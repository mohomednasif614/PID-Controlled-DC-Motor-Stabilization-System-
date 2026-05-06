import pigpio
import time
import csv

# --- PINS & SETTINGS ---
PWMA = 18; AIN1 = 6; AIN2 = 5
ENC_A = 27; ENC_B = 17  
PPR = 350.0  

# TEST PARAMETERS
TEST_PWM_PERCENT = 40.0  # We will change this for each test!
TEST_DURATION = 3.0      # Run for 3 seconds

# --- SETUP ---
pi = pigpio.pi()
if not pi.connected:
    exit("Error: Could not connect to pigpiod.")

for pin in [PWMA, AIN1, AIN2]: pi.set_mode(pin, pigpio.OUTPUT)
for pin in [ENC_A, ENC_B]: 
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_UP)

# --- ENCODER LOGIC ---
encoder_count = 0
def encoder_cb(gpio, level, tick):
    global encoder_count
    encoder_count += 1 
pi.callback(ENC_A, pigpio.RISING_EDGE, encoder_cb)

# --- START TEST ---
pwm_val = int((TEST_PWM_PERCENT / 100.0) * 255)
filename = f"open_loop_{int(TEST_PWM_PERCENT)}percent.csv"

print(f"Starting Open-Loop Step Response Test at {TEST_PWM_PERCENT}% Power...")

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "PWM %", "RPM"])

    # Turn motor ON (Forward)
    pi.write(AIN1, 1); pi.write(AIN2, 0)
    pi.set_PWM_dutycycle(PWMA, pwm_val)

    start_time = time.time()
    last_time = start_time
    
    try:
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            dt = current_time - last_time
            
            if elapsed_time > TEST_DURATION:
                break # Stop the test after 3 seconds
                
            time.sleep(0.05) # Sample very fast (20 Hz) to catch the acceleration curve
            
            rpm = (encoder_count * 60.0) / (PPR * dt)
            encoder_count = 0 
            
            writer.writerow([round(elapsed_time, 3), TEST_PWM_PERCENT, round(rpm, 1)])
            print(f"Time: {elapsed_time:.2f}s | RPM: {rpm:.1f}")
            
            last_time = current_time

    finally:
        print("\nTest complete. Shutting down motor...")
        pi.write(AIN1, 0); pi.write(AIN2, 0)
        pi.set_PWM_dutycycle(PWMA, 0)
        pi.stop()
        print(f"Data saved to {filename}")