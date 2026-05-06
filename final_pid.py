import pigpio
import time
import csv

# --- PINS (Custom PCB) ---
PWMA = 18; AIN1 = 6; AIN2 = 5
ENC_A = 27; ENC_B = 17  

# --- SETTINGS & TUNING ---
SETPOINT = 100.0  
PPR = 350.0       

# TEST 1 GAINS (Initial Baseline)
KP = 0.5
KI = 1.0
KD = 0.0

# --- SETUP ---
pi = pigpio.pi()
if not pi.connected: exit("Error: Run 'sudo pigpiod'.")

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

# --- MOTOR CONTROLLER ---
def set_motor(power):
    power = max(min(power, 100), -100) 
    if power > 0:   pi.write(AIN1, 1); pi.write(AIN2, 0)
    elif power < 0: pi.write(AIN1, 0); pi.write(AIN2, 1)
    else:           pi.write(AIN1, 0); pi.write(AIN2, 0)
    pi.set_PWM_dutycycle(PWMA, int(abs(power) * 2.55))

# --- MAIN PID LOOP WITH DATA LOGGING ---
integral = 0
prev_error = 0
start_time = time.time()
last_time = start_time

print(f"Running PID Test. Target: {SETPOINT} RPM")
print(f"Gains -> KP: {KP}, KI: {KI}, KD: {KD}")
print("Press Ctrl+C to stop after 6 seconds...")

with open('motor_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Setpoint", "RPM"])

    try:
        while True:
            time.sleep(0.1) # 10 Hz loop
            
            current_time = time.time()
            dt = current_time - last_time
            elapsed_time = current_time - start_time
            
            # Read Sensor
            rpm = (encoder_count * 60.0) / (PPR * dt)
            encoder_count = 0 
            
            # Calculate Errors
            error = SETPOINT - rpm
            integral = integral + (error * dt)
            integral = max(min(integral, 150), -150) # Anti-windup
            derivative = (error - prev_error) / dt
            
            # Apply PID
            output = (KP * error) + (KI * integral) + (KD * derivative)
            set_motor(output)
            
            # Save Data
            writer.writerow([round(elapsed_time, 3), SETPOINT, round(rpm, 1)])
            print(f"Time: {elapsed_time:.1f}s | RPM: {rpm:5.1f} | Error: {error:5.1f}")
            
            prev_error = error
            last_time = current_time

    except KeyboardInterrupt:
        print("\nStopping motor...")
        set_motor(0) 
        pi.stop()
        print("Data saved to motor_data.csv!")