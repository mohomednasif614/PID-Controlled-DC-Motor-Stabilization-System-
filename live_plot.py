import pigpio
import time
import sys
import matplotlib.pyplot as plt

# --- PIN DEFINITIONS & CONSTANTS ---
PWMA = 18
AIN1 = 6
AIN2 = 5
ENC_A = 17
ENC_B = 27

# UPDATE THIS TO YOUR CALIBRATED NUMBER!
PULSES_PER_REV = 1400.0  

PWM_SPEED = 150
FILTER_SIZE = 5  

# --- SYSTEM SETUP ---
pi = pigpio.pi()
if not pi.connected:
    print("Error: Could not connect to pigpio daemon.")
    sys.exit()

pi.set_mode(PWMA, pigpio.OUTPUT)
pi.set_mode(AIN1, pigpio.OUTPUT)
pi.set_mode(AIN2, pigpio.OUTPUT)
pi.set_mode(ENC_A, pigpio.INPUT)
pi.set_mode(ENC_B, pigpio.INPUT)
pi.set_pull_up_down(ENC_A, pigpio.PUD_UP)
pi.set_pull_up_down(ENC_B, pigpio.PUD_UP)

pulse_count = 0

def encoder_callback(gpio, level, tick):
    global pulse_count
    b_state = pi.read(ENC_B)
    if level == 1:
        if b_state == 0: pulse_count += 1
        else: pulse_count -= 1
    else:
        if b_state == 1: pulse_count += 1
        else: pulse_count -= 1

cb_a = pi.callback(ENC_A, pigpio.EITHER_EDGE, encoder_callback)

# --- MATPLOTLIB LIVE GRAPH SETUP ---
plt.ion() # Turn on interactive mode
fig, ax = plt.subplots(figsize=(10, 6)) # Create a nice, large 10x6 inch window
fig.canvas.manager.set_window_title('Live Motor RPM')

# Lists to store the data for the graph
time_data = []
raw_rpm_data = []
filtered_rpm_data = []
rpm_history = [] # For the moving average math

print("Starting motor... Close the graph window or press Ctrl+C to stop.")

try:
    pi.write(AIN1, 1)
    pi.write(AIN2, 0)
    pi.set_PWM_frequency(PWMA, 1000)
    pi.set_PWM_dutycycle(PWMA, PWM_SPEED)

    start_time = time.time()
    last_time = start_time
    last_count = 0

    # We will keep the graph window open as long as the figure exists
    while plt.fignum_exists(fig.number):
        time.sleep(0.1) # 10 Hz update rate
        
        current_time = time.time()
        current_count = pulse_count 
        
        dt = current_time - last_time
        dpulses = current_count - last_count
        
        # Math calculations
        raw_rpm = (dpulses / PULSES_PER_REV) / dt * 60.0
        rpm_history.append(raw_rpm)
        if len(rpm_history) > FILTER_SIZE:
            rpm_history.pop(0) 
        filtered_rpm = sum(rpm_history) / len(rpm_history)
        
        elapsed_time = current_time - start_time
        
        # Save data to lists
        time_data.append(elapsed_time)
        raw_rpm_data.append(raw_rpm)
        filtered_rpm_data.append(filtered_rpm)
        
        # Keep only the last 100 data points on the screen so it scrolls cleanly
        if len(time_data) > 100:
            time_data.pop(0)
            raw_rpm_data.pop(0)
            filtered_rpm_data.pop(0)
            
        # Draw the updated graph
        ax.clear()
        ax.set_title(f"Live Speed Test (PWM: {PWM_SPEED})")
        ax.set_xlabel("Time (Seconds)")
        ax.set_ylabel("Speed (RPM)")
        
        # Plot both lines with different colors and styles
        ax.plot(time_data, raw_rpm_data, label='Raw Sensor Noises', color='lightgray', linestyle='--')
        ax.plot(time_data, filtered_rpm_data, label='Filtered (Smooth)', color='blue', linewidth=2)
        
        ax.legend(loc="upper left")
        ax.grid(True)
        
        # Pause briefly to let the system draw the screen
        plt.pause(0.01)
        
        last_time = current_time
        last_count = current_count

except KeyboardInterrupt:
    print("\nTest interrupted.")

finally:
    print("Shutting down motor...")
    pi.set_PWM_dutycycle(PWMA, 0)
    pi.write(AIN1, 0)
    pi.write(AIN2, 0)
    cb_a.cancel() 
    pi.stop()
    plt.close() # Close the graph window safely