import pigpio
import time
import csv
import sys

# --- PINS (Custom PCB) ---
PWMA = 18; AIN1 = 6; AIN2 = 5
ENC_A = 27; ENC_B = 17  

# --- SETTINGS & TUNING ---
PPR = 350.0       

# FINAL OPTIMAL GAINS (PI Controller)
KP = 0.5
KI = 1.7
KD = 0.01

# --- SETUP ---
pi = pigpio.pi()
if not pi.connected: 
    print("Error: Run 'sudo pigpiod' first.")
    sys.exit()

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
    
# --- THE AUTO-PLOTTER FUNCTION ---
def plot_results(filename, title):
    # LAZY IMPORT: Only loads the graph after the motor finishes!
    import matplotlib.pyplot as plt 
    
    print(f"\n>> Loading data and generating graph...")
    plt.close('all') 
    times, setpoints, rpms = [], [], []
    
    try:
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            next(reader) # Skip header
            for row in reader:
                times.append(float(row[0]))
                setpoints.append(float(row[1]))
                rpms.append(float(row[2]))
                
        if len(times) < 2:
            print(">> Error: Not enough data in CSV. The motor test may have failed.")
            return

        TARGET = setpoints[-1] if setpoints[-1] != 0 else 100.0
        upper_bound = TARGET * 1.02
        lower_bound = TARGET * 0.98

        # --- CALCULATE RUBRIC METRICS ---
        max_rpm = max(rpms)
        overshoot_pct = ((max_rpm - TARGET) / TARGET) * 100
        if overshoot_pct < 0: overshoot_pct = 0.0

        ss_rpm = sum(rpms[-10:]) / 10 if len(rpms) >= 10 else rpms[-1]
        ss_error_pct = (abs(TARGET - ss_rpm) / TARGET) * 100

        settling_time = None
        for i in range(len(rpms)-1, -1, -1):
            if rpms[i] > upper_bound or rpms[i] < lower_bound:
                if i < len(times) - 1:
                    settling_time = times[i+1]
                break
        if settling_time is None: settling_time = times[0]

        t_10 = next((times[i] for i, r in enumerate(rpms) if r >= TARGET * 0.1), None)
        t_90 = next((times[i] for i, r in enumerate(rpms) if r >= TARGET * 0.9), None)
        rise_time = (t_90 - t_10) if (t_90 is not None and t_10 is not None) else 0.0

        # --- CREATE THE PLOT ---
        plt.figure(figsize=(10, 6))
        plt.fill_between(times, lower_bound, upper_bound, color='gray', alpha=0.2, label='+/- 2% Settling Band')
        plt.plot(times, rpms, label='Motor Speed (RPM)', color='blue', linewidth=2)
        plt.axhline(y=TARGET, color='red', linestyle='--', label='Target (Setpoint)')

        stats_text = (
            f"--- Performance Metrics ---\n"
            f"Rise Time: {rise_time:.2f} s\n"
            f"Settling Time: {settling_time:.2f} s\n"
            f"Max Overshoot: {overshoot_pct:.1f}%\n"
            f"Steady-State Error: {ss_error_pct:.1f}%"
        )
        plt.text(0.95, 0.05, stats_text, transform=plt.gca().transAxes, fontsize=12,
                 verticalalignment='bottom', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        plt.title(title, fontsize=14, fontweight='bold')
        plt.xlabel('Time (Seconds)', fontsize=12)
        plt.ylabel('Speed (RPM)', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend(loc='upper right')

        image_name = filename.replace('.csv', '_plot.png')
        plt.savefig(image_name, dpi=300, bbox_inches='tight')
        
        print(f">> Graph opened! Close the graph window to return to the Main Menu.")
        plt.show()
        
    except Exception as e:
        print(f">> Critical Error generating plot: {e}")
        
    # --- THE CORE PID CONTROLLER FUNCTION ---
def run_motor_test(setpoint, duration, filename="test_data.csv"):
    global encoder_count
    integral = 0
    prev_error = 0
    encoder_count = 0 
    
    rpm_history = []
    FILTER_SIZE = 3  

    print(f"\n>> Starting Test: Target = {setpoint} RPM | Duration = {duration}s")
    print(">> Press Ctrl+C to stop early if needed.\n")

    try:
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Setpoint", "RPM"])

            time.sleep(0.5) 
            start_time = time.time()
            last_time = start_time

            while True:
                time.sleep(0.1) 
                
                current_time = time.time()
                dt = current_time - last_time
                if dt <= 0: dt = 0.001 
                
                elapsed_time = current_time - start_time
                
                if elapsed_time >= duration:
                    print(f"\n>> Test completed successfully ({duration}s).")
                    break 
                
                raw_rpm = (encoder_count * 60.0) / (PPR * dt)
                encoder_count = 0 
                
                rpm_history.append(raw_rpm)
                if len(rpm_history) > FILTER_SIZE:
                    rpm_history.pop(0) 
                rpm = sum(rpm_history) / len(rpm_history) 
                
                error = setpoint - rpm
                integral = integral + (error * dt)
                integral = max(min(integral, 150), -150) 
                derivative = (error - prev_error) / dt
                
                output = (KP * error) + (KI * integral) + (KD * derivative)
                set_motor(output)
                
                writer.writerow([round(elapsed_time, 3), setpoint, round(rpm, 1)])
                print(f"Time: {elapsed_time:.1f}s | Target: {setpoint} | RPM: {rpm:5.1f}")
                
                prev_error = error
                last_time = current_time

    except KeyboardInterrupt:
        print("\n>> Test interrupted manually.")
    except Exception as e:
        print(f"\n>> CRITICAL MOTOR ERROR: {e}") 
    finally:
        set_motor(0) 
        print(f">> Motor stopped safely. Data saved to '{filename}'.")
        print(">> Preparing graph...")
        time.sleep(1)
        
# --- THE MENU INTERFACE ---
def main_menu():
    while True:
        print("\n" + "="*45)
        print("      DC MOTOR CONTROL SYSTEM - MAIN MENU")
        print("="*45)
        print("1. Run Standard PID Test (100 RPM, 10s) [Module 3]")
        print("2. Run Custom Target Speed Test         [Module 4]")
        print("3. Run Load Disturbance Test (15s)      [Module 4]")
        print("4. Exit System")
        print("="*45)
        
        choice = input("Select a test mode (1-4): ")
        
        if choice == '1':
            filename = "mod3_standard.csv"
            run_motor_test(setpoint=100.0, duration=10.0, filename=filename)
            plot_results(filename, "Module 3: Standard PID Step Response")
            
        elif choice == '2':
            try:
                speed = float(input("\nEnter Target RPM (e.g., 75, 125): "))
                filename = f"mod4_speed_{int(speed)}.csv"
                run_motor_test(setpoint=speed, duration=10.0, filename=filename)
                plot_results(filename, f"Module 4: Target Speed Test ({speed} RPM)")
            except ValueError:
                print("\nInvalid input. Please enter numbers only.")
                
        elif choice == '3':
            print("\nGet ready to manually pinch the motor shaft after 3 seconds!")
            filename = "mod4_disturbance.csv"
            run_motor_test(setpoint=100.0, duration=15.0, filename=filename)
            plot_results(filename, "Module 4: Load Disturbance Recovery")
            
        elif choice == '4':
            print("\nShutting down system. Goodbye!")
            set_motor(0)
            pi.stop()
            sys.exit()
            
        else:
            print("\nInvalid choice. Please select a number between 1 and 4.")

if __name__ == "__main__":
    main_menu()