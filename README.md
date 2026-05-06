# PID-Controlled-DC-Motor-Stabilization-System-

This repository contains the hardware design and software implementation for a closed-loop motor speed control system. Developed as a group assignment for the Control System Engineering (IE3034) module at SLIIT, the project demonstrates how PID control theory is applied to stabilize a physical DC motor under varying loads and setpoints.

Key Features

Hardware: Integrated a Raspberry Pi 4 with a TB6612FNG motor driver and a 6V N20 geared DC motor.  

Custom PCB: Designed in EasyEDA with high-current trace widths (1mm), power filtering capacitors, and overcurrent protection via PTF-15 fuses.  

PID Implementation: A Python-based controller utilizing the pigpio library for hardware-level timing accuracy.  

Signal Conditioning: Features a 3-point moving average filter to mitigate encoder quantization noise and anti-windup logic to prevent integrator saturation.  

Performance: Achieved a settling time of 1.82 seconds with <1% overshoot and 0.0% steady-state error.
