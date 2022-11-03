# esp32_joint_controller

## Short description
ESP32 motion control firmware for a custom joint control board for my 3D printed collaborative 7DOF robotic arm. 

# Features
- Controls a hybrid stepper motor via TMC2160 
  - Closed Loop Stepper Mode
  - Field Oriented Control Servo Cascade Motion Control
    - Position, Velocity and torque loop
- CAN and UART-Interface
- Processes AS5048A motor and shaft encoder
- Kalman Filter for state estimation (Velocities, Accelerations)
- Neural Network PID Gain-Scheduling
- Neural Network feedforward control
- S-Curve Motion Planning


