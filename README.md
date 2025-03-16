# Quadcopter Drone Project with DC 180 Motors

This project implements a complete control system for a quadcopter drone using an Arduino Nano, MPU-6050 gyroscope, HC-05 Bluetooth module, and DC 180 motors with 5045 propellers.

## Hardware Components

- Arduino Nano
- MPU-6050 gyroscope/accelerometer
- HC-05 Bluetooth module
- 4x DC 180 motors (small brushed motors)
- 4x 5045 propellers
- IRL520N MOSFETs (for motor control)
- 1K and 10K resistors (for MOSFET gate control)
- MT3606 step-up DC-DC converter
- 3.7V 1S LiPo battery

## DC 180 Motor Specifications

The DC 180 motors used in this build are small brushed motors with the following characteristics:
- Operating voltage: 3-5V
- No-load current: ~0.2A at 3.7V
- Stall current: ~1.0A at 3.7V
- Shaft diameter: 1mm
- Motor dimensions: approximately 8.5mm diameter x 20mm length
- Weight: ~5g each

These motors require specific tuning of the PID controller and motor parameters, which has been implemented in the code.

## Advanced Features

This drone control system includes several advanced features for improved stability and safety:

1. **Complementary Filtering**: Combines gyroscope and accelerometer data for more accurate angle estimation and reduced drift.

2. **Emergency Landing System**: Automatically initiates a controlled descent when connection is lost or angles exceed safe limits.

3. **Motor Synchronization**: Monitors and balances motor outputs to compensate for differences between motors.

4. **Motor Test Mode**: Allows testing each motor individually to verify proper operation and identify motor positions.

5. **Auto-Leveling System**: Provides stable hovering and automatically returns to level when controls are released.

6. **MultiWii-Inspired Features**: Includes advanced features from the popular MultiWii flight controller:
   - Multiple flight modes (Normal, Headless, Angle-Limited)
   - Low-pass filtering for smoother response
   - Throttle PID Attenuation (TPA) to prevent oscillations at high throttle
   - Advanced motor mixing table for more consistent control
   
   See the [multiwii_features.md](multiwii_features.md) file for detailed information.

See the [advanced_features.md](advanced_features.md) file for detailed information on these features.

## Circuit Connections

Please follow the circuit diagram provided. Key connections:

1. **MPU-6050 to Arduino:**
   - MPU-6050 SDA → Arduino A4
   - MPU-6050 SCL → Arduino A5
   - MPU-6050 VCC → 5V
   - MPU-6050 GND → GND

2. **HC-05 Bluetooth module to Arduino:**
   - HC-05 TX → Arduino pin 8 (RX)
   - HC-05 RX → Arduino pin 7 (TX)
   - HC-05 VCC → 5V
   - HC-05 GND → GND

3. **Motor Control (MOSFETs):**
   - Arduino pin 3 → Gate of MOSFET for Front Left motor (via 1K resistor)
   - Arduino pin 5 → Gate of MOSFET for Front Right motor (via 1K resistor)
   - Arduino pin 6 → Gate of MOSFET for Rear Left motor (via 1K resistor)
   - Arduino pin 9 → Gate of MOSFET for Rear Right motor (via 1K resistor)
   - Connect 10K resistors between each MOSFET gate and ground
   - Connect motors between battery positive and MOSFET drains
   - MOSFET sources connect to ground

4. **Power Supply:**
   - 3.7V LiPo battery → MT3606 step-up converter
   - MT3606 output (5V) → Arduino VIN and other 5V components

## Required Libraries

1. Wire.h (included with Arduino IDE)
2. MPU6050_6Axis_MotionApps20.h (from the [I2Cdev library](https://github.com/jrowberg/i2cdevlib))
3. RemoteXY.h (from [RemoteXY.com](https://remotexy.com/))

## DC 180 Motor Mounting Tips

1. **Frame Construction:**
   - Use a lightweight frame to maximize flight time (carbon fiber, balsa wood, or even sturdy foam)
   - Ensure the frame is rigid enough to minimize vibrations
   - Ideal motor-to-motor distance: 100-120mm for stability with these motors

2. **Motor Mounting:**
   - Use rubber grommets or foam pads between motors and frame to reduce vibration
   - Ensure motors are securely mounted with zip ties or small screws
   - Verify that all motors are perfectly level for stable flight

3. **Propeller Installation:**
   - Carefully balance all propellers before installation
   - For DC 180 motors, use 5045 propellers (50mm diameter, 4.5mm pitch)
   - Ensure clockwise and counter-clockwise propellers are installed correctly
   - Front-left and rear-right: counter-clockwise propellers
   - Front-right and rear-left: clockwise propellers

## Setup Instructions

1. Install the required libraries in your Arduino IDE
2. Connect the hardware according to the circuit diagram
3. Upload the drone.ino sketch to your Arduino Nano
4. Install the RemoteXY app on your Android device
5. Configure the RemoteXY app according to the instructions in remotexy_config.txt
6. Calibrate the gyroscope before first flight

## Flying Instructions

1. Place the drone on a level surface
2. Connect to the HC-05 module from the RemoteXY app
3. Move the throttle to minimum position
4. Toggle the arm switch to ON
5. Gradually increase throttle to take off
6. Use the control sticks to maneuver the drone
7. To land, gradually decrease throttle
8. Toggle the arm switch to OFF after landing

### Flight Modes

To switch between flight modes:
1. Hold the throttle all the way down
2. Hold the yaw stick all the way right for 1 second
3. The LED will blink to indicate the current mode:
   - 1 blink: Normal mode
   - 2 blinks: Headless mode (orientation independent)
   - 3 blinks: Angle-limited mode (beginner friendly)

## Troubleshooting

- **Motors not spinning:** 
  - Check that throttle is above MOTOR_MIN_VALUE (40 in the code)
  - Verify MOSFET connections and resistor values
  - Ensure battery has sufficient charge

- **Uneven lift or drone flips:**
  - Verify all propellers are correctly installed (CW vs CCW)
  - Check if any motors are weaker than others (replace if necessary)
  - Balance the drone's weight distribution

- **Unstable flight or oscillations:**
  - Reduce PID values further (especially D-term)
  - Check for excessive vibrations from unbalanced propellers
  - Increase mass in the center of the drone for more stability
  - Try using Angle-Limited mode for more stable flight

- **Short flight time:**
  - Use a higher capacity battery (at least 800mAh recommended)
  - Reduce weight where possible
  - Check for excessive current draw from any motor

- **No Bluetooth connection:** 
  - Check HC-05 module wiring and power
  - Verify pairing code (usually 1234)

## Safety Precautions

- Always fly in open areas away from people
- Start with low throttle to test stability
- Always disarm motors immediately after landing
- Keep fingers away from propellers when the drone is armed
- Monitor battery voltage to prevent sudden power loss
- DC 180 motors can overheat - allow cooling time between flights

## Fine-Tuning for DC 180 Motors

The code includes optimized parameters for DC 180 motors. You may need to adjust:

```arduino
// Motor parameters
#define MOTOR_MIN_VALUE 40     // Minimum value to start spinning the DC 180 motors
#define MOTOR_IDLE_VALUE 55    // Idle speed (just enough to keep motors running)
#define THROTTLE_SAFE_START 60 // Minimum throttle to arm (prevent sudden jumps)

// PID control variables
float pid_p_gain_roll = 0.9;   // Roll Proportional gain
float pid_i_gain_roll = 0.02;  // Roll Integral gain
float pid_d_gain_roll = 12.0;  // Roll Derivative gain
```

If the drone is unstable or oscillating, try reducing these values further. If it responds too slowly, gradually increase them. 