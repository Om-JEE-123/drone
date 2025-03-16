# Drone Wiring Diagram Explanation

This document provides detailed explanations of the wiring connections shown in the circuit diagram.

## Main Components

1. **Arduino Nano**
   - Serves as the main controller
   - Processes gyroscope data and remote control inputs
   - Controls motors via PWM signals to MOSFETs

2. **MPU-6050 Gyroscope/Accelerometer**
   - Connected to Arduino via I2C (A4/SDA and A5/SCL)
   - Provides orientation data for stabilization

3. **HC-05 Bluetooth Module**
   - Connected to Arduino digital pins 7 and 8
   - Enables wireless control via the RemoteXY app

4. **IRL520N MOSFETs (4x)**
   - Act as electronic switches for the motors
   - Each controlled by a PWM signal from Arduino
   - Enable variable speed control of motors

5. **DC 180 Motors (4x)**
   - Front Left, Front Right, Rear Left, Rear Right
   - Connected to MOSFETs for speed control

6. **MT3606 Step-up Converter**
   - Boosts the 3.7V battery voltage to 5V
   - Powers the Arduino and other components

## Detailed Connections

### Power Supply Connections
1. 3.7V Battery connects to:
   - MT3606 step-up converter input
   - Positive terminal to all four motors

2. MT3606 5V output connects to:
   - Arduino Nano VIN pin
   - HC-05 VCC pin
   - MPU-6050 VCC pin

3. Ground connections:
   - Arduino GND
   - HC-05 GND
   - MPU-6050 GND
   - MOSFET sources
   - MT3606 GND
   - Battery negative terminal

### Arduino to MPU-6050 Connections
1. Arduino A4 (SDA) → MPU-6050 SDA
2. Arduino A5 (SCL) → MPU-6050 SCL
3. Arduino 5V → MPU-6050 VCC
4. Arduino GND → MPU-6050 GND

### Arduino to HC-05 Connections
1. Arduino Digital Pin 7 (TX) → HC-05 RX
2. Arduino Digital Pin 8 (RX) → HC-05 TX
3. Arduino 5V → HC-05 VCC
4. Arduino GND → HC-05 GND

### Arduino to MOSFETs Connections
1. Front Left Motor:
   - Arduino Digital Pin 3 → 1K resistor → MOSFET Gate
   - 10K resistor between MOSFET Gate and GND
   - Motor connected between battery+ and MOSFET Drain
   - MOSFET Source connected to GND

2. Front Right Motor:
   - Arduino Digital Pin 5 → 1K resistor → MOSFET Gate
   - 10K resistor between MOSFET Gate and GND
   - Motor connected between battery+ and MOSFET Drain
   - MOSFET Source connected to GND

3. Rear Left Motor:
   - Arduino Digital Pin 6 → 1K resistor → MOSFET Gate
   - 10K resistor between MOSFET Gate and GND
   - Motor connected between battery+ and MOSFET Drain
   - MOSFET Source connected to GND

4. Rear Right Motor:
   - Arduino Digital Pin 9 → 1K resistor → MOSFET Gate
   - 10K resistor between MOSFET Gate and GND
   - Motor connected between battery+ and MOSFET Drain
   - MOSFET Source connected to GND

## Understanding MOSFET Control

The 1K and 10K resistors create a simple gate driver circuit for each MOSFET:
- The 1K resistor limits current from the Arduino to the MOSFET gate
- The 10K resistor pulls the gate to ground when the Arduino pin is LOW
- When Arduino outputs PWM, the MOSFET switches at that duty cycle
- This controls the effective power to each motor

## Important Safety Notes

1. **Double-check all connections** before powering on the circuit
2. Add a power switch between the battery and circuit
3. Consider adding a low-voltage cutoff circuit to protect the battery
4. Ensure all ground connections are properly made
5. Use appropriate wire gauge for the motor connections (18-22 AWG recommended)
6. Add capacitors (100-220μF) across motor terminals to reduce noise

## Troubleshooting Wiring Issues

1. **Motors not responding:**
   - Check PWM signals at Arduino pins
   - Verify MOSFET gate voltage (should be 3-5V when ON)
   - Check resistance values

2. **Erratic behavior:**
   - Check ground connections
   - Verify MPU-6050 connections
   - Add filtering capacitors

3. **Communication issues:**
   - Verify HC-05 connections
   - Check TX/RX are not reversed
   - Verify baud rate settings 