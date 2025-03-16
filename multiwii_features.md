# MultiWii-Inspired Features for DC 180 Motor Drone

This document explains the advanced features implemented from MultiWii flight controller firmware to enhance the performance and usability of your DC 180 motor drone.

## What is MultiWii?

MultiWii is a popular open-source flight controller software originally developed for Arduino boards using gyroscopes from Nintendo Wii controllers. It has evolved into a sophisticated platform for multirotor aircraft and is known for its stability and customizability.

## Key Features Implemented

### 1. Low-Pass Filtering (LPF)

```arduino
#define LPF_FACTOR 0.8  // Low-pass filter factor for gyro data
gyro_roll_lpf = gyro_roll_lpf * LPF_FACTOR + gyro_roll_input_raw * (1 - LPF_FACTOR);
```

**Benefits:**
- Smooths out sensor noise and vibrations
- Reduces "jitters" during flight
- Creates more predictable drone behavior
- Especially helpful for small DC 180 motors which can produce high-frequency vibrations

### 2. Multiple Flight Modes

Three flight modes have been implemented:

1. **Normal Mode** - Standard flying mode with full control
2. **Headless Mode** - Orientation-independent control (the drone moves relative to the pilot, not the drone's heading)
3. **Angle-Limited Mode** - Restricted control angles perfect for beginners

**How to Switch Modes:**
- Hold the throttle stick all the way down and the yaw stick all the way right for 1 second
- The LED will blink to indicate the current mode (1 blink = Normal, 2 blinks = Headless, 3 blinks = Angle-Limited)

### 3. Throttle PID Attenuation (TPA)

```arduino
float tpa_scale = 1.0 - constrain((throttle - TPA_BREAKPOINT) / 
                               (float)(MOTOR_MAX_VALUE - TPA_BREAKPOINT), 0.0, 1.0) * TPA_FACTOR;
```

**Benefits:**
- Automatically reduces PID gains at higher throttle settings
- Prevents oscillations when motors are running at high power
- Particularly useful for small DC 180 motors that can become "twitchy" at high throttle
- Makes tuning easier as PID values work well across the entire throttle range

### 4. Motor Mixing Table

```arduino
const float MOTOR_MIX[4][3] = {
  // ROLL,    PITCH,    YAW
  {  1.0,     1.0,     -1.0 },    // FRONT LEFT
  { -1.0,     1.0,      1.0 },    // FRONT RIGHT
  {  1.0,    -1.0,      1.0 },    // REAR LEFT
  { -1.0,    -1.0,     -1.0 }     // REAR RIGHT
};
```

**Benefits:**
- More consistent motor response
- Easier tuning and maintenance
- Better handling of complex flight maneuvers
- Improved stability during quick movements

### 5. Headless Mode

This mode makes the drone fly relative to the pilot regardless of which way the front of the drone is pointing.

**How it Works:**
- When you first arm the drone, it records the initial orientation as a reference
- All stick inputs are then interpreted relative to this initial orientation
- Makes flying much easier for beginners who struggle with orientation

## Optimal Settings for DC 180 Motors

These MultiWii-inspired features have been carefully tuned for DC 180 motors:

- **LPF_FACTOR**: Set to 0.8 for good balance between smoothing and responsiveness
- **TPA_FACTOR**: Set to 0.2 (20% reduction at full throttle)
- **TPA_BREAKPOINT**: Starts at throttle value 150 (about 60% throttle)
- **Mixing Scale**: Motor mixing scaled to 0.75 for DC 180 motors to prevent over-correction

## Tuning Guidelines

If you want to adjust these features:

1. **LPF_FACTOR**:
   - Higher values (closer to 1.0) = more filtering but more lag
   - Lower values (closer to 0.0) = less filtering but more responsive
   - Increase if you notice jittering, decrease if response feels delayed

2. **TPA_FACTOR**:
   - Higher values = more PID reduction at high throttle
   - Lower values = less PID reduction at high throttle
   - Increase if you notice oscillations at high throttle

3. **TPA_BREAKPOINT**:
   - Determines at what throttle level TPA starts to take effect
   - Lower value = TPA affects more of the throttle range
   - Higher value = TPA only affects high throttle

## Benefits for Your DC 180 Motor Drone

The combination of these MultiWii-inspired features provides:

1. **Smoother Flight** - Reduced vibrations and oscillations
2. **Better Stability** - More consistent PID control across all throttle ranges
3. **Easier Control** - Multiple flight modes for different skill levels
4. **Longer Motor Life** - Smoother operation means less stress on motors
5. **Better Battery Efficiency** - More precise control uses less power

These features make your drone easier to fly, more stable, and better suited to the characteristics of DC 180 motors. 