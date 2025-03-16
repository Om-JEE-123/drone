/*
 * Quadcopter Drone Control 
 * 
 * Hardware:
 * - Arduino Nano
 * - MPU-6050 gyroscope/accelerometer
 * - HC-05 Bluetooth module
 * - 4x DC 180 motors with 5045 propellers
 * - IRL520N MOSFETs for motor control
 * - MT3606 step-up DC-DC booster
 * - Battery: 3.7v 1S
 * 
 * Uses RemoteXY for Android app control
 */

//============= LIBRARIES =============
#include <Wire.h>              // I2C communication
#include <RemoteXY.h>          // RemoteXY library
#include <MPU6050.h>          // Lighter MPU6050 library (no DMP)
#include <SoftwareSerial.h>    // Software Serial for Bluetooth communication
#include <avr/pgmspace.h>      // Required for PROGMEM
#include <avr/power.h>         // Power management for AVR
#include <I2Cdev.h>
#include <EEPROM.h>
#include "remotexy.h"

// Memory optimization for Arduino Nano
#define USE_REDUCED_MEMORY 1

// Conditionally compile features based on available memory
#if USE_REDUCED_MEMORY
  #define MAX_DIAG_HISTORY 1       // Reduced history for diagnostics
  #define GYRO_CALIBRATION_SAMPLES 1000  // Reduced calibration samples
#else
  #define MAX_DIAG_HISTORY 3
  #define GYRO_CALIBRATION_SAMPLES 2000
#endif

//============= REMOTEXY CONFIGURATION =============
// RemoteXY connection settings
#define REMOTEXY_MODE__SOFTWARESERIAL
#define REMOTEXY_SERIAL_RX BLUETOOTH_RX
#define REMOTEXY_SERIAL_TX BLUETOOTH_TX
#define REMOTEXY_SERIAL_SPEED 9600
#define REMOTEXY_ACCESS_PASSWORD ""

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF_DATA[] =   // 138 bytes
  { 255,5,0,0,0,131,0,16,31,1,
  2,0,68,25,22,11,2,26,31,31,
  79,78,0,79,70,70,0,4,0,18,
  49,12,12,2,26,2,0,68,9,22,
  11,2,26,31,31,79,78,0,79,70,
  70,0,5,0,5,60,40,40,2,26,31,
  5,32,30,30,30,30,2,26,2,1,
  44,9,12,12,2,26,129,0,5,8,
  18,6,17,84,104,114,111,116,116,108,
  101,0,129,0,44,26,27,6,17,89,
  97,119,0,129,0,4,73,14,6,17,
  80,105,116,99,104,0,129,0,56,73,
  12,6,17,82,111,108,108,0,67,4,
  12,89,32,7,2,26,24 };
  
// This structure defines all the variables and events of your control interface 
struct {
  // Input variables
  uint8_t arm_switch; // =1 if switch ON and =0 if OFF
  int8_t throttle_joystick; // from -100 to 100
  uint8_t calibrate_switch; // =1 if switch ON and =0 if OFF
  int8_t yaw_joystick; // from -100 to 100, horizontal
  int8_t roll_joystick; // from -100 to 100, horizontal
  int8_t pitch_joystick; // from -100 to 100, vertical
  uint8_t connect_flag;  // =1 if wire connected, else =0

  // Add missing joystick variables for processRadioControl function
  int8_t joystick_1_x; // from -100 to 100
  int8_t joystick_1_y; // from -100 to 100
  int8_t joystick_2_x; // from -100 to 100
  int8_t joystick_2_y; // from -100 to 100

  // Output variables
  uint8_t led_power;
  uint8_t battery_level;
  int8_t roll_angle;  // to display current roll angle
  int8_t pitch_angle; // to display current pitch angle
  char flight_time[8]; // to display flight time

} RemoteXY;
#pragma pack(pop)

// Create RemoteXY object
RemoteXY remotexy;  // Use the RemoteXY structure directly

// Fix RemoteXY initialization
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600
#define REMOTEXY_BLUETOOTH_RX 7
#define REMOTEXY_BLUETOOTH_TX 8

// Add proper PID structure
struct PIDData {
    float P, I, D;
    float lastError;
    float errorSum;
    float maxI;
    float maxOutput;
};

PIDData rollPID = {0.6, 0.01, 8.0, 0, 0, 50, 150};
PIDData pitchPID = {0.6, 0.01, 8.0, 0, 0, 50, 150};
PIDData yawPID = {1.8, 0.005, 0.0, 0, 0, 50, 150};

// Fix unresolved remotexy reference
RemoteXY_Class remotexy(RemoteXY_CONF_DATA, &REMOTEXY_SERIAL);

// Add gyro initialization check
bool initGyro() {
    Wire.begin();
    delay(100); // Give MPU time to startup
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }
    
    // Set gyro full scale range
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    
    return true;
}

// Fix motor timing control
void writeMotorsSafely() {
    static unsigned long lastWrite = 0;
    const unsigned long MIN_WRITE_INTERVAL = 2000; // microseconds
    
    unsigned long now = micros();
    if (now - lastWrite < MIN_WRITE_INTERVAL) {
        delayMicroseconds(MIN_WRITE_INTERVAL - (now - lastWrite));
    }
    
    writeMotors();
    lastWrite = micros();
}

// Add proper type definitions
typedef struct {
    int16_t raw;
    float filtered;
    float offset;
} GyroAxis;

GyroAxis gyroX, gyroY, gyroZ;

// Fix missing function declarations
void processRadioControl();
void updateIMU();
void calculatePID();
void mixMotors();
void writeMotors();
void check_failsafe();
float applyExpo(float input, float expo_factor);
void handle_emergency_landing();

//============= PIN DEFINITIONS =============
// Motor control pins (PWM)
#define MOTOR_FL_PIN  3   // Front Left motor pin (Timer2)
#define MOTOR_FR_PIN 5   // Front Right motor pin (Timer0)
#define MOTOR_BL_PIN   6   // Rear Left motor pin (Timer0)
#define MOTOR_BR_PIN  9   // Rear Right motor pin (Timer1)

// HC-05 Bluetooth module pins
#define BLUETOOTH_TX 7   // Connect to RX of HC-05
#define BLUETOOTH_RX 8   // Connect to TX of HC-05

// LED indicator pin (if available)
#define LED_PIN 13       // Built-in LED on Arduino Nano

// Battery voltage monitoring pin
#define BATTERY_PIN A0   // Analog pin for battery voltage monitoring

//============= MOTOR PARAMETERS =============
// DC 180 Motor specific parameters (tuned for safer first flight)
#define MOTOR_MIN_VALUE 45     // Minimum value to start spinning the DC 180 motors (increased for reliability)
#define MOTOR_IDLE_VALUE 60    // Idle speed (just enough to keep motors running but not lifting)
#define MOTOR_MAX_VALUE 220    // Maximum safe motor value for first flight (reduced from 240)
#define THROTTLE_SAFE_START 65 // Minimum throttle to arm (prevent sudden jumps)
#define MOSFET_DELAY_US 2      // Microseconds delay for MOSFET switching (deadtime)

//============= BATTERY MONITORING =============
#define R1 100000.0            // 100K resistor from battery to analog pin (for voltage divider)
#define R2 10000.0             // 10K resistor from analog pin to ground
#define VOLTAGE_DIVIDER_RATIO ((R1 + R2) / R2)  // Calculated divider ratio
#define BATTERY_LOW_VOLTAGE 3.5     // Low battery voltage threshold for 1S LiPo (Volts)
#define BATTERY_CRITICAL_VOLTAGE 3.3 // Critical battery voltage threshold for 1S LiPo (Volts)
#define BATTERY_CHECK_INTERVAL 5000  // Battery check interval in milliseconds (5 seconds)

//============= CONTROL PARAMETERS =============
#define EXPO_FACTOR 0.7        // Increased exponential curve factor for smoother control during first flight
#define THROTTLE_CURVE_FACTOR 0.9  // Higher values give more sensitivity at low throttle
#define YAW_CONTROL_DEADBAND 8     // Increased deadband for yaw to prevent unwanted rotation on first flight
#define ROLL_PITCH_DEADBAND 5      // Increased deadband for roll and pitch to prevent drift
#define LPF_FACTOR 0.85         // Increased low-pass filter factor for smoother response
#define TPA_FACTOR 0.2         // Throttle PID Attenuation factor (MultiWii-inspired)
#define TPA_BREAKPOINT 150     // Throttle breakpoint for PID attenuation
#define ANTIGRAVITY_FACTOR 0.1 // Anti-gravity compensation factor (MultiWii-inspired)
#define THROTTLE_EXPO 0.2      // Throttle expo for more precise low throttle control
#define SUPER_EXPO_FACTOR 0.7  // Super expo factor for even more precise stick control

//============= FLIGHT MODES =============
#define MODE_NORMAL 0          // Normal flight mode
#define MODE_HEADLESS 1        // Headless mode (orientation independent)
#define MODE_ANGLE_LIMIT 2     // Angle-limited mode (beginner mode)
int flight_mode = MODE_ANGLE_LIMIT; // Default to angle-limited mode for first flight

//============= FAILSAFE PARAMETERS =============
#define CONNECTION_TIMEOUT 1000    // Milliseconds before triggering connection failsafe
#define AUTO_DISARM_TIMEOUT 30000  // Auto-disarm after 30 seconds of inactivity (if armed)

//============= ADVANCED PARAMETERS =============
#define MOTOR_SYNC_INTERVAL 10000  // Motor synchronization interval (10 seconds)
#define ACCEL_WEIGHT 0.02          // Weight for accelerometer in complementary filter
#define GYRO_WEIGHT 0.98           // Weight for gyroscope in complementary filter
#define MAX_ANGLE_LIMIT 20.0       // Maximum allowed angle in degrees (reduced for first flight)
#define LANDING_DESCENT_RATE 5     // Rate of descent during emergency landing
#define LANDING_TIMEOUT 3000       // Time for emergency landing (3 seconds)
#define MOTOR_DIFFERENCE_LIMIT 40  // Maximum allowed difference between motors
#define MOSFET_DEADTIME 2          // Microseconds of deadtime for MOSFET switching
#define MOTOR_SPIN_UP_TIME 500     // Time in ms to gradually increase motor speed on arm

//============= MPU6050 SETUP =============
MPU6050 mpu;

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

// Add missing MPU6050 variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accel_x, accel_y, accel_z;
float acc_total_vector;
float angle_roll, angle_pitch;
boolean gyro_initialized = false;
float acc_roll, acc_pitch;

// Add missing constants
#define GYRO_SENSITIVITY 131.0  // For 250 deg/s range
#define ACCEL_SENSITIVITY 16384.0  // For 2g range
#define MAX_SAFE_ANGLE 60.0  // Maximum safe angle before failsafe

// Add missing control variables
float roll_setpoint = 0;
float pitch_setpoint = 0;
float yaw_setpoint = 0;
float throttle_smooth = 0;
float yaw = 0;
float pitch = 0;
float roll = 0;

// Add missing failsafe variables
int failsafe_counter = 0;
String last_failsafe_reason = "";
boolean is_failsafe_mode = false;
unsigned long emergency_landing_timer = 0;

// Add missing debug flag
#define ENABLE_DEBUG true

// PID control variables - Fine-tuned for DC 180 motors
float pid_p_gain_roll = 0.6;   // Roll Proportional gain (reduced for first flight stability)
float pid_i_gain_roll = 0.01;  // Roll Integral gain (reduced to prevent oscillation)
float pid_d_gain_roll = 8.0;   // Roll Derivative gain (reduced for first flight)
int pid_max_roll = 150;        // Maximum output of the PID controller (reduced for first flight)

float pid_p_gain_pitch = 0.6;  // Pitch Proportional gain
float pid_i_gain_pitch = 0.01; // Pitch Integral gain
float pid_d_gain_pitch = 8.0;  // Pitch Derivative gain
int pid_max_pitch = 150;       // Maximum output of the PID controller

float pid_p_gain_yaw = 1.8;    // Yaw Proportional gain (reduced for first flight)
float pid_i_gain_yaw = 0.005;  // Yaw Integral gain
float pid_d_gain_yaw = 0.0;    // Yaw Derivative gain
int pid_max_yaw = 150;         // Maximum output of the PID controller

// PID calculation variables
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

// Motor variables
volatile int motor_fl = 0;  // Motor 1 speed
volatile int motor_fr = 0; // Motor 2 speed
volatile int motor_bl = 0;   // Motor 3 speed
volatile int motor_br = 0;  // Motor 4 speed
int throttle = 0;
boolean motors_armed = false;

// Calibration variables
boolean calibration_mode = false;   // Gyro calibration mode
int calibration_counter = 0;        // Calibration sample counter
float gyro_x_cal, gyro_y_cal, gyro_z_cal;

// Timing variables
unsigned long loop_timer;
unsigned long time_now;
unsigned long battery_timer;
unsigned long connection_timer;
unsigned long last_activity_timer;
unsigned long led_timer;

// Status variables
boolean low_battery = false;
boolean critical_battery = false;
boolean connection_lost = false;
boolean led_state = false;
float battery_voltage = 0.0;
boolean low_battery_warning = false; // Flag to track if low battery warning was displayed

// Advanced control variables
unsigned long motor_sync_timer = 0;
boolean auto_level_enabled = true;
boolean emergency_landing = false;
float accel_roll, accel_pitch;
float filtered_roll, filtered_pitch;
int motor_test_sequence = 0;        // Current test sequence step
unsigned long motor_test_timer = 0; // Timer for test sequence
boolean motor_test_mode = false;    // Motor test mode
int motor_differences[4] = {0, 0, 0, 0};  // Store motor differences for balancing

// Add after existing variables
boolean first_arm = true;           // First arm after power-up
unsigned long last_gyro_read = 0;   // Track last successful gyro read
boolean preflight_check_done = false; // Flag to indicate if preflight checks passed

// Angle and heading references for headless mode
float initial_yaw = 0.0;
boolean headless_reference_set = false;

// LPF filtered values
float gyro_roll_input_raw, gyro_pitch_input_raw, gyro_yaw_input_raw;
float gyro_roll_lpf, gyro_pitch_lpf, gyro_yaw_lpf;

// Motor mixing table (MultiWii-inspired)
const float MOTOR_MIX[4][3] = {
  // ROLL,    PITCH,    YAW
  {  1.0,     1.0,     -1.0 },    // FRONT LEFT
  { -1.0,     1.0,      1.0 },    // FRONT RIGHT
  {  1.0,    -1.0,      1.0 },    // REAR LEFT
  { -1.0,    -1.0,     -1.0 }     // REAR RIGHT
};

// Add to global variable declarations
int landing_throttle = 0; // Throttle value during emergency landing

// Add after existing variables
boolean anti_gravity_enabled = true;  // Enable anti-gravity feature for better maneuverability
float antigravity_gain = 0.0;         // Current anti-gravity gain value
float throttle_boost = 0.0;           // Boost for quick throttle changes
float throttle_smooth = 0.0;          // Smoothed throttle value

// Add to global variables
boolean first_flight_mode = true;  // Enable first flight safety features
unsigned long hover_start_time = 0; // Track hover time for first flight
boolean hover_warning_given = false; // Track if hover warning was given
boolean led_warning_active = false; // Track if LED warning is active
unsigned long led_warning_start = 0; // Track start time of LED warning
boolean led_mode_change_active = false; // Track if mode change LED sequence is active
unsigned long led_mode_change_start = 0; // Track start time of mode change sequence
int led_mode_change_count = 0; // Number of blinks needed for mode change

// Add to global variables
boolean led_emergency_active = false; // Track if emergency LED pattern is active
unsigned long led_emergency_start = 0; // Track start time of emergency LED pattern

// Add to global variables
unsigned long calibrate_hold_timer = 0; // Timer for calibrate button hold

// Add to global variables
#define DYNAMIC_FILTER_ENABLED true  // Enable dynamic filtering
float noise_threshold_roll = 8.0;   // Threshold above which gyro noise is considered high 
float noise_threshold_pitch = 8.0;
float noise_threshold_yaw = 10.0;
float gyro_noise_roll = 0;          // Detected noise levels
float gyro_noise_pitch = 0;
float gyro_noise_yaw = 0;
boolean high_gyro_noise = false;    // Flag for high noise conditions
float filter_gain_scale = 1.0;      // Scale factor for filter strength

// Add to global variables section
// Accelerometer calibration values
int16_t acc_1G = 16384;       // This is the 1G reference value for the MPU6050 with +-2G range
float acc_offset[3] = {0, 0, 0};  // Offsets for each axis (x, y, z)
float acc_gain[3] = {1.0, 1.0, 1.0};  // Gain adjustments for each axis
boolean acc_calibrated = false;   // Flag to indicate if accelerometer is calibrated

// Add to global variables
#define AUTO_LEVEL_STRENGTH 0.8f  // Strength of the auto-level correction (0.0 to 1.0)
#define AUTO_LEVEL_MAX_ANGLE 30.0  // Maximum auto-level influence in degrees
float auto_level_roll = 0;       // Auto-level correction for roll
float auto_level_pitch = 0;      // Auto-level correction for pitch

// Add to global variables
// Power management variables
#define POWER_MANAGEMENT_ENABLED true   // Enable power management features
#define BATTERY_PERFORMANCE_COMPENSATION true // Enable battery sag compensation
float battery_full_voltage = 4.2;       // Full battery voltage
float battery_full_throttle_scale = 1.0; // Scale factor at full voltage
float battery_low_throttle_scale = 1.3; // Max scale factor at low voltage
float power_throttle_scale = 1.0;       // Current throttle scaling factor
float battery_capacity_remaining = 1.0;  // Estimate of remaining capacity (0.0 to 1.0)
unsigned long power_check_timer = 0;     // Timer for power management updates

// Add to global variables
#define MOTOR_DIAGNOSTICS_ENABLED true // Enable motor diagnostics
#define MAX_DIAG_DURATION 10000       // Maximum diagnostic run time (10 seconds)
#define DIAG_MIN_THROTTLE MOTOR_MIN_VALUE + 10 // Minimum power for diagnostics
#define DIAG_MAX_THROTTLE MOTOR_MIN_VALUE + 30 // Maximum power for diagnostics
boolean motor_diagnostics_active = false;    // Flag for active diagnostic
uint8_t current_diag_motor = 0;             // Currently tested motor (0-3)
unsigned long motor_diag_start_time = 0;     // Start time of diagnostics
unsigned long motor_diag_step_time = 0;      // Timing of test steps
uint8_t motor_diag_step = 0;                // Current step in the test
float motor_variance[4] = {0, 0, 0, 0};     // Measured variance in motor output
boolean motor_overheat_detected = false;    // Overheat detection flag

// Add to global variables section
#define MAX_DIAG_HISTORY 3  // Store last 3 diagnostic results
#if USE_REDUCED_MEMORY
// Simplified diagnostic history
float motor_variance_history[MAX_DIAG_HISTORY][4] = {{0}};  // Less history tracked
#else
// Full diagnostic history
float motor_variance_history[MAX_DIAG_HISTORY][4] = {{0}};
#endif
unsigned long diag_history_time[MAX_DIAG_HISTORY] = {0};  // When diagnostics were run
uint8_t current_history_index = 0;  // Current position in history array

// Add to global variables section
boolean motor_stall_detected[4] = {false, false, false, false};  // Track motor stalls

// Add to global variables section
unsigned long motor_response_time[4] = {0, 0, 0, 0};  // Motor response time in ms

// Add to global variables
#define FAILSAFE_DELAY 10          // 1 second (10 * 0.1s) delay before activating failsafe
#define FAILSAFE_OFF_DELAY 20      // 2 second (20 * 0.1s) delay before turning off motors
#define FAILSAFE_THROTTLE (THROTTLE_SAFE_START + 10)  // Throttle level during failsafe
static boolean failsafe_active = false;
static unsigned long failsafe_start_time = 0;
static uint8_t failsafe_stage = 0; // 0=inactive, 1=level, 2=land, 3=cut

// Additional failsafe parameters
#define FAILSAFE_TIMEOUT 1000           // Milliseconds before triggering failsafe when no receiver updates
#define MIN_FAILSAFE_THROTTLE 70        // Minimum throttle during failsafe descent
#define MAX_FAILSAFE_THROTTLE 120       // Maximum throttle during failsafe descent
#define FAILSAFE_DESCENT_START_DELAY 2000 // Delay before starting descent (ms)
#define FAILSAFE_DESCENT_DURATION 5000  // Duration of the descent (ms)

// Add to global variables section
#define MOTOR_SAFETY_CHECK_INTERVAL 500  // Check motors every 500ms
#define MOTOR_OVERRUN_TIMEOUT 20000      // 20 seconds maximum motor runtime at high speed
#define MOTOR_HIGH_THRESHOLD 180         // Threshold for detecting high motor output (0-255)
boolean motor_timeout_warning = false;   // Flag for high motor runtime warning
unsigned long motor_high_start_time = 0;  // When motors went to high power
unsigned long motor_safety_timer = 0;     // Timer for regular safety checks

// Add to global variables
#define VBAT_SCALE_FACTOR 1.0      // Fine tune this for your voltage divider
#define VBAT_LOW_HYSTERESIS 0.05   // Hysteresis for low battery warnings (volts)
#define VBAT_CELL_CAPACITY 450     // Estimated battery capacity in mAh
float vbat_compensation_factor = 0.0; // Current compensation amount
float vbat_filtered = 0.0;        // Filtered battery voltage
boolean vbat_alarm_triggered = false; // Prevents alarm from toggling

// Add to global variables
#define THRUST_BOOST_ENABLED true    // Enable thrust boost feature
#define THRUST_BOOST_FACTOR 0.15     // Boost factor (0.0 to 0.3)
float throttle_change_rate = 0.0;    // Rate of throttle change
float thrust_boost_amount = 0.0;     // Current thrust boost amount
float last_throttle_value = 0.0;     // Previous throttle value for calculation
unsigned long last_throttle_calc_time = 0; // Last time we calculated throttle change

// Add to global variables
#define WIND_ESTIMATION_ENABLED true  // Enable wind estimation
float wind_strength = 0.0;           // Estimated wind strength
float wind_direction = 0.0;          // Estimated wind direction (0-360 degrees)
float wind_compensation_gain = 0.05; // How much to compensate for wind

// Add to global variables
float gyro_x = 0, gyro_y = 0, gyro_z = 0;  // Raw gyro values used in check_gyro_health

// Add to global variables
// Utility function for mapping float values (Arduino's map only works with integers)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Add to global variables section
// motor_test_value is used in motor diagnostics
uint8_t motor_test_value = 0;

// Add after existing function declarations
/**
 * Apply accelerometer calibration to raw readings
 * Applies offset and gain corrections to accelerometer data
 */
void apply_acc_calibration(float *acc_x, float *acc_y, float *acc_z) {
  // Apply offset correction
  *acc_x = (*acc_x - acc_offset[0]) * acc_gain[0];
  *acc_y = (*acc_y - acc_offset[1]) * acc_gain[1];
  *acc_z = (*acc_z - acc_offset[2]) * acc_gain[2];
}

// Add to global variables
float roll_comp = 0.0;  // Roll compensation for wind (degrees)
float pitch_comp = 0.0; // Pitch compensation for wind (degrees)

// Change line with function declarations - remove erroneous code
void calculate_auto_level();
void update_remotexy_status();

// Function to disarm motors
void go_disarm() {
  if (motors_armed) {
    motors_armed = false;
    motor_fl = 0;
    motor_fr = 0;
    motor_bl = 0;
    motor_br = 0;
    
    // Write to motor pins immediately using safe function
    writeMotors();
    
    Serial.println(F("Motors disarmed"));
  }
}

//=============== SIMPLIFIED PREFLIGHT CHECKS FUNCTION ======================
boolean run_basic_preflight_checks() {
  // Check battery voltage
  enhanced_battery_management();
  Serial.print(battery_voltage);
  if (battery_voltage < 3.5) {
    Serial.println(" (LOW) - Charge battery before flight");
    return false;
  } else {
    Serial.println(" (OK)");
  }
  
  // Check gyro health (MultiWii-inspired)
  Serial.print("Checking gyro health... ");
  if (!check_gyro_health()) {
    Serial.println("FAILED - Gyro not functioning properly");
    return false;
  } else {
    Serial.println("OK");
  }
  
  // Check RemoteXY connection
  if (RemoteXY.connect_flag) {
    Serial.println("Connected");
  } else {
    Serial.println("Not connected - Connect remote before flight");
    return false;
  }
  
  // Check motor pins
  pinMode(MOTOR_FL_PIN, OUTPUT);
  pinMode(MOTOR_FR_PIN, OUTPUT);
  pinMode(MOTOR_BL_PIN, OUTPUT);
  pinMode(MOTOR_BR_PIN, OUTPUT);
  
  Serial.println("Basic pre-flight checks PASSED");
  return true;
}

//=============== PRE-FLIGHT CHECKS FUNCTION ======================
boolean run_preflight_checks() {
  boolean all_checks_passed = true;
  
  // Check MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
    all_checks_passed = false;
  }
  
  // Check battery voltage
  enhanced_battery_management();
  Serial.print(battery_voltage);
  if (battery_voltage < 3.5) {
    Serial.println(" (LOW) - Charge battery before flight");
    all_checks_passed = false;
  } else {
    Serial.println(" (OK)");
  }
  
  // Check RemoteXY connection
  if (RemoteXY.connect_flag) {
    Serial.println("Connected");
  } else {
    Serial.println("Not connected - Connect remote before flight");
    all_checks_passed = false;
  }
  
  // Check motor pins
  pinMode(MOTOR_FL_PIN, OUTPUT);
  pinMode(MOTOR_FR_PIN, OUTPUT);
  pinMode(MOTOR_BL_PIN, OUTPUT);
  pinMode(MOTOR_BR_PIN, OUTPUT);
  
  // Final result
  if (all_checks_passed) {
    Serial.println("All pre-flight checks PASSED");
  } else {
    Serial.println("Some pre-flight checks FAILED");
  }
  return all_checks_passed;
}

//=============== LED STATUS UPDATE FUNCTION ======================
void update_led_status() {
  unsigned long current_time = millis();
  
  // Emergency LED pattern takes highest priority
  if (led_emergency_active) {
    // Fast triple-blink pattern repeating (SOS-like)
    unsigned long elapsed = current_time - led_emergency_start;
    if (elapsed < 1000) { // Show pattern for 1 second
      // Fast 10Hz triple-blink (50ms on, 50ms off)
      unsigned long cycle = elapsed % 300; // 300ms per complete pattern
      if (cycle < 50 || (cycle >= 100 && cycle < 150) || (cycle >= 200 && cycle < 250)) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      // After pattern completes, emergency LED reverts to regular state
      // but emergency landing continues
      led_emergency_active = false;
    }
    return; // Skip other patterns during emergency
  }
  
  // Mode change LED indication takes high priority
  if (led_mode_change_active) {
    // Each mode change blink is 200ms on, 200ms off (2.5Hz)
    unsigned long elapsed = current_time - led_mode_change_start;
    unsigned long cycle = elapsed % 400;
    int blink_number = elapsed / 400;
    
    if (blink_number < led_mode_change_count) {
      // Still have blinks to do
      if (cycle < 200) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      // All blinks done
      led_mode_change_active = false;
      digitalWrite(LED_PIN, led_state); // Return to normal state
    }
    return; // Skip other LED patterns during mode change
  }
  
  // Handle LED warning mode (non-blocking)
  if (led_warning_active) {
    // Warning blinks 5 times rapidly at 5Hz (100ms on, 100ms off)
    if (current_time - led_warning_start < 1000) { // 1 second total
      // Blink at 5Hz during warning
      if ((current_time - led_warning_start) % 200 < 100) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      // End of warning sequence
      led_warning_active = false;
      digitalWrite(LED_PIN, led_state); // Return to normal state
    }
    return; // Skip other LED patterns during warning
  }
  
  // Normal LED status patterns
  if (critical_battery) {
    // Rapid blink for critical battery - 4Hz (125ms on, 125ms off)
    if (current_time % 250 < 125) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else if (low_battery) {
    // Slow blink for low battery - 1Hz (500ms on, 500ms off)
    if (current_time % 1000 < 500) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else if (calibration_mode) {
    // Fast blink during calibration - 10Hz (50ms on, 50ms off)
    if (current_time % 100 < 50) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else if (motors_armed) {
    // Solid on when armed
    digitalWrite(LED_PIN, HIGH);
    led_state = true;
  } else {
    // Very slow blink when disarmed and everything ok - 0.5Hz (1s on, 1s off)
    if (current_time % 2000 < 1000) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    led_state = (current_time % 2000 < 1000);
  }
}

//=============== SETUP FUNCTION ======================
void setup() {
  // Power saving - disable unused peripherals
  if (USE_REDUCED_MEMORY) {
    power_adc_disable(); // Will be re-enabled for battery reading
    power_spi_disable(); // Disable SPI if not used
    // Don't disable TWI (Wire/I2C) as it's needed for MPU6050
  }
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn on LED during startup
  
  // Initialize hardware Serial for debugging (optional)
  Serial.begin(115200);
  Serial.println(F("Drone initialization starting..."));
  
  // Configure motor control pins as outputs and ensure motors are off
  pinMode(MOTOR_FL_PIN, OUTPUT);
  pinMode(MOTOR_FR_PIN, OUTPUT);
  pinMode(MOTOR_BL_PIN, OUTPUT);
  pinMode(MOTOR_BR_PIN, OUTPUT);
  
  analogWrite(MOTOR_FL_PIN, 0);
  analogWrite(MOTOR_FR_PIN, 0);
  analogWrite(MOTOR_BL_PIN, 0);
  analogWrite(MOTOR_BR_PIN, 0);
  
  // Configure Bluetooth pins for HC05
  pinMode(BLUETOOTH_RX, INPUT);
  pinMode(BLUETOOTH_TX, OUTPUT);
  digitalWrite(BLUETOOTH_TX, HIGH); // Default idle state for serial
  
  // Configure battery monitoring pin
  pinMode(BATTERY_PIN, INPUT);
  
  // Re-enable ADC for battery reading
  power_adc_enable();
  
  // Initialize the RemoteXY object - use direct initialization to prevent memory leak
  RemoteXY_Init(); // Initialize RemoteXY
  
  // Initialize the MPU6050
  Wire.begin();
  Wire.setClock(200000); // 200kHz is more stable than 400kHz for Arduino Nano
  
  // Add timeout protection for I2C bus
  // Wire.setWireTimeout(3000, true); // Removed for compatibility with older Arduino versions
  
  delay(100); // Brief delay for MPU-6050 to stabilize power
  
  // Check if MPU6050 is responding
  Wire.beginTransmission(0x68); // MPU6050 I2C address
  boolean mpu_present = (Wire.endTransmission() == 0);
  
  // Initialize MPU6050 with simpler approach (no DMP)
  if (mpu_present) {
    Serial.println("MPU6050 connected");
    mpu.initialize();
    
    // Set basic calibration values
    mpu.setXGyroOffset(72);
    mpu.setYGyroOffset(25);
    mpu.setZGyroOffset(-12);
    mpu.setZAccelOffset(1670);
    
    // Simple configuration for gyro
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    
    Serial.println("MPU6050 initialized");
    
    // Perform enhanced gyro calibration
    perform_gyro_calibration();
  } else {
    Serial.println("MPU6050 not detected - check wiring");
  }
  
  // Set initial timing variables
  loop_timer = micros();
  battery_timer = millis();
  connection_timer = millis();
  last_activity_timer = millis();
  
  // Initialize control variables
  motor_fl = 0;
  motor_fr = 0;
  motor_bl = 0;
  motor_br = 0;
  pid_i_mem_roll = 0;
  pid_i_mem_pitch = 0;
  pid_i_mem_yaw = 0;
  throttle = 0;
  
  // Optimize PWM frequency for smoother motor control
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    // WARNING: Modifying Timer0 affects millis(), delay(), etc.
    // Only modify Timer 1 and Timer 2 for safety
    
    // Timer 1 - Pin 9 (MOTOR_BR_PIN) - set to 31372.55 Hz
    TCCR1B = (TCCR1B & 0b11111000) | 0x01; // No prescaler (1)
    
    // Timer 2 - Pins 3 & 11 (MOTOR_FL_PIN) - set to 31372.55 Hz
    TCCR2B = (TCCR2B & 0b11111000) | 0x01; // No prescaler (1)
    
    // Leave Timer 0 at default settings (pins 5 & 6)
    // DO NOT MODIFY: TCCR0B = (TCCR0B & 0b11111000) | 0x03; // Default /64 prescaler (~977 Hz)
  #endif
  
  // Run pre-flight checks
  preflight_check_done = run_basic_preflight_checks();
  
  // Display first flight mode status
  if (first_flight_mode) {
    Serial.println("*** FIRST FLIGHT MODE ENABLED ***");
  }
  
  // Blink LED to indicate ready state
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }
  digitalWrite(LED_PIN, LOW);
  
  // Check initial battery voltage
  enhanced_battery_management();
  
  Serial.println("Setup complete");
  
  // Configure PWM frequencies for Arduino Nano
  // This optimizes motor control while avoiding timer conflicts
  setupPWMforNano();
}

// Configure PWM frequencies for Arduino Nano
// This optimizes motor control while avoiding timer conflicts
void setupPWMforNano() {
  // NOTE: Timer0 (pins 5, 6) is used by Arduino core functions like millis() and delay()
  // So we should be careful about modifying it
  
  // Timer1 (pins 9, 10) - increase frequency for smoother motor control
  TCCR1A = 0;                  // Reset control register A
  TCCR1B = 0;                  // Reset control register B
  TCCR1A |= (1 << WGM10);      // Fast PWM 8-bit
  TCCR1B |= (1 << WGM12);      // Fast PWM 8-bit
  TCCR1A |= (1 << COM1A1);     // Clear OC1A on compare match, set at bottom
  TCCR1A |= (1 << COM1B1);     // Clear OC1B on compare match, set at bottom
  TCCR1B |= (1 << CS10);       // No prescaler (31372.55 Hz)
  
  // Timer2 (pins 3, 11) - increase frequency
  TCCR2A = 0;                  // Reset control register A
  TCCR2B = 0;                  // Reset control register B
  TCCR2A |= (1 << WGM20);      // Fast PWM
  TCCR2A |= (1 << WGM21);      // Fast PWM
  TCCR2A |= (1 << COM2A1);     // Clear OC2A on compare match, set at bottom
  TCCR2A |= (1 << COM2B1);     // Clear OC2B on compare match, set at bottom
  TCCR2B |= (1 << CS20);       // No prescaler (31372.55 Hz)
}

//=============== MAIN LOOP FUNCTION ======================
void loop() {
  // Add timing control
  static unsigned long lastLoopTime = 0;
  const unsigned long LOOP_INTERVAL = 2500; // 400Hz loop rate
  
  unsigned long now = micros();
  if (now - lastLoopTime < LOOP_INTERVAL) {
      return;
  }
  lastLoopTime = now;
  
  // Read data from RemoteXY app with error handling
  RemoteXY_Handler();
  
  // Check connection status
  if (RemoteXY.connect_flag == 1) {
    // Connection is active, reset timer
    connection_timer = millis();
    connection_lost = false;
  } else if (timeSince(connection_timer) > CONNECTION_TIMEOUT) {
    // If connection lost for too long, trigger failsafe
    connection_lost = true;
  }
  
  // Update IMU data
  updateIMU();

  // Update battery status
  if (timeSince(battery_timer) > BATTERY_CHECK_INTERVAL) {
    enhanced_battery_management();
    battery_timer = millis();
  }
  
  // Process radio control inputs with filtering
  processRadioControl();
  
  // Check for failsafe conditions
  check_failsafe();
  
  // Calculate auto-level corrections
  calculate_auto_level();
  
  // Estimate wind force for compensation
  estimateWindForce();
  
  // Apply dynamic filtering to gyro data
  applyDynamicFiltering();
  
  // Calculate PID outputs
  calculatePID();

  // Mix motor outputs
  mixMotors();
  
  // Write to motors with MOSFET protection
  writeMotorsSafely();
  
  // Update RemoteXY status
  update_remotexy_status();
  
  // Update LED status
  update_led_status();
  
  // Handle emergency landing if active
  if (emergency_landing) {
    handle_emergency_landing();
  }
  
  // Run motor diagnostics if active
  if (motor_diagnostics_active) {
    run_motor_diagnostics();
  }
}

// Motor diagnostic functions
/**
 * Starts the motor diagnostics mode to test each motor individually
 * Only allows diagnostics when motors are disarmed
 */
void start_motor_diagnostics() {
  // Only allow diagnostics when disarmed
  if (motors_armed) {
    Serial.println("ERROR: Cannot start diagnostics while motors are armed");
    return;
  }
  
  // Check for low battery before starting diagnostics
  if (battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
    Serial.println("ERROR: Battery voltage too low for diagnostics");
    return;
  }
  
  // Make sure all motors are off before starting diagnostics
  motor_fl = 0;
  motor_fr = 0;
  motor_bl = 0;
  motor_br = 0;
  analogWrite(MOTOR_FL_PIN, 0);
  analogWrite(MOTOR_FR_PIN, 0);
  analogWrite(MOTOR_BL_PIN, 0);
  analogWrite(MOTOR_BR_PIN, 0);
  
  // Add a safety delay
  delay(500);
  
  // Set flag to indicate diagnostics mode is active
  motor_diagnostics_active = true;
  
  // Initialize diagnostic variables
  motor_diag_step = 0;
  motor_test_value = 0;
  motor_diag_start_time = millis();
  
  // Visual indication that diagnostics mode is active
  digitalWrite(LED_PIN, HIGH);
  
  // Log to serial
  Serial.println(F("Motor diagnostics mode activated"));
  Serial.println(F("Testing each motor in sequence"));
}

/**
 * Runs the motor diagnostics sequence
 * Tests each motor individually and checks for proper operation
 */
void run_motor_diagnostics() {
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - motor_diag_start_time;
  
  // Reset all motors to zero by default
  motor_fl = 0;
  motor_fr = 0;
  motor_bl = 0;
  motor_br = 0;
  
  // Check for exit condition - calibrate switch turned off
  if (!RemoteXY.calibrate_switch) {
    // Exit diagnostics mode
    motor_diagnostics_active = false;
    
    // Reset all motors
    motor_fl = 0;
    motor_fr = 0;
    motor_bl = 0;
    motor_br = 0;
    
    // Write to motor pins
    analogWrite(MOTOR_FL_PIN, motor_fl);
    analogWrite(MOTOR_FR_PIN, motor_fr);
    analogWrite(MOTOR_BL_PIN, motor_bl);
    analogWrite(MOTOR_BR_PIN, motor_br);
    
    // Visual indication that diagnostics mode is exited
    digitalWrite(LED_PIN, LOW);
    
    // Log to serial
    Serial.println(F("Motor diagnostics mode deactivated"));
    
    return;
  }
  
  // Diagnostic sequence timing
  // Each motor runs for 3 seconds, with 1 second pause between motors
  // Total cycle is 16 seconds (4 motors * 4 seconds each)
  unsigned long cycle_time = elapsed_time % 16000;
  
  // Determine which motor to test based on cycle time
  if (cycle_time < 4000) {
    // Test front left motor (0-3 seconds on, 3-4 seconds off)
    if (cycle_time < 3000) {
      motor_fl = MOTOR_IDLE_VALUE + 20;
      
      // Visual indication - LED on during motor test
      digitalWrite(LED_PIN, HIGH);
      
      // Update RemoteXY
      sprintf(RemoteXY.flight_time, "FL");
    } else {
      // Pause between motors
      digitalWrite(LED_PIN, LOW);
    }
  } else if (cycle_time < 8000) {
    // Test front right motor (4-7 seconds on, 7-8 seconds off)
    if (cycle_time < 7000) {
      motor_fr = MOTOR_IDLE_VALUE + 20;
      
      // Visual indication
      digitalWrite(LED_PIN, HIGH);
      
      // Update RemoteXY
      sprintf(RemoteXY.flight_time, "FR");
    } else {
      // Pause between motors
      digitalWrite(LED_PIN, LOW);
    }
  } else if (cycle_time < 12000) {
    // Test back left motor (8-11 seconds on, 11-12 seconds off)
    if (cycle_time < 11000) {
      motor_bl = MOTOR_IDLE_VALUE + 20;
      
      // Visual indication
      digitalWrite(LED_PIN, HIGH);
      
      // Update RemoteXY
      sprintf(RemoteXY.flight_time, "BL");
    } else {
      // Pause between motors
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    // Test back right motor (12-15 seconds on, 15-16 seconds off)
    if (cycle_time < 15000) {
      motor_br = MOTOR_IDLE_VALUE + 20;
      
      // Visual indication
      digitalWrite(LED_PIN, HIGH);
      
      // Update RemoteXY
      sprintf(RemoteXY.flight_time, "BR");
    } else {
      // Pause between motors
      digitalWrite(LED_PIN, LOW);
    }
  }
  
  // Write to motor pins
  analogWrite(MOTOR_FL_PIN, motor_fl);
  analogWrite(MOTOR_FR_PIN, motor_fr);
  analogWrite(MOTOR_BL_PIN, motor_bl);
  analogWrite(MOTOR_BR_PIN, motor_br);
}

// Forward declarations for motor diagnostic functions
void start_motor_diagnostics();
void run_motor_diagnostics();

// Forward declarations for other functions used in the main loop
void calculate_auto_level();
void update_remotexy_status();


// Function to disarm motors
void go_disarm() {
  if (motors_armed) {
    motors_armed = false;
    motor_fl = 0;
    motor_fr = 0;
    motor_bl = 0;
    motor_br = 0;
    
    // Write to motor pins immediately using safe function
    writeMotors();
    
    Serial.println(F("Motors disarmed"));
  }
}

// Function to handle emergency landing
void handle_emergency_landing() {
  if (!emergency_landing) return;
  
  unsigned long landing_time = millis() - emergency_landing_timer;
  
  // Stage 1: Level the drone (first 2 seconds)
  if (landing_time < 2000) {
    // Set zero targets for roll and pitch to level the craft
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;
    
    // Maintain current throttle to avoid sudden drops
    // But ensure minimum safe value
    if (throttle < THROTTLE_SAFE_START + 20) {
      throttle = THROTTLE_SAFE_START + 20;
    }
  } 
  // Stage 2: Controlled descent (next 5 seconds)
  else if (landing_time < 7000) {
    // Maintain level attitude
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;
    
    // Gradually reduce throttle for a controlled descent
    // Start from whatever throttle we had at the beginning of stage 2
    if (landing_time >= 2000 && landing_time < 2100) {
      landing_throttle = throttle; // Initialize landing throttle at start of stage 2
    }
    
    // Reduce throttle by 5 units per second
    int throttle_reduction = 5 * ((landing_time - 2000) / 1000);
    throttle = max(landing_throttle - throttle_reduction, THROTTLE_SAFE_START);
  }
  // Stage 3: Cut motors
  else {
    // Disarm motors
    go_disarm();
    emergency_landing = false;
    Serial.println("Emergency landing complete - motors disarmed");
  }
}

// Function to trigger emergency landing
void trigger_emergency_landing() {
  emergency_landing = true;
  emergency_landing_timer = millis();
  Serial.println("Emergency landing triggered");
}

// Function to update IMU data with critical section protection
void updateIMU() {
  // Update the last gyro read time before attempting a read
  last_gyro_read = millis();
  
  // Initialize fail detection
  static uint8_t consecutive_failures = 0;
  static float last_good_gyro_x = 0, last_good_gyro_y = 0, last_good_gyro_z = 0;
  static float last_good_accel_x = 0, last_good_accel_y = 0, last_good_accel_z = 0;
  
  // Local variables to store readings atomically
  float gyro_x_local, gyro_y_local, gyro_z_local;
  float accel_x_local, accel_y_local, accel_z_local;
  
  // Get 6-axis motion sensor readings with error handling
  bool read_success = true;
  
  // Disable interrupts during critical read
  noInterrupts();
  
  // Check if MPU is responsive
  Wire.beginTransmission(0x68); // MPU6050 address
  if (Wire.endTransmission() != 0) {
    read_success = false;
  } else {
    // Use the MPU library for reading
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Basic validation - check that values are not extreme outliers
    if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
      // All zeros probably means read failure
      read_success = false;
    }
    
    if (abs(gx) > 32000 || abs(gy) > 32000 || abs(gz) > 32000 ||
        abs(ax) > 32000 || abs(ay) > 32000 || abs(az) > 32000) {
      // Extreme values likely indicate bad read
      read_success = false;
    }
  }
  
  // Re-enable interrupts after I2C reading completes
  interrupts();
  
  // Handle read failures
  if (!read_success) {
    consecutive_failures++;
    
    if (consecutive_failures > 5) {
      if (ENABLE_DEBUG && Serial) {
        Serial.println(F("WARNING: IMU read failures, using last values"));
      }
      
      // Use last good values if we've had multiple failures
      gyro_x = last_good_gyro_x;
      gyro_y = last_good_gyro_y;
      gyro_z = last_good_gyro_z;
      
      accel_x = last_good_accel_x;
      accel_y = last_good_accel_y;
      accel_z = last_good_accel_z;
      
      // Attempt to reset the I2C bus
      if (consecutive_failures == 10) {
        Wire.end();
        delay(10);
        Wire.begin();
        Wire.setClock(400000);
        Wire.setWireTimeout(3000, true);
        if (ENABLE_DEBUG && Serial) {
          Serial.println(F("ALERT: Resetting I2C bus"));
        }
      }
    }
    return;
  }
  
  // Reset failure counter on success
  consecutive_failures = 0;
  
  // Convert to proper units and store in local variables
  gyro_x_local = (float)gx / GYRO_SENSITIVITY - gyro_x_cal;
  gyro_y_local = (float)gy / GYRO_SENSITIVITY - gyro_y_cal;
  gyro_z_local = (float)gz / GYRO_SENSITIVITY - gyro_z_cal;
  
  accel_x_local = (float)ax / ACCEL_SENSITIVITY;
  accel_y_local = (float)ay / ACCEL_SENSITIVITY;
  accel_z_local = (float)az / ACCEL_SENSITIVITY;
  
  // Check for unreasonable values
  bool gyro_values_valid = true;
  if (abs(gyro_x_local) > 500 || abs(gyro_y_local) > 500 || abs(gyro_z_local) > 500) {
    gyro_values_valid = false;
    if (ENABLE_DEBUG && Serial) {
      Serial.println(F("WARNING: Unreasonable gyro values detected"));
    }
  }
  
  // Update globals only if values are valid
  if (gyro_values_valid) {
    gyro_x = gyro_x_local;
    gyro_y = gyro_y_local;
    gyro_z = gyro_z_local;
    
    accel_x = accel_x_local;
    accel_y = accel_y_local;
    accel_z = accel_z_local;
    
    // Store good values for fallback
    last_good_gyro_x = gyro_x;
    last_good_gyro_y = gyro_y;
    last_good_gyro_z = gyro_z;
    
    last_good_accel_x = accel_x;
    last_good_accel_y = accel_y;
    last_good_accel_z = accel_z;
  }
  
  // Calculate roll and pitch angles from accelerometer
  acc_total_vector = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));
  
  // Prevent division by zero
  if (acc_total_vector > 0.5 && acc_total_vector < 2.0) {
    // Calculate angle using accelerometer data
    acc_roll = asin(accel_y / acc_total_vector) * RAD_TO_DEG; 
    acc_pitch = asin(-accel_x / acc_total_vector) * RAD_TO_DEG;
  }
  
  // For the first time, initialize with accelerometer data
  if (!gyro_initialized) {
    angle_roll = acc_roll;
    angle_pitch = acc_pitch;
    gyro_initialized = true;
    } else {
    // Complementary filter to combine gyro and accelerometer data
    angle_roll = angle_roll * 0.98 + acc_roll * 0.02;
    angle_pitch = angle_pitch * 0.98 + acc_pitch * 0.02;
  }
}

// Dynamic notch filter for gyro noise reduction - inspired by MultiWii
float applyDynamicNotchFilter(float input, float *lastInput, float *lastOutput, float cutoffFreq) {
  // Simple second-order IIR filter implementation
  // This is a simplified version of a notch filter that adapts to the detected noise frequency
  
  // Calculate filter coefficient based on cutoff frequency
  // The cutoff frequency is adjusted based on detected noise
  float alpha = 0.5f / (1.0f + cutoffFreq);
  
  // Apply the filter
  float output = alpha * (input + *lastInput) + (1.0f - 2.0f * alpha) * (*lastOutput);
  
  // Update state variables
  *lastInput = input;
  *lastOutput = output;
  
  return output;
}

// Function to detect dominant noise frequency in gyro data
float detectNoiseFrequency() {
  // This is a simplified implementation
  // In a full implementation, this would use FFT or other spectral analysis
  
  // For now, we'll use a simple heuristic based on the noise amplitude
  float noiseAmplitude = max(max(gyro_noise_roll, gyro_noise_pitch), gyro_noise_yaw);
  
  // Map noise amplitude to an estimated frequency range (30-200 Hz)
  // Higher noise typically corresponds to higher frequency vibrations
  float estimatedFreq = mapFloat(constrain(noiseAmplitude, 0, 20), 0, 20, 30, 200);
  
  return estimatedFreq;
}

/**
 * Applies dynamic filtering to gyro data based on detected noise levels
 * Adjusts filter strength based on noise conditions
 */
void applyDynamicFiltering() {
  // Only apply if dynamic filtering is enabled
  if (!DYNAMIC_FILTER_ENABLED) return;
  
  // Simple low-pass filter implementation
  // Filter coefficient is adjusted based on detected noise levels
  float filter_strength = LPF_FACTOR;
  
  // Increase filter strength if high noise is detected
  if (gyro_noise_roll > noise_threshold_roll || 
      gyro_noise_pitch > noise_threshold_pitch || 
      gyro_noise_yaw > noise_threshold_yaw) {
    
    // Calculate how much to increase filter strength
    float noise_factor = max(max(gyro_noise_roll / noise_threshold_roll, 
                                 gyro_noise_pitch / noise_threshold_pitch),
                             gyro_noise_yaw / noise_threshold_yaw);
    
    // Scale filter strength based on noise (more filtering for more noise)
    filter_strength = LPF_FACTOR + ((1.0 - LPF_FACTOR) * min(noise_factor * 0.5, 0.9));
    
    // Ensure filter strength stays within reasonable bounds
    filter_strength = constrain(filter_strength, LPF_FACTOR, 0.98);
  }
  
  // Apply low-pass filter to gyro data
  gyro_roll_input = gyro_roll_input * filter_strength + gyro_roll_input_raw * (1.0 - filter_strength);
  gyro_pitch_input = gyro_pitch_input * filter_strength + gyro_pitch_input_raw * (1.0 - filter_strength);
  gyro_yaw_input = gyro_yaw_input * filter_strength + gyro_yaw_input_raw * (1.0 - filter_strength);
}

// Improved PID controller based on MultiWii's implementation
void calculatePID() {
  float error, PTerm, ITerm, DTerm, delta;
  static float lastError[3] = {0, 0, 0};
  static float errorGyroI[3] = {0, 0, 0};
  static unsigned long lastPIDTime = 0;
  
  // Calculate time since last PID calculation using overflow-safe method
  unsigned long currentTime = micros();
  float cycleTime;
  
  // Handle micros() overflow using safer method
  if (currentTime < lastPIDTime) {
    // micros() has overflowed
    cycleTime = ((0xFFFFFFFF - lastPIDTime) + currentTime + 1) / 1000000.0f;
  } else {
    cycleTime = (currentTime - lastPIDTime) / 1000000.0f;
  }
  
  lastPIDTime = currentTime;
  
  // Limit cycle time to prevent huge jumps
  cycleTime = constrain(cycleTime, 0.001f, 0.1f);
  
  // Apply TPA (Throttle PID Attenuation) - reduce PID gains at higher throttle
  float tpaFactor = 1.0f;
  if (throttle > TPA_BREAKPOINT) {
    tpaFactor = 1.0f - (TPA_FACTOR * (throttle - TPA_BREAKPOINT) / (MOTOR_MAX_VALUE - TPA_BREAKPOINT));
    tpaFactor = constrain(tpaFactor, 0.5f, 1.0f);
  }
  
  // Calculate Anti-Gravity factor - helps maintain altitude during quick throttle changes
  static float lastThrottle = 0;
  float throttleChange = (throttle - lastThrottle) * cycleTime * 100.0f;
  lastThrottle = throttle;
  
  // Only apply positive changes (throttle increases)
  float antiGravityThrottleD = constrain(throttleChange, 0, 1000);
  float antiGravityFactor = 1.0f + (antiGravityThrottleD * ANTIGRAVITY_FACTOR / 1000.0f);
  
  // Process ROLL axis
  // Get current gyro rate
  float gyroRate = gyro_roll_input;
  // Get desired rotation rate
  float targetRate = pid_roll_setpoint;
    // Calculate error
    error = targetRate - gyroRate;
    // Calculate P term with TPA
  float pGain = pid_p_gain_roll * tpaFactor;
    PTerm = error * pGain;
    
  // Anti-windup: only accumulate I if not at limits
  float pidOutput = PTerm + errorGyroI[0];
  int16_t iMax = pid_max_roll / 2;
  bool isMaximized = (pidOutput >= iMax) && (error > 0);
  bool isMinimized = (pidOutput <= -iMax) && (error < 0);
  
  // Accumulate I-term only if not at limits
  if (!isMaximized && !isMinimized) {
    float iGain = pid_i_gain_roll * tpaFactor;
    // Apply anti-gravity
    if (anti_gravity_enabled) {
      iGain *= antiGravityFactor;
    }
    errorGyroI[0] += error * iGain * cycleTime;
  }
  
  // Hard limit I-term to prevent excessive windup
  errorGyroI[0] = constrain(errorGyroI[0], -iMax, iMax);
  
  // Reset I-term on large stick movements
  if (abs(RemoteXY.roll_joystick) > 70) {
    errorGyroI[0] = 0;
  }
  
  ITerm = errorGyroI[0];
    
    // Calculate D term with TPA
  float dGain = pid_d_gain_roll * tpaFactor;
    
    // Calculate delta (gyro derivative)
  delta = (error - lastError[0]) / cycleTime;
  // Limit delta to reasonable values to prevent spikes
  delta = constrain(delta, -2000.0f, 2000.0f); 
  lastError[0] = error;
  
  // Apply low-pass filter to D to reduce noise sensitivity
  static float lastDTerm = 0;
  
  // More sophisticated filtering for D term to reduce noise
  const float dTermFilterCutoff = 20.0f; // 20Hz cutoff
  const float RC = 1.0f / (2.0f * PI * dTermFilterCutoff);
  const float alpha = cycleTime / (cycleTime + RC);
  
  // Calculate filtered D term
    DTerm = delta * dGain;
  DTerm = lastDTerm + alpha * (DTerm - lastDTerm);
  lastDTerm = DTerm;
    
    // Combine PID terms
  pid_output_roll = constrain(PTerm + ITerm - DTerm, -pid_max_roll, pid_max_roll);

  // Process PITCH axis (similar to roll)
  gyroRate = gyro_pitch_input;
  targetRate = pid_pitch_setpoint;
  error = targetRate - gyroRate;
  pGain = pid_p_gain_pitch * tpaFactor;
  PTerm = error * pGain;
  
  pidOutput = PTerm + errorGyroI[1];
  iMax = pid_max_pitch / 2;
  isMaximized = (pidOutput >= iMax) && (error > 0);
  isMinimized = (pidOutput <= -iMax) && (error < 0);
  
  if (!isMaximized && !isMinimized) {
    float iGain = pid_i_gain_pitch * tpaFactor;
    if (anti_gravity_enabled) {
      iGain *= antiGravityFactor;
    }
    errorGyroI[1] += error * iGain * cycleTime;
  }
  
  errorGyroI[1] = constrain(errorGyroI[1], -iMax, iMax);
  
  if (abs(RemoteXY.pitch_joystick) > 70) {
    errorGyroI[1] = 0;
  }
  
  ITerm = errorGyroI[1];
  
  dGain = pid_d_gain_pitch * tpaFactor;
  delta = (error - lastError[1]) / cycleTime;
  lastError[1] = error;
  
  static float lastDTermPitch = 0;
  DTerm = delta * dGain;
  DTerm = lastDTermPitch + alpha * (DTerm - lastDTermPitch);
  lastDTermPitch = DTerm;
  
  pid_output_pitch = constrain(PTerm + ITerm - DTerm, -pid_max_pitch, pid_max_pitch);
  
  // Process YAW axis (similar but with different gains)
  gyroRate = gyro_yaw_input;
  targetRate = pid_yaw_setpoint;
  error = targetRate - gyroRate;
  PTerm = error * pid_p_gain_yaw;
  
  pidOutput = PTerm + errorGyroI[2];
  iMax = pid_max_yaw / 2;
  isMaximized = (pidOutput >= iMax) && (error > 0);
  isMinimized = (pidOutput <= -iMax) && (error < 0);
  
  if (!isMaximized && !isMinimized) {
    errorGyroI[2] += error * pid_i_gain_yaw * cycleTime;
  }
  
  errorGyroI[2] = constrain(errorGyroI[2], -iMax, iMax);
  
  // Reset I-term on large yaw movements
  if (abs(RemoteXY.yaw_joystick) > 70) {
    errorGyroI[2] = 0;
  }
  
  ITerm = errorGyroI[2];
  
  dGain = pid_d_gain_yaw;
  delta = (error - lastError[2]) / cycleTime;
  lastError[2] = error;
  
  static float lastDTermYaw = 0;
  DTerm = delta * dGain;
  DTerm = lastDTermYaw + alpha * (DTerm - lastDTermYaw);
  lastDTermYaw = DTerm;
  
  pid_output_yaw = constrain(PTerm + ITerm - DTerm, -pid_max_yaw, pid_max_yaw);
}

// Modify the mixMotors function to include thrust boost calculation and application
void mixMotors() {
  // Only mix motors if armed or in diagnostic mode
  if (!motors_armed && !motor_diagnostics_active && !motor_test_mode) {
    motor_fl = 0;
    motor_fr = 0;
    motor_bl = 0;
    motor_br = 0;
    return;
  }
  
  // Calculate thrust boost before motor mixing (MultiWii-inspired)
  // Calculate time delta with overflow protection
  static unsigned long last_thrust_calc_time = 0;
  unsigned long current_time = millis();
  float dt = timeSince(last_thrust_calc_time) / 1000.0f; // Convert to seconds
  last_thrust_calc_time = current_time;
  
  // Limit dt to avoid huge jumps
  dt = constrain(dt, 0.001f, 0.1f);
  
  // Calculate throttle change rate (units per second)
  static float last_throttle = 0;
  float throttle_change_rate = (throttle - last_throttle) / dt;
  last_throttle = throttle;
  
  // Calculate boost amount - only boost on throttle increase
  float thrust_boost = 0;
  if (throttle_change_rate > 0 && THRUST_BOOST_ENABLED && !emergency_landing) {
    // Scale it to make it appropriate for small drone with DC motors
    thrust_boost = throttle_change_rate * 0.015 * THRUST_BOOST_FACTOR;
    // Limit maximum boost
    thrust_boost = constrain(thrust_boost, 0.0, 30.0);
  }
  
  // MultiWii-style motor mixing for quadcopter X configuration
  motor_fl = throttle - pid_output_roll + pid_output_pitch - pid_output_yaw; // Front Left  - CCW
  motor_fr = throttle + pid_output_roll + pid_output_pitch + pid_output_yaw; // Front Right - CW
  motor_bl = throttle - pid_output_roll - pid_output_pitch + pid_output_yaw; // Back Left   - CW
  motor_br = throttle + pid_output_roll - pid_output_pitch - pid_output_yaw; // Back Right  - CCW
  
  // Apply thrust boost to all motors if calculated
  if (thrust_boost > 0) {
    motor_fl += thrust_boost;
    motor_fr += thrust_boost;
    motor_bl += thrust_boost;
    motor_br += thrust_boost;
  }
  
  // Find maximum motor value
  int max_motor = max(
    max(motor_fl, motor_fr), 
    max(motor_bl, motor_br)
  );
  
  // If any motor exceeds MOTOR_MAX_VALUE, reduce all proportionally
  if (max_motor > MOTOR_MAX_VALUE) {
    float reduction_factor = (float)MOTOR_MAX_VALUE / max_motor;
    motor_fl *= reduction_factor;
    motor_fr *= reduction_factor;
    motor_bl *= reduction_factor;
    motor_br *= reduction_factor;
  }
  
  // Apply minimum motor value to spinning motors
  if (throttle > THROTTLE_SAFE_START) {
    if (motor_fl > 0) motor_fl = max(MOTOR_MIN_VALUE, motor_fl);
    if (motor_fr > 0) motor_fr = max(MOTOR_MIN_VALUE, motor_fr);
    if (motor_bl > 0) motor_bl = max(MOTOR_MIN_VALUE, motor_bl);
    if (motor_br > 0) motor_br = max(MOTOR_MIN_VALUE, motor_br);
  } else {
    // Motors off if throttle is below start threshold or not armed
    motor_fl = 0;
    motor_fr = 0;
    motor_bl = 0;
    motor_br = 0;
  }
  
  // Final safety check - limit all motors to allowed range
  motor_fl = constrain(motor_fl, 0, MOTOR_MAX_VALUE);
  motor_fr = constrain(motor_fr, 0, MOTOR_MAX_VALUE);
  motor_bl = constrain(motor_bl, 0, MOTOR_MAX_VALUE);
  motor_br = constrain(motor_br, 0, MOTOR_MAX_VALUE);
  
  // We don't write to the motors here - that's handled in the main loop with writeMotors()
}

// Function to check gyro health based on MultiWii
boolean check_gyro_health() {
  // Check if we're getting valid gyro readings
  static unsigned long last_gyro_check = 0;
  static boolean gyro_health_status = true;
  
  // Only check periodically to avoid excessive calculations
  unsigned long current_time = millis();
  if (current_time - last_gyro_check < 500) {
    return gyro_health_status; // Return last status if checked recently
  }
  
  last_gyro_check = current_time;
  
  // Check if gyro values are within reasonable range
  // Typical gyro values should be within +/- 2000 deg/s for most movements
  if (abs(gyro_x) > 2000 || abs(gyro_y) > 2000 || abs(gyro_z) > 2000) {
    gyro_health_status = false;
    return false;
  }
  
  // Check for stuck values - if gyro values haven't changed at all, that's suspicious
  static float last_gyro_x = 0;
  static float last_gyro_y = 0;
  static float last_gyro_z = 0;
  static unsigned long stuck_counter = 0;
  
  if (gyro_x == last_gyro_x && gyro_y == last_gyro_y && gyro_z == last_gyro_z) {
    stuck_counter++;
    if (stuck_counter > 5) { // If values are stuck for 5 consecutive checks
      gyro_health_status = false;
      return false;
    }
  } else {
    stuck_counter = 0; // Reset counter if values are changing
  }
  
  // Store current values for next comparison
  last_gyro_x = gyro_x;
  last_gyro_y = gyro_y;
  last_gyro_z = gyro_z;
  
  // Check if we're getting new readings at all
  if (current_time - last_gyro_read > 100) { // No gyro readings for 100ms
    gyro_health_status = false;
    return false;
  }
  
  // If we got here, gyro seems healthy
  gyro_health_status = true;
  return true;
}

// Enhanced battery management function optimized for 1S LiPo
void enhanced_battery_management() {
  // Read battery voltage with oversampling for better accuracy
  const int numSamples = 8;
  int rawTotal = 0;
  
  for (int i = 0; i < numSamples; i++) {
    rawTotal += analogRead(BATTERY_PIN);
    delayMicroseconds(100); // Small delay between readings
  }
  
  int rawAverage = rawTotal / numSamples;
  
  // Calculate voltage using voltage divider ratio
  float measured_voltage = ((float)rawAverage / 1023.0) * 5.0 * VOLTAGE_DIVIDER_RATIO;
  
  // Apply digital low-pass filter to reduce noise
  static float filtered_voltage = 0;
  
  if (filtered_voltage == 0) {
    // Initialize on first reading
    filtered_voltage = measured_voltage;
  } else {
    // Low-pass filter with hysteresis for stable readings
    // Use more aggressive filtering for small changes to filter noise
    // but faster response for real battery voltage drops
    float voltage_diff = abs(measured_voltage - filtered_voltage);
    float filter_strength;
    
    if (voltage_diff < 0.05) {
      // Small change - probably noise, filter heavily
      filter_strength = 0.95; // 95% previous, 5% new
    } else if (voltage_diff < 0.2) {
      // Medium change - might be real, medium filtering
      filter_strength = 0.8; // 80% previous, 20% new
    } else {
      // Large change - likely real, respond faster
      filter_strength = 0.5; // 50% previous, 50% new
    }
    
    // Apply the appropriate filter strength
    filtered_voltage = filtered_voltage * filter_strength + 
                      measured_voltage * (1.0 - filter_strength);
  }
  
  // Update global battery voltage
  battery_voltage = filtered_voltage;
  
  // Check for 1S LiPo cells (3.7V nominal)
  // Adjust voltage thresholds based on current load
  // Battery under load will have lower voltage (sag), compensate based on throttle
  float load_compensation = 0.0;
  
  // Only apply load compensation when motors are running
  if (throttle > THROTTLE_SAFE_START) {
    // Calculate load-based compensation (higher throttle = more compensation)
    // Normalize throttle to 0-1 range from THROTTLE_SAFE_START to MOTOR_MAX_VALUE
    float normalized_throttle = (float)(throttle - THROTTLE_SAFE_START) / 
                              (MOTOR_MAX_VALUE - THROTTLE_SAFE_START);
    
    // Maximum compensation of 0.3V at full throttle
    load_compensation = normalized_throttle * 0.3;
  }
  
  // Adjusted thresholds based on current load
  float low_voltage_threshold = BATTERY_LOW_VOLTAGE + load_compensation;
  float critical_voltage_threshold = BATTERY_CRITICAL_VOLTAGE + load_compensation;
  
  // Battery state detection with hysteresis to prevent flickering
  if (battery_voltage < critical_voltage_threshold) {
    critical_battery = true;
    low_battery = true;
    
    // Log critical battery state
    if (ENABLE_DEBUG && Serial) {
      Serial.print("CRITICAL BATTERY: ");
      Serial.println(battery_voltage);
    }
  } else if (battery_voltage < low_voltage_threshold) {
    // Only set low battery, not critical
    low_battery = true;
    
    // Clear critical flag only if voltage is significantly above critical
    if (battery_voltage > (critical_voltage_threshold + 0.1)) {
      critical_battery = false;
    }
    
    // Log low battery state
    if (ENABLE_DEBUG && Serial && !low_battery_warning) {
      Serial.print("LOW BATTERY: ");
      Serial.println(battery_voltage);
      low_battery_warning = true;
    }
  } else if (battery_voltage > (low_voltage_threshold + 0.1)) {
    // Clear flags only if voltage is significantly above thresholds
    low_battery = false;
    critical_battery = false;
    low_battery_warning = false;
  }
  
  // Calculate battery percentage based on LiPo discharge curve
  uint8_t battery_percentage;
  
  if (battery_voltage >= 4.0) {
    // 80-100% range
    battery_percentage = mapFloat(battery_voltage, 4.0, 4.2, 80, 100);
  } else if (battery_voltage >= 3.7) {
    // 50-80% range
    battery_percentage = mapFloat(battery_voltage, 3.7, 4.0, 50, 80);
  } else if (battery_voltage >= 3.5) {
    // 20-50% range
    battery_percentage = mapFloat(battery_voltage, 3.5, 3.7, 20, 50);
  } else if (battery_voltage >= 3.3) {
    // 0-20% range
    battery_percentage = mapFloat(battery_voltage, 3.3, 3.5, 0, 20);
  } else {
    battery_percentage = 0;
  }
  
  // Update RemoteXY battery level
  RemoteXY.battery_level = constrain(battery_percentage, 0, 100);
  
  // Also update the battery compensation factor for power management
  float nominal_voltage = 3.7; // 1S LiPo nominal voltage
  float voltage_diff = nominal_voltage - battery_voltage;
  
  // Only compensate if voltage is below nominal
  if (voltage_diff > 0) {
    // Max compensation of 30% at critical voltage
    float max_compensation = 0.3;
    float compensation_range = nominal_voltage - BATTERY_CRITICAL_VOLTAGE;
    
    // Calculate compensation factor (0-0.3 range)
    vbat_compensation_factor = (voltage_diff / compensation_range) * max_compensation;
    vbat_compensation_factor = constrain(vbat_compensation_factor, 0.0, max_compensation);
  } else {
    vbat_compensation_factor = 0.0;
  }
}

// Enhanced gyro calibration routine inspired by MultiWii
void perform_gyro_calibration() {
  const int calibration_samples = GYRO_CALIBRATION_SAMPLES;  // Use the defined sample count
  const int discard_samples = 100;       // Discard first samples (settling time)
  
  Serial.println(F("Starting gyro calibration - keep drone still..."));
  
  // Visual indicator that calibration is in progress
  digitalWrite(LED_PIN, HIGH);
  
  // Variables for calibration
  long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  int valid_samples = 0;
  int16_t ax, ay, az, gx, gy, gz;
  
  // Variables for movement detection
  float gyro_x_min = 0, gyro_x_max = 0;
  float gyro_y_min = 0, gyro_y_max = 0;
  float gyro_z_min = 0, gyro_z_max = 0;
  boolean first_sample = true;
  
  // Discard first samples (sensor settling)
  for (int i = 0; i < discard_samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(1);
  }
  
  // Collect calibration samples
  for (int i = 0; i < calibration_samples; i++) {
    // Read raw gyro data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to degrees per second
    float gyro_x = gx / 131.0;
    float gyro_y = gy / 131.0;
    float gyro_z = gz / 131.0;
    
    // Track min/max for movement detection
    if (first_sample) {
      gyro_x_min = gyro_x_max = gyro_x;
      gyro_y_min = gyro_y_max = gyro_y;
      gyro_z_min = gyro_z_max = gyro_z;
      first_sample = false;
    } else {
      gyro_x_min = min(gyro_x_min, gyro_x);
      gyro_x_max = max(gyro_x_max, gyro_x);
      gyro_y_min = min(gyro_y_min, gyro_y);
      gyro_y_max = max(gyro_y_max, gyro_y);
      gyro_z_min = min(gyro_z_min, gyro_z);
      gyro_z_max = max(gyro_z_max, gyro_z);
    }
    
    // Accumulate values
    gyro_x_sum += gx;
    gyro_y_sum += gy;
    gyro_z_sum += gz;
    valid_samples++;
    
    // Blink LED every 200 samples to show progress
    if (i % 200 == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    delay(1); // Small delay for stability
  }
  
  // Check if drone was moved during calibration
  float gyro_x_diff = gyro_x_max - gyro_x_min;
  float gyro_y_diff = gyro_y_max - gyro_y_min;
  float gyro_z_diff = gyro_z_max - gyro_z_min;
  
  // If movement detected, warn user but still use the values
  if (gyro_x_diff > 15 || gyro_y_diff > 15 || gyro_z_diff > 15) {
    Serial.println("WARNING: Movement detected during calibration!");
    Serial.println("Calibration values may be inaccurate.");
    
    // Warning LED pattern
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
  }
  
  // Calculate average offsets
  gyro_x_cal = (float)gyro_x_sum / valid_samples;
  gyro_y_cal = (float)gyro_y_sum / valid_samples;
  gyro_z_cal = (float)gyro_z_sum / valid_samples;
  
  // Apply offsets to MPU6050
  mpu.setXGyroOffset((int16_t)round(gyro_x_cal));
  mpu.setYGyroOffset((int16_t)round(gyro_y_cal));
  mpu.setZGyroOffset((int16_t)round(gyro_z_cal));
  
  // Verify calibration with a few more readings
  gyro_x_sum = gyro_y_sum = gyro_z_sum = 0;
  for (int i = 0; i < 100; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyro_x_sum += gx;
    gyro_y_sum += gy;
    gyro_z_sum += gz;
    delay(1);
  }
  
  // Calculate residual error
  float residual_x = abs((float)gyro_x_sum / 100.0);
  float residual_y = abs((float)gyro_y_sum / 100.0);
  float residual_z = abs((float)gyro_z_sum / 100.0);
  
  Serial.println("Gyro calibration complete!");
  Serial.print("Offsets: X=");
  Serial.print(gyro_x_cal);
  Serial.print(" Y=");
  Serial.print(gyro_y_cal);
  Serial.print(" Z=");
  Serial.println(gyro_z_cal);
  
  Serial.print("Residual error: X=");
  Serial.print(residual_x);
  Serial.print(" Y=");
  Serial.print(residual_y);
  Serial.print(" Z=");
  Serial.println(residual_z);
  
  // Success indication
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

/**
 * Handles failsafe conditions when communication is lost or critical errors occur
 * Implements a safe landing procedure to minimize damage
 */
void handleFailsafe() {
  static unsigned long failsafe_start_time = 0;
  static float descent_throttle = 0;
  
  // Check for failsafe conditions
  boolean signal_lost = connection_lost;
  boolean gyro_failure = !check_gyro_health();
  
  if (signal_lost || critical_battery || gyro_failure) {
    // Activate failsafe if not already active
    if (!failsafe_active) {
      failsafe_active = true;
      failsafe_start_time = millis();
      
      // Set initial descent throttle based on current throttle
      // but ensure it's within safe range for controlled descent
      descent_throttle = constrain(throttle, MIN_FAILSAFE_THROTTLE, MAX_FAILSAFE_THROTTLE);
      
      // Log failsafe activation
      Serial.println(F("FAILSAFE ACTIVATED"));
      if (signal_lost) Serial.println(F("- Signal lost"));
      if (critical_battery) Serial.println(F("- Critical battery"));
      if (gyro_failure) Serial.println(F("- Gyro failure"));
    }
    
    // Override controls for safe landing
    throttle = descent_throttle;
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;
    
    // Gradually reduce throttle for gentle landing
    unsigned long failsafe_duration = millis() - failsafe_start_time;
    if (failsafe_duration > FAILSAFE_DESCENT_START_DELAY) {
      float reduction_factor = (float)(failsafe_duration - FAILSAFE_DESCENT_START_DELAY) / FAILSAFE_DESCENT_DURATION;
      reduction_factor = constrain(reduction_factor, 0.0, 1.0);
      
      descent_throttle = MAX_FAILSAFE_THROTTLE - (reduction_factor * (MAX_FAILSAFE_THROTTLE - MIN_FAILSAFE_THROTTLE));
    }
    
    // Visual indication of failsafe mode
    if (millis() % 200 < 100) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    // Reset failsafe if conditions are resolved
    if (failsafe_active) {
      failsafe_active = false;
      Serial.println(F("FAILSAFE DEACTIVATED"));
    }
  }
}

// Add to global variables section
unsigned long last_receiver_update = 0; // Track last time we received input from remote

// Fix the processReceiverInput function to include super expo handling
/**
 * Processes receiver input from RemoteXY and updates control variables
 * Updates last_receiver_update timestamp for failsafe detection
 */
void processReceiverInput() {
  static float prev_throttle = 0;
  static float prev_roll = 0;
  static float prev_pitch = 0;
  static float prev_yaw = 0;
  
  // Read data from RemoteXY app
  RemoteXY_Handler();
  
  // Check if we have a valid connection
  if (RemoteXY.connect_flag == 1) {
    // Check if any control inputs have changed significantly
    boolean input_changed = false;
    
    // Check for significant changes in control inputs
    if (abs(RemoteXY.throttle_joystick - prev_throttle) > 1 ||
        abs(RemoteXY.roll_joystick - prev_roll) > 1 ||
        abs(RemoteXY.pitch_joystick - prev_pitch) > 1 ||
        abs(RemoteXY.yaw_joystick - prev_yaw) > 1) {
      input_changed = true;
    }
    
    // Update previous values
    prev_throttle = RemoteXY.throttle_joystick;
    prev_roll = RemoteXY.roll_joystick;
    prev_pitch = RemoteXY.pitch_joystick;
    prev_yaw = RemoteXY.yaw_joystick;
    
    // If inputs changed or it's been a while since last update
    if (input_changed || (millis() - last_receiver_update > 500)) {
      // Update the timestamp for failsafe detection
      last_receiver_update = millis();
      
      // Process throttle with expo
      float throttle_input = mapFloat(RemoteXY.throttle_joystick, -100, 100, 0, 1);
      throttle_input = applyExpo(throttle_input, THROTTLE_EXPO);
      throttle = mapFloat(throttle_input, 0, 1, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
      
      // Process roll, pitch and yaw with expo
      float roll_setpoint_temp = applyExpo(mapFloat(RemoteXY.roll_joystick, -100, 100, -1, 1), EXPO_FACTOR);
      roll_setpoint_temp = applyDeadband(roll_setpoint_temp, ROLL_PITCH_DEADBAND / 100.0);
      pid_roll_setpoint = roll_setpoint_temp * MAX_ANGLE_LIMIT;
      
      float pitch_setpoint_temp = applyExpo(mapFloat(RemoteXY.pitch_joystick, -100, 100, -1, 1), EXPO_FACTOR);
      pitch_setpoint_temp = applyDeadband(pitch_setpoint_temp, ROLL_PITCH_DEADBAND / 100.0);
      pid_pitch_setpoint = pitch_setpoint_temp * MAX_ANGLE_LIMIT;
      
      float yaw_setpoint_temp = applyExpo(mapFloat(RemoteXY.yaw_joystick, -100, 100, -1, 1), EXPO_FACTOR);
      yaw_setpoint_temp = applyDeadband(yaw_setpoint_temp, YAW_CONTROL_DEADBAND / 100.0);
      pid_yaw_setpoint = yaw_setpoint_temp * 180.0; // Convert to degrees per second
      
      // Apply super expo if enabled
      if (SUPER_EXPO_FACTOR > 0) {
        pid_roll_setpoint = applySuperExpo(pid_roll_setpoint / MAX_ANGLE_LIMIT, SUPER_EXPO_FACTOR, 1.0) * MAX_ANGLE_LIMIT;
        pid_pitch_setpoint = applySuperExpo(pid_pitch_setpoint / MAX_ANGLE_LIMIT, SUPER_EXPO_FACTOR, 1.0) * MAX_ANGLE_LIMIT;
        pid_yaw_setpoint = applySuperExpo(pid_yaw_setpoint / 180.0, SUPER_EXPO_FACTOR, 1.0) * 180.0;
      }
    }
  }
}

/**
 * Applies exponential curve to control inputs for smoother response
 */
float applyExpo(float input, float expo_factor) {
  return input * (1 - expo_factor + expo_factor * input * input);
}

/**
 * Applies super exponential curve for even more precise control
 */
float applySuperExpo(float input, float super_expo, float max_value) {
  float input_abs = abs(input);
  float result = input_abs * (1 - super_expo + super_expo * input_abs * input_abs * input_abs);
  return (input >= 0) ? result : -result;
}

/**
 * Applies deadband to control inputs to prevent drift
 */
float applyDeadband(float input, float deadband) {
  if (abs(input) < deadband) {
    return 0;
  } else {
    return input > 0 ? 
      mapFloat(input, deadband, 1, 0, 1) : 
      mapFloat(input, -1, -deadband, -1, 0);
  }
}

/**
 * Calculates auto-level corrections based on IMU data
 * Uses filtered roll and pitch angles to generate correction values
 */
void calculate_auto_level() {
  // Only calculate auto-level if enabled
  if (!auto_level_enabled) {
    auto_level_roll = 0;
    auto_level_pitch = 0;
    return;
  }
  
  // Calculate error between current angle and desired angle (0 degrees)
  float roll_error = 0 - filtered_roll;
  float pitch_error = 0 - filtered_pitch;
  
  // Apply auto-level gain
  // Higher values make the drone level itself more aggressively
  float auto_level_gain = 0.15;
  
  // Apply angle limit to prevent over-correction
  roll_error = constrain(roll_error, -MAX_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
  pitch_error = constrain(pitch_error, -MAX_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
  
  // Calculate auto-level correction
  // This will be added to the pilot's setpoint
  auto_level_roll = roll_error * auto_level_gain;
  auto_level_pitch = pitch_error * auto_level_gain;
  
  // Apply additional damping based on gyro rates to prevent oscillation
  float damping_factor = 0.05;
  auto_level_roll -= gyro_roll_input * damping_factor;
  auto_level_pitch -= gyro_pitch_input * damping_factor;
  
  // Limit the maximum auto-level correction
  auto_level_roll = constrain(auto_level_roll, -5, 5);
  auto_level_pitch = constrain(auto_level_pitch, -5, 5);
}

/**
 * Updates the RemoteXY interface with current drone status
 * Displays battery level, orientation, and flight time
 */
void update_remotexy_status() {
  // Only update every 250ms to reduce Bluetooth traffic
  static unsigned long last_update = 0;
  if (millis() - last_update < 250) return;
  last_update = millis();
  
  // Update battery level indicator (0-100%)
  int battery_percentage = 0;
  
  // Map battery voltage to percentage based on LiPo discharge curve
  // 1S LiPo battery (3.7V nominal)
  if (battery_voltage >= 4.0) {
    // 80-100% range
    battery_percentage = mapFloat(battery_voltage, 4.0, 4.2, 80, 100);
  } else if (battery_voltage >= 3.7) {
    // 50-80% range
    battery_percentage = mapFloat(battery_voltage, 3.7, 4.0, 50, 80);
  } else if (battery_voltage >= 3.5) {
    // 20-50% range
    battery_percentage = mapFloat(battery_voltage, 3.5, 3.7, 20, 50);
  } else if (battery_voltage >= 3.3) {
    // 0-20% range
    battery_percentage = mapFloat(battery_voltage, 3.3, 3.5, 0, 20);
  }
  
  // Update RemoteXY battery level
  RemoteXY.battery_level = constrain(battery_percentage, 0, 100);
  
  // Update orientation display
  // Convert to int8_t range (-100 to 100)
  RemoteXY.roll_angle = constrain((int8_t)filtered_roll, -100, 100);
  RemoteXY.pitch_angle = constrain((int8_t)filtered_pitch, -100, 100);
  
  // Update flight time (seconds since power-on)
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  seconds %= 60;
  
  // Format as MM:SS
  sprintf(RemoteXY.flight_time, "%02lu:%02lu", minutes, seconds);
  
  // Update LED status based on armed state and battery level
  if (motors_armed) {
    RemoteXY.led_power = 1; // LED on when armed
  } else {
    // Blink LED when disarmed
    RemoteXY.led_power = (millis() % 1000 < 500) ? 1 : 0;
  }
  
  // Override LED for low battery warning
  if (low_battery) {
    // Fast blink for low battery
    RemoteXY.led_power = (millis() % 400 < 200) ? 1 : 0;
  }
  
  // Override LED for critical battery warning
  if (critical_battery) {
    // Very fast blink for critical battery
    RemoteXY.led_power = (millis() % 200 < 100) ? 1 : 0;
  }
}

// Implement wind force estimation function to calculate roll_comp and pitch_comp
void estimateWindForce() {
  // Only estimate wind force if wind estimation is enabled and motors are armed
  if (!WIND_ESTIMATION_ENABLED || !motors_armed) {
    roll_comp = 0.0;
    pitch_comp = 0.0;
    return;
  }
  
  // Use PID outputs as indicators of persistent corrections needed
  static float avg_roll_correction = 0;
  static float avg_pitch_correction = 0;
  
  // Low-pass filter PID outputs to detect sustained corrections
  avg_roll_correction = avg_roll_correction * 0.98f + pid_output_roll * 0.02f;
  avg_pitch_correction = avg_pitch_correction * 0.98f + pid_output_pitch * 0.02f;
  
  // Calculate wind compensation values (opposite to correction direction)
  roll_comp = -avg_roll_correction * wind_compensation_gain;
  pitch_comp = -avg_pitch_correction * wind_compensation_gain;
  
  // Limit maximum compensation
  roll_comp = constrain(roll_comp, -5.0f, 5.0f);
  pitch_comp = constrain(pitch_comp, -5.0f, 5.0f);
}

// Add radio signal filtering to handle interference and noise
float filterRadioSignal(float raw_value, float previous_value) {
  // Simple exponential filter to smooth radio inputs
  const float alpha = 0.3; // Filter strength (lower = stronger filtering)
  return alpha * raw_value + (1 - alpha) * previous_value;
}

// Process radio control data with filtering
void processRadioControl() {
  // Guard clause to prevent crashes if RemoteXY is not initialized
  if (remotexy == NULL) {
    return;
  }
  
  // Previous values for filtering (static to maintain values between calls)
  static float prev_throttle = 0;
  static float prev_yaw = 0;
  static float prev_pitch = 0;
  static float prev_roll = 0;
  
  // Check if we have a valid connection
  if (RemoteXY.connect_flag == 1) {
    // Check if any control inputs have changed significantly
    boolean input_changed = false;
    
    // Check for significant changes in control inputs
    if (abs(RemoteXY.throttle_joystick - prev_throttle) > 1 ||
        abs(RemoteXY.roll_joystick - prev_roll) > 1 ||
        abs(RemoteXY.pitch_joystick - prev_pitch) > 1 ||
        abs(RemoteXY.yaw_joystick - prev_yaw) > 1) {
      input_changed = true;
    }
    
    // Update previous values
    prev_throttle = RemoteXY.throttle_joystick;
    prev_roll = RemoteXY.roll_joystick;
    prev_pitch = RemoteXY.pitch_joystick;
    prev_yaw = RemoteXY.yaw_joystick;
    
    // If inputs changed or it's been a while since last update
    if (input_changed || (millis() - last_receiver_update > 500)) {
      // Update the timestamp for failsafe detection
      last_receiver_update = millis();
      
      // Process throttle with expo
      float throttle_input = mapFloat(RemoteXY.throttle_joystick, -100, 100, 0, 1);
      throttle_input = applyExpo(throttle_input, THROTTLE_EXPO);
      throttle = mapFloat(throttle_input, 0, 1, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
      
      // Process roll, pitch and yaw with expo
      float roll_setpoint_temp = applyExpo(mapFloat(RemoteXY.roll_joystick, -100, 100, -1, 1), EXPO_FACTOR);
      roll_setpoint_temp = applyDeadband(roll_setpoint_temp, ROLL_PITCH_DEADBAND / 100.0);
      pid_roll_setpoint = roll_setpoint_temp * MAX_ANGLE_LIMIT;
      
      float pitch_setpoint_temp = applyExpo(mapFloat(RemoteXY.pitch_joystick, -100, 100, -1, 1), EXPO_FACTOR);
      pitch_setpoint_temp = applyDeadband(pitch_setpoint_temp, ROLL_PITCH_DEADBAND / 100.0);
      pid_pitch_setpoint = pitch_setpoint_temp * MAX_ANGLE_LIMIT;
      
      float yaw_setpoint_temp = applyExpo(mapFloat(RemoteXY.yaw_joystick, -100, 100, -1, 1), EXPO_FACTOR);
      yaw_setpoint_temp = applyDeadband(yaw_setpoint_temp, YAW_CONTROL_DEADBAND / 100.0);
      pid_yaw_setpoint = yaw_setpoint_temp * 180.0; // Convert to degrees per second
      
      // Apply super expo if enabled
      if (SUPER_EXPO_FACTOR > 0) {
        pid_roll_setpoint = applySuperExpo(pid_roll_setpoint / MAX_ANGLE_LIMIT, SUPER_EXPO_FACTOR, 1.0) * MAX_ANGLE_LIMIT;
        pid_pitch_setpoint = applySuperExpo(pid_pitch_setpoint / MAX_ANGLE_LIMIT, SUPER_EXPO_FACTOR, 1.0) * MAX_ANGLE_LIMIT;
        pid_yaw_setpoint = applySuperExpo(pid_yaw_setpoint / 180.0, SUPER_EXPO_FACTOR, 1.0) * 180.0;
      }
    }
  }
}

// Enhanced failsafe handler function
void check_failsafe() {
  // Current time for timeout calculations
  unsigned long current_time = millis();
  
  // Check for various failure conditions
  bool failsafe_triggered = false;
  String failsafe_reason = "";
  
  // 1. Check for connection loss
  if (connection_lost) {
    failsafe_triggered = true;
    failsafe_reason = "Radio connection lost";
  }
  
  // 2. Check for battery voltage too low (critical)
  if (battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
    failsafe_triggered = true;
    failsafe_reason = "Critical battery voltage";
  }
  
  // 3. Check for gyro read timeout
  if (current_time - last_gyro_read > 500) { // 500ms without gyro data
    failsafe_triggered = true;
    failsafe_reason = "Gyro timeout";
  }
  
  // 4. Check for excessive angle (crash detection)
  if (abs(angle_roll) > MAX_SAFE_ANGLE || abs(angle_pitch) > MAX_SAFE_ANGLE) {
    failsafe_triggered = true;
    failsafe_reason = "Excessive angle";
  }
  
  // 5. Check for motor imbalance (possible hardware failure)
  int motor_diff = max(
    max(abs(motor_fl - motor_fr), abs(motor_fl - motor_bl)),
    max(abs(motor_fl - motor_br), max(abs(motor_fr - motor_bl), abs(motor_fr - motor_br)))
  );
  
  if (motor_diff > 100 && throttle > THROTTLE_SAFE_START) { // Only check at higher throttle
    failsafe_triggered = true;
    failsafe_reason = "Motor imbalance";
  }
  
  // Execute failsafe if triggered
  if (failsafe_triggered) {
    // Log the failsafe reason if Serial is available
    if (ENABLE_DEBUG && Serial) {
      Serial.print("FAILSAFE: ");
      Serial.println(failsafe_reason);
    }
    
    // Increment counter if same failsafe
    if (failsafe_reason == last_failsafe_reason) {
      failsafe_counter++;
    } else {
      // Reset counter for new type of failsafe
      failsafe_counter = 1;
      last_failsafe_reason = failsafe_reason;
    }
    
    // Execute appropriate failsafe action based on severity
    if (failsafe_reason == "Critical battery voltage" || 
        failsafe_reason == "Excessive angle" ||
        failsafe_counter > 20) {
      // Critical failsafe - immediate landing
      execute_emergency_landing();
    } else {
      // Less critical - controlled descent
      execute_controlled_descent();
    }
  } else {
    // Reset failsafe counter if no failsafe condition exists
    failsafe_counter = 0;
    last_failsafe_reason = "";
  }
}

// Execute gradual descent for non-critical failsafe
void execute_controlled_descent() {
  // Gradually reduce throttle while maintaining stability
  throttle = max(1000, throttle - 5);
  
  // Keep attitude control active for stabilized descent
  is_failsafe_mode = true;
}

// Execute immediate landing for critical failsafe
void execute_emergency_landing() {
  // Cut motors immediately for critical situations
  throttle = 1000;
  motor_fl = 0;
  motor_fr = 0;
  motor_bl = 0;
  motor_br = 0;
  writeMotors(); // Use safe motor writing
  
  is_failsafe_mode = true;
  
  // Disarm after landing
  if (emergency_landing_timer == 0) {
    emergency_landing_timer = millis();
  } else if (millis() - emergency_landing_timer > 2000) {
    motors_armed = false; // Changed from 'armed' to 'motors_armed' for consistency
    is_failsafe_mode = false;
    emergency_landing_timer = 0;
  }
}

// Safely compare time intervals accounting for millis() overflow
unsigned long timeSince(unsigned long timestamp) {
  // Handle millis() overflow which occurs every ~50 days
  return (millis() >= timestamp) ? (millis() - timestamp) : (0xFFFFFFFF - timestamp + millis());
}

#define PI 3.14159265358979323846f

// Custom I2C timeout function 
bool i2cReadWithTimeout(int deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t length) {
  unsigned long startTime = millis();
  const unsigned long i2cTimeout = 100; // 100ms timeout
  
  // Start transmission
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  byte result = Wire.endTransmission(false); // False to not release the bus
  
  if (result != 0) {
    return false; // Failed to start transmission
  }
  
  // Request data
  Wire.requestFrom(deviceAddress, length, true);
  
  // Read with timeout
  uint8_t bytesRead = 0;
  while (bytesRead < length) {
    if (Wire.available()) {
      data[bytesRead++] = Wire.read();
    } else {
      // Check timeout
      if (timeSince(startTime) > i2cTimeout) {
        return false; // Timeout
      }
      // Small delay to avoid overwhelming the bus
      delayMicroseconds(10);
    }
  }
  
  return true;
}

// Safe motor writing function with deadtime to protect MOSFETs
void writeMotors() {
  // Ensure all motor values are in valid range before writing
  uint8_t safeMotorFL = constrain(motor_fl, 0, 255);
  uint8_t safeMotorFR = constrain(motor_fr, 0, 255);
  uint8_t safeMotorBL = constrain(motor_bl, 0, 255);
  uint8_t safeMotorBR = constrain(motor_br, 0, 255);
  
  // Apply deadtime between MOSFET switching to prevent shoot-through
  analogWrite(MOTOR_FL_PIN, safeMotorFL);
  delayMicroseconds(MOSFET_DELAY_US);
  
  analogWrite(MOTOR_FR_PIN, safeMotorFR);
  delayMicroseconds(MOSFET_DELAY_US);
  
  analogWrite(MOTOR_BL_PIN, safeMotorBL);
  delayMicroseconds(MOSFET_DELAY_US);
  
  analogWrite(MOTOR_BR_PIN, safeMotorBR);
}
