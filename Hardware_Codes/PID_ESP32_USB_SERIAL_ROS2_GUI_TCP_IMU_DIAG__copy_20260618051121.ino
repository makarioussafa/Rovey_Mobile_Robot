// ============================================================
// ESP32 Mecanum Robot - Predefined Path + Obstacle Avoidance
// MASTER ESP32
//
// micro-ROS NODE: PID / drivetrain / path / obstacle decision node.
// No HTTP server is started. Communication is through ROS 2 topics via
// USB serial micro-ROS agents running on the Raspberry Pi.
//
// Obstacle handling logic:
//   ANY stable sensor detection stops the robot immediately.
//   The sensor ESP32 sends a debounced/stable mask, not raw flicker.
//
//   SIDE-ONLY CASES:
//     S1 only, S3 only, or S1+S3 without S2:
//       stop -> wait until side sensors clear -> wait 3 stable seconds
//       -> continue the same path segment.
//
//   FRONT CASES:
//     S2 only, S1+S2, or S2+S3:
//       stop -> 5 s static/dynamic check.
//       If all sensors clear -> dynamic obstacle -> continue same segment.
//       If S2 remains active -> arm Pi toy recognition.
//       If TOY is confirmed -> move forward 12 cm -> wait for mechanism task
//         and homing to finish -> continue path.
//       If no TOY is confirmed -> perform full avoidance sequence:
//         rotate +90 for the normal S2-only case -> drive side offset
//         -> rotate back to path heading -> drive forward bypass
//         -> rotate -90 -> drive side offset -> rotate back to path heading
//         -> continue path.
//
//   FULL BLOCK CASE:
//     S1+S2+S3:
//       stop and wait indefinitely while all three are active.
//       When any sensor clears, re-evaluate the new mask using the rules above.
//
// ROS 2 / micro-ROS alignment:
//   This PID ESP32 subscribes to sensor/mecanism topics from the
//   Sensor/Mechanism ESP32 and publishes command topics back to it.
//   MPU6050 remains on local I2C; ESP-to-ESP I2C is no longer used.
// ============================================================

#include <Arduino.h>
// WiFi transport removed: USB serial micro-ROS is used.
#include <Wire.h>
#include <math.h>
#include "driver/gpio.h"
#include <freertos/queue.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// =====================================================
// USB SERIAL micro-ROS IMPORTANT NOTE
// =====================================================
// The ESP32 USB Serial port is reserved exclusively for micro-ROS XRCE-DDS.
// Do NOT print normal debug text to Serial while the micro-ROS serial agent is running,
// because debug bytes will corrupt the micro-ROS stream.
// Debug prints are redirected to this no-op object. Use ROS 2 topics on the Pi for monitoring.
struct NullDebugSerialClass {
  void begin(unsigned long) {}
  template<typename... Args> void print(Args...) {}
  template<typename... Args> void println(Args...) {}
  template<typename... Args> void printf(const char*, Args...) {}
};
static NullDebugSerialClass DebugSerial;

// =====================================================
// USB Serial - micro-ROS transport
// =====================================================
#define MICRO_ROS_SERIAL_BAUD 115200

// =====================================================
// IMU Pins and Address
// =====================================================
#define SDA_PIN   21
#define SCL_PIN   22
#define MPU_ADDR  0x68

// =====================================================
// Motor Pin Definitions
// =====================================================
#define BR_DN1       0
#define BR_DN2       2
#define BR_PWM_PIN  15

#define FL_DN1      25
#define FL_DN2      33

#define FR_DN1      26
#define FR_DN2      27

#define BL_DN1       4
#define BL_DN2      16

#define FL_PWM_PIN  32
#define FR_PWM_PIN  14
#define BL_PWM_PIN  17

// =====================================================
// Encoder Pin Definitions
// =====================================================
#define FL_A  35
#define FL_B  34

#define FR_A  36
#define FR_B  39

#define BL_A  23
#define BL_B  19

#define BR_A   5
#define BR_B  18

// =====================================================
// Robot and Motor Constants
// =====================================================
#define WHEEL_R     0.0485f
#define ROBOT_LW    0.31828f
#define MAX_RAD_S   21.99f
#define MAX_MPS     (MAX_RAD_S * WHEEL_R)
#define MAX_WZ      2.3f

const float CPR[4] = {
  1562.0f,  // FL - measured from run_encoder_imuPPr.csv, one output-shaft revolution
  1579.0f,  // FR
  1635.0f,  // BL
  1618.0f   // BR
};

// Wheel coordinate correction.
// Order is FL, FR, BL, BR.
//
// MOTOR_SIGN fixes motor-driver polarity only. The new FL JGA25-370 motor
// drives opposite to the other three motors for the same H-bridge command,
// so FL must be inverted here instead of changing the mecanum equations.
//
// ENCODER_SIGN is intentionally separate. Leave FL at +1 first: after the FL
// motor output is inverted, a correct forward wheel rotation should report
// positive encoder ticks. Change only this constant if a lifted-wheel test
// proves a wheel's encoder counts opposite to the robot-coordinate direction.
const int8_t MOTOR_SIGN[4] = {
  -1,  // FL
  +1,  // FR
  +1,  // BL
  +1   // BR
};

const int8_t ENCODER_SIGN[4] = {
  +1,  // FL
  +1,  // FR
  +1,  // BL
  +1   // BR
};

// =====================================================
// PWM Settings
// =====================================================
#define PWM_FREQ      5000
#define PWM_RES_BITS  8
#define PWM_MIN       130
#define PWM_MIN_MOVE  130
#define PWM_MIN_LATERAL 185
#define PWM_MIN_TURN  215
#define PWM_MIN_REAR_OPPOSED  190
#define PWM_MAX       255

#define WHEEL_FF_GAIN                 0.18f
#define WHEEL_RESPONSE_SETPOINT_RAD_S 3.0f
#define WHEEL_RESPONSE_MIN_RATIO      0.35f
#define WHEEL_RESPONSE_RELEASE_RATIO  0.55f
#define WHEEL_WRONG_DIR_RAD_S         0.70f
#define WHEEL_STALL_TIME_MS           160u
#define WHEEL_RECOVERY_STEP_PWM       35
#define WHEEL_RECOVERY_MAX_PWM        120
#define WHEEL_RECOVERY_DECAY_PWM      4
#define WHEEL_RECOVERY_STEP_PERIOD_MS 80u
#define REAR_OPPOSED_ASSIST_PWM       35

#define FL_PWM_CH  0
#define FR_PWM_CH  1
#define BL_PWM_CH  2
#define BR_PWM_CH  3

// =====================================================
// PID Loop
// =====================================================
#define PID_HZ  100
#define PID_DT  (1.0f / PID_HZ)
#define PLANNER_HZ  100
#define PLANNER_DT  (1.0f / PLANNER_HZ)

// =====================================================
// Motion Profile
// =====================================================
#define ACCEL_TIME    0.70f
#define DECEL_TIME    0.35f
#define ACCEL_TIME_W  0.38f
#define DECEL_TIME_W  0.18f

// =====================================================
// Move / Turn Controller
// =====================================================
#define DIST_TOLERANCE_M      0.015f
#define ANGLE_TOLERANCE_DEG   4.0f

#define SLOWDOWN_DISTANCE_M   0.25f
#define MIN_MOVE_SPEED_MPS    0.10f
#define MIN_TURN_WZ           0.95f

#define KP_MOVE_YAW        1.35f
#define KI_MOVE_YAW        0.25f
#define KD_MOVE_YAW        0.22f
#define MAX_MOVE_YAW_CORR  0.42f
#define MAX_MANUAL_YAW_CORR 0.72f
#define MANUAL_YAW_SLOWDOWN_DEG      10.0f
#define MANUAL_YAW_FULL_SLOWDOWN_DEG 35.0f
#define MANUAL_YAW_SLOWDOWN_RATE_DPS 25.0f
#define MANUAL_YAW_MIN_LINEAR_SCALE  0.35f
#define MOVE_YAW_I_LIMIT   0.35f
#define MOVE_YAW_DEADBAND_DEG      2.5f
#define MOVE_YAW_RATE_DEADBAND_DPS 2.0f
// Small encoder displacement trim only. Yaw hold is the primary line holder:
// if a segment starts at 0 deg and drifts to 55 deg, Wz drives back toward 0.
#define KP_CROSSTRACK       0.55f
#define MAX_CROSSTRACK_CORR_MPS 0.08f
#define CROSSTRACK_DEADBAND_M 0.020f
// Separate signs are used so straight-line yaw correction can be reversed
// without reversing the already-correct absolute turn commands.
#define MOVE_YAW_CONTROL_SIGN    1.0f
#define TURN_YAW_CONTROL_SIGN    1.0f

#define KP_TURN  3.20f
#define KD_TURN  0.24f
#define TURN_STOP_GYRO_DPS  4.5f
#define TURN_FINE_ZONE_DEG  30.0f
#define TURN_FINE_MIN_WZ    0.75f
#define TURN_STALL_GYRO_DPS 5.0f
#define TURN_STALL_TIME_MS  180u
#define TURN_STALL_BOOST_WZ 1.15f

// =====================================================
// Predefined Path + Obstacle Avoidance Settings
// =====================================================
// ESP-to-ESP communication is ROS 2 topics over USB serial micro-ROS through the Pi micro-ROS agent.

// ROS 2 topic protocol:
//   Subscribed from Sensor/Mechanism ESP32:
//     /robot/sensors/mask          std_msgs/UInt8   bit0=S1 right, bit1=S2 middle, bit2=S3 left
//     /robot/sensors/decision      std_msgs/UInt8   0=NONE, 1=TOY, 2=OBSTACLE
//     /robot/mechanism/state       std_msgs/UInt8   0=IDLE, 1=RUNNING, 2=DONE, 3=ERROR
//   Published to Sensor/Mechanism ESP32:
//     /robot/cmd/clear_decision    std_msgs/Bool    true pulse = clear consumed decision
//     /robot/cmd/arm_vision        std_msgs/Bool    true = arm Pi vision window
//     /robot/cmd/mechanism         std_msgs/UInt8   1=run pickup sequence after 12 cm approach
//     /robot/pid/encoder_packet    std_msgs/Int32   packed encoder + command diagnostics
//
#define DECISION_NONE      0u
#define DECISION_TOY       1u
#define DECISION_OBSTACLE  2u

#define MECH_STATE_IDLE     0u
#define MECH_STATE_RUNNING  1u
#define MECH_STATE_DONE     2u
#define MECH_STATE_ERROR    3u
#define MECH_CMD_RUN        1u

#define SENSOR_CMD_CLEAR_DECISION  0xA1u
#define SENSOR_CMD_ARM_VISION      0xA2u

#define GUI_MODE_MANUAL  0u
#define GUI_MODE_AUTO    1u

#define PID_GUI_CMD_MODE          1u
#define PID_GUI_CMD_AUTO_START    2u
#define PID_GUI_CMD_AUTO_ABORT    3u
#define PID_GUI_CMD_MANUAL_DRIVE  4u
#define PID_GUI_CMD_CAL_IMU       5u
#define PID_GUI_CMD_RESET_ZERO    6u

#define MANUAL_DIR_STOP        0u
#define MANUAL_DIR_FWD         1u
#define MANUAL_DIR_BACK        2u
#define MANUAL_DIR_LEFT        3u
#define MANUAL_DIR_RIGHT       4u
#define MANUAL_DIR_FWD_LEFT    5u
#define MANUAL_DIR_FWD_RIGHT   6u
#define MANUAL_DIR_BACK_LEFT   7u
#define MANUAL_DIR_BACK_RIGHT  8u
#define MANUAL_DIR_ROT_L       9u
#define MANUAL_DIR_ROT_R       10u

#define MANUAL_DRIVE_TIMEOUT_MS  450u
#define MANUAL_SPEED_MIN_PERCENT 60.0f

// PATH_SPEED_PERCENT controls the normal predefined forward segments only.
// TURN_SPEED_PERCENT controls only in-place turns.
// AVOID_SPEED_PERCENT controls only the turn/side-offset/forward/side-offset avoidance sequence.
// Keep path speed modest because straight-line behavior is now improved; tune turns/avoidance separately.
#define PATH_SPEED_PERCENT       30.0f
#define TURN_SPEED_PERCENT       60.0f
#define AVOID_SPEED_PERCENT      50.0f
#define MANUAL_LINEAR_SPEED_PERCENT 70.0f
#define MANUAL_LATERAL_SPEED_SCALE  0.70f

#define STATIC_CHECK_TIME_MS     5000
#define PI_TOY_WAIT_MS          15000
#define SIDE_CLEAR_CONTINUE_DELAY_MS 3000
#define AUTO_RESTART_AFTER_FINISH_MS 800u
#define AVOID_SIDE_DISTANCE_M    0.35f
#define AVOID_FORWARD_DISTANCE_M 1.20f
#define TOY_APPROACH_DISTANCE_M  0.12f
#define TOY_APPROACH_SPEED_PERCENT 25.0f

// Mask bits - a bit is SET when that sensor IS detecting
#define SENSOR_1_RIGHT_MASK   0x01
#define SENSOR_2_MIDDLE_MASK  0x02
#define SENSOR_3_LEFT_MASK    0x04

// =====================================================
// IMU Settings
// =====================================================
#define GYRO_SENS  131.0f
#define ACC_SENS   16384.0f

#define MPU_REG_WHO_AM_I       0x75
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_SMPLRT_DIV     0x19
#define MPU_REG_CONFIG         0x1A
#define MPU_REG_GYRO_CONFIG    0x1B
#define MPU_REG_ACCEL_CONFIG   0x1C
#define MPU_REG_ACCEL_XOUT_H   0x3B

#define IMU_CAL_SAMPLES  1200
#define IMU_TASK_MS      4
#define IMU_I2C_CLOCK_HZ 400000
#define IMU_LIVE_TIMEOUT_MS 500u

// =====================================================
// Wheel Index
// =====================================================
enum {
  W_FL = 0,
  W_FR = 1,
  W_BL = 2,
  W_BR = 3
};

// =====================================================
// Robot Mode
// =====================================================
enum RobotMode {
  MODE_IDLE = 0,
  MODE_MOVE_DISTANCE,
  MODE_STRAFE_DISTANCE,
  MODE_TURN_ANGLE
};

// =====================================================
// Motor Pin Map
// =====================================================
struct MotorPin {
  uint8_t dn1, dn2, pwm, ch;
};

const MotorPin MOT[4] = {
  { FL_DN1, FL_DN2, FL_PWM_PIN, FL_PWM_CH },
  { FR_DN1, FR_DN2, FR_PWM_PIN, FR_PWM_CH },
  { BL_DN1, BL_DN2, BL_PWM_PIN, BL_PWM_CH },
  { BR_DN1, BR_DN2, BR_PWM_PIN, BR_PWM_CH }
};

// =====================================================
// Encoder State
// =====================================================
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t ticks[4] = {0, 0, 0, 0};
volatile uint32_t encoderResetGeneration = 0;

// =====================================================
// PID State
// =====================================================
struct WheelPID {
  float kp, ki, kd;
  float integral, prevErr;
  float integralLimit;
  float outMin, outMax;
};

WheelPID pid[4] = {
  // Lower Ki + lower PWM_MIN reduces wheel fighting, start jerk, and drift buildup.
  { 18.0f, 8.0f, 0.08f, 0.0f, 0.0f, 8.0f, -255.0f, 255.0f },  // FL
  { 17.0f, 8.0f, 0.08f, 0.0f, 0.0f, 8.0f, -255.0f, 255.0f },  // FR
  { 18.0f, 8.0f, 0.08f, 0.0f, 0.0f, 8.0f, -255.0f, 255.0f },  // BL
  { 17.0f, 8.0f, 0.08f, 0.0f, 0.0f, 8.0f, -255.0f, 255.0f }   // BR
};

volatile float   setpointRad[4]  = {0, 0, 0, 0};
volatile float   measuredRad[4]  = {0, 0, 0, 0};
volatile int16_t motorPwmOut[4]  = {0, 0, 0, 0};
volatile int16_t wheelRecoveryBoostPwm[4] = {0, 0, 0, 0};
uint32_t wheelBadResponseStartMs[4] = {0, 0, 0, 0};
uint32_t wheelLastRecoveryStepMs[4] = {0, 0, 0, 0};

// =====================================================
// Motion State
// =====================================================
volatile RobotMode robotMode = MODE_IDLE;
volatile float gSpd = 0.25f;

volatile float targVx = 0.0f, targVy = 0.0f, targWz = 0.0f;
volatile float profVx = 0.0f, profVy = 0.0f, profWz = 0.0f;

volatile float moveTargetDistanceM = 0.0f;
volatile float moveStartForwardM   = 0.0f;
volatile float moveStartLateralM   = 0.0f;
volatile float moveTargetYawDeg    = 0.0f;
volatile float moveDoneM           = 0.0f;
volatile float moveRemainingM      = 0.0f;

volatile float strafeTargetDistanceM = 0.0f;
volatile float strafeStartForwardM   = 0.0f;
volatile float strafeStartLateralM   = 0.0f;
volatile float strafeTargetYawDeg    = 0.0f;
volatile float strafeDoneM           = 0.0f;
volatile float strafeRemainingM      = 0.0f;

volatile float turnTargetYawDeg  = 0.0f;
volatile float turnAngleErrorDeg = 0.0f;
volatile float moveYawIntegralRad = 0.0f;
volatile uint32_t turnStallStartMs = 0;
volatile bool turnStallBoostActive = false;

volatile bool goalReached = false;

// =====================================================
// Odometry State
// =====================================================
volatile float odomX = 0.0f;
volatile float odomY = 0.0f;

// =====================================================
// IMU State
// =====================================================
volatile bool  imuReady                = false;
volatile bool  imuCalibrating          = false;
volatile bool  imuCalibrationRequested = false;
volatile bool  imuInitOk               = false;
volatile uint8_t imuWhoAmI             = 0;
volatile uint16_t imuCalibrationValidSamples = 0;
volatile uint16_t imuCalibrationTotalSamples = 0;

volatile float imuRollDeg  = 0.0f;
volatile float imuPitchDeg = 0.0f;
volatile float imuYawDeg   = 0.0f;

volatile float gyroXDegS = 0.0f;
volatile float gyroYDegS = 0.0f;
volatile float gyroZDegS = 0.0f;
volatile int16_t imuTempD10 = 0;
volatile int16_t imuRawAx = 0;
volatile int16_t imuRawAy = 0;
volatile int16_t imuRawAz = 0;
volatile int16_t imuRawGx = 0;
volatile int16_t imuRawGy = 0;
volatile int16_t imuRawGz = 0;

float axOffset = 0.0f, ayOffset = 0.0f, azOffset = 0.0f;
float gxOffset = 0.0f, gyOffset = 0.0f, gzOffset = 0.0f;

float accXf = 0.0f, accYf = 0.0f, accZf = 1.0f;
float gyroZf = 0.0f;

uint32_t lastImuMicros = 0;
volatile uint32_t imuReadOkCount = 0;
volatile uint32_t imuReadFailCount = 0;
volatile uint32_t imuReadInvalidCount = 0;
volatile uint32_t lastImuOkMs = 0;
volatile uint32_t lastImuFailMs = 0;

// =====================================================
// Status Tracking
// =====================================================
volatile char     lastCmd   = '-';
volatile uint32_t cmdCount  = 0;
volatile uint32_t lastCmdMs = 0;

// =====================================================
// Path / Obstacle State
// obstacleState values:
//   0 = clear
//   1 = detected / stopping
//   2 = waiting  (static-check or side-clear wait in progress)
//   3 = dynamic  (obstacle cleared, continuing)
//   4 = avoiding (full avoidance sequence in progress)
//   5 = all sensors blocked (waiting indefinitely until mask changes)
// =====================================================
SemaphoreHandle_t i2cMutex = NULL;   // protects the local MPU6050 I2C bus only

volatile uint8_t  sensorMask               = 0;
volatile uint8_t  userDecision             = DECISION_NONE;
volatile uint8_t  mechanismState           = MECH_STATE_IDLE;
volatile uint8_t  lastStaticObstacleMask   = 0;
volatile int8_t   lastAvoidSide            = 0;
volatile uint8_t  obstacleState            = 0;
volatile int      currentPathStep          = 0;
volatile bool     pathRunning              = false;
volatile bool     pathFinished             = false;
volatile bool     pathAborted              = false;
volatile float    currentSegmentDoneM      = 0.0f;
volatile float    currentSegmentRemainingM = 0.0f;

volatile uint8_t  guiMode                  = GUI_MODE_AUTO;  // AUTO is default, but waits for GUI start.
volatile bool     autoStartRequested       = false;
volatile bool     autoAbortRequested       = false;
volatile uint8_t  manualDriveDir           = MANUAL_DIR_STOP;
volatile uint8_t  manualDriveSpeed         = 0;
volatile uint32_t lastManualDriveMs        = 0;
volatile float    manualHoldYawDeg         = 0.0f;
volatile float    manualHoldForwardM       = 0.0f;
volatile float    manualHoldLateralM       = 0.0f;
volatile bool     manualHoldYawValid       = false;
volatile uint8_t  manualHoldYawDir         = MANUAL_DIR_STOP;
volatile uint8_t  lastGuiCommand           = 0;
volatile uint8_t  lastGuiDirection         = MANUAL_DIR_STOP;
volatile uint8_t  lastGuiValue             = 0;
volatile uint32_t lastGuiCommandMs         = 0;

// ROS sensor receive diagnostics
volatile uint32_t sensorReadOk   = 0;
volatile uint32_t sensorReadFail = 0;

// =====================================================
// micro-ROS State
// =====================================================
rcl_node_t ros_node;
rclc_support_t ros_support;
rcl_allocator_t ros_allocator;
rclc_executor_t ros_executor;

rcl_subscription_t sensor_mask_sub;
rcl_subscription_t decision_sub;
rcl_subscription_t mechanism_state_sub;
rcl_subscription_t pid_control_sub;
rcl_publisher_t clear_decision_pub;
rcl_publisher_t arm_vision_pub;
rcl_publisher_t mechanism_cmd_pub;
rcl_publisher_t pid_obstacle_state_pub;
rcl_publisher_t pid_path_step_pub;
rcl_publisher_t pid_path_flags_pub;
rcl_publisher_t pid_static_mask_pub;
rcl_publisher_t pid_imu_packet_pub;
rcl_publisher_t pid_encoder_packet_pub;

std_msgs__msg__UInt8 sensor_mask_msg;
std_msgs__msg__UInt8 decision_msg;
std_msgs__msg__UInt8 mechanism_state_msg;
std_msgs__msg__UInt16 pid_control_msg;
std_msgs__msg__Bool clear_decision_msg;
std_msgs__msg__Bool arm_vision_msg;
std_msgs__msg__UInt8 mechanism_cmd_msg;
std_msgs__msg__UInt8 pid_obstacle_state_msg;
std_msgs__msg__UInt8 pid_path_step_msg;
std_msgs__msg__UInt8 pid_path_flags_msg;
std_msgs__msg__UInt8 pid_static_mask_msg;
std_msgs__msg__Int32 pid_imu_packet_msg;
std_msgs__msg__Int32 pid_encoder_packet_msg;

volatile bool microRosReady = false;
volatile uint32_t rosSensorMaskRxCount = 0;
volatile uint32_t rosDecisionRxCount = 0;
volatile uint32_t rosMechanismRxCount = 0;
volatile uint32_t rosClearTxCount = 0;
volatile uint32_t rosArmTxCount = 0;
volatile uint32_t rosMechanismCmdTxCount = 0;
volatile uint32_t rosCommandQueueDropCount = 0;
volatile uint32_t rosPidStatusTxCount = 0;

#define ROS_CMD_CLEAR_DECISION        1u
#define ROS_CMD_ARM_VISION            2u
#define ROS_CMD_MECHANISM             3u
#define CLEAR_DECISION_BURST_COUNT    3u
#define ARM_VISION_BURST_COUNT        5u
#define MECHANISM_CMD_BURST_COUNT     5u
#define ROS_CMD_BURST_INTERVAL_MS     5u
#define ROS_PID_STATUS_PERIOD_MS      200u
#define IMU_PACKET_SCALE              100000L
#define IMU_PACKET_OFFSET             50000L
#define ENCODER_PACKET_SCALE          1000000L
#define ENCODER_PACKET_OFFSET         500000L

#define IMU_FIELD_YAW_CDEG    1L
#define IMU_FIELD_PITCH_CDEG  2L
#define IMU_FIELD_ROLL_CDEG   3L
#define IMU_FIELD_TEMP_D10    4L
#define IMU_FIELD_STATUS      5L
#define PID_FIELD_WHEEL_FL_PCT 6L
#define PID_FIELD_WHEEL_FR_PCT 7L
#define PID_FIELD_WHEEL_BL_PCT 8L
#define PID_FIELD_WHEEL_BR_PCT 9L
#define PID_FIELD_WHEEL_AVG_PCT 10L
#define IMU_FIELD_WHO_AM_I     11L
#define IMU_FIELD_READ_OK_MOD  12L
#define IMU_FIELD_READ_FAIL_MOD 13L
#define IMU_FIELD_CAL_VALID    14L
#define IMU_FIELD_LAST_OK_AGE_MS 15L
#define IMU_FIELD_INIT_OK      16L
#define IMU_FIELD_RAW_AX       17L
#define IMU_FIELD_RAW_AY       18L
#define IMU_FIELD_RAW_AZ       19L
#define IMU_FIELD_RAW_GX       20L
#define IMU_FIELD_RAW_GY       21L
#define IMU_FIELD_RAW_GZ       22L
#define IMU_FIELD_READ_INVALID_MOD 23L

#define ENC_FIELD_TICKS_FL             1L
#define ENC_FIELD_TICKS_FR             2L
#define ENC_FIELD_TICKS_BL             3L
#define ENC_FIELD_TICKS_BR             4L
#define ENC_FIELD_DELTA_FL             5L
#define ENC_FIELD_DELTA_FR             6L
#define ENC_FIELD_DELTA_BL             7L
#define ENC_FIELD_DELTA_BR             8L
#define ENC_FIELD_FORWARD_MM           9L
#define ENC_FIELD_LATERAL_MM          10L
#define ENC_FIELD_ODOM_X_MM           11L
#define ENC_FIELD_ODOM_Y_MM           12L
#define ENC_FIELD_ROBOT_MODE          13L
#define ENC_FIELD_GUI_MODE            14L
#define ENC_FIELD_MANUAL_DIR          15L
#define ENC_FIELD_MANUAL_SPEED        16L
#define ENC_FIELD_LAST_GUI_COMMAND    17L
#define ENC_FIELD_LAST_GUI_DIRECTION  18L
#define ENC_FIELD_LAST_GUI_VALUE      19L
#define ENC_FIELD_LAST_GUI_AGE_MS     20L
#define ENC_FIELD_PATH_STEP           21L
#define ENC_FIELD_OBSTACLE_STATE      22L
#define ENC_FIELD_PATH_FLAGS          23L
#define ENC_FIELD_SEGMENT_DONE_MM     24L
#define ENC_FIELD_SEGMENT_REMAIN_MM   25L
#define ENC_FIELD_TARGET_VX_MMS       26L
#define ENC_FIELD_TARGET_VY_MMS       27L
#define ENC_FIELD_TARGET_WZ_MRADS     28L
#define ENC_FIELD_PROFILE_VX_MMS      29L
#define ENC_FIELD_PROFILE_VY_MMS      30L
#define ENC_FIELD_PROFILE_WZ_MRADS    31L

struct RosCommand {
  uint8_t type;
  uint8_t value;
  uint8_t repeats;
};

QueueHandle_t rosCommandQueue = NULL;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ DebugSerial.printf("micro-ROS error at line %d: %d\n", __LINE__, (int)temp_rc); return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ DebugSerial.printf("micro-ROS soft error at line %d: %d\n", __LINE__, (int)temp_rc); }}

// =====================================================
// micro-ROS Helpers
// =====================================================
void softwareResetZeroState();
void readTicks(int32_t snap[4]);
float getForwardEncoderDistanceM();
float getLateralEncoderDistanceM();

uint8_t remapManualDirectionForOperatorView(uint8_t dir) {
  // The GUI labels stay unchanged. This swaps left/right at the firmware layer
  // for the operator standing face-to-face with the robot.
  switch (dir) {
    case MANUAL_DIR_LEFT:       return MANUAL_DIR_RIGHT;
    case MANUAL_DIR_RIGHT:      return MANUAL_DIR_LEFT;
    case MANUAL_DIR_FWD_LEFT:   return MANUAL_DIR_FWD_RIGHT;
    case MANUAL_DIR_FWD_RIGHT:  return MANUAL_DIR_FWD_LEFT;
    case MANUAL_DIR_BACK_LEFT:  return MANUAL_DIR_BACK_RIGHT;
    case MANUAL_DIR_BACK_RIGHT: return MANUAL_DIR_BACK_LEFT;
    default:                    return dir;
  }
}

bool manualDirectionAllowedBySensors(uint8_t dir) {
  uint8_t m = sensorMask & 0x07;

  if (dir == MANUAL_DIR_STOP ||
      dir == MANUAL_DIR_ROT_L ||
      dir == MANUAL_DIR_ROT_R) {
    return true;
  }

  if ((m & SENSOR_2_MIDDLE_MASK) &&
      (dir == MANUAL_DIR_FWD ||
       dir == MANUAL_DIR_FWD_LEFT ||
       dir == MANUAL_DIR_FWD_RIGHT)) {
    return false;
  }

  if ((m & SENSOR_1_RIGHT_MASK) &&
      (dir == MANUAL_DIR_RIGHT ||
       dir == MANUAL_DIR_FWD_RIGHT ||
       dir == MANUAL_DIR_BACK_RIGHT)) {
    return false;
  }

  if ((m & SENSOR_3_LEFT_MASK) &&
      (dir == MANUAL_DIR_LEFT ||
       dir == MANUAL_DIR_FWD_LEFT ||
       dir == MANUAL_DIR_BACK_LEFT)) {
    return false;
  }

  return true;
}

void sensorMaskCallback(const void * msgin) {
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  sensorMask = msg->data & 0x07;
  sensorReadOk++;
  rosSensorMaskRxCount++;
}

void decisionCallback(const void * msgin) {
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  uint8_t d = msg->data;
  if (d <= DECISION_OBSTACLE) {
    userDecision = d;
    rosDecisionRxCount++;
  }
}

void mechanismStateCallback(const void * msgin) {
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  uint8_t s = msg->data;
  if (s <= MECH_STATE_ERROR) {
    mechanismState = s;
    rosMechanismRxCount++;
  }
}

void pidControlCallback(const void * msgin) {
  const std_msgs__msg__UInt16 * msg = (const std_msgs__msg__UInt16 *)msgin;
  uint16_t packed = msg->data;
  uint8_t command = (uint8_t)((packed >> 12) & 0x0F);
  uint8_t dir = (uint8_t)((packed >> 8) & 0x0F);
  uint8_t value = (uint8_t)(packed & 0xFF);

  lastGuiCommand = command;
  lastGuiDirection = dir;
  lastGuiValue = value;
  lastGuiCommandMs = millis();
  cmdCount++;

  switch (command) {
    case PID_GUI_CMD_MODE:
      if (value <= GUI_MODE_AUTO) {
        guiMode = value;
        if (guiMode == GUI_MODE_MANUAL) {
          autoAbortRequested = true;
          pathAborted = true;
          manualDriveDir = MANUAL_DIR_STOP;
          manualDriveSpeed = 0;
        } else {
          autoAbortRequested = false;
        }
      }
      break;

    case PID_GUI_CMD_AUTO_START:
      if (guiMode == GUI_MODE_AUTO) {
        autoStartRequested = true;
        autoAbortRequested = false;
        pathAborted = false;
        pathFinished = false;
      }
      break;

    case PID_GUI_CMD_AUTO_ABORT:
      autoAbortRequested = true;
      autoStartRequested = false;
      pathAborted = true;
      manualDriveDir = MANUAL_DIR_STOP;
      manualDriveSpeed = 0;
      break;

    case PID_GUI_CMD_MANUAL_DRIVE:
      if (dir > MANUAL_DIR_ROT_R) dir = MANUAL_DIR_STOP;
      dir = remapManualDirectionForOperatorView(dir);
      if (value > 100) value = 100;
      if (dir != MANUAL_DIR_STOP && value < (uint8_t)MANUAL_SPEED_MIN_PERCENT) {
        value = (uint8_t)MANUAL_SPEED_MIN_PERCENT;
      }
      manualDriveDir = dir;
      manualDriveSpeed = (dir == MANUAL_DIR_STOP) ? 0 : value;
      lastManualDriveMs = millis();
      break;

    case PID_GUI_CMD_CAL_IMU:
      imuCalibrationRequested = true;
      break;

    case PID_GUI_CMD_RESET_ZERO:
      softwareResetZeroState();
      break;

    default:
      break;
  }
}

bool setupMicroRosNode() {
  set_microros_transports();
  delay(200);

  // Keep retrying until the Raspberry Pi micro-ROS serial agent is available.
  // This prevents the ESP32 from silently failing if it boots before the agent.
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(500);
  }

  ros_allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&ros_support, 0, NULL, &ros_allocator));
  RCCHECK(rclc_node_init_default(&ros_node, "pid_path_node", "", &ros_support));

  RCCHECK(rclc_subscription_init_best_effort(
    &sensor_mask_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/sensors/mask"));

  RCCHECK(rclc_subscription_init_best_effort(
    &decision_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/sensors/decision"));

  RCCHECK(rclc_subscription_init_best_effort(
    &mechanism_state_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/mechanism/state"));

  RCCHECK(rclc_subscription_init_best_effort(
    &pid_control_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "/robot/cmd/pid_control"));

  RCCHECK(rclc_publisher_init_best_effort(
    &clear_decision_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/robot/cmd/clear_decision"));

  RCCHECK(rclc_publisher_init_best_effort(
    &arm_vision_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/robot/cmd/arm_vision"));

  RCCHECK(rclc_publisher_init_best_effort(
    &mechanism_cmd_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/cmd/mechanism"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pid_obstacle_state_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/pid/obstacle_state"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pid_path_step_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/pid/path_step"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pid_path_flags_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/pid/path_flags"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pid_static_mask_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/pid/static_mask"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pid_imu_packet_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/robot/pid/imu_packet"));

  RCCHECK(rclc_publisher_init_best_effort(
    &pid_encoder_packet_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/robot/pid/encoder_packet"));

  RCCHECK(rclc_executor_init(&ros_executor, &ros_support.context, 4, &ros_allocator));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &sensor_mask_sub, &sensor_mask_msg, &sensorMaskCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &decision_sub, &decision_msg, &decisionCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &mechanism_state_sub, &mechanism_state_msg, &mechanismStateCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &pid_control_sub, &pid_control_msg, &pidControlCallback, ON_NEW_DATA));

  microRosReady = true;
  DebugSerial.println("micro-ROS PID/path node ready.");
  return true;
}

void enqueueRosCommand(uint8_t type, uint8_t value, uint8_t repeats) {
  if (!rosCommandQueue) {
    rosCommandQueueDropCount++;
    return;
  }

  RosCommand cmd;
  cmd.type = type;
  cmd.value = value;
  cmd.repeats = repeats;

  if (xQueueSendToBack(rosCommandQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
    rosCommandQueueDropCount++;
  }
}

void publishClearDecisionCommandFromMicroRosTask() {
  clear_decision_msg.data = true;
  RCSOFTCHECK(rcl_publish(&clear_decision_pub, &clear_decision_msg, NULL));
  rosClearTxCount++;
}

void publishArmVisionCommandFromMicroRosTask(bool arm) {
  arm_vision_msg.data = arm;

  RCSOFTCHECK(rcl_publish(&arm_vision_pub, &arm_vision_msg, NULL));
  rosArmTxCount++;
}

void publishMechanismCommandFromMicroRosTask(uint8_t cmd) {
  mechanism_cmd_msg.data = cmd;

  RCSOFTCHECK(rcl_publish(&mechanism_cmd_pub, &mechanism_cmd_msg, NULL));
  rosMechanismCmdTxCount++;
}

int32_t makeImuPacket(int32_t field, int32_t value) {
  if (value < -49999L) value = -49999L;
  if (value >  49999L) value =  49999L;
  return (field * IMU_PACKET_SCALE) + (value + IMU_PACKET_OFFSET);
}

void publishImuPacketFromMicroRosTask(int32_t field, int32_t value) {
  pid_imu_packet_msg.data = makeImuPacket(field, value);
  RCSOFTCHECK(rcl_publish(&pid_imu_packet_pub, &pid_imu_packet_msg, NULL));
}

int32_t makeEncoderPacket(int32_t field, int32_t value) {
  if (value < -499999L) value = -499999L;
  if (value >  499999L) value =  499999L;
  return (field * ENCODER_PACKET_SCALE) + (value + ENCODER_PACKET_OFFSET);
}

void publishEncoderPacketFromMicroRosTask(int32_t field, int32_t value) {
  pid_encoder_packet_msg.data = makeEncoderPacket(field, value);
  RCSOFTCHECK(rcl_publish(&pid_encoder_packet_pub, &pid_encoder_packet_msg, NULL));
}

bool isImuLiveNow() {
  if (!imuReady || imuCalibrating) return false;
  return (uint32_t)(millis() - lastImuOkMs) <= IMU_LIVE_TIMEOUT_MS;
}

int32_t wheelPercentFromRadS(float radS) {
  float pct = (radS / MAX_RAD_S) * 100.0f;
  if (pct < -100.0f) pct = -100.0f;
  if (pct >  100.0f) pct =  100.0f;
  return (int32_t)pct;
}

void publishWheelTelemetryFromMicroRosTask() {
  float fl = measuredRad[W_FL];
  float fr = measuredRad[W_FR];
  float bl = measuredRad[W_BL];
  float br = measuredRad[W_BR];
  float avg = (fabsf(fl) + fabsf(fr) + fabsf(bl) + fabsf(br)) * 25.0f / MAX_RAD_S;
  if (avg > 100.0f) avg = 100.0f;

  publishImuPacketFromMicroRosTask(PID_FIELD_WHEEL_FL_PCT, wheelPercentFromRadS(fl));
  publishImuPacketFromMicroRosTask(PID_FIELD_WHEEL_FR_PCT, wheelPercentFromRadS(fr));
  publishImuPacketFromMicroRosTask(PID_FIELD_WHEEL_BL_PCT, wheelPercentFromRadS(bl));
  publishImuPacketFromMicroRosTask(PID_FIELD_WHEEL_BR_PCT, wheelPercentFromRadS(br));
  publishImuPacketFromMicroRosTask(PID_FIELD_WHEEL_AVG_PCT, (int32_t)avg);
}

void publishEncoderTelemetryFromMicroRosTask() {
  static int32_t previousTicks[4] = {0, 0, 0, 0};
  static bool havePreviousTicks = false;
  static uint32_t seenEncoderResetGeneration = 0;

  int32_t snap[4];
  int32_t delta[4] = {0, 0, 0, 0};
  readTicks(snap);

  uint32_t resetGenerationNow = encoderResetGeneration;
  if (!havePreviousTicks || seenEncoderResetGeneration != resetGenerationNow) {
    havePreviousTicks = true;
    seenEncoderResetGeneration = resetGenerationNow;
  } else {
    for (int i = 0; i < 4; i++) {
      delta[i] = snap[i] - previousTicks[i];
    }
  }

  for (int i = 0; i < 4; i++) {
    previousTicks[i] = snap[i];
  }

  uint8_t flags = 0;
  if (pathRunning)  flags |= 0x01;
  if (pathFinished) flags |= 0x02;
  if (pathAborted)  flags |= 0x04;

  uint32_t nowMs = millis();
  uint32_t commandAgeMs = (lastGuiCommandMs == 0) ? 499999u : (uint32_t)(nowMs - lastGuiCommandMs);
  if (commandAgeMs > 499999u) commandAgeMs = 499999u;

  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TICKS_FL, snap[W_FL]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TICKS_FR, snap[W_FR]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TICKS_BL, snap[W_BL]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TICKS_BR, snap[W_BR]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_DELTA_FL, delta[W_FL]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_DELTA_FR, delta[W_FR]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_DELTA_BL, delta[W_BL]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_DELTA_BR, delta[W_BR]);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_FORWARD_MM, (int32_t)lroundf(getForwardEncoderDistanceM() * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_LATERAL_MM, (int32_t)lroundf(getLateralEncoderDistanceM() * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_ODOM_X_MM, (int32_t)lroundf(odomX * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_ODOM_Y_MM, (int32_t)lroundf(odomY * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_ROBOT_MODE, (int32_t)robotMode);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_GUI_MODE, (int32_t)guiMode);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_MANUAL_DIR, (int32_t)manualDriveDir);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_MANUAL_SPEED, (int32_t)manualDriveSpeed);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_LAST_GUI_COMMAND, (int32_t)lastGuiCommand);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_LAST_GUI_DIRECTION, (int32_t)lastGuiDirection);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_LAST_GUI_VALUE, (int32_t)lastGuiValue);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_LAST_GUI_AGE_MS, (int32_t)commandAgeMs);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_PATH_STEP, (int32_t)currentPathStep);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_OBSTACLE_STATE, (int32_t)obstacleState);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_PATH_FLAGS, (int32_t)flags);
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_SEGMENT_DONE_MM, (int32_t)lroundf(currentSegmentDoneM * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_SEGMENT_REMAIN_MM, (int32_t)lroundf(currentSegmentRemainingM * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TARGET_VX_MMS, (int32_t)lroundf(targVx * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TARGET_VY_MMS, (int32_t)lroundf(targVy * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_TARGET_WZ_MRADS, (int32_t)lroundf(targWz * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_PROFILE_VX_MMS, (int32_t)lroundf(profVx * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_PROFILE_VY_MMS, (int32_t)lroundf(profVy * 1000.0f));
  publishEncoderPacketFromMicroRosTask(ENC_FIELD_PROFILE_WZ_MRADS, (int32_t)lroundf(profWz * 1000.0f));
}

void publishPidStatusFromMicroRosTask() {
  uint8_t flags = 0;
  if (pathRunning)  flags |= 0x01;
  if (pathFinished) flags |= 0x02;
  if (pathAborted)  flags |= 0x04;

  pid_obstacle_state_msg.data = obstacleState;
  pid_path_step_msg.data = (uint8_t)constrain(currentPathStep, 0, 255);
  pid_path_flags_msg.data = flags;
  pid_static_mask_msg.data = lastStaticObstacleMask & 0x07;

  RCSOFTCHECK(rcl_publish(&pid_obstacle_state_pub, &pid_obstacle_state_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pid_path_step_pub, &pid_path_step_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pid_path_flags_pub, &pid_path_flags_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pid_static_mask_pub, &pid_static_mask_msg, NULL));

  publishImuPacketFromMicroRosTask(IMU_FIELD_STATUS, isImuLiveNow() ? 1L : 0L);
  publishImuPacketFromMicroRosTask(IMU_FIELD_YAW_CDEG,   (int32_t)(imuYawDeg * 100.0f));
  publishImuPacketFromMicroRosTask(IMU_FIELD_PITCH_CDEG, (int32_t)(imuPitchDeg * 100.0f));
  publishImuPacketFromMicroRosTask(IMU_FIELD_ROLL_CDEG,  (int32_t)(imuRollDeg * 100.0f));
  publishImuPacketFromMicroRosTask(IMU_FIELD_TEMP_D10,   (int32_t)imuTempD10);
  publishWheelTelemetryFromMicroRosTask();

  uint32_t lastOkAgeMs = (lastImuOkMs == 0) ? 49999u : (uint32_t)(millis() - lastImuOkMs);
  if (lastOkAgeMs > 49999u) lastOkAgeMs = 49999u;

  publishImuPacketFromMicroRosTask(IMU_FIELD_WHO_AM_I, (int32_t)imuWhoAmI);
  publishImuPacketFromMicroRosTask(IMU_FIELD_READ_OK_MOD, (int32_t)(imuReadOkCount % 50000u));
  publishImuPacketFromMicroRosTask(IMU_FIELD_READ_FAIL_MOD, (int32_t)(imuReadFailCount % 50000u));
  publishImuPacketFromMicroRosTask(IMU_FIELD_CAL_VALID, (int32_t)imuCalibrationValidSamples);
  publishImuPacketFromMicroRosTask(IMU_FIELD_LAST_OK_AGE_MS, (int32_t)lastOkAgeMs);
  publishImuPacketFromMicroRosTask(IMU_FIELD_INIT_OK, imuInitOk ? 1L : 0L);
  publishImuPacketFromMicroRosTask(IMU_FIELD_RAW_AX, (int32_t)imuRawAx);
  publishImuPacketFromMicroRosTask(IMU_FIELD_RAW_AY, (int32_t)imuRawAy);
  publishImuPacketFromMicroRosTask(IMU_FIELD_RAW_AZ, (int32_t)imuRawAz);
  publishImuPacketFromMicroRosTask(IMU_FIELD_RAW_GX, (int32_t)imuRawGx);
  publishImuPacketFromMicroRosTask(IMU_FIELD_RAW_GY, (int32_t)imuRawGy);
  publishImuPacketFromMicroRosTask(IMU_FIELD_RAW_GZ, (int32_t)imuRawGz);
  publishImuPacketFromMicroRosTask(IMU_FIELD_READ_INVALID_MOD, (int32_t)(imuReadInvalidCount % 50000u));
  publishEncoderTelemetryFromMicroRosTask();
  rosPidStatusTxCount++;
}

void publishClearDecisionCommand() {
  // Request only. The micro-ROS task owns the serial transport and does the real publish.
  enqueueRosCommand(ROS_CMD_CLEAR_DECISION, true, CLEAR_DECISION_BURST_COUNT);
}

void publishArmVisionCommand(bool arm) {
  // Request only. The micro-ROS task publishes /robot/cmd/arm_vision.
  // The Sensor/Mechanism ESP republishes /vision/arm for the Pi vision node.
  enqueueRosCommand(ROS_CMD_ARM_VISION, arm, ARM_VISION_BURST_COUNT);
}

void publishMechanismRunCommand() {
  // Request only. The micro-ROS task publishes /robot/cmd/mechanism = MECH_CMD_RUN.
  enqueueRosCommand(ROS_CMD_MECHANISM, MECH_CMD_RUN, MECHANISM_CMD_BURST_COUNT);
}

void taskMicroRos(void* pv) {
  TickType_t t = xTaskGetTickCount();
  RosCommand activeCmd;
  bool hasActiveCmd = false;
  uint32_t lastCommandPublishMs = 0;
  uint32_t lastPidStatusPublishMs = 0;

  while (true) {
    if (microRosReady) {
      // Keep every micro-ROS operation in this one task. USB serial micro-ROS is
      // fragile when one task spins while another task publishes.
      RCSOFTCHECK(rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(2)));

      uint32_t nowMs = millis();

      if (!hasActiveCmd && rosCommandQueue) {
        if (xQueueReceive(rosCommandQueue, &activeCmd, 0) == pdTRUE) {
          hasActiveCmd = true;
          lastCommandPublishMs = 0;
        }
      }

      if (hasActiveCmd &&
          activeCmd.repeats > 0 &&
          (lastCommandPublishMs == 0 ||
           (uint32_t)(nowMs - lastCommandPublishMs) >= ROS_CMD_BURST_INTERVAL_MS)) {
        lastCommandPublishMs = nowMs;

        if (activeCmd.type == ROS_CMD_CLEAR_DECISION) {
          publishClearDecisionCommandFromMicroRosTask();
        } else if (activeCmd.type == ROS_CMD_ARM_VISION) {
          publishArmVisionCommandFromMicroRosTask(activeCmd.value != 0);
        } else if (activeCmd.type == ROS_CMD_MECHANISM) {
          publishMechanismCommandFromMicroRosTask(activeCmd.value);
        }

        activeCmd.repeats--;
        if (activeCmd.repeats == 0) {
          hasActiveCmd = false;
        }
      }

      if ((uint32_t)(nowMs - lastPidStatusPublishMs) >= ROS_PID_STATUS_PERIOD_MS) {
        lastPidStatusPublishMs = nowMs;
        publishPidStatusFromMicroRosTask();
      }
    }
    vTaskDelayUntil(&t, pdMS_TO_TICKS(2));
  }
}

// =====================================================
// Utility Functions
// =====================================================
float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float signFloat(float v) {
  if (v > 0.0f) return  1.0f;
  if (v < 0.0f) return -1.0f;
  return 0.0f;
}

float wrapAngle180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float angleDiffDeg(float target, float current) {
  return wrapAngle180(target - current);
}

float stepToward(float current, float target, float maxStep) {
  float err = target - current;
  if (err >  maxStep) return current + maxStep;
  if (err < -maxStep) return current - maxStep;
  return target;
}

float crossTrackCorrectionMps(float errorM) {
  if (fabsf(errorM) < CROSSTRACK_DEADBAND_M) {
    return 0.0f;
  }
  return clampFloat(KP_CROSSTRACK * errorM,
                    -MAX_CROSSTRACK_CORR_MPS,
                    MAX_CROSSTRACK_CORR_MPS);
}

// =====================================================
// Encoder ISRs
// =====================================================
void IRAM_ATTR isr_fl_a() {
  int a = gpio_get_level((gpio_num_t)FL_A);
  int b = gpio_get_level((gpio_num_t)FL_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_FL] += (a != b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_fl_b() {
  int a = gpio_get_level((gpio_num_t)FL_A);
  int b = gpio_get_level((gpio_num_t)FL_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_FL] += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_fr_a() {
  int a = gpio_get_level((gpio_num_t)FR_A);
  int b = gpio_get_level((gpio_num_t)FR_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_FR] += (a != b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_fr_b() {
  int a = gpio_get_level((gpio_num_t)FR_A);
  int b = gpio_get_level((gpio_num_t)FR_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_FR] += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_bl_a() {
  int a = gpio_get_level((gpio_num_t)BL_A);
  int b = gpio_get_level((gpio_num_t)BL_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_BL] += (a != b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_bl_b() {
  int a = gpio_get_level((gpio_num_t)BL_A);
  int b = gpio_get_level((gpio_num_t)BL_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_BL] += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_br_a() {
  int a = gpio_get_level((gpio_num_t)BR_A);
  int b = gpio_get_level((gpio_num_t)BR_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_BR] += (a != b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isr_br_b() {
  int a = gpio_get_level((gpio_num_t)BR_A);
  int b = gpio_get_level((gpio_num_t)BR_B);
  portENTER_CRITICAL_ISR(&encMux);
  ticks[W_BR] += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}

void readTicks(int32_t snap[4]) {
  portENTER_CRITICAL(&encMux);
  snap[W_FL] = (int32_t)ENCODER_SIGN[W_FL] * ticks[W_FL];
  snap[W_FR] = (int32_t)ENCODER_SIGN[W_FR] * ticks[W_FR];
  snap[W_BL] = (int32_t)ENCODER_SIGN[W_BL] * ticks[W_BL];
  snap[W_BR] = (int32_t)ENCODER_SIGN[W_BR] * ticks[W_BR];
  portEXIT_CRITICAL(&encMux);
}

void resetTicks() {
  portENTER_CRITICAL(&encMux);
  ticks[W_FL]=0; ticks[W_FR]=0; ticks[W_BL]=0; ticks[W_BR]=0;
  encoderResetGeneration++;
  portEXIT_CRITICAL(&encMux);
}

// =====================================================
// Encoder Distance Helpers
// =====================================================
float ticksToRad(int32_t t, uint8_t wheel) {
  return ((float)t / CPR[wheel]) * 2.0f * PI;
}

float getForwardEncoderDistanceM() {
  int32_t snap[4]; readTicks(snap);
  float w0 = ticksToRad(snap[W_FL], W_FL);
  float w1 = ticksToRad(snap[W_FR], W_FR);
  float w2 = ticksToRad(snap[W_BL], W_BL);
  float w3 = ticksToRad(snap[W_BR], W_BR);
  return (WHEEL_R / 4.0f) * (w0 + w1 + w2 + w3);
}

float getLateralEncoderDistanceM() {
  int32_t snap[4]; readTicks(snap);
  float w0 = ticksToRad(snap[W_FL], W_FL);
  float w1 = ticksToRad(snap[W_FR], W_FR);
  float w2 = ticksToRad(snap[W_BL], W_BL);
  float w3 = ticksToRad(snap[W_BR], W_BR);
  return (WHEEL_R / 4.0f) * (-w0 + w1 + w2 - w3);   // positive = strafe left
}

// =====================================================
// PWM Compatibility (Arduino core v2 and v3)
// =====================================================
void setupPwmPin(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(uint8_t pin, uint8_t channel, int duty) {
  duty = constrain(duty, 0, PWM_MAX);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

bool manualTurnCommandActive();
bool lateralStrafeCommandActive();
bool rearOpposedAssistActive(uint8_t i);
void resetWheelRecovery(uint8_t i);
void updateWheelResponseRecovery(uint8_t i, float sp, float meas);
int16_t applyWheelRecoveryAssist(uint8_t i, int16_t pidPwm, float sp);

// =====================================================
// Motor Driver
// =====================================================
void motorWrite(uint8_t i, int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (abs(pwm) < 2) {
    digitalWrite(MOT[i].dn1, LOW); digitalWrite(MOT[i].dn2, LOW);
    writePwm(MOT[i].pwm, MOT[i].ch, 0);
    motorPwmOut[i] = 0;
    return;
  }
  int16_t drivePwm = constrain((int16_t)(pwm * MOTOR_SIGN[i]), -255, 255);
  uint8_t minPwm = PWM_MIN;
  RobotMode modeSnapshot = robotMode;
  if (modeSnapshot == MODE_TURN_ANGLE || manualTurnCommandActive()) {
    minPwm = PWM_MIN_TURN;
  } else if (lateralStrafeCommandActive()) {
    minPwm = PWM_MIN_LATERAL;
  } else if (modeSnapshot == MODE_MOVE_DISTANCE || modeSnapshot == MODE_STRAFE_DISTANCE) {
    minPwm = PWM_MIN_MOVE;
  }
  if (rearOpposedAssistActive(i) && minPwm < PWM_MIN_REAR_OPPOSED) {
    minPwm = PWM_MIN_REAR_OPPOSED;
  }
  uint8_t mag = (uint8_t)(minPwm + (abs(drivePwm) * (PWM_MAX - minPwm)) / 255);
  if (drivePwm > 0) { digitalWrite(MOT[i].dn1, HIGH); digitalWrite(MOT[i].dn2, LOW); }
  else              { digitalWrite(MOT[i].dn1, LOW);  digitalWrite(MOT[i].dn2, HIGH); }
  writePwm(MOT[i].pwm, MOT[i].ch, mag);
  motorPwmOut[i] = pwm;
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) motorWrite(i, 0);
}

// =====================================================
// PID Functions
// =====================================================
void resetPid(uint8_t i) { pid[i].integral = 0.0f; pid[i].prevErr = 0.0f; }
void resetAllPid()        { for (int i = 0; i < 4; i++) resetPid(i); }

bool manualTurnCommandActive() {
  return robotMode == MODE_IDLE &&
         guiMode == GUI_MODE_MANUAL &&
         manualDriveSpeed > 0 &&
         (manualDriveDir == MANUAL_DIR_ROT_L || manualDriveDir == MANUAL_DIR_ROT_R);
}

bool lateralStrafeCommandActive() {
  RobotMode modeSnapshot = robotMode;
  if (modeSnapshot == MODE_STRAFE_DISTANCE) return true;

  if (modeSnapshot != MODE_IDLE ||
      guiMode != GUI_MODE_MANUAL ||
      manualDriveSpeed == 0) {
    return false;
  }

  return manualDriveDir == MANUAL_DIR_LEFT ||
         manualDriveDir == MANUAL_DIR_RIGHT ||
         manualDriveDir == MANUAL_DIR_FWD_LEFT ||
         manualDriveDir == MANUAL_DIR_FWD_RIGHT ||
         manualDriveDir == MANUAL_DIR_BACK_LEFT ||
         manualDriveDir == MANUAL_DIR_BACK_RIGHT;
}

bool rearOpposedAssistActive(uint8_t i) {
  if (i != W_BL && i != W_BR) return false;
  float bl = setpointRad[W_BL];
  float br = setpointRad[W_BR];
  if (fabsf(bl) < WHEEL_RESPONSE_SETPOINT_RAD_S ||
      fabsf(br) < WHEEL_RESPONSE_SETPOINT_RAD_S) {
    return false;
  }
  return signFloat(bl) != signFloat(br);
}

void resetWheelRecovery(uint8_t i) {
  wheelBadResponseStartMs[i] = 0;
  wheelLastRecoveryStepMs[i] = 0;
  wheelRecoveryBoostPwm[i] = 0;
}

void updateWheelResponseRecovery(uint8_t i, float sp, float meas) {
  float absSp = fabsf(sp);
  if (absSp < WHEEL_RESPONSE_SETPOINT_RAD_S) {
    resetWheelRecovery(i);
    return;
  }

  float absMeas = fabsf(meas);
  float ratio = absMeas / absSp;
  bool wrongDirection =
    absMeas > WHEEL_WRONG_DIR_RAD_S && signFloat(meas) != signFloat(sp);
  bool weakResponse = ratio < WHEEL_RESPONSE_MIN_RATIO;
  uint32_t nowMs = millis();

  if (wrongDirection || weakResponse) {
    if (wheelBadResponseStartMs[i] == 0) {
      wheelBadResponseStartMs[i] = nowMs;
      wheelLastRecoveryStepMs[i] = 0;
    }

    if ((uint32_t)(nowMs - wheelBadResponseStartMs[i]) >= WHEEL_STALL_TIME_MS &&
        (wheelLastRecoveryStepMs[i] == 0 ||
         (uint32_t)(nowMs - wheelLastRecoveryStepMs[i]) >= WHEEL_RECOVERY_STEP_PERIOD_MS)) {
      int16_t step = wrongDirection ? (WHEEL_RECOVERY_STEP_PWM * 2) : WHEEL_RECOVERY_STEP_PWM;
      int16_t nextBoost = wheelRecoveryBoostPwm[i] + step;
      if (nextBoost > WHEEL_RECOVERY_MAX_PWM) nextBoost = WHEEL_RECOVERY_MAX_PWM;
      wheelRecoveryBoostPwm[i] = nextBoost;
      wheelLastRecoveryStepMs[i] = nowMs;
      if (wrongDirection) resetPid(i);
    }
    return;
  }

  wheelBadResponseStartMs[i] = 0;
  wheelLastRecoveryStepMs[i] = 0;
  if (ratio > WHEEL_RESPONSE_RELEASE_RATIO && wheelRecoveryBoostPwm[i] > 0) {
    int16_t nextBoost = wheelRecoveryBoostPwm[i] - WHEEL_RECOVERY_DECAY_PWM;
    wheelRecoveryBoostPwm[i] = (nextBoost > 0) ? nextBoost : 0;
  }
}

int16_t applyWheelRecoveryAssist(uint8_t i, int16_t pidPwm, float sp) {
  float commandSign = signFloat(sp);
  if (commandSign == 0.0f) return pidPwm;

  float ffPwm = commandSign * WHEEL_FF_GAIN * (fabsf(sp) / MAX_RAD_S) * 255.0f;
  int16_t assistPwm = wheelRecoveryBoostPwm[i];
  if (rearOpposedAssistActive(i)) {
    assistPwm += REAR_OPPOSED_ASSIST_PWM;
  }

  float out = (float)pidPwm + ffPwm;
  if (assistPwm > 0) {
    float assistSigned = commandSign * (float)assistPwm;
    if (wheelRecoveryBoostPwm[i] > 0 &&
        signFloat(out) == -commandSign &&
        fabsf(out) > 20.0f) {
      out = assistSigned;
    } else {
      out += assistSigned;
    }
  }

  return (int16_t)clampFloat(out, -255.0f, 255.0f);
}

float pidCompute(uint8_t i, float sp, float meas) {
  float err = sp - meas;
  pid[i].integral += err * PID_DT;
  pid[i].integral  = clampFloat(pid[i].integral,
                                -pid[i].integralLimit, pid[i].integralLimit);
  float deriv    = (err - pid[i].prevErr) / PID_DT;
  pid[i].prevErr = err;
  float out = (pid[i].kp * err) + (pid[i].ki * pid[i].integral) + (pid[i].kd * deriv);
  return clampFloat(out, pid[i].outMin, pid[i].outMax);
}

// =====================================================
// Mecanum IK
// =====================================================
void mecanumIK(float Vx, float Vy, float Wz) {
  float r = 1.0f / WHEEL_R;
  setpointRad[W_FL] = r * (Vx - Vy - ROBOT_LW * Wz);
  setpointRad[W_FR] = r * (Vx + Vy + ROBOT_LW * Wz);
  setpointRad[W_BL] = r * (Vx + Vy - ROBOT_LW * Wz);
  setpointRad[W_BR] = r * (Vx - Vy + ROBOT_LW * Wz);
  float peak = 0.001f;
  for (int i = 0; i < 4; i++) if (fabsf(setpointRad[i]) > peak) peak = fabsf(setpointRad[i]);
  if (peak > MAX_RAD_S) {
    float scale = MAX_RAD_S / peak;
    for (int i = 0; i < 4; i++) setpointRad[i] *= scale;
  }
}

void setSetpointsZero() { for (int i = 0; i < 4; i++) setpointRad[i] = 0.0f; }

// =====================================================
// IMU Low-Level
// =====================================================
bool mpuWrite(uint8_t reg, uint8_t value) {
  bool ok = false;
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(value);
  ok = (Wire.endTransmission() == 0);
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return ok;
}

bool mpuReadRegister(uint8_t reg, uint8_t &value) {
  bool ok = false;
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0) {
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) == 1) {
      value = Wire.read();
      ok = true;
    }
  }
  while (Wire.available()) Wire.read();
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return ok;
}

bool readWhoAmIWithRetry(uint8_t &whoOut) {
  whoOut = 0;
  for (uint8_t attempt = 0; attempt < 10; attempt++) {
    uint8_t who = 0;
    if (mpuReadRegister(MPU_REG_WHO_AM_I, who)) {
      whoOut = who;
      if (who == 0x68) return true;
    }
    delay(20);
  }
  return false;
}

bool mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  bool ok = false;
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0) {
    uint8_t received = Wire.requestFrom((uint8_t)MPU_ADDR, len);
    if (received == len) {
      for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
      ok = true;
    }
  }
  while (Wire.available()) Wire.read();
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return ok;
}

bool imuRawSampleLooksAlive(int16_t axRaw, int16_t ayRaw, int16_t azRaw) {
  uint32_t axAbs = (uint32_t)((axRaw < 0) ? -(long)axRaw : (long)axRaw);
  uint32_t ayAbs = (uint32_t)((ayRaw < 0) ? -(long)ayRaw : (long)ayRaw);
  uint32_t azAbs = (uint32_t)((azRaw < 0) ? -(long)azRaw : (long)azRaw);
  return (axAbs + ayAbs + azAbs) > 4000u;
}

bool readMPURaw(int16_t &axRaw, int16_t &ayRaw, int16_t &azRaw,
                int16_t &tempRaw,
                int16_t &gxRaw, int16_t &gyRaw, int16_t &gzRaw) {
  uint8_t data[14];
  if (!mpuReadBytes(MPU_REG_ACCEL_XOUT_H, data, 14)) return false;
  axRaw = (int16_t)((data[0]  << 8) | data[1]);
  ayRaw = (int16_t)((data[2]  << 8) | data[3]);
  azRaw = (int16_t)((data[4]  << 8) | data[5]);
  tempRaw = (int16_t)((data[6] << 8) | data[7]);
  gxRaw = (int16_t)((data[8]  << 8) | data[9]);
  gyRaw = (int16_t)((data[10] << 8) | data[11]);
  gzRaw = (int16_t)((data[12] << 8) | data[13]);
  imuRawAx = axRaw;
  imuRawAy = ayRaw;
  imuRawAz = azRaw;
  imuRawGx = gxRaw;
  imuRawGy = gyRaw;
  imuRawGz = gzRaw;
  imuTempD10 = (int16_t)roundf((((float)tempRaw / 340.0f) + 36.53f) * 10.0f);
  return true;
}

bool initMPU() {
  delay(100);
  bool ok = true;
  uint8_t whoBefore = 0;

  imuReady = false;
  imuInitOk = false;
  imuWhoAmI = 0;

  if (!readWhoAmIWithRetry(whoBefore)) {
    imuWhoAmI = whoBefore;
    return false;
  }
  imuWhoAmI = whoBefore;

  ok &= mpuWrite(MPU_REG_PWR_MGMT_1, 0x80); delay(100);  // hardware reset
  ok &= mpuWrite(MPU_REG_PWR_MGMT_1, 0x01); delay(100);  // wake, X gyro PLL clock
  ok &= mpuWrite(MPU_REG_SMPLRT_DIV, 9);                 // 100 Hz sample rate with DLPF
  ok &= mpuWrite(MPU_REG_CONFIG, 0x03);                   // DLPF_CFG=3
  ok &= mpuWrite(MPU_REG_GYRO_CONFIG, 0x00);              // +/-250 dps
  ok &= mpuWrite(MPU_REG_ACCEL_CONFIG, 0x00);             // +/-2g

  uint8_t whoAfter = 0;
  bool whoAfterOk = readWhoAmIWithRetry(whoAfter);
  imuWhoAmI = whoAfter;

  imuInitOk = ok && whoAfterOk && (whoAfter == 0x68);
  return imuInitOk;
}

void stopRobotHard() {
  robotMode = MODE_IDLE;
  targVx=0; targVy=0; targWz=0;
  profVx=0; profVy=0; profWz=0;
  moveYawIntegralRad = 0.0f;
  turnStallStartMs = 0;
  turnStallBoostActive = false;
  setSetpointsZero();
  resetAllPid();
  stopAllMotors();
}

void calibrateMPU(uint16_t samples) {
  imuCalibrating = true; imuReady = false;
  imuCalibrationTotalSamples = samples;
  imuCalibrationValidSamples = 0;
  lastImuOkMs = 0;
  stopRobotHard();
  if (!imuInitOk) {
    DebugSerial.println("IMU calibration requested while MPU init is invalid. Retrying init.");
    if (!initMPU()) {
      imuCalibrating = false;
      DebugSerial.println("IMU calibration refused: MPU initialization is not valid.");
      return;
    }
  }
  uint8_t who = 0;
  if (!readWhoAmIWithRetry(who)) {
    imuWhoAmI = who;
    imuInitOk = false;
    imuCalibrating = false;
    DebugSerial.println("IMU calibration refused: WHO_AM_I is not 0x68.");
    return;
  }
  imuWhoAmI = who;
  DebugSerial.println("IMU calibration started. Keep robot still.");
  double axSum=0,aySum=0,azSum=0,gxSum=0,gySum=0,gzSum=0;
  uint16_t valid=0;
  delay(2000);
  for (uint16_t i=0; i<samples; i++) {
    int16_t axR,ayR,azR,tempR,gxR,gyR,gzR;
    if (readMPURaw(axR,ayR,azR,tempR,gxR,gyR,gzR)) {
      if (imuRawSampleLooksAlive(axR, ayR, azR)) {
        axSum+=axR; aySum+=ayR; azSum+=azR;
        gxSum+=gxR; gySum+=gyR; gzSum+=gzR;
        valid++;
      } else {
        imuReadInvalidCount++;
      }
    } else {
      imuReadFailCount++;
      lastImuFailMs = millis();
    }
    delay(2);
  }
  imuCalibrationValidSamples = valid;
  uint16_t minValid = (uint16_t)((uint32_t)samples * 9u / 10u);
  if (imuInitOk && valid >= minValid) {
    axOffset=(float)(axSum/valid)/ACC_SENS;
    ayOffset=(float)(aySum/valid)/ACC_SENS;
    azOffset=(float)(azSum/valid)/ACC_SENS-1.0f;
    gxOffset=(float)(gxSum/valid)/GYRO_SENS;
    gyOffset=(float)(gySum/valid)/GYRO_SENS;
    gzOffset=(float)(gzSum/valid)/GYRO_SENS;
    imuRollDeg=0; imuPitchDeg=0; imuYawDeg=0;
    gyroXDegS=0; gyroYDegS=0; gyroZDegS=0;
    accXf=0; accYf=0; accZf=1; gyroZf=0;
    lastImuMicros=micros();
    lastImuOkMs=millis();
    imuReady=true;
    DebugSerial.println("IMU calibration done.");
    DebugSerial.printf("Gyro offsets deg/s: %.6f %.6f %.6f\n",gxOffset,gyOffset,gzOffset);
  } else {
    imuReady=false;
    DebugSerial.println("IMU calibration FAILED.");
  }
  imuCalibrating=false;
}

void updateIMU() {
  if (!imuReady || imuCalibrating) return;
  int16_t axRaw,ayRaw,azRaw,tempRaw,gxRaw,gyRaw,gzRaw;
  if (!readMPURaw(axRaw,ayRaw,azRaw,tempRaw,gxRaw,gyRaw,gzRaw)) {
    imuReadFailCount++;
    lastImuFailMs=millis();
    return;
  }
  if (!imuRawSampleLooksAlive(axRaw, ayRaw, azRaw)) {
    imuReadInvalidCount++;
    lastImuFailMs=millis();
    return;
  }
  imuReadOkCount++;
  lastImuOkMs=millis();
  uint32_t now=micros();
  float dt=(now-lastImuMicros)/1000000.0f;
  lastImuMicros=now;
  if (dt<=0||dt>0.1f) return;
  float ax=((float)axRaw/ACC_SENS)-axOffset;
  float ay=((float)ayRaw/ACC_SENS)-ayOffset;
  float az=((float)azRaw/ACC_SENS)-azOffset;
  float gx=((float)gxRaw/GYRO_SENS)-gxOffset;
  float gy=((float)gyRaw/GYRO_SENS)-gyOffset;
  float gz=((float)gzRaw/GYRO_SENS)-gzOffset;
  const float accA=0.15f,gyroA=0.15f;
  accXf=(1-accA)*accXf+accA*ax; accYf=(1-accA)*accYf+accA*ay;
  accZf=(1-accA)*accZf+accA*az; gyroZf=(1-gyroA)*gyroZf+gyroA*gz;
  gyroXDegS=gx; gyroYDegS=gy; gyroZDegS=gyroZf;
  float rollAcc =atan2f(accYf,accZf)*180.0f/PI;
  float pitchAcc=atan2f(-accXf,sqrtf(accYf*accYf+accZf*accZf))*180.0f/PI;
  imuRollDeg =0.98f*(imuRollDeg +gx*dt)+0.02f*rollAcc;
  imuPitchDeg=0.98f*(imuPitchDeg+gy*dt)+0.02f*pitchAcc;
  imuYawDeg +=gyroZf*dt;
  imuYawDeg  =wrapAngle180(imuYawDeg);
}

// =====================================================
// Motion Planner
// =====================================================
void updateManualDriveTargets() {
  uint32_t nowMs = millis();

  if (guiMode != GUI_MODE_MANUAL ||
      manualDriveDir == MANUAL_DIR_STOP ||
      manualDriveSpeed == 0 ||
      (uint32_t)(nowMs - lastManualDriveMs) > MANUAL_DRIVE_TIMEOUT_MS) {
    targVx = 0;
    targVy = 0;
    targWz = 0;
    manualDriveDir = MANUAL_DIR_STOP;
    manualDriveSpeed = 0;
    manualHoldYawValid = false;
    manualHoldYawDir = MANUAL_DIR_STOP;
    return;
  }

  if (!manualDirectionAllowedBySensors(manualDriveDir)) {
    targVx = 0.0f;
    targVy = 0.0f;
    targWz = 0.0f;
    manualDriveDir = MANUAL_DIR_STOP;
    manualDriveSpeed = 0;
    manualHoldYawValid = false;
    manualHoldYawDir = MANUAL_DIR_STOP;
    return;
  }

  float pct = clampFloat((float)manualDriveSpeed, MANUAL_SPEED_MIN_PERCENT, 100.0f) / 100.0f;
  float manualLinearMax = (MANUAL_LINEAR_SPEED_PERCENT / 100.0f) * MAX_MPS;
  float manualLateralMax = manualLinearMax * MANUAL_LATERAL_SPEED_SCALE;
  float manualAngularMax = (TURN_SPEED_PERCENT / 100.0f) * MAX_WZ;
  float linear = clampFloat(pct * manualLinearMax, MIN_MOVE_SPEED_MPS, manualLinearMax);
  float lateral = clampFloat(pct * manualLateralMax, MIN_MOVE_SPEED_MPS, manualLateralMax);
  float angular = clampFloat(pct * manualAngularMax, TURN_FINE_MIN_WZ, manualAngularMax);
  const float diag = 0.70710678f;

  targVx = 0;
  targVy = 0;
  targWz = 0;

  switch (manualDriveDir) {
    case MANUAL_DIR_FWD:        targVx =  linear; break;
    case MANUAL_DIR_BACK:       targVx = -linear; break;
    case MANUAL_DIR_LEFT:       targVy =  lateral; break;
    case MANUAL_DIR_RIGHT:      targVy = -lateral; break;
    case MANUAL_DIR_FWD_LEFT:   targVx =  linear * diag; targVy =  lateral * diag; break;
    case MANUAL_DIR_FWD_RIGHT:  targVx =  linear * diag; targVy = -lateral * diag; break;
    case MANUAL_DIR_BACK_LEFT:  targVx = -linear * diag; targVy =  lateral * diag; break;
    case MANUAL_DIR_BACK_RIGHT: targVx = -linear * diag; targVy = -lateral * diag; break;
    case MANUAL_DIR_ROT_L:      targWz =  angular; break;
    case MANUAL_DIR_ROT_R:      targWz = -angular; break;
    default: break;
  }

  bool translationalManualMove = (targVx != 0.0f || targVy != 0.0f) && targWz == 0.0f;
  if (translationalManualMove) {
    if (!isImuLiveNow()) {
      targVx = 0.0f;
      targVy = 0.0f;
      targWz = 0.0f;
      manualDriveDir = MANUAL_DIR_STOP;
      manualDriveSpeed = 0;
      manualHoldYawValid = false;
      manualHoldYawDir = MANUAL_DIR_STOP;
      return;
    }

    if (!manualHoldYawValid || manualHoldYawDir != manualDriveDir) {
      manualHoldYawDeg = imuYawDeg;
      manualHoldForwardM = getForwardEncoderDistanceM();
      manualHoldLateralM = getLateralEncoderDistanceM();
      manualHoldYawValid = true;
      manualHoldYawDir = manualDriveDir;
      moveYawIntegralRad = 0.0f;
    }

    float ye = angleDiffDeg(manualHoldYawDeg, imuYawDeg);
    float yeRad = ye * PI / 180.0f;
    float gyroZRadS = gyroZDegS * PI / 180.0f;
    moveYawIntegralRad = clampFloat(moveYawIntegralRad + (yeRad * PLANNER_DT), -MOVE_YAW_I_LIMIT, MOVE_YAW_I_LIMIT);
    float corr = clampFloat((KP_MOVE_YAW * yeRad) + (KI_MOVE_YAW * moveYawIntegralRad) - (KD_MOVE_YAW * gyroZRadS), -MAX_MANUAL_YAW_CORR, MAX_MANUAL_YAW_CORR);

    if (fabsf(ye) < MOVE_YAW_DEADBAND_DEG && fabsf(gyroZDegS) < MOVE_YAW_RATE_DEADBAND_DPS) {
      corr = 0.0f;
      moveYawIntegralRad = 0.0f;
    }

    targWz = MOVE_YAW_CONTROL_SIGN * corr;

    float yawAbs = fabsf(ye);
    float rateAbs = fabsf(gyroZDegS);
    if (yawAbs > MANUAL_YAW_SLOWDOWN_DEG || rateAbs > MANUAL_YAW_SLOWDOWN_RATE_DPS) {
      float yawScale = 1.0f;
      if (yawAbs > MANUAL_YAW_SLOWDOWN_DEG) {
        yawScale = 1.0f - ((yawAbs - MANUAL_YAW_SLOWDOWN_DEG) /
                           (MANUAL_YAW_FULL_SLOWDOWN_DEG - MANUAL_YAW_SLOWDOWN_DEG));
      }
      float rateScale = 1.0f;
      if (rateAbs > MANUAL_YAW_SLOWDOWN_RATE_DPS) {
        rateScale = MANUAL_YAW_SLOWDOWN_RATE_DPS / rateAbs;
      }
      float linearScale = clampFloat(fminf(yawScale, rateScale),
                                     MANUAL_YAW_MIN_LINEAR_SCALE, 1.0f);
      targVx *= linearScale;
      targVy *= linearScale;
    }

    float linearNorm = sqrtf((targVx * targVx) + (targVy * targVy));
    if (linearNorm > 0.001f) {
      float ux = targVx / linearNorm;
      float uy = targVy / linearNorm;
      float nx = -uy;
      float ny = ux;
      float dx = getForwardEncoderDistanceM() - manualHoldForwardM;
      float dy = getLateralEncoderDistanceM() - manualHoldLateralM;
      float crossTrackErrorM = (dx * nx) + (dy * ny);
      float lineCorr = -crossTrackCorrectionMps(crossTrackErrorM);
      targVx += lineCorr * nx;
      targVy += lineCorr * ny;
    }
  } else {
    manualHoldYawValid = false;
    manualHoldYawDir = MANUAL_DIR_STOP;
  }
}

void updateMotionPlanner() {
  RobotMode mode = robotMode;
  if (mode == MODE_IDLE) {
    updateManualDriveTargets();
    return;
  }

  if (!isImuLiveNow()) {
    pathAborted = true;
    stopRobotHard();
    return;
  }

  float spd = clampFloat(gSpd, 0.05f, 1.0f);

  if (mode == MODE_MOVE_DISTANCE) {
    float now   = getForwardEncoderDistanceM();
    float moved = now - moveStartForwardM;
    float rem   = moveTargetDistanceM - moved;
    moveDoneM=moved; moveRemainingM=rem;
    if (fabsf(rem)<=DIST_TOLERANCE_M) { goalReached=true; stopRobotHard(); return; }
    float dir=signFloat(rem);
    float ms=clampFloat(spd*MAX_MPS,MIN_MOVE_SPEED_MPS,MAX_MPS);
    float cs=ms; if(fabsf(rem)<SLOWDOWN_DISTANCE_M) cs=clampFloat(ms*fabsf(rem)/SLOWDOWN_DISTANCE_M,MIN_MOVE_SPEED_MPS,ms);
    float lateralErrorM = moveStartLateralM - getLateralEncoderDistanceM();
    targVx=dir*cs; targVy=crossTrackCorrectionMps(lateralErrorM);
    float ye=angleDiffDeg(moveTargetYawDeg,imuYawDeg);
    float yeRad = ye * PI / 180.0f;
    float gyroZRadS = gyroZDegS * PI / 180.0f;
    moveYawIntegralRad = clampFloat(moveYawIntegralRad + (yeRad * PLANNER_DT), -MOVE_YAW_I_LIMIT, MOVE_YAW_I_LIMIT);
    float corr=clampFloat((KP_MOVE_YAW*yeRad) + (KI_MOVE_YAW*moveYawIntegralRad) - (KD_MOVE_YAW*gyroZRadS), -MAX_MOVE_YAW_CORR, MAX_MOVE_YAW_CORR);
    if (fabsf(ye) < MOVE_YAW_DEADBAND_DEG && fabsf(gyroZDegS) < MOVE_YAW_RATE_DEADBAND_DPS) {
      corr = 0.0f;
      moveYawIntegralRad = 0.0f;
    }
    targWz=MOVE_YAW_CONTROL_SIGN * corr; turnAngleErrorDeg=ye;
  }
  else if (mode == MODE_STRAFE_DISTANCE) {
    float now   = getLateralEncoderDistanceM();
    float moved = now - strafeStartLateralM;
    float rem   = strafeTargetDistanceM - moved;
    strafeDoneM=moved; strafeRemainingM=rem;
    if (fabsf(rem)<=DIST_TOLERANCE_M) { goalReached=true; stopRobotHard(); return; }
    float dir=signFloat(rem);
    float ms=clampFloat(spd*MAX_MPS,MIN_MOVE_SPEED_MPS,MAX_MPS);
    float cs=ms; if(fabsf(rem)<SLOWDOWN_DISTANCE_M) cs=clampFloat(ms*fabsf(rem)/SLOWDOWN_DISTANCE_M,MIN_MOVE_SPEED_MPS,ms);
    float forwardErrorM = strafeStartForwardM - getForwardEncoderDistanceM();
    targVx=crossTrackCorrectionMps(forwardErrorM); targVy=dir*cs;
    float ye=angleDiffDeg(strafeTargetYawDeg,imuYawDeg);
    float yeRad = ye * PI / 180.0f;
    float gyroZRadS = gyroZDegS * PI / 180.0f;
    moveYawIntegralRad = clampFloat(moveYawIntegralRad + (yeRad * PLANNER_DT), -MOVE_YAW_I_LIMIT, MOVE_YAW_I_LIMIT);
    float corr=clampFloat((KP_MOVE_YAW*yeRad) + (KI_MOVE_YAW*moveYawIntegralRad) - (KD_MOVE_YAW*gyroZRadS), -MAX_MOVE_YAW_CORR, MAX_MOVE_YAW_CORR);
    if (fabsf(ye) < MOVE_YAW_DEADBAND_DEG && fabsf(gyroZDegS) < MOVE_YAW_RATE_DEADBAND_DPS) {
      corr = 0.0f;
      moveYawIntegralRad = 0.0f;
    }
    targWz=MOVE_YAW_CONTROL_SIGN * corr; turnAngleErrorDeg=ye;
  }
  else if (mode == MODE_TURN_ANGLE) {
    float errDeg = angleDiffDeg(turnTargetYawDeg, imuYawDeg);
    float errRad = errDeg * PI / 180.0f;
    float gyroZRadS = gyroZDegS * PI / 180.0f;
    turnAngleErrorDeg = errDeg;

    // Turn PD controller:
    //   P term rotates toward the target angle.
    //   D term damps current gyro rotation to reduce overshoot.
    if (fabsf(errDeg) <= ANGLE_TOLERANCE_DEG && fabsf(gyroZDegS) < TURN_STOP_GYRO_DPS) {
      goalReached = true;
      stopRobotHard();
      return;
    }

    float wMax = clampFloat(spd * MAX_WZ, MIN_TURN_WZ, MAX_WZ);

    // Braking limit becomes smaller as the angle error gets smaller.
    float wBrake = sqrtf(2.0f * (MAX_WZ / DECEL_TIME_W) * fabsf(errRad));
    float wLimit = clampFloat(fminf(wMax, wBrake), TURN_FINE_MIN_WZ, wMax);

    bool turnStalled = false;
    if (fabsf(errDeg) > ANGLE_TOLERANCE_DEG && fabsf(gyroZDegS) < TURN_STALL_GYRO_DPS) {
      if (turnStallStartMs == 0) {
        turnStallStartMs = millis();
      }
      if ((uint32_t)(millis() - turnStallStartMs) >= TURN_STALL_TIME_MS) {
        turnStalled = true;
      }
    } else {
      turnStallStartMs = 0;
      turnStallBoostActive = false;
    }
    turnStallBoostActive = turnStalled;
    if (turnStallBoostActive) {
      wLimit = clampFloat(fmaxf(wLimit, TURN_STALL_BOOST_WZ), TURN_STALL_BOOST_WZ, wMax);
    }

    // Signed PD output. If the robot is still spinning fast toward the target,
    // the gyro term reduces command before overshoot happens.
    float wCmd = (KP_TURN * errRad) - (KD_TURN * gyroZRadS);
    wCmd = clampFloat(wCmd, -wLimit, wLimit);

    // Avoid motor stiction, but use a much smaller minimum near the target.
    float minAllowed = (fabsf(errDeg) <= TURN_FINE_ZONE_DEG) ? TURN_FINE_MIN_WZ : MIN_TURN_WZ;
    if (turnStallBoostActive) {
      minAllowed = fmaxf(minAllowed, TURN_STALL_BOOST_WZ);
    }
    if (fabsf(wCmd) < minAllowed && fabsf(errDeg) > ANGLE_TOLERANCE_DEG) {
      // If gyro is already high, allow PD braking/reversal instead of forcing
      // more speed toward the target.
      if (fabsf(gyroZDegS) < 8.0f || signFloat(wCmd) == signFloat(errRad)) {
        wCmd = signFloat(errRad) * minAllowed;
      }
    }

    targVx = 0;
    targVy = 0;
    targWz = TURN_YAW_CONTROL_SIGN * wCmd;
  }
}

// =====================================================
// String Helpers
// =====================================================
const char* modeName(RobotMode mode) {
  switch (mode) {
    case MODE_IDLE:            return "IDLE";
    case MODE_MOVE_DISTANCE:   return "MOVE_DISTANCE";
    case MODE_STRAFE_DISTANCE: return "STRAFE_DISTANCE";
    case MODE_TURN_ANGLE:      return "TURN_ANGLE";
    default:                   return "UNKNOWN";
  }
}

const char* pathStepName(int step) {
  switch (step) {
    case 0: return "INITIALIZING";
    case 1: return "FWD 2.77 m  (heading 0)";
    case 2: return "TURN +90 deg";
    case 3: return "FWD 1.55 m  (heading +90)";
    case 4: return "TURN +90 deg";
    case 5: return "FWD 2.77 m  (heading -180)";
    case 6: return "TURN +90 deg";
    case 7: return "FWD 1.55 m  (heading -90)";
    case 8: return "TURN +90 deg";
    case 9: return "COMPLETE";
    default: return "UNKNOWN";
  }
}

const char* obstacleStateName(uint8_t state) {
  switch (state) {
    case 0: return "CLEAR";
    case 1: return "DETECTED - STOPPING";
    case 2: return "WAITING - STATIC CHECK (5s)";
    case 3: return "DYNAMIC - CONTINUING";
    case 4: return "AVOIDING OBSTACLE";
    case 5: return "ALL SENSORS BLOCKED - WAITING";
    default: return "UNKNOWN";
  }
}

// =====================================================
// Path Helper Functions
// =====================================================
void startMoveDistanceM(float dist, float speedPct, float yaw) {
  stopRobotHard();
  goalReached=false;
  gSpd=clampFloat(speedPct,5.0f,100.0f)/100.0f;
  moveTargetDistanceM=dist;
  moveStartForwardM=getForwardEncoderDistanceM();
  moveStartLateralM=getLateralEncoderDistanceM();
  moveTargetYawDeg=yaw;
  moveDoneM=0; moveRemainingM=dist; turnAngleErrorDeg=0; moveYawIntegralRad=0.0f;
  resetAllPid();
  robotMode=MODE_MOVE_DISTANCE;
}

void startStrafeDistanceM(float dist, float speedPct, float yaw) {
  stopRobotHard();
  goalReached=false;
  gSpd=clampFloat(speedPct,5.0f,100.0f)/100.0f;
  strafeTargetDistanceM=dist;
  strafeStartLateralM=getLateralEncoderDistanceM();
  strafeStartForwardM=getForwardEncoderDistanceM();
  strafeTargetYawDeg=yaw;
  strafeDoneM=0; strafeRemainingM=dist; turnAngleErrorDeg=0; moveYawIntegralRad=0.0f;
  resetAllPid();
  robotMode=MODE_STRAFE_DISTANCE;
}

void startTurnRelativeDeg(float angle, float speedPct) {
  stopRobotHard();
  goalReached=false;
  gSpd=clampFloat(speedPct,5.0f,100.0f)/100.0f;
  turnTargetYawDeg=wrapAngle180(imuYawDeg+angle);
  turnAngleErrorDeg=angleDiffDeg(turnTargetYawDeg,imuYawDeg);
  moveDoneM=0; moveRemainingM=0; strafeDoneM=0; strafeRemainingM=0; moveYawIntegralRad=0.0f;
  turnStallStartMs=0; turnStallBoostActive=false;
  resetAllPid();
  robotMode=MODE_TURN_ANGLE;
}

void startTurnAbsoluteDeg(float targetYawDeg, float speedPct) {
  stopRobotHard();
  goalReached=false;
  gSpd=clampFloat(speedPct,5.0f,100.0f)/100.0f;
  turnTargetYawDeg=wrapAngle180(targetYawDeg);
  turnAngleErrorDeg=angleDiffDeg(turnTargetYawDeg,imuYawDeg);
  moveDoneM=0; moveRemainingM=0; strafeDoneM=0; strafeRemainingM=0; moveYawIntegralRad=0.0f;
  turnStallStartMs=0; turnStallBoostActive=false;
  resetAllPid();
  robotMode=MODE_TURN_ANGLE;
}


void sendSensorEspCommand(uint8_t cmd) {
  // Converted from legacy I2C command to ROS 2 command topic.
  if (cmd == SENSOR_CMD_CLEAR_DECISION) {
    publishClearDecisionCommand();
  } else if (cmd == SENSOR_CMD_ARM_VISION) {
    publishArmVisionCommand(true);
  } else {
    DebugSerial.printf("WARN: unknown sensor/mechanism command 0x%02X\n", cmd);
  }
}

void clearConsumedDecision() {
  userDecision = DECISION_NONE;
  sendSensorEspCommand(SENSOR_CMD_CLEAR_DECISION);
  // Also force the Pi vision gate OFF through the Sensor/Mechanism ESP relay.
  // This tells the Sensor/Mechanism ESP32 to stop accepting stale toy messages.
  publishArmVisionCommand(false);
}

void softwareResetZeroState() {
  stopRobotHard();
  resetTicks();
  resetAllPid();
  setSetpointsZero();

  odomX = 0.0f;
  odomY = 0.0f;
  moveDoneM = 0.0f;
  moveRemainingM = 0.0f;
  moveStartForwardM = 0.0f;
  moveStartLateralM = 0.0f;
  strafeDoneM = 0.0f;
  strafeRemainingM = 0.0f;
  strafeStartLateralM = 0.0f;
  strafeStartForwardM = 0.0f;
  manualHoldForwardM = 0.0f;
  manualHoldLateralM = 0.0f;
  currentSegmentDoneM = 0.0f;
  currentSegmentRemainingM = 0.0f;

  currentPathStep = 0;
  pathRunning = false;
  pathFinished = false;
  pathAborted = false;
  obstacleState = 0;
  lastStaticObstacleMask = 0;
  lastAvoidSide = 0;

  guiMode = GUI_MODE_AUTO;
  autoStartRequested = false;
  autoAbortRequested = false;
  manualDriveDir = MANUAL_DIR_STOP;
  manualDriveSpeed = 0;
  manualHoldYawDeg = imuYawDeg;
  manualHoldYawValid = false;
  manualHoldYawDir = MANUAL_DIR_STOP;

  imuReadOkCount = 0;
  imuReadFailCount = 0;
  lastImuFailMs = 0;

  clearConsumedDecision();
  imuCalibrationRequested = true;
}

const char* decisionName(uint8_t d) {
  switch (d) {
    case DECISION_TOY:      return "TOY";
    case DECISION_OBSTACLE: return "OBSTACLE";
    default:                return "NONE";
  }
}

const char* mechanismStateName(uint8_t s) {
  switch (s) {
    case MECH_STATE_IDLE:    return "IDLE";
    case MECH_STATE_RUNNING: return "RUNNING";
    case MECH_STATE_DONE:    return "DONE";
    case MECH_STATE_ERROR:   return "ERROR";
    default:                 return "UNKNOWN";
  }
}

uint8_t waitForPiToyOrAutoObstacleDecision() {
  DebugSerial.println("Static S2 obstacle confirmed: arming Pi vision window on merged sensor/mechanism ESP...");
  DebugSerial.println("Pi has up to 15 s to send TOY after the 5 s static check. Max wait is 20 s total.");

  // Remove any stale decision from a previous object, then arm the merged ESP
  // so it accepts Pi TOY messages only during this static-obstacle window.
  clearConsumedDecision();
  sendSensorEspCommand(SENSOR_CMD_ARM_VISION);

  uint32_t start = millis();
  uint32_t lastArmBurstMs = 0;

  while ((uint32_t)(millis() - start) < PI_TOY_WAIT_MS) {
    if (pathAborted) return DECISION_NONE;

    // Keep re-publishing arm during the decision window.
    // This protects against a lost best-effort packet and makes /vision/arm visible to the Pi.
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastArmBurstMs) >= 400u) {
      lastArmBurstMs = nowMs;
      publishArmVisionCommand(true);
    }

    uint8_t d = userDecision;
    if (d == DECISION_TOY) {
      DebugSerial.println("Pi vision decision received from merged ESP: TOY.");
      return DECISION_TOY;
    }

    // Manual OBSTACLE is no longer required, but keep compatibility if the
    // existing HTTP/Serial command is used during testing.
    if (d == DECISION_OBSTACLE) {
      DebugSerial.println("OBSTACLE decision received from merged ESP.");
      return DECISION_OBSTACLE;
    }

    stopRobotHard();
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  DebugSerial.println("No Pi TOY decision arrived in time -> auto OBSTACLE avoidance.");
  clearConsumedDecision();   // also disarms the Pi vision window on the merged ESP
  return DECISION_OBSTACLE;
}

bool waitForMechanismToFinish() {
  DebugSerial.println("TOY decision: waiting until mechanism task and homing sequence finish...");

  // Ignore stale IDLE/DONE from startup homing. After the 12 cm approach,
  // the robot may continue only after the mechanism reports RUNNING and then
  // returns to DONE or IDLE.
  bool mechanismStarted = false;
  uint32_t lastMechanismRunCmdMs = 0;

  while (true) {
    if (pathAborted) return false;

    uint32_t nowMs = millis();

    uint8_t ms = mechanismState;

    if (ms == MECH_STATE_RUNNING) {
      mechanismStarted = true;
    }

    if (!mechanismStarted &&
        (lastMechanismRunCmdMs == 0 ||
         (uint32_t)(nowMs - lastMechanismRunCmdMs) >= 400u)) {
      lastMechanismRunCmdMs = nowMs;
      publishMechanismRunCommand();
    }

    if (mechanismStarted && (ms == MECH_STATE_DONE || ms == MECH_STATE_IDLE)) {
      DebugSerial.printf("Mechanism task/homing finished/state=%s. Continuing path.\n", mechanismStateName(ms));
      clearConsumedDecision();
      return true;
    }

    if (ms == MECH_STATE_ERROR) {
      DebugSerial.println("ERROR: mechanism ESP reported MECH_STATE_ERROR. Path aborted for safety.");
      pathAborted = true;
      return false;
    }

    stopRobotHard();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

bool waitMotionDoneNoObstacle() {
  while (true) {
    if (goalReached && robotMode==MODE_IDLE) return true;
    if (pathAborted) return false;
    if (!isImuLiveNow()) {
      pathAborted = true;
      stopRobotHard();
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =====================================================
// Avoidance Decision + Obstacle Wait Helpers
// =====================================================

// Full avoidance direction decision.
// Return values after requested reversal:
//   +1 = avoid LEFT
//   -1 = avoid RIGHT
//    0 = no avoidance direction / not a front obstacle case
int8_t decideAvoidSideFromMask(uint8_t mask) {
  bool s1 = (mask & SENSOR_1_RIGHT_MASK)  != 0;
  bool s2 = (mask & SENSOR_2_MIDDLE_MASK) != 0;
  bool s3 = (mask & SENSOR_3_LEFT_MASK)   != 0;

  // All-three block is handled before calling this function.
  if (s1 && s2 && s3) return 0;

  // Reversed avoidance task:
  // S1 + S2 now avoids RIGHT instead of LEFT.
  if (s1 && s2 && !s3) return -1;

  // S2 + S3 now avoids LEFT instead of RIGHT.
  if (!s1 && s2 && s3) return +1;

  // S2 only now avoids RIGHT by default instead of LEFT.
  if (!s1 && s2 && !s3) return -1;

  // S1 only, S3 only, S1 + S3 only:
  // side obstacle cases, not avoidance cases.
  return 0;
}

// 5-second static/dynamic check after S2 is involved.
bool waitForStaticCheck() {
  obstacleState = 2;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < STATIC_CHECK_TIME_MS) {
    if (pathAborted) return false;
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  return true;
}

// Used when S1 and/or S3 detects without S2.
// Robot waits until side sensors clear, then waits 3 stable seconds,
// then continues the same segment.
bool waitUntilSideSensorsClearThenContinue() {
  obstacleState = 2;

  // Wait until S1 and S3 are clear.
  while (true) {
    if (pathAborted) return false;

    uint8_t maskNow = sensorMask & 0x07;

    // If S2 appears while waiting, return to main obstacle logic.
    if (maskNow & SENSOR_2_MIDDLE_MASK) {
      return true;
    }

    bool sideSensorsClear =
      ((maskNow & SENSOR_1_RIGHT_MASK) == 0) &&
      ((maskNow & SENSOR_3_LEFT_MASK)  == 0);

    if (sideSensorsClear) break;

    stopRobotHard();
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // After S1/S3 are clear, wait 3 stable seconds.
  uint32_t clearStart = millis();

  while ((uint32_t)(millis() - clearStart) < SIDE_CLEAR_CONTINUE_DELAY_MS) {
    if (pathAborted) return false;

    uint8_t maskNow = sensorMask & 0x07;

    // If S2 appears during the clear delay, return to main obstacle logic.
    if (maskNow & SENSOR_2_MIDDLE_MASK) {
      return true;
    }

    // If S1 or S3 detects again, restart the 3-second stable timer.
    if ((maskNow & SENSOR_1_RIGHT_MASK) || (maskNow & SENSOR_3_LEFT_MASK)) {
      clearStart = millis();
    }

    stopRobotHard();
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  obstacleState = 0;
  return true;
}

// Used when S1 + S2 + S3 are all detecting.
// Robot waits indefinitely while all three remain active.
// Once the mask changes, the new mask is returned and handled normally.
uint8_t waitWhileAllSensorsBlocked() {
  obstacleState = 5;

  const uint8_t ALL_SENSORS_MASK =
    SENSOR_1_RIGHT_MASK | SENSOR_2_MIDDLE_MASK | SENSOR_3_LEFT_MASK;

  while (true) {
    if (pathAborted) return 0xFF;

    uint8_t maskNow = sensorMask & 0x07;

    if (maskNow != ALL_SENSORS_MASK) {
      return maskNow;
    }

    stopRobotHard();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

bool performAvoidance(int8_t side, float avoidYawDeg) {
  if (side == 0) return false;

  obstacleState = 4;
  lastAvoidSide = side;

  float originalYawDeg = wrapAngle180(avoidYawDeg);
  float firstSideYawDeg = wrapAngle180(originalYawDeg - (90.0f * (float)side));
  float secondSideYawDeg = wrapAngle180(originalYawDeg + (90.0f * (float)side));

  DebugSerial.printf("Turn-based avoidance: side=%+d originalYaw=%.1f firstYaw=%.1f secondYaw=%.1f sideDist=%.3f forwardDist=%.3f\n",
                     side,
                     originalYawDeg,
                     firstSideYawDeg,
                     secondSideYawDeg,
                     AVOID_SIDE_DISTANCE_M,
                     AVOID_FORWARD_DISTANCE_M);

  // Full avoidance logic:
  // 1) rotate 90 deg using the reversed avoidance angle
  //    (S2-only currently selects side=-1, so this is rotate +90 as requested)
  // 2) drive forward by the same old strafe distance, 0.35 m
  // 3) rotate back to the original path heading
  // 4) drive forward by the old forward bypass distance, 1.20 m
  // 5) rotate -90 deg for the normal S2-only case
  // 6) drive forward 0.35 m to return to the original lane
  // 7) rotate back to the original path heading and continue the remaining path
  startTurnAbsoluteDeg(firstSideYawDeg, TURN_SPEED_PERCENT);
  if (!waitMotionDoneNoObstacle()) return false;

  startMoveDistanceM(AVOID_SIDE_DISTANCE_M, AVOID_SPEED_PERCENT, firstSideYawDeg);
  if (!waitMotionDoneNoObstacle()) return false;

  startTurnAbsoluteDeg(originalYawDeg, TURN_SPEED_PERCENT);
  if (!waitMotionDoneNoObstacle()) return false;

  startMoveDistanceM(AVOID_FORWARD_DISTANCE_M, AVOID_SPEED_PERCENT, originalYawDeg);
  if (!waitMotionDoneNoObstacle()) return false;

  startTurnAbsoluteDeg(secondSideYawDeg, TURN_SPEED_PERCENT);
  if (!waitMotionDoneNoObstacle()) return false;

  startMoveDistanceM(AVOID_SIDE_DISTANCE_M, AVOID_SPEED_PERCENT, secondSideYawDeg);
  if (!waitMotionDoneNoObstacle()) return false;

  startTurnAbsoluteDeg(originalYawDeg, TURN_SPEED_PERCENT);
  if (!waitMotionDoneNoObstacle()) return false;

  obstacleState = 0;
  return true;
}

// =====================================================
// Forward Segment with Obstacle Avoidance
// =====================================================
bool runForwardSegmentWithAvoidance(float segDist, float yaw) {
  float completedM = 0.0f;

  while (completedM < segDist - DIST_TOLERANCE_M) {
    if (!isImuLiveNow()) {
      pathAborted = true;
      stopRobotHard();
      return false;
    }

    float cmdDist = segDist - completedM;

    currentSegmentDoneM = completedM;
    currentSegmentRemainingM = cmdDist;

    startMoveDistanceM(cmdDist, PATH_SPEED_PERCENT, yaw);

    while (true) {
      currentSegmentDoneM = completedM + fabsf(moveDoneM);
      currentSegmentRemainingM = segDist - currentSegmentDoneM;

      if (!isImuLiveNow()) {
        pathAborted = true;
        stopRobotHard();
        return false;
      }

      // Sub-segment finished cleanly
      if (goalReached && robotMode == MODE_IDLE) {
        completedM = segDist;
        currentSegmentDoneM = completedM;
        currentSegmentRemainingM = 0.0f;
        obstacleState = 0;
        break;
      }

      uint8_t maskNow = sensorMask & 0x07;

      // ANY sensor detection stops the robot.
      if (maskNow != 0) {
        obstacleState = 1;

        float doneBeforeStop = fabsf(moveDoneM);
        stopRobotHard();

        // Re-evaluate masks until the situation becomes one of:
        //   clear, side-only, or front/S2 obstacle.
        while (true) {
          if (pathAborted) {
            stopRobotHard();
            return false;
          }

          maskNow = sensorMask & 0x07;

          const uint8_t ALL_SENSORS_MASK =
            SENSOR_1_RIGHT_MASK | SENSOR_2_MIDDLE_MASK | SENSOR_3_LEFT_MASK;

          // S1 + S2 + S3:
          // wait indefinitely until at least one sensor clears,
          // then re-evaluate the new mask.
          if (maskNow == ALL_SENSORS_MASK) {
            lastStaticObstacleMask = maskNow;
            maskNow = waitWhileAllSensorsBlocked();

            if (maskNow == 0xFF) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            // Continue the inner re-evaluation with the new mask.
            continue;
          }

          // All sensors clear:
          // wait 3 stable seconds, then continue same segment.
          if (maskNow == 0) {
            obstacleState = 3;

            uint32_t clearStart = millis();
            bool frontAppeared = false;

            while ((uint32_t)(millis() - clearStart) < SIDE_CLEAR_CONTINUE_DELAY_MS) {
              if (pathAborted) {
                stopRobotHard();
                return false;
              }

              uint8_t stableMask = sensorMask & 0x07;

              if (stableMask == ALL_SENSORS_MASK) {
                frontAppeared = true;
                break;
              }

              if (stableMask & SENSOR_2_MIDDLE_MASK) {
                frontAppeared = true;
                break;
              }

              if (stableMask != 0) {
                clearStart = millis();
              }

              stopRobotHard();
              vTaskDelay(pdMS_TO_TICKS(50));
            }

            if (frontAppeared) {
              continue;
            }

            completedM += doneBeforeStop;
            currentSegmentDoneM = completedM;
            currentSegmentRemainingM = segDist - completedM;
            obstacleState = 0;
            break;
          }

          // S1/S3 side-only case, without S2:
          // wait until side sensors clear, then wait 3 stable seconds,
          // then continue same segment.
          if ((maskNow & SENSOR_2_MIDDLE_MASK) == 0) {
            lastStaticObstacleMask = maskNow;

            if (!waitUntilSideSensorsClearThenContinue()) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            // If S2 appeared while the side-wait helper was running,
            // re-evaluate instead of continuing.
            uint8_t afterSideWaitMask = sensorMask & 0x07;
            if (afterSideWaitMask & SENSOR_2_MIDDLE_MASK) {
              continue;
            }

            completedM += doneBeforeStop;
            currentSegmentDoneM = completedM;
            currentSegmentRemainingM = segDist - completedM;
            obstacleState = 0;
            break;
          }

          // S2 is involved:
          // enter the static/dynamic check loop.
          if (!waitForStaticCheck()) return false;

          uint8_t staticMask = sensorMask & 0x07;

          // If all three are still active after the 5-second check,
          // wait indefinitely until any sensor clears,
          // then re-evaluate the new sensor mask.
          if (staticMask == ALL_SENSORS_MASK) {
            lastStaticObstacleMask = staticMask;
            staticMask = waitWhileAllSensorsBlocked();

            if (staticMask == 0xFF) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            // Re-evaluate using the changed mask.
            continue;
          }

          // Dynamic obstacle:
          // If all sensors are clear after the 5-second check,
          // continue from stopped position.
          if (staticMask == 0) {
            obstacleState = 3;

            completedM += doneBeforeStop;
            currentSegmentDoneM = completedM;
            currentSegmentRemainingM = segDist - completedM;

            vTaskDelay(pdMS_TO_TICKS(200));
            obstacleState = 0;
            break;
          }

          // If after the 5-second check S2 is no longer active,
          // but S1/S3 are still active, treat it as side-only wait.
          if ((staticMask & SENSOR_2_MIDDLE_MASK) == 0) {
            lastStaticObstacleMask = staticMask;

            if (!waitUntilSideSensorsClearThenContinue()) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            uint8_t afterSideWaitMask = sensorMask & 0x07;
            if (afterSideWaitMask & SENSOR_2_MIDDLE_MASK) {
              continue;
            }

            completedM += doneBeforeStop;
            currentSegmentDoneM = completedM;
            currentSegmentRemainingM = segDist - completedM;
            obstacleState = 0;
            break;
          }

          // Static front obstacle:
          // S2 is still active. Stop and wait for operator decision from
          // the merged sensor/mechanism ESP micro-ROS node:
          //   TOY      -> approach 12 cm, wait for mechanism task/homing, then continue
          //   OBSTACLE -> receiver performs avoidance, then path continues
          lastStaticObstacleMask = staticMask;

          uint8_t decision = waitForPiToyOrAutoObstacleDecision();
          if (decision == DECISION_NONE) {
            pathAborted = true;
            stopRobotHard();
            return false;
          }

          completedM += doneBeforeStop;

          if (decision == DECISION_TOY) {
            DebugSerial.println("TOY confirmed: moving forward 12 cm before mechanism task.");
            startMoveDistanceM(TOY_APPROACH_DISTANCE_M, TOY_APPROACH_SPEED_PERCENT, yaw);
            if (!waitMotionDoneNoObstacle()) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            completedM += fabsf(moveDoneM);
            currentSegmentDoneM = completedM;
            currentSegmentRemainingM = fmaxf(0.0f, segDist - completedM);
            stopRobotHard();

            publishMechanismRunCommand();
            if (!waitForMechanismToFinish()) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            // Toy was collected after the 12 cm approach. The robot stayed hard-stopped
            // during the mechanism task/homing wait; now resume normal path sensing.
            currentSegmentDoneM = completedM;
            currentSegmentRemainingM = fmaxf(0.0f, segDist - completedM);
            obstacleState = 0;
            break;
          }

          if (decision == DECISION_OBSTACLE) {
            int8_t side = decideAvoidSideFromMask(staticMask);

            // side == 0 here should only happen for an unexpected case.
            // Do not abort immediately; keep the robot stopped and re-evaluate.
            if (side == 0) {
              vTaskDelay(pdMS_TO_TICKS(50));
              continue;
            }

            clearConsumedDecision();

            if (!performAvoidance(side, yaw)) {
              pathAborted = true;
              stopRobotHard();
              return false;
            }

            completedM += AVOID_FORWARD_DISTANCE_M;
          }

          currentSegmentDoneM = completedM;
          currentSegmentRemainingM = segDist - completedM;

          if (completedM >= segDist - DIST_TOLERANCE_M) {
            stopRobotHard();
            obstacleState = 0;
            return true;
          }

          obstacleState = 0;
          break;
        }

        break;
      }

      if (pathAborted) {
        stopRobotHard();
        return false;
      }

      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }

  return true;
}

bool runTurnToYaw(float targetYawDeg) {
  startTurnAbsoluteDeg(targetYawDeg, TURN_SPEED_PERCENT);
  return waitMotionDoneNoObstacle();
}

// =====================================================
// RTOS Tasks
// =====================================================

// Sensor data now arrives through micro-ROS subscriptions.
// ESP-to-ESP I2C polling was removed; Wire is used only for the local MPU6050 IMU.
void taskPredefinedPath(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(1500));

  // Safety gate: do not start moving until the Sensor/Mechanism ESP32
  // has actually published through ROS. If it is disconnected, the robot stays stopped.
  while (rosSensorMaskRxCount == 0 || rosMechanismRxCount == 0) {
    stopRobotHard();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Initial mechanism homing gate:
  // Sensor/Mechanism ESP32 must complete Stepper 1 homing first, then Stepper 2 homing.
  // Path starts only when mechanism state is safe: IDLE or DONE.
  // If homing reports ERROR, the robot must not move.
  while (true) {
    stopRobotHard();

    uint8_t ms = mechanismState;
    if (ms == MECH_STATE_ERROR) {
      DebugSerial.println("ERROR: initial mechanism homing failed. Robot path will not start.");
      pathRunning = false;
      pathFinished = false;
      pathAborted = true;
      vTaskDelete(NULL);
      return;
    }

    if (ms == MECH_STATE_IDLE || ms == MECH_STATE_DONE) {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

waitForGuiAutoStart:
  // AUTO is the default GUI mode. The first run starts after GUI AUTO_START.
  // After a clean mission finish, AUTO mode restarts the path by itself.
  // GUI AUTO_ABORT still stops the current path and prevents automatic restart.
  pathRunning=false;
  if (!pathFinished && !pathAborted) currentPathStep=0;
  while (true) {
    if (guiMode == GUI_MODE_MANUAL) {
      autoStartRequested = false;
      autoAbortRequested = false;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    stopRobotHard();

    bool restartAfterCleanFinish = pathFinished && !pathAborted && !autoAbortRequested;

    if (guiMode == GUI_MODE_AUTO && (autoStartRequested || restartAfterCleanFinish)) {
      if (!isImuLiveNow()) {
        autoStartRequested = false;
        autoAbortRequested = false;
        pathAborted = true;
        pathFinished = false;
        currentPathStep = 0;
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      autoStartRequested = false;
      autoAbortRequested = false;
      pathAborted = false;
      pathFinished = false;
      currentPathStep = 0;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  pathRunning=true; pathFinished=false; pathAborted=false;
  obstacleState=0; lastStaticObstacleMask=0; lastAvoidSide=0;

  currentPathStep=1;
  if (!runForwardSegmentWithAvoidance(2.77f,   0.0f)) goto pathStop;
  currentPathStep=2;
  if (!runTurnToYaw(90.0f))                          goto pathStop;
  currentPathStep=3;
  if (!runForwardSegmentWithAvoidance(1.55f,  90.0f)) goto pathStop;
  currentPathStep=4;
  if (!runTurnToYaw(-180.0f))                        goto pathStop;
  currentPathStep=5;
  if (!runForwardSegmentWithAvoidance(2.77f,-180.0f)) goto pathStop;
  currentPathStep=6;
  if (!runTurnToYaw(-90.0f))                         goto pathStop;
  currentPathStep=7;
  if (!runForwardSegmentWithAvoidance(1.55f, -90.0f)) goto pathStop;
  currentPathStep=8;
  if (!runTurnToYaw(0.0f))                           goto pathStop;

  currentPathStep=9;
  stopRobotHard(); pathFinished=true; pathRunning=false; obstacleState=0;
  autoStartRequested = false;
  vTaskDelay(pdMS_TO_TICKS(AUTO_RESTART_AFTER_FINISH_MS));
  goto waitForGuiAutoStart;

pathStop:
  stopRobotHard(); pathRunning=false;
  if (!pathFinished) pathAborted=true;
  autoStartRequested = false;
  goto waitForGuiAutoStart;
}

void taskIMU(void* pv) {
  lastImuMicros=micros();
  while (true) {
    if (imuCalibrationRequested) { imuCalibrationRequested=false; calibrateMPU(IMU_CAL_SAMPLES); }
    updateIMU();
    vTaskDelay(pdMS_TO_TICKS(IMU_TASK_MS));
  }
}

void taskPlanner(void* pv) {
  TickType_t t=xTaskGetTickCount();
  while (true) { updateMotionPlanner(); vTaskDelayUntil(&t, pdMS_TO_TICKS(1000/PLANNER_HZ)); }
}

void taskProfile(void* pv) {
  TickType_t t=xTaskGetTickCount();
  while (true) {
    const float av=(MAX_MPS/ACCEL_TIME)  *PID_DT;
    const float dv=(MAX_MPS/DECEL_TIME)  *PID_DT;
    const float aw=(MAX_WZ/ACCEL_TIME_W) *PID_DT;
    const float dw=(MAX_WZ/DECEL_TIME_W) *PID_DT;
    bool decX=(profVx>0&&targVx<profVx)||(profVx<0&&targVx>profVx);
    bool decY=(profVy>0&&targVy<profVy)||(profVy<0&&targVy>profVy);
    bool decW=(profWz>0&&targWz<profWz)||(profWz<0&&targWz>profWz);
    profVx=stepToward(profVx,targVx,decX?dv:av);
    profVy=stepToward(profVy,targVy,decY?dv:av);
    profWz=stepToward(profWz,targWz,decW?dw:aw);
    mecanumIK(profVx,profVy,profWz);
    vTaskDelayUntil(&t, pdMS_TO_TICKS(1000/PID_HZ));
  }
}

void taskWheelPID(void* pv) {
  int32_t prev[4]={0,0,0,0};
  uint32_t seenEncoderResetGeneration = encoderResetGeneration;
  TickType_t t=xTaskGetTickCount();
  while (true) {
    int32_t snap[4]; readTicks(snap);
    if (seenEncoderResetGeneration != encoderResetGeneration) {
      seenEncoderResetGeneration = encoderResetGeneration;
      for (int i=0; i<4; i++) {
        prev[i]=snap[i];
        measuredRad[i]=0.0f;
        resetPid(i);
        motorWrite(i,0);
      }
      vTaskDelayUntil(&t, pdMS_TO_TICKS(1000/PID_HZ));
      continue;
    }

    bool zeroDriveCommand =
      fabsf(targVx) < 0.001f && fabsf(targVy) < 0.001f && fabsf(targWz) < 0.001f &&
      fabsf(profVx) < 0.001f && fabsf(profVy) < 0.001f && fabsf(profWz) < 0.001f;

    for (int i=0; i<4; i++) {
      float meas=((float)(snap[i]-prev[i])/CPR[i])*(2.0f*PI)/PID_DT;
      prev[i]=snap[i]; measuredRad[i]=meas;
      float sp = setpointRad[i];
      if (zeroDriveCommand && fabsf(sp)<0.01f) {
        measuredRad[i]=0.0f;
        resetPid(i);
        resetWheelRecovery(i);
        motorWrite(i,0);
        continue;
      }
      if (fabsf(sp)<0.01f && fabsf(meas)<1.0f) {
        resetPid(i);
        resetWheelRecovery(i);
        motorWrite(i,0);
        continue;
      }
      updateWheelResponseRecovery(i, sp, meas);
      int16_t pidPwm = (int16_t)pidCompute(i, sp, meas);
      motorWrite(i, applyWheelRecoveryAssist(i, pidPwm, sp));
    }
    vTaskDelayUntil(&t, pdMS_TO_TICKS(1000/PID_HZ));
  }
}

void taskOdom(void* pv) {
  int32_t prev[4]={0,0,0,0};
  uint32_t seenEncoderResetGeneration = encoderResetGeneration;
  TickType_t t=xTaskGetTickCount();
  while (true) {
    int32_t snap[4]; readTicks(snap);
    if (seenEncoderResetGeneration != encoderResetGeneration) {
      seenEncoderResetGeneration = encoderResetGeneration;
      for (int i=0; i<4; i++) prev[i]=snap[i];
      vTaskDelayUntil(&t, pdMS_TO_TICKS(1000/PID_HZ));
      continue;
    }
    float w[4];
    for (int i=0; i<4; i++) { w[i]=((float)(snap[i]-prev[i])/CPR[i])*(2.0f*PI)/PID_DT; prev[i]=snap[i]; }
    float vxB=WHEEL_R/4.0f*(w[W_FL]+w[W_FR]+w[W_BL]+w[W_BR]);
    float vyB=WHEEL_R/4.0f*(-w[W_FL]+w[W_FR]+w[W_BL]-w[W_BR]);
    float th=imuYawDeg*PI/180.0f;
    odomX+=(vxB*cosf(th)-vyB*sinf(th))*PID_DT;
    odomY+=(vxB*sinf(th)+vyB*cosf(th))*PID_DT;
    vTaskDelayUntil(&t, pdMS_TO_TICKS(1000/PID_HZ));
  }
}

void taskPrint(void* pv) {
  TickType_t t=xTaskGetTickCount();
  while (true) {
    int32_t snap[4]; readTicks(snap);
    DebugSerial.println();
    DebugSerial.println("========= ESP32 MECANUM PATH ROBOT =========");
    DebugSerial.printf("Mode: %s | Speed: %.0f%% | Goal: %s\n",
                  modeName(robotMode),gSpd*100.0f,goalReached?"YES":"NO");
    DebugSerial.printf("Path:  step=%d (%s)\n",
                  currentPathStep,pathStepName(currentPathStep));
    DebugSerial.printf("       running=%s | finished=%s | aborted=%s\n",
                  pathRunning?"YES":"NO",pathFinished?"YES":"NO",pathAborted?"YES":"NO");
    DebugSerial.printf("Obstacle: state=%d [%s]\n",
                  obstacleState,obstacleStateName(obstacleState));
    DebugSerial.printf("          mask=0x%02X | lastStaticMask=0x%02X | avoidSide=%+d\n",
                  sensorMask,lastStaticObstacleMask,lastAvoidSide);
    DebugSerial.printf("ROS sensor rx: mask=%lu decision=%lu mech=%lu | clearTx=%lu armTx=%lu\n",
                  (unsigned long)rosSensorMaskRxCount,
                  (unsigned long)rosDecisionRxCount,
                  (unsigned long)rosMechanismRxCount,
                  (unsigned long)rosClearTxCount,
                  (unsigned long)rosArmTxCount);
    DebugSerial.printf("ROS pid status tx=%lu | command queue drops=%lu\n",
                  (unsigned long)rosPidStatusTxCount,
                  (unsigned long)rosCommandQueueDropCount);
    DebugSerial.printf("Segment: done=%.3f m | remaining=%.3f m\n",
                  currentSegmentDoneM,currentSegmentRemainingM);
    DebugSerial.printf("IMU: yaw=%.2f | pitch=%.2f | roll=%.2f | gz=%.3f deg/s\n",
                  imuYawDeg,imuPitchDeg,imuRollDeg,gyroZDegS);
    DebugSerial.printf("Odom: x=%.3f m | y=%.3f m | headErr=%.2f deg\n",
                  odomX,odomY,turnAngleErrorDeg);
    DebugSerial.printf("Target: Vx=%.3f Vy=%.3f Wz=%.3f\n",targVx,targVy,targWz);
    DebugSerial.printf("PWM: FL=%d FR=%d BL=%d BR=%d\n",
                  motorPwmOut[W_FL],motorPwmOut[W_FR],
                  motorPwmOut[W_BL],motorPwmOut[W_BR]);
    DebugSerial.printf("Recovery PWM: FL=%d FR=%d BL=%d BR=%d\n",
                  wheelRecoveryBoostPwm[W_FL],wheelRecoveryBoostPwm[W_FR],
                  wheelRecoveryBoostPwm[W_BL],wheelRecoveryBoostPwm[W_BR]);
    DebugSerial.println("Transport: USB serial micro-ROS @ 115200 baud");
    DebugSerial.println("=============================================");
    vTaskDelayUntil(&t, pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(MICRO_ROS_SERIAL_BAUD);  // Must match the micro-ROS agent -b value: 115200.
  delay(2000);
  DebugSerial.println();
  DebugSerial.println("=== ESP32 Mecanum PID + Encoders + MPU6050 + micro-ROS ===");
  DebugSerial.printf("Path speed: %.0f%% | Turn speed: %.0f%% | Avoid speed: %.0f%%\n", PATH_SPEED_PERCENT, TURN_SPEED_PERCENT, AVOID_SPEED_PERCENT);

  const uint8_t dirPins[]={FL_DN1,FL_DN2,FR_DN1,FR_DN2,BL_DN1,BL_DN2,BR_DN1,BR_DN2};
  for (uint8_t i=0; i<sizeof(dirPins); i++) { pinMode(dirPins[i],OUTPUT); digitalWrite(dirPins[i],LOW); }

  setupPwmPin(FL_PWM_PIN,FL_PWM_CH);
  setupPwmPin(FR_PWM_PIN,FR_PWM_CH);
  setupPwmPin(BL_PWM_PIN,BL_PWM_CH);
  setupPwmPin(BR_PWM_PIN,BR_PWM_CH);
  stopAllMotors();

  pinMode(FL_A,INPUT); pinMode(FL_B,INPUT);
  pinMode(FR_A,INPUT); pinMode(FR_B,INPUT);
  pinMode(BL_A,INPUT_PULLUP); pinMode(BL_B,INPUT_PULLUP);
  pinMode(BR_A,INPUT_PULLUP); pinMode(BR_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_A),isr_fl_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_B),isr_fl_b,CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_A),isr_fr_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_B),isr_fr_b,CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_A),isr_bl_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_B),isr_bl_b,CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_A),isr_br_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_B),isr_br_b,CHANGE);
  resetTicks();

  // Local I2C bus for MPU6050 only. ESP-to-ESP communication is USB serial micro-ROS through the Pi.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(IMU_I2C_CLOCK_HZ);

  i2cMutex=xSemaphoreCreateMutex();
  if (!i2cMutex) { DebugSerial.println("FATAL: I2C mutex failed."); while(true) delay(1000); }

  rosCommandQueue = xQueueCreate(24, sizeof(RosCommand));
  if (!rosCommandQueue) { DebugSerial.println("FATAL: ROS command queue failed."); while(true) delay(1000); }

  if (initMPU()) {
    DebugSerial.println("MPU6050 initialized.");
    calibrateMPU(IMU_CAL_SAMPLES);
  } else {
    DebugSerial.println("MPU6050 FAILED."); imuReady=false;
  }

  DebugSerial.println("Starting USB serial micro-ROS transport to Raspberry Pi agent...");
  if (!setupMicroRosNode()) {
    DebugSerial.println("FATAL: micro-ROS setup failed. Check serial agent, device path, and baud rate.");
    while (true) delay(1000);
  }

  // Core 0 = communication + high-level path/obstacle logic
  xTaskCreatePinnedToCore(taskMicroRos,       "microros",8192, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(taskPredefinedPath, "path",    6144, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskPrint,          "print",   4096, NULL, 1, NULL, 0);

  // Core 1 = real-time movement/control
  xTaskCreatePinnedToCore(taskIMU,            "imu",     4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskPlanner,        "planner", 3072, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskProfile,        "profile", 3072, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskOdom,           "odom",    3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskWheelPID,       "pid",     4096, NULL, 4, NULL, 1);

  DebugSerial.println("All tasks started. AUTO is default; path waits for GUI START AUTO PATH.");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
