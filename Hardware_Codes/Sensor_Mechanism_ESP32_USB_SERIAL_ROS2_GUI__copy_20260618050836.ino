// ============================================================
// Merged Sensor + Mechanism ESP32
// micro-ROS NODE for sensor debounce + mechanism + Pi vision gatekeeper
//
// CORE SPLIT:
//   Core 0: micro-ROS communication + status printing
//   Core 1: sensor debounce task + mechanism sequence task
//
// ROS 2 topics:
//   Publishes:
//     /robot/sensors/mask       std_msgs/UInt8
//     /robot/sensors/decision   std_msgs/UInt8
//     /robot/mechanism/state    std_msgs/UInt8  0=IDLE, 1=RUNNING, 2=DONE, 3=ERROR
//     /vision/arm               std_msgs/Bool
//   Subscribes:
//     /robot/cmd/clear_decision std_msgs/Bool
//     /robot/cmd/arm_vision     std_msgs/Bool
//     /vision/toy_detected      std_msgs/Float32
//     /robot/cmd/mechanism      std_msgs/UInt8  1=run pickup sequence
//
// IMPORTANT PIN-CONFLICT FIX:
//   The old mechanism code used GPIO32 for Stepper 1 MS1.
//   The sensor code uses GPIO32 for S1 RIGHT.
//   Because both programs now run on ONE ESP32, software MS pins are disabled.
//   Set Stepper 1 driver microstepping physically using jumpers/DIP switches.
//   Required assumed mode: 1/8 microstepping.
//
// DEPENDENCIES: ESP32Servo library + micro_ros_arduino library + Adafruit_NeoPixel library
// ============================================================

#include <Arduino.h>
// WiFi transport removed: USB serial micro-ROS is used.
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

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

// ============================================================
// USB Serial — micro-ROS transport
// ============================================================
#define MICRO_ROS_SERIAL_BAUD 115200

// ============================================================
// Decision/state constants. Transport is ROS 2 topics over USB serial micro-ROS.
// ============================================================
#define DECISION_NONE      0u
#define DECISION_TOY       1u
#define DECISION_OBSTACLE  2u

#define MECH_STATE_IDLE     0u
#define MECH_STATE_RUNNING  1u
#define MECH_STATE_DONE     2u
#define MECH_STATE_ERROR    3u

#define MECH_CMD_STOP          0u
#define MECH_CMD_RUN           1u
#define MECH_CMD_SERVO_OPEN    2u
#define MECH_CMD_SERVO_CLOSE   3u
#define MECH_CMD_STEP2_HOME    4u
#define MECH_CMD_STEP2_FWD     5u
#define MECH_CMD_STEP1_FWD     6u
#define MECH_CMD_STEP1_RETURN  7u
#define MECH_CMD_HOME_ALL      8u

#define VISION_ARM_MS           15000u
#define VISION_MIN_CONF_PERMILLE 700u
#define PI_NO_TOY_CONFIRM_MS    14500u

// ============================================================
// Sensors: active-LOW
// ============================================================
#define SENSOR_1_RIGHT_PIN    32
#define SENSOR_2_MIDDLE_PIN   34
#define SENSOR_3_LEFT_PIN     35

#define SENSOR_1_RIGHT_MASK   0x01
#define SENSOR_2_MIDDLE_MASK  0x02
#define SENSOR_3_LEFT_MASK    0x04

#define SENSOR_SAMPLE_MS          2
#define S2_DETECT_DEBOUNCE_MS     8
#define SIDE_DETECT_DEBOUNCE_MS   15
#define CLEAR_DEBOUNCE_MS         35
#define ROS_PUBLISH_PERIOD_MS     20

// ============================================================
// Mechanism pins
// ============================================================
#define STEP1_DIR_PIN    16
#define STEP1_STEP_PIN   17

#define STEP2_STEP_PIN   18
#define STEP2_DIR_PIN    19

#define SERVO_PIN        25
#define LIMIT_PIN        27     // Stepper 2 active-LOW limit switch
#define STEP1_LIMIT_PIN  33     // Stepper 1 active-LOW limit switch; same logic as sensors

// NeoPixel / WS2812 LED signal pin.
// GPIO18 from the standalone LED sketch is NOT used here because GPIO18 is STEP2_STEP_PIN.
#define LED_PIN           26
#define NUM_LEDS          10
#define LED_BRIGHTNESS    80

#define LED_MODE_OFF        0u
#define LED_MODE_DANGER     1u
#define LED_MODE_FOUND_TOY  2u

// Software microstep pins are intentionally disabled because GPIO32 is S1.
#define STEP1_USE_MICROSTEP_PINS  0

#if STEP1_USE_MICROSTEP_PINS
#define STEP1_MS1_PIN    32
#define STEP1_MS2_PIN    33
#define STEP1_MS3_PIN    26
#endif

#define DIR_NEG   LOW
#define DIR_POS   HIGH

// Higher value = slower/quieter.
#define STEP1_HALF_US   1500u
#define STEP2_HALF_US   750u
#define DIR_SETTLE_US   50u
#define YIELD_EVERY_STEPS  100u

#define STEP2_REVERSE_STEPS   1500u

#define STEP1_FULL_STEPS_PER_REV   200u
#define STEP1_PHYSICAL_DEG         140u
#define STEP1_MICROSTEP_DIVISOR    8u

#define STEP1_ANGLE_STEPS \
  ((STEP1_FULL_STEPS_PER_REV * STEP1_MICROSTEP_DIVISOR * STEP1_PHYSICAL_DEG) / 360u)

#define STEP1_FORWARD_STEPS   STEP1_ANGLE_STEPS
#define STEP1_RETURN_STEPS    STEP1_ANGLE_STEPS

// Limit safety: prevents infinite Stepper 1 homing if switch/wiring fails.
// Direction is the same as the Stepper 1 return motion: DIR_POS.
#define STEP1_HOME_MAX_STEPS  (STEP1_RETURN_STEPS * 20u)

#define SERVO_CLOSED_ANGLE    140
#define SERVO_OPEN_ANGLE      60
#define SERVO_DELAY_MS        15

// Limit safety: prevents infinite Stepper 2 homing if switch/wiring fails.
// At 750us half pulse + vTaskDelay(1), this is a large safe ceiling.
#define STEP2_HOME_MAX_STEPS  12000u
#define STEP2_LIMIT_BACKOFF_MS 2000u

Servo mechServo;
SemaphoreHandle_t mechanismStartSem = NULL;
Adafruit_NeoPixel ledStrip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
// Shared state
// ============================================================
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t rawSensorMask     = 0;
volatile uint8_t currentSensorMask = 0;
volatile uint8_t userDecision      = DECISION_NONE;
volatile uint8_t mechanismState    = MECH_STATE_IDLE;
volatile uint8_t pendingMechanismCommand = MECH_CMD_RUN;
volatile bool    mechanismStopRequested  = false;
volatile bool    toyFoundLedActive       = false;
volatile bool    toyFinishedLedOffHold   = false;
volatile uint8_t requestedLedMode        = LED_MODE_OFF;

volatile uint32_t detectionCount   = 0;
volatile uint32_t requestCount     = 0;
volatile uint32_t clearCmdCount    = 0;
volatile uint32_t toyCmdCount      = 0;
volatile uint32_t obstacleCmdCount = 0;

volatile bool     visionArmed          = false;
volatile bool     visionHadS2DuringArm = false;
volatile uint32_t visionArmStartMs     = 0;
volatile uint32_t visionArmCount       = 0;
volatile uint32_t piToyAcceptedCount   = 0;
volatile uint32_t piToyRejectedCount   = 0;
volatile uint32_t lastPiToyMs          = 0;
volatile uint16_t lastPiConfPermille   = 0;

// ============================================================
// micro-ROS state
// ============================================================
rcl_node_t ros_node;
rclc_support_t ros_support;
rcl_allocator_t ros_allocator;
rclc_executor_t ros_executor;

rcl_publisher_t sensor_mask_pub;
rcl_publisher_t decision_pub;
rcl_publisher_t mechanism_state_pub;
rcl_publisher_t vision_arm_pub;

rcl_subscription_t clear_decision_sub;
rcl_subscription_t arm_vision_sub;
rcl_subscription_t toy_detected_sub;
rcl_subscription_t mechanism_cmd_sub;

std_msgs__msg__UInt8 sensor_mask_msg;
std_msgs__msg__UInt8 decision_msg;
std_msgs__msg__UInt8 mechanism_state_msg;
std_msgs__msg__Bool vision_arm_msg;
std_msgs__msg__Bool clear_decision_cmd_msg;
std_msgs__msg__Bool arm_vision_cmd_msg;
std_msgs__msg__Float32 toy_detected_msg;
std_msgs__msg__UInt8 mechanism_cmd_msg;

volatile bool microRosReady = false;
volatile uint32_t rosMaskTxCount = 0;
volatile uint32_t rosDecisionTxCount = 0;
volatile uint32_t rosMechanismTxCount = 0;
volatile uint32_t rosVisionArmTxCount = 0;
volatile uint32_t rosClearRxCount = 0;
volatile uint32_t rosArmRxCount = 0;
volatile uint32_t rosToyRxCount = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ DebugSerial.printf("micro-ROS error at line %d: %d\n", __LINE__, (int)temp_rc); return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ DebugSerial.printf("micro-ROS soft error at line %d: %d\n", __LINE__, (int)temp_rc); }}

// ============================================================
// Forward declarations used by micro-ROS callbacks
// ============================================================
void clearDecisionLocal();
bool acceptPiToyCommand(uint16_t confPermille);
bool acceptPiNoToyCommand(uint16_t confPermille);
bool isVisionArmedNow(uint32_t nowMs, uint32_t *remainingMsOut);

// ============================================================
// Helper names
// ============================================================
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

// ============================================================
// Non-blocking NeoPixel status animation
// ============================================================
uint8_t scale8(uint8_t value, uint8_t scale) {
  return (uint8_t)(((uint16_t)value * (uint16_t)scale) / 255u);
}

void ledFill(uint32_t color) {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    ledStrip.setPixelColor(i, color);
  }
  ledStrip.show();
}

void ledShowOff() {
  ledFill(ledStrip.Color(0, 0, 0));
}

void refreshRequestedLedMode() {
  uint8_t raw;
  uint8_t stable;
  uint8_t decision;
  bool armed;
  bool toyLed;
  bool toyDoneHold;
  uint32_t armStart;
  uint32_t nowMs = millis();

  portENTER_CRITICAL(&stateMux);
  raw = rawSensorMask;
  stable = currentSensorMask;
  decision = userDecision;
  armed = visionArmed;
  toyLed = toyFoundLedActive;
  toyDoneHold = toyFinishedLedOffHold;
  armStart = visionArmStartMs;
  portEXIT_CRITICAL(&stateMux);

  bool armWindowActive = armed && ((uint32_t)(nowMs - armStart) < VISION_ARM_MS);
  bool anySensorDetected = ((raw | stable) & 0x07u) != 0;

  if (toyLed) {
    requestedLedMode = LED_MODE_FOUND_TOY;
  } else if (toyDoneHold) {
    requestedLedMode = LED_MODE_OFF;
  } else if (decision == DECISION_OBSTACLE || armWindowActive || anySensorDetected) {
    requestedLedMode = LED_MODE_DANGER;
  } else {
    requestedLedMode = LED_MODE_OFF;
  }
}

void renderDangerFrame(uint8_t frame) {
  bool flashOn = ((frame / 2u) % 2u) == 0u;
  uint32_t base = flashOn ? ledStrip.Color(255, 0, 0) : ledStrip.Color(35, 0, 0);
  uint32_t dark = ledStrip.Color(8, 0, 0);

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    ledStrip.setPixelColor(i, ((i + frame) % 3u) == 0u ? base : dark);
  }

  uint16_t head = frame % NUM_LEDS;
  uint16_t mirror = (NUM_LEDS - 1u) - head;
  ledStrip.setPixelColor(head, ledStrip.Color(255, 20, 0));
  ledStrip.setPixelColor(mirror, ledStrip.Color(255, 0, 0));
  ledStrip.show();
}

void renderFoundToyFrame(uint8_t frame) {
  uint8_t phase = frame % 36u;
  uint8_t breath = (phase < 18u) ? (uint8_t)(35u + phase * 5u)
                                 : (uint8_t)(35u + (35u - phase) * 5u);
  uint32_t dimCyan = ledStrip.Color(0, scale8(220, breath), scale8(255, breath));

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    ledStrip.setPixelColor(i, dimCyan);
  }

  uint16_t pathLen = (NUM_LEDS > 1u) ? (uint16_t)(NUM_LEDS * 2u - 2u) : 1u;
  uint16_t pos = frame % pathLen;
  if (pos >= NUM_LEDS) {
    pos = pathLen - pos;
  }

  ledStrip.setPixelColor(pos, ledStrip.Color(0, 255, 255));

  if (pos > 0u) {
    ledStrip.setPixelColor(pos - 1u, ledStrip.Color(0, 120, 150));
  }
  if ((pos + 1u) < NUM_LEDS) {
    ledStrip.setPixelColor(pos + 1u, ledStrip.Color(0, 120, 150));
  }

  ledStrip.show();
}

void taskLedAnimation(void *pv) {
  uint8_t activeMode = LED_MODE_OFF;
  uint8_t frame = 0;
  uint32_t lastFrameMs = 0;

  ledShowOff();

  while (true) {
    refreshRequestedLedMode();

    uint8_t nextMode = requestedLedMode;
    uint32_t nowMs = millis();

    if (nextMode != activeMode) {
      activeMode = nextMode;
      frame = 0;
      lastFrameMs = 0;
      if (activeMode == LED_MODE_OFF) {
        ledShowOff();
      }
    }

    if (activeMode != LED_MODE_OFF) {
      uint16_t intervalMs = (activeMode == LED_MODE_DANGER) ? 80u : 55u;
      if (lastFrameMs == 0u || (uint32_t)(nowMs - lastFrameMs) >= intervalMs) {
        lastFrameMs = nowMs;
        if (activeMode == LED_MODE_DANGER) {
          renderDangerFrame(frame);
        } else {
          renderFoundToyFrame(frame);
        }
        frame++;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ============================================================
// Sensor debounce
// ============================================================
uint8_t updateDebouncedBit(uint8_t rawMask,
                           uint8_t stableMask,
                           uint8_t bit,
                           uint16_t &counterMs,
                           uint16_t detectDebounceMs) {
  bool rawDetected    = (rawMask    & bit) != 0;
  bool stableDetected = (stableMask & bit) != 0;

  if (rawDetected == stableDetected) {
    counterMs = 0;
    return stableMask;
  }

  if (counterMs < 60000) counterMs += SENSOR_SAMPLE_MS;

  uint16_t neededMs = rawDetected ? detectDebounceMs : CLEAR_DEBOUNCE_MS;

  if (counterMs >= neededMs) {
    if (rawDetected) stableMask |= bit;
    else             stableMask &= (uint8_t)(~bit);
    counterMs = 0;
  }

  return stableMask;
}

void taskReadSensors(void *pv) {
  uint8_t stableMask = 0;
  uint8_t lastStableMask = 0;

  uint16_t s1CounterMs = 0;
  uint16_t s2CounterMs = 0;
  uint16_t s3CounterMs = 0;

  while (true) {
    uint8_t rawMask = 0;

    if (digitalRead(SENSOR_1_RIGHT_PIN)  == LOW) rawMask |= SENSOR_1_RIGHT_MASK;
    if (digitalRead(SENSOR_2_MIDDLE_PIN) == LOW) rawMask |= SENSOR_2_MIDDLE_MASK;
    if (digitalRead(SENSOR_3_LEFT_PIN)   == LOW) rawMask |= SENSOR_3_LEFT_MASK;

    stableMask = updateDebouncedBit(rawMask, stableMask,
                                    SENSOR_1_RIGHT_MASK,
                                    s1CounterMs,
                                    SIDE_DETECT_DEBOUNCE_MS);

    stableMask = updateDebouncedBit(rawMask, stableMask,
                                    SENSOR_2_MIDDLE_MASK,
                                    s2CounterMs,
                                    S2_DETECT_DEBOUNCE_MS);

    stableMask = updateDebouncedBit(rawMask, stableMask,
                                    SENSOR_3_LEFT_MASK,
                                    s3CounterMs,
                                    SIDE_DETECT_DEBOUNCE_MS);

    portENTER_CRITICAL(&stateMux);
    rawSensorMask = rawMask;
    currentSensorMask = stableMask;
    if (visionArmed && ((rawMask | stableMask) & SENSOR_2_MIDDLE_MASK)) {
      visionHadS2DuringArm = true;
    }
    if (stableMask != 0 && lastStableMask == 0) detectionCount++;
    portEXIT_CRITICAL(&stateMux);

    lastStableMask = stableMask;
    vTaskDelay(pdMS_TO_TICKS(SENSOR_SAMPLE_MS));
  }
}

// ============================================================
// Mechanism helpers
// ============================================================
#if STEP1_USE_MICROSTEP_PINS
void setupStepper1Microstepping() {
  pinMode(STEP1_MS1_PIN, OUTPUT);
  pinMode(STEP1_MS2_PIN, OUTPUT);
  pinMode(STEP1_MS3_PIN, OUTPUT);

  digitalWrite(STEP1_MS1_PIN, HIGH);
  digitalWrite(STEP1_MS2_PIN, HIGH);
  digitalWrite(STEP1_MS3_PIN, LOW);

  DebugSerial.println("Stepper 1 software microstepping enabled: 110 assumed 1/8.");
}
#endif

void stepOnce(uint8_t pin, uint32_t halfUs) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(halfUs);
  digitalWrite(pin, LOW);
  delayMicroseconds(halfUs);
}

bool runStepsConstant(uint8_t stepPin, uint32_t halfUs, uint32_t count) {
  if (count == 0) return true;

  for (uint32_t i = 0; i < count; i++) {
    if (mechanismStopRequested) return false;

    stepOnce(stepPin, halfUs);

    if ((i + 1) % YIELD_EVERY_STEPS == 0) {
      vTaskDelay(1);
    }
  }

  return true;
}

void setStepperDirection(uint8_t dirPin, uint8_t direction) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(DIR_SETTLE_US);
}

void servoSweep(int from, int to) {
  if (from == to) {
    mechServo.write(to);
    vTaskDelay(pdMS_TO_TICKS(SERVO_DELAY_MS));
    return;
  }

  int dir = (to > from) ? 1 : -1;

  for (int a = from; a != to; a += dir) {
    mechServo.write(a);
    vTaskDelay(pdMS_TO_TICKS(SERVO_DELAY_MS));
  }

  mechServo.write(to);
  vTaskDelay(pdMS_TO_TICKS(SERVO_DELAY_MS));
}


bool homeStepper1ToLimit(const char *phaseLabel) {
  DebugSerial.printf(">>> %s: Stepper 1 homing -> +1, seeking Stepper 1 limit switch on GPIO%d\n",
                     phaseLabel,
                     STEP1_LIMIT_PIN);

  setStepperDirection(STEP1_DIR_PIN, DIR_POS);

  // Active-LOW limit: LOW = triggered, HIGH = not triggered.
  // This is intentionally the same logic as your obstacle sensors.
  if (digitalRead(STEP1_LIMIT_PIN) == LOW) {
    DebugSerial.printf(">>> %s note: Stepper 1 limit switch already active - Stepper 1 does not move\n",
                       phaseLabel);
    return true;
  }

  uint32_t steps = 0;
  while (digitalRead(STEP1_LIMIT_PIN) == HIGH) {
    if (mechanismStopRequested) return false;

    stepOnce(STEP1_STEP_PIN, STEP1_HALF_US);
    steps++;

    if (steps >= STEP1_HOME_MAX_STEPS) {
      DebugSerial.printf("ERROR: %s Stepper 1 homing timeout. Limit switch did not trigger.\n",
                         phaseLabel);
      return false;
    }

    vTaskDelay(1);
  }

  DebugSerial.printf(">>> %s done: Stepper 1 limit switch triggered - Stepper 1 stopped after %lu steps\n",
                     phaseLabel,
                     (unsigned long)steps);
  return true;
}

bool backoffStepper2FromLimit(const char *phaseLabel) {
  DebugSerial.printf(">>> %s: Stepper 2 backing off -> +1 for %lu ms\n",
                     phaseLabel,
                     (unsigned long)STEP2_LIMIT_BACKOFF_MS);

  setStepperDirection(STEP2_DIR_PIN, DIR_POS);

  uint32_t startMs = millis();
  uint32_t steps = 0;
  while ((uint32_t)(millis() - startMs) < STEP2_LIMIT_BACKOFF_MS) {
    if (mechanismStopRequested) return false;

    stepOnce(STEP2_STEP_PIN, STEP2_HALF_US);
    steps++;
    vTaskDelay(1);
  }

  DebugSerial.printf(">>> %s: Stepper 2 backoff done after %lu steps\n",
                     phaseLabel,
                     (unsigned long)steps);
  return true;
}

bool homeStepper2ToLimit(const char *phaseLabel) {
  DebugSerial.printf(">>> %s: Stepper 2 homing -> -1, seeking Stepper 2 limit switch on GPIO%d\n",
                     phaseLabel,
                     LIMIT_PIN);

  setStepperDirection(STEP2_DIR_PIN, DIR_NEG);

  // Active-LOW limit: LOW = triggered, HIGH = not triggered.
  if (digitalRead(LIMIT_PIN) == LOW) {
    DebugSerial.printf(">>> %s note: Stepper 2 limit switch already active - backing off\n",
                       phaseLabel);
    return backoffStepper2FromLimit(phaseLabel);
  }

  uint32_t steps = 0;
  while (digitalRead(LIMIT_PIN) == HIGH) {
    if (mechanismStopRequested) return false;

    stepOnce(STEP2_STEP_PIN, STEP2_HALF_US);
    steps++;

    if (steps >= STEP2_HOME_MAX_STEPS) {
      DebugSerial.printf("ERROR: %s Stepper 2 homing timeout. Limit switch did not trigger.\n",
                         phaseLabel);
      return false;
    }

    vTaskDelay(1);
  }

  DebugSerial.printf(">>> %s done: Stepper 2 limit switch triggered - Stepper 2 stopped after %lu steps\n",
                     phaseLabel,
                     (unsigned long)steps);
  return backoffStepper2FromLimit(phaseLabel);
}

bool lowerStepper2ToLimitNoBackoff(const char *phaseLabel) {
  DebugSerial.printf(">>> %s: Stepper 2 pickup lower -> -1, seeking Stepper 2 limit switch on GPIO%d\n",
                     phaseLabel,
                     LIMIT_PIN);

  setStepperDirection(STEP2_DIR_PIN, DIR_NEG);

  // During pickup, reaching the lower limit is the final down position.
  // Do not back off here; the gripper must close while Stepper 2 is still down.
  if (digitalRead(LIMIT_PIN) == LOW) {
    DebugSerial.printf(">>> %s note: Stepper 2 limit switch already active - staying down for gripper close\n",
                       phaseLabel);
    return true;
  }

  uint32_t steps = 0;
  while (digitalRead(LIMIT_PIN) == HIGH) {
    if (mechanismStopRequested) return false;

    stepOnce(STEP2_STEP_PIN, STEP2_HALF_US);
    steps++;

    if (steps >= STEP2_HOME_MAX_STEPS) {
      DebugSerial.printf("ERROR: %s Stepper 2 lower timeout. Limit switch did not trigger.\n",
                         phaseLabel);
      return false;
    }

    vTaskDelay(1);
  }

  DebugSerial.printf(">>> %s done: Stepper 2 lower limit triggered - staying down after %lu steps\n",
                     phaseLabel,
                     (unsigned long)steps);
  return true;
}

bool runMechanismSequenceOnce() {
  DebugSerial.println("=== MECHANISM SEQUENCE START ===");

  DebugSerial.printf(">>> Phase 1: Servo open to %d deg\n", SERVO_OPEN_ANGLE);
  servoSweep(SERVO_OPEN_ANGLE, SERVO_OPEN_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(200));

  DebugSerial.println(">>> Phase 2: Stepper 2 lowers to limit and stays down for gripper close");
  if (!lowerStepper2ToLimitNoBackoff("Phase 2")) {
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(150));

  DebugSerial.printf(">>> Phase 3: Servo close %d deg -> %d deg\n", SERVO_OPEN_ANGLE, SERVO_CLOSED_ANGLE);
  servoSweep(SERVO_OPEN_ANGLE, SERVO_CLOSED_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(200));

  DebugSerial.printf(">>> Phase 4: Stepper 2 -> +1 for %lu ms after gripper close\n",
                     (unsigned long)STEP2_LIMIT_BACKOFF_MS);
  if (!backoffStepper2FromLimit("Phase 4")) {
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(150));

  DebugSerial.printf(">>> Phase 5: Stepper 1 -> -1, physical %lu deg, pulses %lu\n",
                (unsigned long)STEP1_PHYSICAL_DEG,
                (unsigned long)STEP1_FORWARD_STEPS);
  setStepperDirection(STEP1_DIR_PIN, DIR_NEG);
  if (!runStepsConstant(STEP1_STEP_PIN, STEP1_HALF_US, STEP1_FORWARD_STEPS)) {
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(150));

  DebugSerial.printf(">>> Phase 6: Servo open %d deg -> %d deg\n", SERVO_CLOSED_ANGLE, SERVO_OPEN_ANGLE);
  servoSweep(SERVO_CLOSED_ANGLE, SERVO_OPEN_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(150));

  DebugSerial.println(">>> Phase 7: Stepper 1 return is now homing to the Stepper 1 limit switch");
  if (!homeStepper1ToLimit("Phase 7")) {
    return false;
  }

  DebugSerial.println("=== MECHANISM SEQUENCE DONE ===");
  return true;
}

bool runMechanismCommand(uint8_t cmd) {
  mechanismStopRequested = false;

  switch (cmd) {
    case MECH_CMD_RUN:
      return runMechanismSequenceOnce();

    case MECH_CMD_SERVO_OPEN:
      servoSweep(SERVO_CLOSED_ANGLE, SERVO_OPEN_ANGLE);
      return !mechanismStopRequested;

    case MECH_CMD_SERVO_CLOSE:
      servoSweep(SERVO_OPEN_ANGLE, SERVO_CLOSED_ANGLE);
      return !mechanismStopRequested;

    case MECH_CMD_STEP2_HOME:
      return homeStepper2ToLimit("GUI Stepper 2 Home");

    case MECH_CMD_STEP2_FWD:
      setStepperDirection(STEP2_DIR_PIN, DIR_POS);
      return runStepsConstant(STEP2_STEP_PIN, STEP2_HALF_US, STEP2_REVERSE_STEPS);

    case MECH_CMD_STEP1_FWD:
      setStepperDirection(STEP1_DIR_PIN, DIR_NEG);
      return runStepsConstant(STEP1_STEP_PIN, STEP1_HALF_US, STEP1_FORWARD_STEPS);

    case MECH_CMD_STEP1_RETURN:
      return homeStepper1ToLimit("GUI Stepper 1 Return");

    case MECH_CMD_HOME_ALL:
      if (!homeStepper1ToLimit("GUI Home Stepper 1")) {
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(150));
      return homeStepper2ToLimit("GUI Home Stepper 2");

    default:
      return true;
  }
}

void taskMechanism(void *pv) {
  while (true) {
    if (xSemaphoreTake(mechanismStartSem, portMAX_DELAY) == pdTRUE) {
      uint8_t commandToRun;
      bool commandIsToySequence;

      portENTER_CRITICAL(&stateMux);
      commandToRun = pendingMechanismCommand;
      commandIsToySequence = (commandToRun == MECH_CMD_RUN) && toyFoundLedActive;
      mechanismState = MECH_STATE_RUNNING;
      portEXIT_CRITICAL(&stateMux);

      bool ok = runMechanismCommand(commandToRun);

      portENTER_CRITICAL(&stateMux);
      mechanismState = mechanismStopRequested ? MECH_STATE_IDLE : (ok ? MECH_STATE_DONE : MECH_STATE_ERROR);
      mechanismStopRequested = false;
      if (commandToRun == MECH_CMD_RUN) {
        toyFoundLedActive = false;
        toyFinishedLedOffHold = commandIsToySequence;
      }
      portEXIT_CRITICAL(&stateMux);
    }
  }
}

// ============================================================
// Command handling
// ============================================================
void acceptToyCommand() {
  portENTER_CRITICAL(&stateMux);
  userDecision = DECISION_TOY;
  toyFoundLedActive = true;
  toyFinishedLedOffHold = false;
  toyCmdCount++;
  visionArmed = false;
  visionHadS2DuringArm = false;
  portEXIT_CRITICAL(&stateMux);
}

void acceptObstacleCommand() {
  portENTER_CRITICAL(&stateMux);
  userDecision = DECISION_OBSTACLE;
  toyFoundLedActive = false;
  toyFinishedLedOffHold = false;
  obstacleCmdCount++;
  portEXIT_CRITICAL(&stateMux);
}

void clearDecisionLocal() {
  portENTER_CRITICAL(&stateMux);
  userDecision = DECISION_NONE;
  if (mechanismState == MECH_STATE_DONE) mechanismState = MECH_STATE_IDLE;
  toyFoundLedActive = false;
  toyFinishedLedOffHold = false;
  visionArmed = false;
  visionHadS2DuringArm = false;
  clearCmdCount++;
  portEXIT_CRITICAL(&stateMux);
}


bool isVisionArmedNow(uint32_t nowMs, uint32_t *remainingMsOut = nullptr) {
  bool armed;
  uint32_t startMs;

  portENTER_CRITICAL(&stateMux);
  armed = visionArmed;
  startMs = visionArmStartMs;
  portEXIT_CRITICAL(&stateMux);

  if (!armed) {
    if (remainingMsOut) *remainingMsOut = 0;
    return false;
  }

  uint32_t elapsed = nowMs - startMs;
  if (elapsed >= VISION_ARM_MS) {
    portENTER_CRITICAL(&stateMux);
    visionArmed = false;
    visionHadS2DuringArm = false;
    portEXIT_CRITICAL(&stateMux);
    if (remainingMsOut) *remainingMsOut = 0;
    return false;
  }

  if (remainingMsOut) *remainingMsOut = VISION_ARM_MS - elapsed;
  return true;
}

bool shouldIgnorePiVisionInput() {
  uint8_t decision;
  uint8_t mech;

  portENTER_CRITICAL(&stateMux);
  decision = userDecision;
  mech = mechanismState;
  portEXIT_CRITICAL(&stateMux);

  // Once TOY is accepted, PID owns the next step:
  // move forward 12 cm, then publish /robot/cmd/mechanism = MECH_CMD_RUN.
  // Later Pi confidence/false messages must not change the latched decision.
  return (decision == DECISION_TOY ||
          mech == MECH_STATE_RUNNING ||
          mech == MECH_STATE_DONE);
}

bool acceptPiToyCommand(uint16_t confPermille) {
  uint32_t nowMs = millis();
  bool armed = isVisionArmedNow(nowMs, nullptr);

  portENTER_CRITICAL(&stateMux);
  lastPiToyMs = nowMs;
  lastPiConfPermille = confPermille;

  bool s2Stable = (currentSensorMask & SENSOR_2_MIDDLE_MASK) != 0;
  bool s2Raw = (rawSensorMask & SENSOR_2_MIDDLE_MASK) != 0;
  bool s2Ok = s2Stable || s2Raw || visionHadS2DuringArm;
  bool confOk = confPermille >= VISION_MIN_CONF_PERMILLE;
  bool mechanismReadyForLaterStart =
    (mechanismState == MECH_STATE_IDLE ||
     mechanismState == MECH_STATE_DONE ||
     mechanismState == MECH_STATE_ERROR);

  bool accept = armed && s2Ok && confOk && mechanismReadyForLaterStart;

  if (accept) {
    userDecision = DECISION_TOY;
    pendingMechanismCommand = MECH_CMD_RUN;
    mechanismStopRequested = false;
    toyFoundLedActive = true;
    toyFinishedLedOffHold = false;
    toyCmdCount++;
    piToyAcceptedCount++;
    visionArmed = false;
    visionHadS2DuringArm = false;
  } else {
    piToyRejectedCount++;
    DebugSerial.printf("Pi TOY rejected: armed=%d s2Stable=%d s2Raw=%d hadS2=%d conf=%u mech=%s\n",
                  armed ? 1 : 0,
                  s2Stable ? 1 : 0,
                  s2Raw ? 1 : 0,
                  visionHadS2DuringArm ? 1 : 0,
                  (unsigned)confPermille,
                  mechanismStateName(mechanismState));
  }
  portEXIT_CRITICAL(&stateMux);

  return accept;
}


bool acceptPiNoToyCommand(uint16_t confPermille) {
  uint32_t nowMs = millis();
  bool armed = isVisionArmedNow(nowMs, nullptr);
  bool makeObstacleDecision = false;

  portENTER_CRITICAL(&stateMux);
  lastPiToyMs = nowMs;
  lastPiConfPermille = confPermille;

  bool s2Stable = (currentSensorMask & SENSOR_2_MIDDLE_MASK) != 0;
  bool s2Raw = (rawSensorMask & SENSOR_2_MIDDLE_MASK) != 0;
  bool s2Ok = s2Stable || s2Raw || visionHadS2DuringArm;
  uint32_t armedElapsedMs = nowMs - visionArmStartMs;

  // 0.0 from the Pi means "false / no confirmed toy".
  // Do NOT turn it into OBSTACLE immediately, otherwise the robot would avoid
  // before YOLO gets its 8-second first recognition plus 4-second final confirmation chance.
  // After the extended confirmation window passes with no >=70% confirmed toy,
  // convert the active S2 case into OBSTACLE so the PID ESP performs avoidance.
  if (armed &&
      s2Ok &&
      confPermille < VISION_MIN_CONF_PERMILLE &&
      armedElapsedMs >= PI_NO_TOY_CONFIRM_MS &&
      userDecision == DECISION_NONE) {
    userDecision = DECISION_OBSTACLE;
    toyFoundLedActive = false;
    toyFinishedLedOffHold = false;
    obstacleCmdCount++;
    visionArmed = false;
    visionHadS2DuringArm = false;
    makeObstacleDecision = true;
  }

  portEXIT_CRITICAL(&stateMux);
  return makeObstacleDecision;
}

// ============================================================
// micro-ROS callbacks and tasks
// ============================================================
void clearDecisionCallback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    clearDecisionLocal();
    rosClearRxCount++;
  }
}

void armVisionCallback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  uint32_t nowMs = millis();

  portENTER_CRITICAL(&stateMux);
  if (msg->data) {
    bool s2Now = ((rawSensorMask | currentSensorMask) & SENSOR_2_MIDDLE_MASK) != 0;

    // The PID ESP republishes arm=true repeatedly during the vision window.
    // Repeated or late arm=true messages must NOT clear a pending TOY/OBSTACLE
    // decision. PID must explicitly clear the decision after it handles it.
    if (userDecision != DECISION_NONE ||
        mechanismState == MECH_STATE_RUNNING ||
        mechanismState == MECH_STATE_DONE) {
      // Keep the latched decision/state untouched.
    } else if (!visionArmed) {
      userDecision = DECISION_NONE;
      visionArmed = true;
      visionArmStartMs = nowMs;
      visionArmCount++;
      visionHadS2DuringArm = s2Now;
    } else {
      visionHadS2DuringArm = visionHadS2DuringArm || s2Now;
    }
  } else {
    visionArmed = false;
    visionHadS2DuringArm = false;
    userDecision = DECISION_NONE;
  }
  portEXIT_CRITICAL(&stateMux);

  rosArmRxCount++;
}

void toyDetectedCallback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  if (shouldIgnorePiVisionInput()) {
    rosToyRxCount++;
    return;
  }

  float conf = msg->data;

  if (conf > 1.5f) conf = conf / 100.0f;  // accepts 85.0 as 0.85
  if (conf < 0.0f) conf = 0.0f;
  if (conf > 1.0f) conf = 1.0f;

  uint16_t confPermille = (uint16_t)(conf * 1000.0f + 0.5f);

  if (confPermille >= VISION_MIN_CONF_PERMILLE) {
    acceptPiToyCommand(confPermille);
  } else {
    acceptPiNoToyCommand(confPermille);
  }

  rosToyRxCount++;
}

void mechanismCommandCallback(const void * msgin) {
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  uint8_t cmd = msg->data;

  if (cmd == MECH_CMD_STOP) {
    bool wasToyLedActive = toyFoundLedActive;
    mechanismStopRequested = true;
    toyFoundLedActive = false;
    if (wasToyLedActive) {
      toyFinishedLedOffHold = true;
    }
    return;
  }

  if (cmd > MECH_CMD_HOME_ALL) {
    return;
  }

  bool canStart = false;

  portENTER_CRITICAL(&stateMux);
  if (mechanismState == MECH_STATE_IDLE ||
      mechanismState == MECH_STATE_DONE ||
      mechanismState == MECH_STATE_ERROR) {
    pendingMechanismCommand = cmd;
    mechanismStopRequested = false;
    mechanismState = MECH_STATE_RUNNING;
    if (cmd == MECH_CMD_RUN && userDecision == DECISION_TOY) {
      toyFoundLedActive = true;
      toyFinishedLedOffHold = false;
    }
    canStart = true;
  }
  portEXIT_CRITICAL(&stateMux);

  if (canStart && mechanismStartSem) {
    xSemaphoreGive(mechanismStartSem);
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
  RCCHECK(rclc_node_init_default(&ros_node, "sensor_mechanism_node", "", &ros_support));

  RCCHECK(rclc_publisher_init_best_effort(
    &sensor_mask_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/sensors/mask"));

  RCCHECK(rclc_publisher_init_best_effort(
    &decision_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/sensors/decision"));

  RCCHECK(rclc_publisher_init_best_effort(
    &mechanism_state_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/mechanism/state"));

  RCCHECK(rclc_publisher_init_best_effort(
    &vision_arm_pub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/vision/arm"));

  RCCHECK(rclc_subscription_init_best_effort(
    &clear_decision_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/robot/cmd/clear_decision"));

  RCCHECK(rclc_subscription_init_best_effort(
    &arm_vision_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/robot/cmd/arm_vision"));

  RCCHECK(rclc_subscription_init_best_effort(
    &toy_detected_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/vision/toy_detected"));

  RCCHECK(rclc_subscription_init_best_effort(
    &mechanism_cmd_sub,
    &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "/robot/cmd/mechanism"));

  RCCHECK(rclc_executor_init(&ros_executor, &ros_support.context, 4, &ros_allocator));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &clear_decision_sub, &clear_decision_cmd_msg, &clearDecisionCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &arm_vision_sub, &arm_vision_cmd_msg, &armVisionCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &toy_detected_sub, &toy_detected_msg, &toyDetectedCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&ros_executor, &mechanism_cmd_sub, &mechanism_cmd_msg, &mechanismCommandCallback, ON_NEW_DATA));

  microRosReady = true;
  DebugSerial.println("micro-ROS sensor/mechanism node ready.");
  return true;
}

void taskMicroRos(void *pv) {
  TickType_t lastPub = xTaskGetTickCount();

  while (true) {
    if (microRosReady) {
      // Keep all micro-ROS operations in ONE task.
      // Serial micro-ROS is not safe when spin and publish use the same transport from two tasks.
      RCSOFTCHECK(rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(2)));

      TickType_t now = xTaskGetTickCount();
      if ((now - lastPub) >= pdMS_TO_TICKS(ROS_PUBLISH_PERIOD_MS)) {
        lastPub = now;

        uint8_t stable, decision, mech;
        bool vArmed = isVisionArmedNow(millis(), nullptr);

        portENTER_CRITICAL(&stateMux);
        // Transmit detection immediately from RAW, but keep clear controlled by debounce.
        // This makes the drive ESP stop fast without making clear/no-clear flicker.
        uint8_t raw = rawSensorMask & 0x07;
        stable = (currentSensorMask | raw) & 0x07;
        decision = userDecision;
        mech = mechanismState;
        portEXIT_CRITICAL(&stateMux);

        sensor_mask_msg.data = stable;
        decision_msg.data = decision;
        mechanism_state_msg.data = mech;
        vision_arm_msg.data = vArmed;

        RCSOFTCHECK(rcl_publish(&sensor_mask_pub, &sensor_mask_msg, NULL));
        rosMaskTxCount++;

        RCSOFTCHECK(rcl_publish(&decision_pub, &decision_msg, NULL));
        rosDecisionTxCount++;

        RCSOFTCHECK(rcl_publish(&mechanism_state_pub, &mechanism_state_msg, NULL));
        rosMechanismTxCount++;

        RCSOFTCHECK(rcl_publish(&vision_arm_pub, &vision_arm_msg, NULL));
        rosVisionArmTxCount++;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void taskPrintStatus(void *pv) {
  while (true) {
    uint8_t raw, stable, decision, mech;
    uint32_t req, det;

    portENTER_CRITICAL(&stateMux);
    raw = rawSensorMask;
    stable = currentSensorMask;
    decision = userDecision;
    mech = mechanismState;
    req = requestCount;
    det = detectionCount;
    portEXIT_CRITICAL(&stateMux);

    DebugSerial.println();
    DebugSerial.println("========= MERGED SENSOR + MECHANISM ESP =========");
    DebugSerial.printf("Stable mask: 0x%02X | Raw mask: 0x%02X\n", stable, raw);
    DebugSerial.printf("S1 RIGHT=%s | S2 MIDDLE=%s | S3 LEFT=%s\n",
                  (stable & SENSOR_1_RIGHT_MASK) ? "DETECTED" : "clear",
                  (stable & SENSOR_2_MIDDLE_MASK) ? "DETECTED" : "clear",
                  (stable & SENSOR_3_LEFT_MASK) ? "DETECTED" : "clear");
    DebugSerial.printf("Decision=%s | Mechanism=%s\n", decisionName(decision), mechanismStateName(mech));
    DebugSerial.printf("Legacy I2C requests=%lu | detection events=%lu\n", (unsigned long)req, (unsigned long)det);
    DebugSerial.printf("ROS tx: mask=%lu decision=%lu mech=%lu visionArm=%lu | rx: clear=%lu arm=%lu toy=%lu\n",
                  (unsigned long)rosMaskTxCount,
                  (unsigned long)rosDecisionTxCount,
                  (unsigned long)rosMechanismTxCount,
                  (unsigned long)rosVisionArmTxCount,
                  (unsigned long)rosClearRxCount,
                  (unsigned long)rosArmRxCount,
                  (unsigned long)rosToyRxCount);
    DebugSerial.println("Transport: USB serial micro-ROS @ 115200 baud");
    DebugSerial.println("=================================================");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(MICRO_ROS_SERIAL_BAUD);  // Must match the micro-ROS agent -b value: 115200.
  delay(2000);

  DebugSerial.println();
  DebugSerial.println("=== Merged Sensor + Mechanism ESP32 Boot ===");
  DebugSerial.println("Sensors are active-LOW: LOW = detected.");

  ledStrip.begin();
  ledStrip.setBrightness(LED_BRIGHTNESS);
  ledShowOff();

  pinMode(SENSOR_1_RIGHT_PIN,  INPUT_PULLUP);
  pinMode(SENSOR_2_MIDDLE_PIN, INPUT);
  pinMode(SENSOR_3_LEFT_PIN,   INPUT);

  pinMode(STEP1_DIR_PIN,  OUTPUT);
  pinMode(STEP1_STEP_PIN, OUTPUT);
  pinMode(STEP2_DIR_PIN,  OUTPUT);
  pinMode(STEP2_STEP_PIN, OUTPUT);
  pinMode(LIMIT_PIN,      INPUT_PULLUP);
  pinMode(STEP1_LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(STEP1_STEP_PIN, LOW);
  digitalWrite(STEP2_STEP_PIN, LOW);
  digitalWrite(STEP1_DIR_PIN, DIR_NEG);
  digitalWrite(STEP2_DIR_PIN, DIR_NEG);

#if STEP1_USE_MICROSTEP_PINS
  setupStepper1Microstepping();
#else
  DebugSerial.println("Stepper 1 software microstep pins are DISABLED to avoid GPIO32 sensor conflict.");
  DebugSerial.println("Set Stepper 1 driver hardware jumpers/DIP switches to 1/8 microstepping.");
#endif

  ESP32PWM::allocateTimer(0);
  mechServo.setPeriodHertz(50);
  mechServo.attach(SERVO_PIN, 500, 2400);
  mechServo.write(SERVO_OPEN_ANGLE);
  delay(600);

  mechanismStartSem = xSemaphoreCreateBinary();
  if (!mechanismStartSem) {
    DebugSerial.println("FATAL: mechanism semaphore creation failed.");
    while (true) delay(1000);
  }

  // Bring ROS up before initial homing so the Pi/GUI can see the sensor ESP
  // while homing is running or if a limit switch problem occurs.
  portENTER_CRITICAL(&stateMux);
  mechanismState = MECH_STATE_RUNNING;
  portEXIT_CRITICAL(&stateMux);

  DebugSerial.println("Starting USB serial micro-ROS transport to Raspberry Pi agent...");
  if (!setupMicroRosNode()) {
    DebugSerial.println("FATAL: micro-ROS setup failed. Check serial agent, device path, and baud rate.");
    while (true) delay(1000);
  }

  // Core 0 = micro-ROS communication and status
  xTaskCreatePinnedToCore(taskMicroRos,      "uros",     12288, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(taskPrintStatus,   "print",     4096, NULL, 1, NULL, 0);

  // Core 1 = sensor sampling. Mechanism command task starts after initial homing.
  xTaskCreatePinnedToCore(taskReadSensors,   "sensors", 3072, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskLedAnimation,  "led",     3072, NULL, 1, NULL, 1);

  // Initial mechanism homing gate after ESP initialization and ROS setup.
  // Required order is unchanged: Stepper 1 homes first, then Stepper 2 homes.
  // The PID robot path will not start until /robot/mechanism/state becomes IDLE.
  // Stepper 1 homes in DIR_POS; Stepper 2 homes in DIR_NEG.
  bool initialHomeOk = homeStepper1ToLimit("Setup Stepper 1");
  if (initialHomeOk) {
    delay(150);
    initialHomeOk = homeStepper2ToLimit("Setup Stepper 2");
  }

  portENTER_CRITICAL(&stateMux);
  mechanismState = initialHomeOk ? MECH_STATE_IDLE : MECH_STATE_ERROR;
  portEXIT_CRITICAL(&stateMux);
  delay(150);

  // Core 1 = mechanism motion after initial homing is complete
  xTaskCreatePinnedToCore(taskMechanism,     "mech",    6144, NULL, 2, NULL, 1);

  DebugSerial.println("Ready. PID ESP32 and Pi should communicate through ROS 2 topics.");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
