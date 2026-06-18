# ============================================================
# Raspberry Pi ROS 2 Vision Node - DEBUG STAGE VERSION
#
# Built from your working NO-ROS camera logic, then ROS 2 output added.
#
# Subscribes:
#   /vision/arm              std_msgs/Bool
#   /robot/pid/obstacle_state std_msgs/UInt8
#   /robot/pid/static_mask    std_msgs/UInt8
#
# Publishes:
#   /vision/toy_confidence   std_msgs/Float32  live highest toy confidence
#   /vision/toy_detected     std_msgs/Float32  0.0=false, confirmed only after 8s + 4s
#
# Logic:
#   - Always runs camera + YOLO
#   - Prints current stage while running
#   - Sends 0.0 until toy is confirmed
#   - Requires confidence >= 0.70 continuously for 8 seconds first
#   - Then requires another continuous 4 seconds at >= 0.70
#   - Then bursts confirmed toy messages for 2 seconds
# ============================================================

import sys
from unittest.mock import MagicMock
sys.modules["pykms"] = MagicMock()

import threading
import time
from pathlib import Path

try:
    from gpiozero import OutputDevice
except Exception:
    OutputDevice = None

from ultralytics import YOLO
from picamera2 import Picamera2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, UInt8
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MODEL_PATH = Path("/home/paula/best.pt")

BUZZER_GPIO_BCM = 18          # Raspberry Pi physical pin 12
BUZZER_BEEP_COUNT = 3
BUZZER_ON_SECONDS = 0.25
BUZZER_OFF_SECONDS = 0.35
BUZZER_RETRIGGER_GUARD_SECONDS = 1.50

TOY_CLASS_NAME = "toy"
TOY_CONF_THRESHOLD = 0.70
TOY_FIRST_RECOGNITION_SECONDS = 8.0
TOY_FINAL_CONFIRM_SECONDS = 4.0

YOLO_CONF = 0.40
YOLO_IMGSZ = 320
CAMERA_SIZE = (1280, 720)

TOY_EVENT_BURST_SECONDS = 2.0
TOY_EVENT_BURST_INTERVAL_SECONDS = 0.10

CONF_PUBLISH_INTERVAL_SECONDS = 0.10
FALSE_PUBLISH_INTERVAL_SECONDS = 0.10
PRINT_INTERVAL_SECONDS = 0.50

_ros_node = None
_ros_thread = None

_vision_armed = False
_last_arm_state = None

_toy_phase = "FIRST_RECOGNITION"
_toy_seen_start = None
_best_toy_conf = 0.0

_burst_active = False
_burst_end_time = 0.0
_toy_sent_this_arm = False

_last_conf_publish = 0.0
_last_detected_publish = 0.0
_last_print = 0.0

_buzzer = None
_buzzer_lock = threading.Lock()
_buzzer_thread = None
_last_buzzer_trigger = 0.0


def reset_toy_confirmation_state() -> None:
    global _toy_phase, _toy_seen_start, _best_toy_conf
    global _burst_active, _burst_end_time, _toy_sent_this_arm

    _toy_phase = "FIRST_RECOGNITION"
    _toy_seen_start = None
    _best_toy_conf = 0.0
    _burst_active = False
    _burst_end_time = 0.0
    _toy_sent_this_arm = False


class AutoToyRosNode(Node):
    def __init__(self):
        super().__init__("pi_auto_toy_debug_node")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.toy_conf_pub = self.create_publisher(
            Float32,
            "/vision/toy_confidence",
            qos,
        )

        self.toy_detected_pub = self.create_publisher(
            Float32,
            "/vision/toy_detected",
            qos,
        )

        self.arm_sub = self.create_subscription(
            Bool,
            "/vision/arm",
            self._arm_callback,
            qos,
        )

        self.obstacle_state_sub = self.create_subscription(
            UInt8,
            "/robot/pid/obstacle_state",
            self._obstacle_state_callback,
            qos,
        )

        self.static_mask_sub = self.create_subscription(
            UInt8,
            "/robot/pid/static_mask",
            self._static_mask_callback,
            qos,
        )

        self._last_obstacle_state = 0
        self._last_static_mask = 0

    def _arm_callback(self, msg: Bool) -> None:
        global _vision_armed, _last_arm_state

        new_state = bool(msg.data)
        changed = (_last_arm_state is None) or (new_state != _last_arm_state)

        _vision_armed = new_state
        _last_arm_state = new_state

        if changed:
            print(f"\n[ARM CALLBACK] /vision/arm = {_vision_armed}", flush=True)

        if changed and _vision_armed:
            reset_toy_confirmation_state()
            print("[STAGE] NEW_ARM_WINDOW -> reset timers, ready for 8s + 4s toy confirmation", flush=True)

        if not _vision_armed:
            reset_toy_confirmation_state()

    def publish_confidence(self, conf: float) -> None:
        msg = Float32()
        msg.data = float(conf)
        self.toy_conf_pub.publish(msg)

    def publish_toy_detected(self, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        self.toy_detected_pub.publish(msg)

    def _obstacle_state_callback(self, msg: UInt8) -> None:
        state = int(msg.data)
        if self._last_obstacle_state == 0 and state != 0:
            trigger_buzzer_alert(f"obstacle_state={state}")
        self._last_obstacle_state = state

    def _static_mask_callback(self, msg: UInt8) -> None:
        mask = int(msg.data) & 0x07
        if self._last_static_mask == 0 and mask != 0:
            trigger_buzzer_alert(f"static_mask={mask:03b}")
        self._last_static_mask = mask


def init_buzzer() -> None:
    global _buzzer

    if OutputDevice is None:
        print(
            "[BUZZER] gpiozero is not available. Buzzer output disabled.",
            flush=True,
        )
        return

    if _buzzer is not None:
        return

    try:
        _buzzer = OutputDevice(BUZZER_GPIO_BCM, active_high=True, initial_value=False)
        print(
            f"[BUZZER] Ready on BCM GPIO{BUZZER_GPIO_BCM} "
            f"(physical pin 12).",
            flush=True,
        )
    except Exception as exc:
        _buzzer = None
        print(f"[BUZZER] GPIO init failed: {exc}. Buzzer output disabled.", flush=True)


def shutdown_buzzer() -> None:
    global _buzzer

    if _buzzer is None:
        return

    try:
        _buzzer.off()
        _buzzer.close()
    except Exception:
        pass

    _buzzer = None


def _buzzer_beep_worker(reason: str) -> None:
    global _buzzer

    if _buzzer is None:
        return

    print(f"[BUZZER] Alert: {reason}. Beeping {BUZZER_BEEP_COUNT} times.", flush=True)

    try:
        for beep_index in range(BUZZER_BEEP_COUNT):
            _buzzer.on()
            time.sleep(BUZZER_ON_SECONDS)
            _buzzer.off()
            if beep_index < BUZZER_BEEP_COUNT - 1:
                time.sleep(BUZZER_OFF_SECONDS)
    except Exception as exc:
        try:
            _buzzer.off()
        except Exception:
            pass
        print(f"[BUZZER] Beep failed: {exc}", flush=True)


def trigger_buzzer_alert(reason: str) -> None:
    global _buzzer_thread, _last_buzzer_trigger

    now = time.monotonic()

    with _buzzer_lock:
        if _buzzer is None:
            return

        if (now - _last_buzzer_trigger) < BUZZER_RETRIGGER_GUARD_SECONDS:
            return

        if _buzzer_thread is not None and _buzzer_thread.is_alive():
            return

        _last_buzzer_trigger = now
        _buzzer_thread = threading.Thread(
            target=_buzzer_beep_worker,
            args=(reason,),
            daemon=True,
        )
        _buzzer_thread.start()


def init_ros() -> None:
    global _ros_node, _ros_thread

    if _ros_node is not None:
        return

    rclpy.init(args=None)
    _ros_node = AutoToyRosNode()

    def spin_thread() -> None:
        rclpy.spin(_ros_node)

    _ros_thread = threading.Thread(target=spin_thread, daemon=True)
    _ros_thread.start()


def shutdown_ros() -> None:
    global _ros_node

    if _ros_node is not None:
        _ros_node.destroy_node()
        _ros_node = None

    if rclpy.ok():
        rclpy.shutdown()


def get_class_name(model, cls_id) -> str:
    names = getattr(model, "names", {})
    return str(names.get(int(cls_id), int(cls_id))).strip().lower()


def get_highest_toy_conf(result, model):
    highest_toy_conf = 0.0
    toy_boxes = 0
    all_detections = []

    if result is None or result.boxes is None:
        return highest_toy_conf, toy_boxes, all_detections

    for box in result.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        cls_name = get_class_name(model, cls_id)

        all_detections.append(f"{cls_name}:{conf:.2f}")

        if cls_name == TOY_CLASS_NAME:
            toy_boxes += 1
            highest_toy_conf = max(highest_toy_conf, conf)

    return highest_toy_conf, toy_boxes, all_detections


def publish_false(now: float) -> None:
    global _last_detected_publish

    if _ros_node is None:
        return

    if (now - _last_detected_publish) >= FALSE_PUBLISH_INTERVAL_SECONDS:
        _ros_node.publish_toy_detected(0.0)
        _last_detected_publish = now


def publish_true_burst(now: float, conf: float) -> None:
    global _last_detected_publish

    if _ros_node is None:
        return

    if (now - _last_detected_publish) >= TOY_EVENT_BURST_INTERVAL_SECONDS:
        _ros_node.publish_toy_detected(conf)
        _last_detected_publish = now
        print(f"[PUBLISH] /vision/toy_detected = {conf:.3f}  CONFIRMED TOY", flush=True)


def print_stage(
    stage: str,
    toy_conf: float,
    phase_elapsed: float,
    phase_required: float,
    toy_boxes: int,
    detections,
) -> None:
    global _last_print

    now = time.monotonic()
    if (now - _last_print) < PRINT_INTERVAL_SECONDS:
        return

    _last_print = now

    if detections:
        det_text = ", ".join(detections[:6])
    else:
        det_text = "none"

    print(
        f"[STAGE={stage}] "
        f"armed={_vision_armed} | "
        f"phase={_toy_phase} | "
        f"toy_conf={toy_conf:.3f} | "
        f"best={_best_toy_conf:.3f} | "
        f"timer={phase_elapsed:.2f}/{phase_required:.1f}s | "
        f"toy_boxes={toy_boxes} | "
        f"detections=[{det_text}]",
        flush=True,
    )


def update_ros_logic(toy_conf: float, toy_boxes: int, detections) -> None:
    global _toy_phase, _toy_seen_start, _best_toy_conf
    global _burst_active, _burst_end_time, _toy_sent_this_arm
    global _last_conf_publish

    now = time.monotonic()
    phase_elapsed = 0.0
    phase_required = TOY_FIRST_RECOGNITION_SECONDS
    stage = "STARTING"

    if _ros_node is not None and (now - _last_conf_publish) >= CONF_PUBLISH_INTERVAL_SECONDS:
        _ros_node.publish_confidence(toy_conf)
        _last_conf_publish = now

    if not _vision_armed:
        stage = "NOT_ARMED_PUBLISHING_FALSE"
        reset_toy_confirmation_state()
        publish_false(now)
        print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)
        return

    if _burst_active:
        if now < _burst_end_time:
            stage = "CONFIRMED_TOY_BURST"
            publish_true_burst(now, max(_best_toy_conf, toy_conf))
            phase_elapsed = TOY_FINAL_CONFIRM_SECONDS
            phase_required = TOY_FINAL_CONFIRM_SECONDS
            print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)
            return

        _burst_active = False
        _toy_sent_this_arm = True
        stage = "TOY_BURST_DONE_WAITING_ARM_OFF"
        publish_false(now)
        print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)
        return

    if _toy_sent_this_arm:
        stage = "TOY_ALREADY_SENT_WAITING_ARM_OFF"
        publish_false(now)
        print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)
        return

    if toy_conf >= TOY_CONF_THRESHOLD:
        if _toy_seen_start is None:
            _toy_seen_start = now
            _best_toy_conf = toy_conf
            stage = "FIRST_RECOGNITION_TIMER_STARTED"
        else:
            _best_toy_conf = max(_best_toy_conf, toy_conf)
            stage = "FIRST_RECOGNITION_COUNTING"

        phase_elapsed = now - _toy_seen_start

        if _toy_phase == "FIRST_RECOGNITION":
            phase_required = TOY_FIRST_RECOGNITION_SECONDS
            if phase_elapsed >= TOY_FIRST_RECOGNITION_SECONDS:
                _toy_phase = "FINAL_CONFIRMATION"
                _toy_seen_start = now
                phase_elapsed = 0.0
                phase_required = TOY_FINAL_CONFIRM_SECONDS
                stage = "FIRST_RECOGNITION_PASSED_FINAL_TIMER_STARTED"
            publish_false(now)
            print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)
            return

        phase_required = TOY_FINAL_CONFIRM_SECONDS
        stage = "FINAL_CONFIRMATION_COUNTING"

        if phase_elapsed >= TOY_FINAL_CONFIRM_SECONDS:
            _burst_active = True
            _burst_end_time = now + TOY_EVENT_BURST_SECONDS
            stage = "TOY_CONFIRMED_STARTING_BURST"
            print(
                f"\n[CONFIRMED] toy_conf stayed >= {TOY_CONF_THRESHOLD:.2f} "
                f"for {TOY_FIRST_RECOGNITION_SECONDS:.1f}s first recognition "
                f"+ {TOY_FINAL_CONFIRM_SECONDS:.1f}s final confirmation. "
                f"Starting burst for {TOY_EVENT_BURST_SECONDS:.1f}s.",
                flush=True,
            )
            publish_true_burst(now, _best_toy_conf)
        else:
            publish_false(now)

        print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)
        return

    if _toy_seen_start is not None:
        print(
            f"[RESET] confidence dropped below {TOY_CONF_THRESHOLD:.2f}: "
            f"toy_conf={toy_conf:.3f}. Returning to first 8s recognition.",
            flush=True,
        )

    reset_toy_confirmation_state()
    stage = "ARMED_SEARCHING_PUBLISHING_FALSE"
    publish_false(now)
    print_stage(stage, toy_conf, phase_elapsed, phase_required, toy_boxes, detections)


def main() -> None:
    if not MODEL_PATH.exists():
        print("ERROR: Model not found at:", MODEL_PATH.resolve(), flush=True)
        print("Put your YOLO best.pt file here:", MODEL_PATH, flush=True)
        return

    print("Loading YOLO model from:", MODEL_PATH, flush=True)
    model = YOLO(str(MODEL_PATH))
    print("Model loaded OK", flush=True)
    print("Model classes:", model.names, flush=True)

    print("Initialising ROS 2 node...", flush=True)
    init_buzzer()
    init_ros()
    print("ROS 2 node up.", flush=True)
    print("Sub : /vision/arm              std_msgs/Bool", flush=True)
    print("Sub : /robot/pid/obstacle_state std_msgs/UInt8  buzzer alert", flush=True)
    print("Sub : /robot/pid/static_mask    std_msgs/UInt8  buzzer alert", flush=True)
    print("Pub : /vision/toy_confidence   std_msgs/Float32  live confidence", flush=True)
    print("Pub : /vision/toy_detected     std_msgs/Float32  0.0=false, after 8s+4s confirmed toy", flush=True)

    print("Starting Pi camera...", flush=True)
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_preview_configuration(
            main={"format": "RGB888", "size": CAMERA_SIZE}
        )
    )
    picam2.start()
    time.sleep(2.0)
    print("Camera ready. Running headless debug node. Press Ctrl+C to quit.", flush=True)

    try:
        while True:
            frame_rgb = picam2.capture_array()

            results = model(
                frame_rgb,
                imgsz=YOLO_IMGSZ,
                conf=YOLO_CONF,
                verbose=False,
            )

            result = results[0]
            toy_conf, toy_boxes, detections = get_highest_toy_conf(result, model)

            update_ros_logic(toy_conf, toy_boxes, detections)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nCtrl+C received. Exiting...", flush=True)

    finally:
        print("Stopping camera and shutting down ROS...", flush=True)
        try:
            picam2.stop()
        except Exception:
            pass
        shutdown_buzzer()
        shutdown_ros()
        print("Done.", flush=True)


if __name__ == "__main__":
    main()
