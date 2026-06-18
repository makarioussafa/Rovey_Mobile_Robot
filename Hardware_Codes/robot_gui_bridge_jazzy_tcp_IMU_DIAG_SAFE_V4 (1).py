#!/usr/bin/env python3
# ============================================================
# Raspberry Pi ROS2 Jazzy GUI Bridge
#
# PC Tkinter GUI connects wirelessly to this Pi TCP server.
# This bridge translates simple GUI text commands into ROS2 topics,
# and streams semicolon STATUS lines back to the GUI.
#
# Run on Pi:
#   source /opt/ros/jazzy/setup.bash
#   export ROS_DOMAIN_ID=0
#   unset ROS_LOCALHOST_ONLY
#   export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
#   python3 robot_gui_bridge_jazzy_tcp.py
# ============================================================

import socket
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, UInt8, UInt16, Int32


TCP_HOST = "0.0.0.0"
TCP_PORT = 8765
STATUS_HZ = 10.0
LINK_TIMEOUT_S = 1.5


MODE_MANUAL = 0
MODE_AUTO = 1

PID_GUI_CMD_MODE = 1
PID_GUI_CMD_AUTO_START = 2
PID_GUI_CMD_AUTO_ABORT = 3
PID_GUI_CMD_MANUAL_DRIVE = 4
PID_GUI_CMD_CAL_IMU = 5
PID_GUI_CMD_RESET_ZERO = 6

DIRECTION_CODES = {
    "STOP": 0,
    "FWD": 1,
    "BACK": 2,
    "LEFT": 3,
    "RIGHT": 4,
    "FWD_LEFT": 5,
    "FWD_RIGHT": 6,
    "BACK_LEFT": 7,
    "BACK_RIGHT": 8,
    "ROT_L": 9,
    "ROT_R": 10,
}

MECHANISM_CODES = {
    "STOP": 0,
    "RUN": 1,
    "SERVO_OPEN": 2,
    "SERVO_CLOSE": 3,
    "STEP2_HOME": 4,
    "STEP2_FWD": 5,
    "STEP1_FWD": 6,
    "STEP1_RETURN": 7,
    "HOME_ALL": 8,
}

MECH_STATE_NAMES = {
    0: "IDLE",
    1: "RUNNING",
    2: "DONE",
    3: "ERROR",
}


class RobotGuiBridge(Node):
    def __init__(self):
        super().__init__("robot_gui_bridge_tcp")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pid_control_pub = self.create_publisher(UInt16, "/robot/cmd/pid_control", qos)
        self.mechanism_cmd_pub = self.create_publisher(UInt8, "/robot/cmd/mechanism", qos)

        self.create_subscription(UInt8, "/robot/sensors/mask", self._sensor_mask_cb, qos)
        self.create_subscription(UInt8, "/robot/sensors/decision", self._decision_cb, qos)
        self.create_subscription(UInt8, "/robot/mechanism/state", self._mechanism_state_cb, qos)
        self.create_subscription(Bool, "/vision/arm", self._vision_arm_cb, qos)

        self.create_subscription(UInt8, "/robot/pid/obstacle_state", self._obstacle_state_cb, qos)
        self.create_subscription(UInt8, "/robot/pid/path_step", self._path_step_cb, qos)
        self.create_subscription(UInt8, "/robot/pid/path_flags", self._path_flags_cb, qos)
        self.create_subscription(UInt8, "/robot/pid/static_mask", self._static_mask_cb, qos)
        self.create_subscription(Int32, "/robot/pid/imu_packet", self._imu_packet_cb, qos)

        self.lock = threading.Lock()
        self.tcp_clients: List[socket.socket] = []
        self.running = True

        self.sensor_mask = 0
        self.decision = 0
        self.mechanism_state = 0
        self.vision_armed = False

        self.obstacle_state = 0
        self.path_step = 0
        self.path_running = False
        self.path_finished = False
        self.path_aborted = False
        self.static_mask = 0

        self.manual_direction = 0
        self.manual_speed = 0
        self.imu_ok = False
        self.imu_ready_flag = False
        self.last_imu_rx = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.imu_temp = 0.0
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.avg_wheel_speed = 0.0
        self.imu_who = 0
        self.imu_init_ok = False
        self.imu_read_ok_count = 0
        self.imu_read_fail_count = 0
        self.imu_cal_valid = 0
        self.imu_age_ms = 0
        self.imu_raw_ax = 0
        self.imu_raw_ay = 0
        self.imu_raw_az = 0
        self.imu_raw_gx = 0
        self.imu_raw_gy = 0
        self.imu_raw_gz = 0
        self.imu_invalid_count = 0

        now = time.monotonic()
        self.last_sensor_rx = 0.0
        self.last_drive_rx = 0.0
        self.last_client_rx = now

        self.tcp_thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        self.tcp_thread.start()
        self.status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self.status_thread.start()

        self.get_logger().info(f"TCP GUI bridge listening on {TCP_HOST}:{TCP_PORT}")

    def _sensor_mask_cb(self, msg: UInt8):
        with self.lock:
            self.sensor_mask = int(msg.data) & 0x07
            self.last_sensor_rx = time.monotonic()

    def _decision_cb(self, msg: UInt8):
        with self.lock:
            self.decision = int(msg.data)
            self.last_sensor_rx = time.monotonic()

    def _mechanism_state_cb(self, msg: UInt8):
        with self.lock:
            self.mechanism_state = int(msg.data)
            self.last_sensor_rx = time.monotonic()

    def _vision_arm_cb(self, msg: Bool):
        with self.lock:
            self.vision_armed = bool(msg.data)
            self.last_sensor_rx = time.monotonic()

    def _obstacle_state_cb(self, msg: UInt8):
        with self.lock:
            self.obstacle_state = int(msg.data)
            self.last_drive_rx = time.monotonic()

    def _path_step_cb(self, msg: UInt8):
        with self.lock:
            self.path_step = int(msg.data)
            self.last_drive_rx = time.monotonic()

    def _path_flags_cb(self, msg: UInt8):
        flags = int(msg.data)
        with self.lock:
            self.path_running = bool(flags & 0x01)
            self.path_finished = bool(flags & 0x02)
            self.path_aborted = bool(flags & 0x04)
            self.last_drive_rx = time.monotonic()

    def _static_mask_cb(self, msg: UInt8):
        with self.lock:
            self.static_mask = int(msg.data) & 0x07
            self.last_drive_rx = time.monotonic()

    def _imu_packet_cb(self, msg: Int32):
        raw = int(msg.data)
        field = raw // 100000
        value = (raw % 100000) - 50000

        with self.lock:
            self.last_imu_rx = time.monotonic()
            if field == 1:
                self.yaw = value / 100.0
            elif field == 2:
                self.pitch = value / 100.0
            elif field == 3:
                self.roll = value / 100.0
            elif field == 4:
                self.imu_temp = value / 10.0
            elif field == 5:
                self.imu_ready_flag = bool(value & 0x01)
                self.imu_ok = self.imu_ready_flag
            elif field == 6:
                self.wheel_speeds[0] = float(value)
            elif field == 7:
                self.wheel_speeds[1] = float(value)
            elif field == 8:
                self.wheel_speeds[2] = float(value)
            elif field == 9:
                self.wheel_speeds[3] = float(value)
            elif field == 10:
                self.avg_wheel_speed = float(value)
            elif field == 11:
                self.imu_who = int(value)
            elif field == 12:
                self.imu_read_ok_count = int(value)
            elif field == 13:
                self.imu_read_fail_count = int(value)
            elif field == 14:
                self.imu_cal_valid = int(value)
            elif field == 15:
                self.imu_age_ms = int(value)
            elif field == 16:
                self.imu_init_ok = bool(value)
            elif field == 17:
                self.imu_raw_ax = int(value)
            elif field == 18:
                self.imu_raw_ay = int(value)
            elif field == 19:
                self.imu_raw_az = int(value)
            elif field == 20:
                self.imu_raw_gx = int(value)
            elif field == 21:
                self.imu_raw_gy = int(value)
            elif field == 22:
                self.imu_raw_gz = int(value)
            elif field == 23:
                self.imu_invalid_count = int(value)
            self.last_drive_rx = time.monotonic()

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def publish_u8(self, pub, value: int):
        msg = UInt8()
        msg.data = int(value) & 0xFF
        pub.publish(msg)

    def publish_u16(self, pub, value: int):
        msg = UInt16()
        msg.data = int(value) & 0xFFFF
        pub.publish(msg)

    def publish_pid_control(self, command: int, direction: int = 0, value: int = 0):
        packed = ((command & 0x0F) << 12) | ((direction & 0x0F) << 8) | (value & 0xFF)
        self.publish_u16(self.pid_control_pub, packed)

    def handle_command(self, line: str):
        line = line.strip()
        if not line:
            return

        with self.lock:
            self.last_client_rx = time.monotonic()

        if line == "PING":
            return

        if line == "AUTO:START":
            self.publish_pid_control(PID_GUI_CMD_AUTO_START)
            self.get_logger().info("GUI command: AUTO START")
            return

        if line == "AUTO:ABORT":
            self.publish_pid_control(PID_GUI_CMD_AUTO_ABORT)
            with self.lock:
                self.manual_direction = 0
                self.manual_speed = 0
            self.get_logger().info("GUI command: AUTO ABORT")
            return

        if line.startswith("MODE:"):
            mode_name = line.split(":", 1)[1].strip().upper()
            mode_value = MODE_AUTO if mode_name == "AUTO" else MODE_MANUAL
            self.publish_pid_control(PID_GUI_CMD_MODE, value=mode_value)
            self.get_logger().info(f"GUI command: MODE {mode_name}")
            return

        if line.startswith("DRIVE:"):
            parts = line.split(":")
            if len(parts) >= 2 and parts[1] == "CAL_IMU":
                self.publish_pid_control(PID_GUI_CMD_CAL_IMU)
                self.get_logger().info("GUI command: CALIBRATE IMU")
                return

            if len(parts) >= 3:
                direction = parts[1].strip().upper()
                try:
                    speed = max(0, min(100, int(parts[2])))
                except ValueError:
                    speed = 0

                code = DIRECTION_CODES.get(direction, 0)
                self.publish_pid_control(PID_GUI_CMD_MANUAL_DRIVE, direction=code, value=speed)
                with self.lock:
                    self.manual_direction = code
                    self.manual_speed = speed
                return

        if line.startswith("MECH:"):
            cmd = line.split(":", 1)[1].strip().upper()
            code = MECHANISM_CODES.get(cmd)
            if code is not None:
                self.publish_u8(self.mechanism_cmd_pub, code)
                self.get_logger().info(f"GUI command: MECH {cmd}")
            return

        if line == "RESET:ESP":
            self.publish_pid_control(PID_GUI_CMD_RESET_ZERO)
            self.publish_u8(self.mechanism_cmd_pub, MECHANISM_CODES["HOME_ALL"])
            with self.lock:
                self.manual_direction = 0
                self.manual_speed = 0
                self.path_running = False
                self.path_finished = False
                self.path_aborted = False
                self.path_step = 0
            self.get_logger().info("GUI command: RESET/ZERO PID state and HOME mechanism")
            return

        self.get_logger().warn(f"Unknown GUI command: {line}")

    def status_line(self) -> str:
        now = time.monotonic()
        with self.lock:
            sensor_ok = (now - self.last_sensor_rx) < LINK_TIMEOUT_S
            drive_ok = (now - self.last_drive_rx) < LINK_TIMEOUT_S
            imu_ok = self.imu_ready_flag and ((now - self.last_imu_rx) < LINK_TIMEOUT_S)
            mech_state = self.mechanism_state
            mech_busy = mech_state == 1
            manual_translation_requires_imu = self.manual_direction in {
                DIRECTION_CODES["FWD"],
                DIRECTION_CODES["BACK"],
                DIRECTION_CODES["LEFT"],
                DIRECTION_CODES["RIGHT"],
                DIRECTION_CODES["FWD_LEFT"],
                DIRECTION_CODES["FWD_RIGHT"],
                DIRECTION_CODES["BACK_LEFT"],
                DIRECTION_CODES["BACK_RIGHT"],
            }
            manual_motion_allowed = (
                self.manual_direction != 0 and
                not (manual_translation_requires_imu and not imu_ok)
            )
            blocked_reason = (
                "IMU_OFFLINE"
                if manual_translation_requires_imu and not imu_ok
                else "NONE"
            )

            fields = {
                "sensor": int(sensor_ok),
                "pi_sensor": int(sensor_ok),
                "pi_drive": int(drive_ok),
                "drive": int(drive_ok),
                "mask": self.sensor_mask,
                "s1": int(bool(self.sensor_mask & 0x01)),
                "s2": int(bool(self.sensor_mask & 0x02)),
                "s3": int(bool(self.sensor_mask & 0x04)),
                "decision": self.decision,
                "mech": MECH_STATE_NAMES.get(mech_state, "UNKNOWN"),
                "mech_busy": int(mech_busy),
                "cmd": self.manual_direction,
                "moving": int(manual_motion_allowed),
                "imu": int(imu_ok),
                "yaw": f"{self.yaw:.2f}",
                "pitch": f"{self.pitch:.2f}",
                "roll": f"{self.roll:.2f}",
                "imu_temp": f"{self.imu_temp:.1f}",
                "imu_who": self.imu_who,
                "imu_init": int(self.imu_init_ok),
                "imu_ok_count": self.imu_read_ok_count,
                "imu_fail_count": self.imu_read_fail_count,
                "imu_cal_valid": self.imu_cal_valid,
                "imu_age_ms": self.imu_age_ms,
                "imu_raw_ax": self.imu_raw_ax,
                "imu_raw_ay": self.imu_raw_ay,
                "imu_raw_az": self.imu_raw_az,
                "imu_raw_gx": self.imu_raw_gx,
                "imu_raw_gy": self.imu_raw_gy,
                "imu_raw_gz": self.imu_raw_gz,
                "imu_invalid_count": self.imu_invalid_count,
                "path_step": self.path_step,
                "path_running": int(self.path_running),
                "path_finished": int(self.path_finished),
                "path_aborted": int(self.path_aborted),
                "obstacle_state": self.obstacle_state,
                "static_mask": self.static_mask,
                "vision_armed": int(self.vision_armed),
                "avg_speed": f"{self.avg_wheel_speed:.1f}",
                "w1": f"{self.wheel_speeds[0]:.1f}",
                "w2": f"{self.wheel_speeds[1]:.1f}",
                "w3": f"{self.wheel_speeds[2]:.1f}",
                "w4": f"{self.wheel_speeds[3]:.1f}",
                "blocked": blocked_reason,
                "sensor_ip": "ROS2",
                "drive_ip": "ROS2",
            }

        return "STATUS;" + ";".join(f"{k}={v}" for k, v in fields.items()) + "\n"

    def broadcast_status(self):
        data = self.status_line().encode("utf-8")
        dead = []
        with self.lock:
            clients = list(self.tcp_clients)

        for client in clients:
            try:
                client.sendall(data)
            except OSError:
                dead.append(client)

        if dead:
            with self.lock:
                self.tcp_clients = [c for c in self.tcp_clients if c not in dead]
            for client in dead:
                try:
                    client.close()
                except OSError:
                    pass

    def _status_loop(self):
        period = 1.0 / STATUS_HZ
        while self.running:
            self.broadcast_status()
            time.sleep(period)

    def _tcp_server_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_HOST, TCP_PORT))
        server.listen(4)
        server.settimeout(0.5)

        while self.running:
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f"GUI client connected from {addr[0]}:{addr[1]}")
            client.settimeout(0.5)
            with self.lock:
                self.tcp_clients.append(client)
            threading.Thread(target=self._client_loop, args=(client, addr), daemon=True).start()

    def _client_loop(self, client: socket.socket, addr):
        buf = ""
        try:
            while self.running:
                try:
                    data = client.recv(1024)
                    if not data:
                        break
                    buf += data.decode("utf-8", errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self.handle_command(line)
                except socket.timeout:
                    continue
        finally:
            with self.lock:
                self.tcp_clients = [c for c in self.tcp_clients if c is not client]
            try:
                client.close()
            except OSError:
                pass
            self.get_logger().info(f"GUI client disconnected from {addr[0]}:{addr[1]}")


def main():
    rclpy.init()
    node = RobotGuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
