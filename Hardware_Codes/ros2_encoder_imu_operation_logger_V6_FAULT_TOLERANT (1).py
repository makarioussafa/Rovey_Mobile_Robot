#!/usr/bin/env python3
"""
ROS 2 encoder + IMU operation logger for the V6 fault-tolerant PID ESP32 firmware.

Subscribes:
  /robot/pid/imu_packet      std_msgs/Int32   existing IMU/wheel packet stream
  /robot/pid/encoder_packet  std_msgs/Int32   V6 encoder/command/fault packet stream

Run on the Raspberry Pi after sourcing ROS 2:
  source /opt/ros/jazzy/setup.bash
  python3 ros2_encoder_imu_operation_logger_V6_FAULT_TOLERANT.py

Optional:
  python3 ros2_encoder_imu_operation_logger_V6_FAULT_TOLERANT.py --csv run1.csv --rate 5
"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32


IMU_PACKET_SCALE = 100000
IMU_PACKET_OFFSET = 50000
ENCODER_PACKET_SCALE = 1000000
ENCODER_PACKET_OFFSET = 500000


IMU_FIELDS = {
    1: "yaw_cdeg",
    2: "pitch_cdeg",
    3: "roll_cdeg",
    4: "temp_d10",
    5: "imu_status",
    6: "wheel_fl_pct",
    7: "wheel_fr_pct",
    8: "wheel_bl_pct",
    9: "wheel_br_pct",
    10: "wheel_avg_pct",
    11: "who_am_i",
    12: "read_ok_mod",
    13: "read_fail_mod",
    14: "cal_valid",
    15: "last_ok_age_ms",
    16: "init_ok",
    17: "raw_ax",
    18: "raw_ay",
    19: "raw_az",
    20: "raw_gx",
    21: "raw_gy",
    22: "raw_gz",
    23: "read_invalid_mod",
}


ENCODER_FIELDS = {
    1: "ticks_fl",
    2: "ticks_fr",
    3: "ticks_bl",
    4: "ticks_br",
    5: "delta_fl",
    6: "delta_fr",
    7: "delta_bl",
    8: "delta_br",
    9: "forward_mm",
    10: "lateral_mm",
    11: "odom_x_mm",
    12: "odom_y_mm",
    13: "robot_mode",
    14: "gui_mode",
    15: "manual_dir",
    16: "manual_speed",
    17: "last_gui_command",
    18: "last_gui_direction",
    19: "last_gui_value",
    20: "last_gui_age_ms",
    21: "path_step",
    22: "obstacle_state",
    23: "path_flags",
    24: "segment_done_mm",
    25: "segment_remaining_mm",
    26: "target_vx_mms",
    27: "target_vy_mms",
    28: "target_wz_mrads",
    29: "profile_vx_mms",
    30: "profile_vy_mms",
    31: "profile_wz_mrads",
    32: "encoder_health_mask",
    33: "encoder_fault_mask",
    34: "reject_fl_mod",
    35: "reject_fr_mod",
    36: "reject_bl_mod",
    37: "reject_br_mod",
    38: "raw_ticks_fl",
    39: "raw_ticks_fr",
    40: "raw_ticks_bl",
    41: "raw_ticks_br",
    42: "fault_reason_fl",
    43: "fault_reason_fr",
    44: "fault_reason_bl",
    45: "fault_reason_br",
}


PID_COMMAND_NAMES = {
    0: "NONE",
    1: "MODE",
    2: "AUTO_START",
    3: "AUTO_ABORT",
    4: "MANUAL_DRIVE",
    5: "CAL_IMU",
    6: "RESET_ZERO",
}


DIRECTION_NAMES = {
    0: "STOP",
    1: "FWD",
    2: "BACK",
    3: "LEFT",
    4: "RIGHT",
    5: "FWD_LEFT",
    6: "FWD_RIGHT",
    7: "BACK_LEFT",
    8: "BACK_RIGHT",
    9: "ROT_L",
    10: "ROT_R",
}


ROBOT_MODE_NAMES = {
    0: "IDLE",
    1: "MOVE_DISTANCE",
    2: "STRAFE_DISTANCE",
    3: "TURN_ANGLE",
}


GUI_MODE_NAMES = {
    0: "MANUAL",
    1: "AUTO",
}


OBSTACLE_STATE_NAMES = {
    0: "CLEAR",
    1: "DETECTED_STOPPING",
    2: "WAITING",
    3: "DYNAMIC_CONTINUING",
    4: "AVOIDING",
    5: "ALL_BLOCKED",
}


FAULT_REASON_NAMES = {
    0: "NONE",
    1: "ZERO_CMD_NOISE",
    2: "IMPOSSIBLE_DELTA",
    3: "WRONG_SIGN",
}


CSV_COLUMNS = [
    "timestamp",
    "monotonic_s",
    "imu_age_s",
    "encoder_age_s",
    "imu_status",
    "imu_init",
    "imu_who_hex",
    "yaw_deg",
    "pitch_deg",
    "roll_deg",
    "imu_temp_c",
    "imu_ok_count",
    "imu_fail_count",
    "imu_invalid_count",
    "imu_last_ok_age_ms",
    "imu_cal_valid",
    "imu_raw_ax",
    "imu_raw_ay",
    "imu_raw_az",
    "imu_raw_gx",
    "imu_raw_gy",
    "imu_raw_gz",
    "wheel_fl_pct",
    "wheel_fr_pct",
    "wheel_bl_pct",
    "wheel_br_pct",
    "wheel_avg_pct",
    "ticks_fl",
    "ticks_fr",
    "ticks_bl",
    "ticks_br",
    "delta_fl",
    "delta_fr",
    "delta_bl",
    "delta_br",
    "forward_mm",
    "lateral_mm",
    "odom_x_mm",
    "odom_y_mm",
    "robot_mode",
    "robot_mode_name",
    "gui_mode",
    "gui_mode_name",
    "manual_dir",
    "manual_dir_name",
    "manual_speed",
    "last_gui_command",
    "last_gui_command_name",
    "last_gui_direction",
    "last_gui_direction_name",
    "last_gui_value",
    "last_gui_age_ms",
    "path_step",
    "obstacle_state",
    "obstacle_state_name",
    "path_flags",
    "path_running",
    "path_finished",
    "path_aborted",
    "segment_done_mm",
    "segment_remaining_mm",
    "target_vx_mms",
    "target_vy_mms",
    "target_wz_mrads",
    "profile_vx_mms",
    "profile_vy_mms",
    "profile_wz_mrads",
    "encoder_health_mask",
    "encoder_fault_mask",
    "encoder_fl_healthy",
    "encoder_fr_healthy",
    "encoder_bl_healthy",
    "encoder_br_healthy",
    "encoder_fl_fault",
    "encoder_fr_fault",
    "encoder_bl_fault",
    "encoder_br_fault",
    "reject_fl_mod",
    "reject_fr_mod",
    "reject_bl_mod",
    "reject_br_mod",
    "raw_ticks_fl",
    "raw_ticks_fr",
    "raw_ticks_bl",
    "raw_ticks_br",
    "fault_reason_fl",
    "fault_reason_fl_name",
    "fault_reason_fr",
    "fault_reason_fr_name",
    "fault_reason_bl",
    "fault_reason_bl_name",
    "fault_reason_br",
    "fault_reason_br_name",
]


def decode_packet(raw: int, scale: int, offset: int) -> tuple[int, int]:
    field = raw // scale
    value = (raw % scale) - offset
    return field, value


def field_value(values: dict[int, int], field: int, default=""):
    return values.get(field, default)


def scaled_value(values: dict[int, int], field: int, scale: float, default=""):
    if field not in values:
        return default
    return values[field] * scale


def packet_age(last_rx: float) -> str:
    if last_rx <= 0.0:
        return ""
    return f"{time.monotonic() - last_rx:.3f}"


def mask_bit(mask, bit: int):
    if mask == "":
        return ""
    return int(bool(int(mask) & (1 << bit)))


class EncoderImuOperationLogger(Node):
    def __init__(self, csv_path: Path, rate_hz: float, print_live: bool):
        super().__init__("encoder_imu_operation_logger")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.imu_values: dict[int, int] = {}
        self.encoder_values: dict[int, int] = {}
        self.last_imu_rx = 0.0
        self.last_encoder_rx = 0.0
        self.imu_packet_count = 0
        self.encoder_packet_count = 0
        self.row_count = 0
        self.last_print = time.monotonic()
        self.print_live = print_live

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()
        self.csv_file.flush()

        self.create_subscription(Int32, "/robot/pid/imu_packet", self.on_imu_packet, qos)
        self.create_subscription(Int32, "/robot/pid/encoder_packet", self.on_encoder_packet, qos)

        period = 1.0 / max(0.5, rate_hz)
        self.create_timer(period, self.write_snapshot)

        self.get_logger().info(f"Logging encoder + IMU operation data to: {csv_path}")
        self.get_logger().info("Waiting for /robot/pid/encoder_packet and /robot/pid/imu_packet ...")

    def on_imu_packet(self, msg: Int32):
        field, value = decode_packet(int(msg.data), IMU_PACKET_SCALE, IMU_PACKET_OFFSET)
        if field in IMU_FIELDS:
            self.imu_values[field] = value
            self.imu_packet_count += 1
            self.last_imu_rx = time.monotonic()

    def on_encoder_packet(self, msg: Int32):
        field, value = decode_packet(int(msg.data), ENCODER_PACKET_SCALE, ENCODER_PACKET_OFFSET)
        if field in ENCODER_FIELDS:
            self.encoder_values[field] = value
            self.encoder_packet_count += 1
            self.last_encoder_rx = time.monotonic()

    def make_row(self) -> dict[str, object]:
        imu = self.imu_values
        enc = self.encoder_values

        who = field_value(imu, 11, "")
        who_hex = "" if who == "" else f"0x{int(who):02X}"

        robot_mode = field_value(enc, 13, "")
        gui_mode = field_value(enc, 14, "")
        manual_dir = field_value(enc, 15, "")
        last_cmd = field_value(enc, 17, "")
        last_dir = field_value(enc, 18, "")
        obstacle_state = field_value(enc, 22, "")
        flags = field_value(enc, 23, 0)
        health_mask = field_value(enc, 32, "")
        fault_mask = field_value(enc, 33, "")
        fault_reason_fl = field_value(enc, 42, "")
        fault_reason_fr = field_value(enc, 43, "")
        fault_reason_bl = field_value(enc, 44, "")
        fault_reason_br = field_value(enc, 45, "")

        return {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "monotonic_s": f"{time.monotonic():.3f}",
            "imu_age_s": packet_age(self.last_imu_rx),
            "encoder_age_s": packet_age(self.last_encoder_rx),
            "imu_status": field_value(imu, 5, ""),
            "imu_init": field_value(imu, 16, ""),
            "imu_who_hex": who_hex,
            "yaw_deg": scaled_value(imu, 1, 0.01),
            "pitch_deg": scaled_value(imu, 2, 0.01),
            "roll_deg": scaled_value(imu, 3, 0.01),
            "imu_temp_c": scaled_value(imu, 4, 0.1),
            "imu_ok_count": field_value(imu, 12, ""),
            "imu_fail_count": field_value(imu, 13, ""),
            "imu_invalid_count": field_value(imu, 23, ""),
            "imu_last_ok_age_ms": field_value(imu, 15, ""),
            "imu_cal_valid": field_value(imu, 14, ""),
            "imu_raw_ax": field_value(imu, 17, ""),
            "imu_raw_ay": field_value(imu, 18, ""),
            "imu_raw_az": field_value(imu, 19, ""),
            "imu_raw_gx": field_value(imu, 20, ""),
            "imu_raw_gy": field_value(imu, 21, ""),
            "imu_raw_gz": field_value(imu, 22, ""),
            "wheel_fl_pct": field_value(imu, 6, ""),
            "wheel_fr_pct": field_value(imu, 7, ""),
            "wheel_bl_pct": field_value(imu, 8, ""),
            "wheel_br_pct": field_value(imu, 9, ""),
            "wheel_avg_pct": field_value(imu, 10, ""),
            "ticks_fl": field_value(enc, 1, ""),
            "ticks_fr": field_value(enc, 2, ""),
            "ticks_bl": field_value(enc, 3, ""),
            "ticks_br": field_value(enc, 4, ""),
            "delta_fl": field_value(enc, 5, ""),
            "delta_fr": field_value(enc, 6, ""),
            "delta_bl": field_value(enc, 7, ""),
            "delta_br": field_value(enc, 8, ""),
            "forward_mm": field_value(enc, 9, ""),
            "lateral_mm": field_value(enc, 10, ""),
            "odom_x_mm": field_value(enc, 11, ""),
            "odom_y_mm": field_value(enc, 12, ""),
            "robot_mode": robot_mode,
            "robot_mode_name": ROBOT_MODE_NAMES.get(robot_mode, ""),
            "gui_mode": gui_mode,
            "gui_mode_name": GUI_MODE_NAMES.get(gui_mode, ""),
            "manual_dir": manual_dir,
            "manual_dir_name": DIRECTION_NAMES.get(manual_dir, ""),
            "manual_speed": field_value(enc, 16, ""),
            "last_gui_command": last_cmd,
            "last_gui_command_name": PID_COMMAND_NAMES.get(last_cmd, ""),
            "last_gui_direction": last_dir,
            "last_gui_direction_name": DIRECTION_NAMES.get(last_dir, ""),
            "last_gui_value": field_value(enc, 19, ""),
            "last_gui_age_ms": field_value(enc, 20, ""),
            "path_step": field_value(enc, 21, ""),
            "obstacle_state": obstacle_state,
            "obstacle_state_name": OBSTACLE_STATE_NAMES.get(obstacle_state, ""),
            "path_flags": flags,
            "path_running": int(bool(flags & 0x01)),
            "path_finished": int(bool(flags & 0x02)),
            "path_aborted": int(bool(flags & 0x04)),
            "segment_done_mm": field_value(enc, 24, ""),
            "segment_remaining_mm": field_value(enc, 25, ""),
            "target_vx_mms": field_value(enc, 26, ""),
            "target_vy_mms": field_value(enc, 27, ""),
            "target_wz_mrads": field_value(enc, 28, ""),
            "profile_vx_mms": field_value(enc, 29, ""),
            "profile_vy_mms": field_value(enc, 30, ""),
            "profile_wz_mrads": field_value(enc, 31, ""),
            "encoder_health_mask": health_mask,
            "encoder_fault_mask": fault_mask,
            "encoder_fl_healthy": mask_bit(health_mask, 0),
            "encoder_fr_healthy": mask_bit(health_mask, 1),
            "encoder_bl_healthy": mask_bit(health_mask, 2),
            "encoder_br_healthy": mask_bit(health_mask, 3),
            "encoder_fl_fault": mask_bit(fault_mask, 0),
            "encoder_fr_fault": mask_bit(fault_mask, 1),
            "encoder_bl_fault": mask_bit(fault_mask, 2),
            "encoder_br_fault": mask_bit(fault_mask, 3),
            "reject_fl_mod": field_value(enc, 34, ""),
            "reject_fr_mod": field_value(enc, 35, ""),
            "reject_bl_mod": field_value(enc, 36, ""),
            "reject_br_mod": field_value(enc, 37, ""),
            "raw_ticks_fl": field_value(enc, 38, ""),
            "raw_ticks_fr": field_value(enc, 39, ""),
            "raw_ticks_bl": field_value(enc, 40, ""),
            "raw_ticks_br": field_value(enc, 41, ""),
            "fault_reason_fl": fault_reason_fl,
            "fault_reason_fl_name": FAULT_REASON_NAMES.get(fault_reason_fl, ""),
            "fault_reason_fr": fault_reason_fr,
            "fault_reason_fr_name": FAULT_REASON_NAMES.get(fault_reason_fr, ""),
            "fault_reason_bl": fault_reason_bl,
            "fault_reason_bl_name": FAULT_REASON_NAMES.get(fault_reason_bl, ""),
            "fault_reason_br": fault_reason_br,
            "fault_reason_br_name": FAULT_REASON_NAMES.get(fault_reason_br, ""),
        }

    def write_snapshot(self):
        if self.last_encoder_rx <= 0.0:
            self.print_waiting_if_needed()
            return

        row = self.make_row()
        self.writer.writerow(row)
        self.csv_file.flush()
        self.row_count += 1

        if self.print_live and (time.monotonic() - self.last_print) >= 1.0:
            self.last_print = time.monotonic()
            self.print_live_summary(row)

    def print_waiting_if_needed(self):
        if self.print_live and (time.monotonic() - self.last_print) >= 1.0:
            self.last_print = time.monotonic()
            print(
                "waiting for encoder packets... "
                f"imu_packets={self.imu_packet_count} encoder_packets={self.encoder_packet_count}",
                flush=True,
            )

    def print_live_summary(self, row: dict[str, object]):
        print(
            "rows={rows} mode={mode} cmd={cmd} dir={direction} speed={speed} "
            "ticks={fl}/{fr}/{bl}/{br} d={dfl}/{dfr}/{dbl}/{dbr} "
            "health=0x{health} fault=0x{fault} raw_fl={raw_fl} "
            "yaw={yaw} imu={imu} obstacle={obstacle}".format(
                rows=self.row_count,
                mode=row["gui_mode_name"],
                cmd=row["last_gui_command_name"],
                direction=row["manual_dir_name"],
                speed=row["manual_speed"],
                fl=row["ticks_fl"],
                fr=row["ticks_fr"],
                bl=row["ticks_bl"],
                br=row["ticks_br"],
                dfl=row["delta_fl"],
                dfr=row["delta_fr"],
                dbl=row["delta_bl"],
                dbr=row["delta_br"],
                health="" if row["encoder_health_mask"] == "" else f"{int(row['encoder_health_mask']):X}",
                fault="" if row["encoder_fault_mask"] == "" else f"{int(row['encoder_fault_mask']):X}",
                raw_fl=row["raw_ticks_fl"],
                yaw=row["yaw_deg"],
                imu=row["imu_status"],
                obstacle=row["obstacle_state_name"],
            ),
            flush=True,
        )

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()


def default_csv_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"encoder_imu_fault_tolerant_log_{stamp}.csv"


def parse_args():
    parser = argparse.ArgumentParser(description="Log PID ESP32 encoder + IMU operation data to CSV.")
    parser.add_argument("--csv", type=Path, default=default_csv_path(), help="CSV output path.")
    parser.add_argument("--rate", type=float, default=5.0, help="CSV rows per second.")
    parser.add_argument("--no-print", action="store_true", help="Disable live console summaries.")
    return parser.parse_args()


def main():
    args = parse_args()
    csv_path = args.csv.expanduser()
    rclpy.init()
    node = EncoderImuOperationLogger(csv_path, args.rate, not args.no_print)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"Log saved: {csv_path}", flush=True)


if __name__ == "__main__":
    main()
