#!/usr/bin/env python3

import math
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tf_transformations


# =========================
# Global variables
# =========================
yaw = 0.0
yaw_zero = None

roll = 0.0
pitch = 0.0

x = 0.0
y = 0.0

x_zero = None
y_zero = None

linear_vel = 0.0
angular_vel = 0.0

last_time = None

tof_distance = None
ir_left_distance = None
ir_right_distance = None

odom_received = False
imu_received = False
tof_received = False
ir_left_received = False
ir_right_received = False

clock_zero_handled = False

node = None
cmd_pub = None

current_waypoint = 0
mission_done = False
pause_start_time = None
mission_enabled = False

log_counter = 0

# Avoidance state machine
avoid_state = None
avoid_side = None
avoid_target_yaw = None
avoid_resume_yaw = None
avoid_start_x = None
avoid_start_y = None

# Obstacle classification state
obstacle_check_active = False
obstacle_check_start_time = None
obstacle_check_yaw_ref = None

# Dynamic obstacle control
dynamic_started = False
dynamic_done = False
gz_path = None

world_name = "pickup_room"
model_name = "dynamic_obstacle"

dynamic_x_start = 1.0
dynamic_x = dynamic_x_start
dynamic_y = -0.25
dynamic_z = 0.10

dynamic_x_target_forward = 0.0
dynamic_x_target_back = 1.0
dynamic_speed = 0.045

dynamic_state = "idle"   # idle -> move_to_zero -> wait -> return_to_start -> done
dynamic_wait_duration = 15.0
dynamic_wait_start_time = None

# Mechanism action clients
arm_client = None
gripper_client = None

# Mechanism sequence state
mechanism_step = 0
mechanism_busy = False
mechanism_done = False


# =========================
# Tunable parameters
# =========================
drive_tolerance = 0.08
angle_tolerance = math.radians(2)
goal_lateral_tolerance = 0.10
realign_angle_threshold = 0.08

min_linear_speed = 0.08
max_linear_speed = 0.20
max_rotate_speed = 0.40
straight_max_angular = 0.10

k_linear = 0.40
k_heading = 0.90
k_lateral = 0.45
k_rotate = 1.20

heading_deadband = 0.03
lateral_deadband = 0.03

tof_block_distance = 0.30
ir_block_distance = 0.15

avoid_turn_angle = math.pi / 2
avoid_shift_distance = 0.60
avoid_forward_speed = 0.08
avoid_shift_max_angular = 0.12
avoid_settle_distance = 0.20

obstacle_check_duration = 4.0

# =========================
# Mechanism poses
# =========================
ARM_HOME_ROT = 0.375
ARM_HOME_LIFT = 0.028

ARM_LEFT_ROT = -0.40
ARM_RIGHT_ROT = 1.50

ARM_DOWN = -0.076
ARM_UP_AFTER_PICK = -0.035

GRIPPER_OPEN_RIGHT = 0.0
GRIPPER_OPEN_LEFT = 0.0

GRIPPER_CLOSE_RIGHT = -0.35
GRIPPER_CLOSE_LEFT = -0.35


# =========================
# Waypoints array
# =========================
waypoints = [
    {
        "type": "drive_straight",
        "name": "go_to_purple_x_near_first_toy",
        "x": 1.05,
        "y": 0.00,
        "yaw_ref": 0.0,
    },
    {
        "type": "mechanism_sequence",
        "name": "pick_at_purple_x",
    },
    {
        "type": "drive_straight",
        "name": "go_to_blue_cube_line",
        "x": 3.45,
        "y": 0.00,
        "yaw_ref": 0.0,
    },
    {
        "type": "rotate_only",
        "name": "face_blue_cube",
        "yaw": -math.pi / 2,
    },
    {
        "type": "drive_straight",
        "name": "go_to_blue_cube",
        "x": 3.40,
        "y": 3.45,
        "yaw_ref": -math.pi / 2,
        "finish_on_forward_only": True,
    },
    {
        "type": "rotate_only",
        "name": "face_dynamic_obstacle",
        "yaw": -math.pi,
    },
    {
        "type": "drive_straight",
        "name": "go_after_dynamic_obstacle",
        "x": 1.00,
        "y": 3.60,
        "yaw_ref": -math.pi,
        "finish_on_forward_only": True,
    },
    {
        "type": "rotate_only",
        "name": "face_blue_cube",
        "yaw": -math.pi * 3 / 2,
    },
    {
        "type": "drive_straight",
        "name": "go_to_intial_position",
        "x": 0.70,
        "y": 0.60,
        "yaw_ref": -math.pi * 3 / 2,
        "finish_on_forward_only": True,
    },
    {
        "type": "rotate_only",
        "name": "rotate_to_intial_angle",
        "yaw": 0.0,
    },
]


# =========================
# Helper functions
# =========================
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def clean_min_range(msg):
    valid_ranges = [
        r for r in msg.ranges
        if math.isfinite(r) and msg.range_min < r < msg.range_max
    ]

    if not valid_ranges:
        return msg.range_max

    return min(valid_ranges)


def sensors_ready():
    return tof_received and ir_left_received and ir_right_received


def publish_cmd(linear_x, angular_z):
    cmd = TwistStamped()
    cmd.header.stamp = node.get_clock().now().to_msg()
    cmd.header.frame_id = "base_link"
    cmd.twist.linear.x = linear_x
    cmd.twist.angular.z = angular_z
    cmd_pub.publish(cmd)


def stop_robot():
    publish_cmd(0.0, 0.0)


def front_blocked():
    return tof_distance is not None and tof_distance < tof_block_distance


def left_blocked():
    return ir_left_distance is not None and ir_left_distance < ir_block_distance


def right_blocked():
    return ir_right_distance is not None and ir_right_distance < ir_block_distance


def any_sensor_blocked():
    return front_blocked() or left_blocked() or right_blocked()


def distance_from(start_x, start_y):
    return math.hypot(x - start_x, y - start_y)


def clear_avoidance():
    global avoid_state, avoid_side, avoid_target_yaw, avoid_resume_yaw
    global avoid_start_x, avoid_start_y

    avoid_state = None
    avoid_side = None
    avoid_target_yaw = None
    avoid_resume_yaw = None
    avoid_start_x = None
    avoid_start_y = None


def clear_obstacle_check():
    global obstacle_check_active, obstacle_check_start_time, obstacle_check_yaw_ref

    obstacle_check_active = False
    obstacle_check_start_time = None
    obstacle_check_yaw_ref = None


def start_obstacle_check(yaw_ref):
    global obstacle_check_active, obstacle_check_start_time, obstacle_check_yaw_ref

    obstacle_check_active = True
    obstacle_check_start_time = node.get_clock().now().nanoseconds / 1e9
    obstacle_check_yaw_ref = yaw_ref

    stop_robot()
    node.get_logger().info(
        f"Obstacle detected by sensor -> stop and classify for {obstacle_check_duration:.1f} sec"
    )
    node.get_logger().info(
        f"CHECK START | tof={tof_distance:.2f} left={ir_left_distance:.2f} right={ir_right_distance:.2f}"
    )


def handle_obstacle_check():
    global obstacle_check_active

    if not obstacle_check_active:
        return False

    stop_robot()

    now = node.get_clock().now().nanoseconds / 1e9
    elapsed = now - obstacle_check_start_time

    node.get_logger().info(
        f"CHECK RUNNING | elapsed={elapsed:.2f}/{obstacle_check_duration:.2f} sec | "
        f"tof={tof_distance:.2f} left={ir_left_distance:.2f} right={ir_right_distance:.2f}"
    )

    if elapsed < obstacle_check_duration:
        return True

    if any_sensor_blocked():
        node.get_logger().info(
            "CHECK RESULT -> obstacle still present -> STATIC obstacle -> start avoidance"
        )
        yaw_ref = obstacle_check_yaw_ref
        clear_obstacle_check()
        start_avoidance(yaw_ref)
        return False

    node.get_logger().info(
        "CHECK RESULT -> obstacle cleared -> DYNAMIC obstacle -> resume straight motion"
    )
    clear_obstacle_check()
    return False


def mechanism_reset():
    global mechanism_step, mechanism_busy, mechanism_done
    mechanism_step = 0
    mechanism_busy = False
    mechanism_done = False


def choose_avoid_side():
    left_is_blocked = left_blocked()
    right_is_blocked = right_blocked()

    left_space = ir_left_distance if ir_left_distance is not None else 0.30
    right_space = ir_right_distance if ir_right_distance is not None else 0.30

    if left_is_blocked and not right_is_blocked:
        return "right"

    if right_is_blocked and not left_is_blocked:
        return "left"

    if left_space > right_space:
        return "left"

    return "right"


def start_avoidance(yaw_ref):
    global avoid_state, avoid_side, avoid_target_yaw, avoid_resume_yaw
    global avoid_start_x, avoid_start_y

    avoid_side = choose_avoid_side()

    if avoid_side == "left":
        avoid_target_yaw = normalize_angle(yaw_ref + avoid_turn_angle)
    else:
        avoid_target_yaw = normalize_angle(yaw_ref - avoid_turn_angle)

    avoid_resume_yaw = yaw_ref
    avoid_start_x = x
    avoid_start_y = y
    avoid_state = "turn_away"

    node.get_logger().info(
        f"Avoidance start | side={avoid_side} | "
        f"tof={tof_distance:.2f} left={ir_left_distance:.2f} right={ir_right_distance:.2f}"
    )


def handle_avoidance():
    global avoid_state, avoid_start_x, avoid_start_y

    if avoid_state is None:
        return False

    if avoid_state == "turn_away":
        done = rotate_to_yaw(avoid_target_yaw)
        if done:
            avoid_state = "shift_side"
            avoid_start_x = x
            avoid_start_y = y
            node.get_logger().info("Avoidance stage -> shift_side")
        return True

    if avoid_state == "shift_side":
        heading_error = normalize_angle(avoid_target_yaw - yaw)
        angular_cmd = clamp(
            k_rotate * heading_error,
            -avoid_shift_max_angular,
            avoid_shift_max_angular,
        )
        publish_cmd(avoid_forward_speed, angular_cmd)

        if distance_from(avoid_start_x, avoid_start_y) >= avoid_shift_distance:
            stop_robot()
            avoid_state = "turn_back"
            node.get_logger().info("Avoidance stage -> turn_back")
        return True

    if avoid_state == "turn_back":
        done = rotate_to_yaw(avoid_resume_yaw)
        if done:
            avoid_state = "settle_forward"
            avoid_start_x = x
            avoid_start_y = y
            node.get_logger().info("Avoidance stage -> settle_forward")
        return True

    if avoid_state == "settle_forward":
        heading_error = normalize_angle(avoid_resume_yaw - yaw)
        angular_cmd = clamp(
            k_rotate * heading_error,
            -avoid_shift_max_angular,
            avoid_shift_max_angular,
        )
        publish_cmd(avoid_forward_speed, angular_cmd)

        if distance_from(avoid_start_x, avoid_start_y) >= avoid_settle_distance:
            stop_robot()
            clear_avoidance()
            node.get_logger().info("Avoidance finished -> resume waypoint tracking")
        return True

    clear_avoidance()
    stop_robot()
    return True


def send_arm_goal(rot_value, prism_value, duration_sec=2.0):
    global mechanism_busy

    if arm_client is None:
        node.get_logger().error("arm_client is not initialized")
        return

    if not arm_client.wait_for_server(timeout_sec=2.0):
        node.get_logger().error("arm_controller action server not available")
        mechanism_busy = False
        return

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = [
        "link_1_of_mechanism_joint",
        "link_2_of_mechanism_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = [rot_value, prism_value]
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

    goal_msg.trajectory.points = [point]

    mechanism_busy = True
    future = arm_client.send_goal_async(goal_msg)
    future.add_done_callback(arm_goal_response_callback)


def send_gripper_goal(right_value, left_value, duration_sec=1.5):
    global mechanism_busy

    if gripper_client is None:
        node.get_logger().error("gripper_client is not initialized")
        return

    if not gripper_client.wait_for_server(timeout_sec=2.0):
        node.get_logger().error("gripper_controller action server not available")
        mechanism_busy = False
        return

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = [
        "right_grip_joint",
        "left_grip_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = [right_value, left_value]
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

    goal_msg.trajectory.points = [point]

    mechanism_busy = True
    future = gripper_client.send_goal_async(goal_msg)
    future.add_done_callback(gripper_goal_response_callback)


def arm_goal_response_callback(future):
    global mechanism_busy

    try:
        goal_handle = future.result()
    except Exception as exc:
        mechanism_busy = False
        node.get_logger().error(f"Arm goal send failed: {exc}")
        return

    if not goal_handle.accepted:
        mechanism_busy = False
        node.get_logger().error("Arm goal rejected")
        return

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(action_result_callback)


def gripper_goal_response_callback(future):
    global mechanism_busy

    try:
        goal_handle = future.result()
    except Exception as exc:
        mechanism_busy = False
        node.get_logger().error(f"Gripper goal send failed: {exc}")
        return

    if not goal_handle.accepted:
        mechanism_busy = False
        node.get_logger().error("Gripper goal rejected")
        return

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(action_result_callback)


def action_result_callback(future):
    global mechanism_busy
    mechanism_busy = False

    try:
        _ = future.result()
        node.get_logger().info("Mechanism action step completed")
    except Exception as exc:
        node.get_logger().error(f"Mechanism action failed: {exc}")


def handle_mechanism_sequence():
    global mechanism_step, mechanism_done, mechanism_busy

    if mechanism_done:
        return True

    if mechanism_busy:
        return False

    if mechanism_step == 0:
        node.get_logger().info("Mechanism step 0 -> rotate left")
        send_arm_goal(ARM_LEFT_ROT, ARM_HOME_LIFT, 2.0)
        mechanism_step += 1
        return False

    if mechanism_step == 1:
        node.get_logger().info("Mechanism step 1 -> prismatic down")
        send_arm_goal(ARM_LEFT_ROT, ARM_DOWN, 2.0)
        mechanism_step += 1
        return False

    if mechanism_step == 2:
        node.get_logger().info("Mechanism step 2 -> gripper close")
        send_gripper_goal(GRIPPER_CLOSE_RIGHT, GRIPPER_CLOSE_LEFT, 1.5)
        mechanism_step += 1
        return False

    if mechanism_step == 3:
        node.get_logger().info("Mechanism step 3 -> rotate right")
        send_arm_goal(ARM_RIGHT_ROT, ARM_DOWN, 2.0)
        mechanism_step += 1
        return False

    if mechanism_step == 4:
        node.get_logger().info("Mechanism step 4 -> prismatic up")
        send_arm_goal(ARM_RIGHT_ROT, ARM_UP_AFTER_PICK, 2.0)
        mechanism_step += 1
        return False

    if mechanism_step == 5:
        node.get_logger().info("Mechanism step 5 -> gripper open")
        send_gripper_goal(GRIPPER_OPEN_RIGHT, GRIPPER_OPEN_LEFT, 1.5)
        mechanism_step += 1
        return False

    if mechanism_step == 6:
        node.get_logger().info("Mechanism step 6 -> return home")
        send_arm_goal(ARM_HOME_ROT, ARM_HOME_LIFT, 2.0)
        mechanism_step += 1
        return False

    mechanism_done = True
    node.get_logger().info("Mechanism sequence finished")
    return True


def advance_waypoint():
    global current_waypoint, mission_done, pause_start_time

    current_waypoint += 1
    pause_start_time = None
    clear_avoidance()
    clear_obstacle_check()
    mechanism_reset()

    if current_waypoint >= len(waypoints):
        mission_done = True
        stop_robot()
        node.get_logger().info("Mission complete")


def drive_straight_to(target_x, target_y, yaw_ref, finish_on_forward_only=False):
    error_x = target_x - x
    error_y = target_y - y

    forward_axis_x = math.cos(yaw_ref)
    forward_axis_y = -math.sin(yaw_ref)

    left_axis_x = math.sin(yaw_ref)
    left_axis_y = math.cos(yaw_ref)

    forward_error = (error_x * forward_axis_x) + (error_y * forward_axis_y)
    lateral_error = (error_x * left_axis_x) + (error_y * left_axis_y)

    if finish_on_forward_only:
        if forward_error <= drive_tolerance:
            stop_robot()
            return True
    else:
        if forward_error <= drive_tolerance and abs(lateral_error) <= goal_lateral_tolerance:
            stop_robot()
            return True

    heading_error = normalize_angle(yaw_ref - yaw)

    heading_error = deadband(heading_error, heading_deadband)
    lateral_error = deadband(lateral_error, lateral_deadband)

    if abs(heading_error) > realign_angle_threshold:
        angular_cmd = clamp(k_rotate * heading_error, -max_rotate_speed, max_rotate_speed)
        publish_cmd(0.0, angular_cmd)
        return False

    if forward_error <= 0.0:
        linear_cmd = 0.0
    else:
        linear_cmd = clamp(k_linear * forward_error, min_linear_speed, max_linear_speed)

    angular_cmd = clamp(
        (k_heading * heading_error) + (k_lateral * lateral_error),
        -straight_max_angular,
        straight_max_angular,
    )

    publish_cmd(linear_cmd, angular_cmd)
    return False


def rotate_to_yaw(target_yaw):
    yaw_error = normalize_angle(target_yaw - yaw)

    if abs(yaw_error) <= angle_tolerance:
        stop_robot()
        return True

    angular_cmd = clamp(k_rotate * yaw_error, -max_rotate_speed, max_rotate_speed)
    publish_cmd(0.0, angular_cmd)
    return False


# =========================
# Dynamic obstacle functions
# =========================
def set_model_pose(model_x, model_y, model_z):
    req = (
        f'name: "{model_name}", '
        f'position: {{x: {model_x}, y: {model_y}, z: {model_z}}}, '
        f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    )

    cmd = [
        gz_path,
        'service',
        '-s', f'/world/{world_name}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000',
        '--req', req,
    ]

    subprocess.run(cmd, capture_output=True, text=True, check=False)


def start_dynamic_obstacle():
    global dynamic_started, dynamic_state, dynamic_done, dynamic_wait_start_time
    global dynamic_x

    if dynamic_started:
        return

    dynamic_started = True
    dynamic_done = False
    dynamic_state = "move_to_zero"
    dynamic_wait_start_time = None
    dynamic_x = dynamic_x_start

    node.get_logger().info("Dynamic obstacle sequence started")


def maybe_start_dynamic_obstacle_for_current_waypoint():
    if current_waypoint >= len(waypoints):
        return

    wp = waypoints[current_waypoint]

    if wp["name"] == "go_after_dynamic_obstacle" and not dynamic_started:
        start_dynamic_obstacle()


def update_dynamic_obstacle():
    global dynamic_x, dynamic_state, dynamic_wait_start_time, dynamic_done

    if not mission_enabled:
        return

    if dynamic_state == "idle" or dynamic_state == "done":
        return

    if dynamic_state == "move_to_zero":
        if dynamic_x > dynamic_x_target_forward:
            dynamic_x -= dynamic_speed

            if dynamic_x <= dynamic_x_target_forward:
                dynamic_x = dynamic_x_target_forward
                set_model_pose(dynamic_x, dynamic_y, dynamic_z)
                dynamic_wait_start_time = node.get_clock().now().nanoseconds / 1e9
                dynamic_state = "wait"
                node.get_logger().info("Dynamic obstacle reached x=0.0, waiting")
                return

            set_model_pose(dynamic_x, dynamic_y, dynamic_z)
        else:
            dynamic_x = dynamic_x_target_forward
            set_model_pose(dynamic_x, dynamic_y, dynamic_z)
            dynamic_wait_start_time = node.get_clock().now().nanoseconds / 1e9
            dynamic_state = "wait"
            node.get_logger().info("Dynamic obstacle reached x=0.0, waiting")
        return

    if dynamic_state == "wait":
        now = node.get_clock().now().nanoseconds / 1e9
        if now - dynamic_wait_start_time >= dynamic_wait_duration:
            dynamic_state = "return_to_start"
            node.get_logger().info("Dynamic obstacle returning to x=1.0")
        return

    if dynamic_state == "return_to_start":
        if dynamic_x < dynamic_x_target_back:
            dynamic_x += dynamic_speed

            if dynamic_x >= dynamic_x_target_back:
                dynamic_x = dynamic_x_target_back
                set_model_pose(dynamic_x, dynamic_y, dynamic_z)
                node.get_logger().info("Dynamic obstacle returned to start and stopped")
                dynamic_state = "done"
                dynamic_done = True
                return

            set_model_pose(dynamic_x, dynamic_y, dynamic_z)
        else:
            dynamic_x = dynamic_x_target_back
            set_model_pose(dynamic_x, dynamic_y, dynamic_z)
            node.get_logger().info("Dynamic obstacle returned to start and stopped")
            dynamic_state = "done"
            dynamic_done = True
        return


# =========================
# Callbacks
# =========================
def imu_callback(msg):
    global yaw, yaw_zero, roll, pitch, imu_received

    q = msg.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    roll_val, pitch_val, raw_yaw = tf_transformations.euler_from_quaternion(quaternion)

    roll = roll_val
    pitch = pitch_val

    if yaw_zero is None:
        yaw_zero = raw_yaw

    yaw = normalize_angle(raw_yaw - yaw_zero)
    imu_received = True


def odom_callback(msg):
    global x, y, x_zero, y_zero
    global linear_vel, angular_vel, odom_received

    raw_x = msg.pose.pose.position.x
    raw_y = msg.pose.pose.position.y

    if x_zero is None:
        x_zero = raw_x
    if y_zero is None:
        y_zero = raw_y

    x = raw_x - x_zero
    y = raw_y - y_zero

    linear_vel = msg.twist.twist.linear.x
    angular_vel = msg.twist.twist.angular.z
    odom_received = True


def tof_callback(msg):
    global tof_distance, tof_received
    tof_distance = clean_min_range(msg)
    tof_received = True


def ir_left_callback(msg):
    global ir_left_distance, ir_left_received
    ir_left_distance = clean_min_range(msg)
    ir_left_received = True


def ir_right_callback(msg):
    global ir_right_distance, ir_right_received
    ir_right_distance = clean_min_range(msg)
    ir_right_received = True


def mission_start_callback(msg):
    global mission_enabled

    if msg.data:
        mission_enabled = True
        node.get_logger().info("Mission START command received")


def mission_stop_callback(msg):
    global mission_enabled

    if msg.data:
        mission_enabled = False
        stop_robot()
        node.get_logger().info("Mission STOP command received")


def clock_callback(msg):
    global x, y, yaw, yaw_zero, x_zero, y_zero, last_time
    global current_waypoint, mission_done, pause_start_time
    global clock_zero_handled, log_counter
    global mission_enabled
    global dynamic_started, dynamic_done, dynamic_state, dynamic_wait_start_time, dynamic_x
    global obstacle_check_active, obstacle_check_start_time, obstacle_check_yaw_ref

    sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    if sim_time < 0.05 and not clock_zero_handled:
        x = 0.0
        y = 0.0
        yaw = 0.0

        yaw_zero = None
        x_zero = None
        y_zero = None

        last_time = None

        current_waypoint = 0
        mission_done = False
        pause_start_time = None
        mission_enabled = False

        log_counter = 0
        clear_avoidance()
        clear_obstacle_check()
        mechanism_reset()

        dynamic_started = False
        dynamic_done = False
        dynamic_state = "idle"
        dynamic_wait_start_time = None
        dynamic_x = dynamic_x_start

        clock_zero_handled = True

    elif sim_time >= 0.05:
        clock_zero_handled = False


# =========================
# Control loop
# =========================
def control_loop():
    global log_counter

    update_dynamic_obstacle()

    if not mission_enabled:
        stop_robot()
        return

    if not odom_received or not imu_received:
        stop_robot()
        return

    if not sensors_ready():
        stop_robot()
        return

    if mission_done:
        stop_robot()
        return

    if current_waypoint >= len(waypoints):
        stop_robot()
        return

    wp = waypoints[current_waypoint]

    maybe_start_dynamic_obstacle_for_current_waypoint()

    log_counter += 1
    if log_counter % 10 == 0:
        node.get_logger().info(
            f"wp={current_waypoint + 1}/{len(waypoints)} {wp['name']} | "
            f"pose=(x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg) | "
            f"tof={tof_distance:.2f} left={ir_left_distance:.2f} right={ir_right_distance:.2f} | "
            f"avoid_state={avoid_state} | mission_enabled={mission_enabled} | dynamic_state={dynamic_state} | "
            f"check_active={obstacle_check_active}"
        )

    if wp["type"] == "mechanism_sequence":
        stop_robot()
        done = handle_mechanism_sequence()
        if done:
            advance_waypoint()
        return

    if wp["type"] == "drive_straight":
        if avoid_state is not None:
            handle_avoidance()
            return

        if obstacle_check_active:
            still_checking = handle_obstacle_check()
            if still_checking:
                return

            if avoid_state is not None:
                handle_avoidance()
                return

        if any_sensor_blocked():
            start_obstacle_check(wp["yaw_ref"])
            return

        done = drive_straight_to(
            wp["x"],
            wp["y"],
            wp["yaw_ref"],
            wp.get("finish_on_forward_only", False),
        )
        if done:
            advance_waypoint()
        return

    if wp["type"] == "rotate_only":
        done = rotate_to_yaw(wp["yaw"])
        if done:
            advance_waypoint()
        return

    stop_robot()
    node.get_logger().error(f"Unknown waypoint type: {wp['type']}")
    advance_waypoint()


# =========================
# Main
# =========================
def main():
    global node, cmd_pub, last_time, gz_path
    global arm_client, gripper_client

    rclpy.init()

    node = Node("waypoints_control")

    arm_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory"
    )

    gripper_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/gripper_controller/follow_joint_trajectory"
    )

    gz_path = shutil.which("gz")
    if gz_path is None:
        node.get_logger().error("gz command not found in PATH")
        rclpy.shutdown()
        return

    cmd_pub = node.create_publisher(
        TwistStamped,
        "/diff_drive_controller/cmd_vel",
        10,
    )

    node.create_subscription(Imu, "/imu", imu_callback, 10)
    node.create_subscription(Odometry, "/diff_drive_controller/odom", odom_callback, 10)
    node.create_subscription(LaserScan, "/tof_scan", tof_callback, 10)
    node.create_subscription(LaserScan, "/ir_left_scan", ir_left_callback, 10)
    node.create_subscription(LaserScan, "/ir_right_scan", ir_right_callback, 10)
    node.create_subscription(Clock, "/clock", clock_callback, 10)

    node.create_subscription(Bool, "/mission_start", mission_start_callback, 10)
    node.create_subscription(Bool, "/mission_stop", mission_stop_callback, 10)

    node.create_timer(0.05, control_loop)

    last_time = None

    node.get_logger().info("Waypoint control node started")
    node.get_logger().info("Obstacle classification enabled: stop -> check -> dynamic or static")
    node.get_logger().info("Dynamic obstacle starts automatically at go_after_dynamic_obstacle")
    node.get_logger().info("Send START with: ros2 topic pub --once /mission_start std_msgs/msg/Bool '{data: true}'")
    node.get_logger().info("Send STOP  with: ros2 topic pub --once /mission_stop std_msgs/msg/Bool '{data: true}'")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()