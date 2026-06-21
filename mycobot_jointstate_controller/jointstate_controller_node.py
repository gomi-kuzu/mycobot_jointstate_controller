import math
import threading
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Int32, String

from pymycobot.mycobot280 import MyCobot280


class MyCobotJointStateController(Node):
    def __init__(self) -> None:
        super().__init__('mycobot_jointstate_controller')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('debug', False)
        self.declare_parameter('command_topic', '/mycobot/joint_commands')
        self.declare_parameter('state_topic', '/mycobot/joint_states')
        self.declare_parameter('ee_pose_topic', '/mycobot/ee_pose')
        self.declare_parameter('joint_state_source', 'device')
        self.declare_parameter('state_publish_rate_hz', 10.0)
        self.declare_parameter('ee_pose_publish_rate_hz', 5.0)
        self.declare_parameter('enable_ee_pose_publish', True)
        self.declare_parameter('base_frame_id', 'mycobot_base')
        self.declare_parameter('ee_frame_id', 'mycobot_ee')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('command_speed', 30)
        self.declare_parameter('command_rate_hz', 30.0)
        self.declare_parameter('position_unit', 'rad')
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('joint_angle_min_deg', [-165.0, -135.0, -150.0, -145.0, -165.0, -180.0])
        self.declare_parameter('joint_angle_max_deg', [165.0, 135.0, 150.0, 145.0, 165.0, 180.0])
        self.declare_parameter('send_only_on_change', False)
        self.declare_parameter('change_threshold_deg', 0.5)
        self.declare_parameter('gripper_command_topic', '/mycobot/gripper/command')
        self.declare_parameter('gripper_value_topic', '/mycobot/gripper/value')
        self.declare_parameter('gripper_calibration_topic', '/mycobot/gripper/calibrate')
        self.declare_parameter('gripper_speed', 30)
        self.declare_parameter('gripper_value_min', 0)
        self.declare_parameter('gripper_value_max', 100)
        self.declare_parameter('profile_io_timing', False)
        self.declare_parameter('io_timing_log_every_n', 120)
        self.declare_parameter('io_timing_outlier_ms', 20.0)

        self._port = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baud').value)
        self._debug = bool(self.get_parameter('debug').value)
        self._command_topic = str(self.get_parameter('command_topic').value)
        self._state_topic = str(self.get_parameter('state_topic').value)
        self._ee_pose_topic = str(self.get_parameter('ee_pose_topic').value)
        self._joint_state_source = str(self.get_parameter('joint_state_source').value).strip().lower()
        self._state_publish_rate_hz = float(self.get_parameter('state_publish_rate_hz').value)
        self._ee_pose_publish_rate_hz = float(self.get_parameter('ee_pose_publish_rate_hz').value)
        self._enable_ee_pose_publish = bool(self.get_parameter('enable_ee_pose_publish').value)
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)
        self._ee_frame_id = str(self.get_parameter('ee_frame_id').value)
        self._queue_size = int(self.get_parameter('queue_size').value)
        self._command_speed = int(self.get_parameter('command_speed').value)
        self._command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self._position_unit = str(self.get_parameter('position_unit').value).strip().lower()
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._joint_angle_min_deg = [float(v) for v in self.get_parameter('joint_angle_min_deg').value]
        self._joint_angle_max_deg = [float(v) for v in self.get_parameter('joint_angle_max_deg').value]
        self._send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self._change_threshold_deg = float(self.get_parameter('change_threshold_deg').value)
        self._gripper_command_topic = str(self.get_parameter('gripper_command_topic').value)
        self._gripper_value_topic = str(self.get_parameter('gripper_value_topic').value)
        self._gripper_calibration_topic = str(self.get_parameter('gripper_calibration_topic').value)
        self._gripper_speed = int(self.get_parameter('gripper_speed').value)
        self._gripper_value_min = int(self.get_parameter('gripper_value_min').value)
        self._gripper_value_max = int(self.get_parameter('gripper_value_max').value)
        self._profile_io_timing = bool(self.get_parameter('profile_io_timing').value)
        self._io_timing_log_every_n = int(self.get_parameter('io_timing_log_every_n').value)
        self._io_timing_outlier_ms = float(self.get_parameter('io_timing_outlier_ms').value)

        if len(self._joint_names) != 6:
            raise ValueError('joint_names must contain exactly 6 names.')
        if len(self._joint_angle_min_deg) != 6 or len(self._joint_angle_max_deg) != 6:
            raise ValueError('joint_angle_min_deg and joint_angle_max_deg must contain exactly 6 values.')
        if self._position_unit not in ('rad', 'deg'):
            raise ValueError("position_unit must be either 'rad' or 'deg'.")
        if self._joint_state_source not in ('device', 'command_echo'):
            raise ValueError("joint_state_source must be either 'device' or 'command_echo'.")
        if self._gripper_value_min >= self._gripper_value_max:
            raise ValueError('gripper_value_min must be smaller than gripper_value_max.')
        if self._io_timing_log_every_n <= 0:
            raise ValueError('io_timing_log_every_n must be greater than 0.')
        if self._io_timing_outlier_ms < 0.0:
            raise ValueError('io_timing_outlier_ms must be >= 0.0.')

        self._joint_index_by_name: Dict[str, int] = {
            name: idx for idx, name in enumerate(self._joint_names)
        }

        self._lock = threading.Lock()
        self._mc_lock = threading.Lock()
        self._command_event = threading.Event()
        self._running = True
        self._target_angles_deg: Optional[List[float]] = None
        self._last_sent_angles_deg: Optional[List[float]] = None
        self._command_min_interval_sec = 1.0 / max(self._command_rate_hz, 0.1)
        self._last_send_monotonic = 0.0
        self._io_timing = {
            'send_angles': {'count': 0, 'sum_ms': 0.0, 'min_ms': float('inf'), 'max_ms': 0.0},
            'get_angles': {'count': 0, 'sum_ms': 0.0, 'min_ms': float('inf'), 'max_ms': 0.0},
            'get_coords': {'count': 0, 'sum_ms': 0.0, 'min_ms': float('inf'), 'max_ms': 0.0},
        }

        self._mc = self._connect_mycobot()
        self._seed_last_sent_from_robot()

        self._sub = self.create_subscription(
            JointState,
            self._command_topic,
            self._on_joint_state,
            self._queue_size,
        )
        self._gripper_command_sub = self.create_subscription(
            String,
            self._gripper_command_topic,
            self._on_gripper_command,
            self._queue_size,
        )
        self._gripper_value_sub = self.create_subscription(
            Int32,
            self._gripper_value_topic,
            self._on_gripper_value,
            self._queue_size,
        )
        self._gripper_calibration_sub = self.create_subscription(
            Empty,
            self._gripper_calibration_topic,
            self._on_gripper_calibration,
            self._queue_size,
        )

        self._joint_state_pub = self.create_publisher(JointState, self._state_topic, self._queue_size)
        self._ee_pose_pub = self.create_publisher(PoseStamped, self._ee_pose_topic, self._queue_size)

        self._command_thread = threading.Thread(target=self._command_worker, daemon=True)
        self._command_thread.start()
        joint_state_period = 1.0 / max(self._state_publish_rate_hz, 0.1)
        self._joint_state_timer = self.create_timer(joint_state_period, self._publish_joint_state)
        ee_state_period = 1.0 / max(self._ee_pose_publish_rate_hz, 0.1)
        self._ee_pose_timer = self.create_timer(ee_state_period, self._publish_ee_pose)

        self.get_logger().info(
            f'myCobot JointState controller started: command_topic={self._command_topic}, '
            f'state_topic={self._state_topic}, ee_pose_topic={self._ee_pose_topic}, '
            f'joint_state_source={self._joint_state_source}, '
            f'gripper_command_topic={self._gripper_command_topic}, '
            f'gripper_value_topic={self._gripper_value_topic}, '
            f'gripper_calibration_topic={self._gripper_calibration_topic}, '
            f'port={self._port}, baud={self._baud}, unit={self._position_unit}, '
            f'command_rate_hz={self._command_rate_hz}, '
            f'joint_state_publish_rate_hz={self._state_publish_rate_hz}, '
            f'ee_pose_publish_rate_hz={self._ee_pose_publish_rate_hz}, '
            f'enable_ee_pose_publish={self._enable_ee_pose_publish}, '
            f'profile_io_timing={self._profile_io_timing}, '
            f'io_timing_log_every_n={self._io_timing_log_every_n}, '
            f'io_timing_outlier_ms={self._io_timing_outlier_ms}'
        )

    def _record_io_timing(self, name: str, elapsed_ms: float) -> None:
        if not self._profile_io_timing:
            return

        stats = self._io_timing[name]
        stats['count'] += 1
        stats['sum_ms'] += elapsed_ms
        stats['min_ms'] = min(stats['min_ms'], elapsed_ms)
        stats['max_ms'] = max(stats['max_ms'], elapsed_ms)

        if elapsed_ms >= self._io_timing_outlier_ms:
            self.get_logger().warning(
                f'[IO timing][{name}] outlier {elapsed_ms:.2f} ms'
            )

        if stats['count'] % self._io_timing_log_every_n == 0:
            avg_ms = stats['sum_ms'] / stats['count']
            self.get_logger().info(
                f'[IO timing][{name}] samples={stats["count"]}, '
                f'avg_ms={avg_ms:.3f}, min_ms={stats["min_ms"]:.3f}, '
                f'max_ms={stats["max_ms"]:.3f}'
            )

    def _connect_mycobot(self) -> MyCobot280:
        mc = MyCobot280(self._port, self._baud, debug=self._debug)

        # Keep the initialization sequence close to the verified standalone script.
        time.sleep(1.2)
        self._safe_init_call('power_on', mc.power_on)
        self._safe_init_call('resume', mc.resume)
        self._safe_init_call('clear_queue', mc.clear_queue)
        self._safe_init_call('set_free_mode(0)', lambda: mc.set_free_mode(0))
        self._safe_init_call('focus_all_servos', mc.focus_all_servos)
        time.sleep(0.5)

        return mc

    def _safe_init_call(self, name: str, fn) -> None:
        try:
            fn()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f'Initialization call {name} failed: {exc}')

    def _seed_last_sent_from_robot(self) -> None:
        """Initialize command-echo state from a one-shot device read to avoid startup deadlock."""
        try:
            with self._mc_lock:
                angles_deg = self._mc.get_angles()
            if angles_deg and len(angles_deg) >= 6:
                self._last_sent_angles_deg = [float(v) for v in angles_deg[:6]]
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f'Initial get_angles failed: {exc}')

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.position:
            return

        target_deg = self._decode_joint_state_to_deg(msg)
        if target_deg is None:
            return

        with self._lock:
            self._target_angles_deg = target_deg

        # 専用送信スレッドに通知し、ROSコールバックをI/O待ちで塞がない。
        self._command_event.set()

    def _decode_joint_state_to_deg(self, msg: JointState) -> Optional[List[float]]:
        current = [0.0] * 6
        if self._last_sent_angles_deg is not None:
            current = list(self._last_sent_angles_deg)

        if msg.name:
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    break
                idx = self._joint_index_by_name.get(name)
                if idx is None:
                    continue
                current[idx] = self._to_deg(msg.position[i])
        else:
            for i in range(min(6, len(msg.position))):
                current[i] = self._to_deg(msg.position[i])

        for i in range(6):
            low = self._joint_angle_min_deg[i]
            high = self._joint_angle_max_deg[i]
            current[i] = max(low, min(high, current[i]))

        return current

    def _to_deg(self, value: float) -> float:
        if self._position_unit == 'rad':
            return math.degrees(value)
        return value

    def _send_latest_command(self) -> None:
        with self._lock:
            target = None if self._target_angles_deg is None else list(self._target_angles_deg)

        if target is None:
            return

        now_mono = time.monotonic()
        if (now_mono - self._last_send_monotonic) < self._command_min_interval_sec:
            return

        if self._send_only_on_change and self._last_sent_angles_deg is not None:
            max_delta = max(
                abs(a - b) for a, b in zip(target, self._last_sent_angles_deg)
            )
            if max_delta < self._change_threshold_deg:
                return

        try:
            t0 = time.perf_counter() if self._profile_io_timing else None
            with self._mc_lock:
                self._mc.send_angles(target, self._command_speed)
            if t0 is not None:
                self._record_io_timing('send_angles', (time.perf_counter() - t0) * 1000.0)
            self._last_sent_angles_deg = target
            self._last_send_monotonic = now_mono
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Failed to send command to myCobot: {exc}')

    def _command_worker(self) -> None:
        while self._running:
            self._command_event.wait(timeout=self._command_min_interval_sec)
            self._command_event.clear()
            if not self._running:
                break
            self._send_latest_command()

    def destroy_node(self) -> bool:
        self._running = False
        self._command_event.set()
        if hasattr(self, '_command_thread') and self._command_thread.is_alive():
            self._command_thread.join(timeout=1.0)
        return super().destroy_node()

    def _on_gripper_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if not cmd:
            return

        try:
            if cmd in ('open', 'release'):
                with self._mc_lock:
                    self._mc.set_gripper_state(0, self._gripper_speed)
                self.get_logger().info('Gripper command: open')
            elif cmd in ('close', 'grasp'):
                with self._mc_lock:
                    self._mc.set_gripper_state(1, self._gripper_speed)
                self.get_logger().info('Gripper command: close')
            elif cmd in ('calibrate', 'calibration'):
                with self._mc_lock:
                    self._mc.set_gripper_calibration()
                self.get_logger().info('Gripper command: calibration')
            elif cmd == 'init':
                with self._mc_lock:
                    self._mc.init_gripper()
                self.get_logger().info('Gripper command: init')
            elif cmd == 'stop':
                with self._mc_lock:
                    self._mc.gripper_stop()
                self.get_logger().info('Gripper command: stop')
            else:
                self.get_logger().warning(
                    'Unknown gripper command. Use open/close/calibrate/init/stop.'
                )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Failed to run gripper command "{cmd}": {exc}')

    def _on_gripper_value(self, msg: Int32) -> None:
        value = int(msg.data)
        value = max(self._gripper_value_min, min(self._gripper_value_max, value))
        try:
            with self._mc_lock:
                self._mc.set_gripper_value(value, self._gripper_speed)
            self.get_logger().info(f'Gripper value command: {value}')
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Failed to set gripper value {value}: {exc}')

    def _on_gripper_calibration(self, _msg: Empty) -> None:
        try:
            with self._mc_lock:
                self._mc.set_gripper_calibration()
            self.get_logger().info('Gripper calibration command received')
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Failed to calibrate gripper: {exc}')

    def _publish_joint_state(self) -> None:
        now = self.get_clock().now().to_msg()

        if self._joint_state_source == 'command_echo':
            with self._lock:
                angles_deg = None if self._last_sent_angles_deg is None else list(self._last_sent_angles_deg)

            if angles_deg and len(angles_deg) >= 6:
                joint_msg = JointState()
                joint_msg.header.stamp = now
                joint_msg.header.frame_id = self._base_frame_id
                joint_msg.name = list(self._joint_names)
                joint_msg.position = [math.radians(float(v)) for v in angles_deg[:6]]
                self._joint_state_pub.publish(joint_msg)
            return

        try:
            t0 = time.perf_counter() if self._profile_io_timing else None
            with self._mc_lock:
                angles_deg = self._mc.get_angles()
            if t0 is not None:
                self._record_io_timing('get_angles', (time.perf_counter() - t0) * 1000.0)
            if angles_deg and len(angles_deg) >= 6:
                joint_msg = JointState()
                joint_msg.header.stamp = now
                joint_msg.header.frame_id = self._base_frame_id
                joint_msg.name = list(self._joint_names)
                joint_msg.position = [math.radians(float(v)) for v in angles_deg[:6]]
                self._joint_state_pub.publish(joint_msg)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f'Failed to fetch/publish joint state: {exc}')

    def _publish_ee_pose(self) -> None:
        if not self._enable_ee_pose_publish:
            return

        now = self.get_clock().now().to_msg()
        try:
            t0 = time.perf_counter() if self._profile_io_timing else None
            with self._mc_lock:
                coords = self._mc.get_coords()
            if t0 is not None:
                self._record_io_timing('get_coords', (time.perf_counter() - t0) * 1000.0)
            if coords and len(coords) >= 6:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = now
                pose_msg.header.frame_id = self._base_frame_id
                # myCobot coords are typically [x, y, z, rx, ry, rz] with mm and deg.
                pose_msg.pose.position.x = float(coords[0]) / 1000.0
                pose_msg.pose.position.y = float(coords[1]) / 1000.0
                pose_msg.pose.position.z = float(coords[2]) / 1000.0
                qx, qy, qz, qw = self._rpy_deg_to_quaternion(
                    float(coords[3]), float(coords[4]), float(coords[5])
                )
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw
                self._ee_pose_pub.publish(pose_msg)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f'Failed to fetch/publish ee pose: {exc}')

    def _rpy_deg_to_quaternion(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MyCobotJointStateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
