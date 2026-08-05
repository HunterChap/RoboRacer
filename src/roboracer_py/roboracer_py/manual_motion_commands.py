import math
import time
from typing import Callable, List, Optional

from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from .motion_segments import MotionSegment
    from . import motion_profiles
except ImportError:
    from motion_segments import MotionSegment
    import motion_profiles


class ManualMotionController:
    """Terminal motion controller with persistent shared speed/steering settings."""

    _EPSILON = 1e-6

    def __init__(
        self,
        node,
        drive_mode_pub,
        transmitter_cmd_pub,
        *,
        manual_speed_mps: float,
        manual_reverse_speed_mps: float,
        manual_turn_speed_mps: float,
        manual_turn_angular_z: float,
        preset_move_duration_sec: float,
        preset_back_duration_sec: float,
        preset_turn_duration_sec: float,
        circle_speed_mps: float,
        circle_angular_z: float,
        max_abs_speed_mps: float,
        max_abs_angular_z: float,
        wheelbase_m: float = 0.33,
        max_steering_angle_rad: float = 0.50,
        turn_speed_follows_forward: bool = False,
        avoid_forward_distance_m: float = 1.0,
        avoid_turn_duration_sec: float = 2.4,
    ):
        self.node = node
        self.drive_mode_pub = drive_mode_pub
        self.transmitter_cmd_pub = transmitter_cmd_pub

        self.max_abs_speed_mps = abs(float(max_abs_speed_mps))
        self.max_abs_angular_z = abs(float(max_abs_angular_z))
        self.wheelbase_m = max(self._EPSILON, abs(float(wheelbase_m)))
        self.max_steering_angle_rad = abs(float(max_steering_angle_rad))

        self.forward_speed_mps = self.clamp(
            abs(float(manual_speed_mps)),
            0.0,
            self.max_abs_speed_mps,
        )
        self.reverse_speed_mps = -self.clamp(
            abs(float(manual_reverse_speed_mps)),
            0.0,
            self.max_abs_speed_mps,
        )
        self.turn_speed_mps = self.clamp(
            abs(float(manual_turn_speed_mps)),
            0.0,
            self.max_abs_speed_mps,
        )
        self.turn_speed_follows_forward = bool(turn_speed_follows_forward)

        initial_turn_speed = max(
            self.turn_speed_mps,
            self.forward_speed_mps,
            self._EPSILON,
        )
        initial_angle = math.atan(
            self.wheelbase_m
            * abs(float(manual_turn_angular_z))
            / initial_turn_speed
        )
        self.steering_angle_rad = self.clamp(
            initial_angle,
            0.0,
            self.max_steering_angle_rad,
        )

        self.preset_move_duration_sec = max(
            0.0,
            float(preset_move_duration_sec),
        )
        self.preset_back_duration_sec = max(
            0.0,
            float(preset_back_duration_sec),
        )
        self.preset_turn_duration_sec = max(
            0.0,
            float(preset_turn_duration_sec),
        )

        # Kept for compatibility with existing launch parameters.
        self.circle_speed_mps = float(circle_speed_mps)
        self.circle_angular_z = float(circle_angular_z)

        # Scripted avoidance maneuver: travel the configured forward
        # distance, then execute a time-calibrated left or right turn.
        self.avoid_forward_distance_m = max(
            self._EPSILON,
            abs(float(avoid_forward_distance_m)),
        )
        self.avoid_turn_duration_sec = max(
            self._EPSILON,
            float(avoid_turn_duration_sec),
        )

        self.continuous_twist: Optional[Twist] = None
        self.active_manual_action: Optional[str] = None
        self.manual_status = 'STOPPED'
        self.motion_completion_mode = 'manual_hold'

        self.timed_action_end_time: Optional[float] = None

        self.angle_provider: Optional[Callable[[], Optional[float]]] = None
        self.angle_start_rad: Optional[float] = None
        self.angle_target_rad: Optional[float] = None
        self.angle_fallback_end_time: Optional[float] = None

        self.distance_provider: Optional[Callable[[], Optional[float]]] = None
        self.distance_start_m: Optional[float] = None
        self.distance_target_m: Optional[float] = None

        self.profile_segments: List[MotionSegment] = []
        self.profile_start_time: Optional[float] = None
        self.profile_total_duration: float = 0.0
        self.profile_completion_action: Optional[str] = None

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(float(value), max_value))

    def make_twist(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = self.clamp(
            linear_x,
            -self.max_abs_speed_mps,
            self.max_abs_speed_mps,
        )
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.clamp(
            angular_z,
            -self.max_abs_angular_z,
            self.max_abs_angular_z,
        )
        return cmd

    def make_stop_twist(self):
        return self.make_twist(0.0, 0.0)

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = str(mode)
        self.drive_mode_pub.publish(msg)

    def publish_cmd(self, cmd: Twist):
        self.transmitter_cmd_pub.publish(cmd)

    def clear_active_motion(self):
        self.continuous_twist = None
        self.active_manual_action = None
        self.motion_completion_mode = 'manual_hold'
        self.timed_action_end_time = None

        self.angle_provider = None
        self.angle_start_rad = None
        self.angle_target_rad = None
        self.angle_fallback_end_time = None

        self.distance_provider = None
        self.distance_start_m = None
        self.distance_target_m = None

        self.profile_segments = []
        self.profile_start_time = None
        self.profile_total_duration = 0.0
        self.profile_completion_action = None

    def has_active_motion(self) -> bool:
        return (
            self.continuous_twist is not None
            or self.profile_start_time is not None
        )

    def is_temporary_auto_motion(self) -> bool:
        return self.has_active_motion() and self.motion_completion_mode == 'auto_active'

    def get_status_label(self):
        return self.manual_status

    def get_effective_turn_speed(self) -> float:
        if self.turn_speed_follows_forward:
            return self.forward_speed_mps
        return self.turn_speed_mps

    def get_effective_turn_angular_z(self) -> float:
        speed = abs(self.get_effective_turn_speed())
        if speed <= self._EPSILON or self.steering_angle_rad <= self._EPSILON:
            return 0.0

        angular_z = (
            speed
            / self.wheelbase_m
            * math.tan(self.steering_angle_rad)
        )
        return self.clamp(
            angular_z,
            0.0,
            self.max_abs_angular_z,
        )

    def get_settings_text(self) -> str:
        if self.turn_speed_follows_forward:
            turn_speed = (
                f'{self.get_effective_turn_speed():.3f} m/s '
                f'(follows v)'
            )
        else:
            turn_speed = (
                f'{self.turn_speed_mps:.3f} m/s '
                f'(independent)'
            )

        return (
            f'forward={self.forward_speed_mps:.3f} m/s, '
            f'reverse={self.reverse_speed_mps:.3f} m/s, '
            f'turn_speed={turn_speed}, '
            f'steering={self.steering_angle_rad:.3f} rad '
            f'({math.degrees(self.steering_angle_rad):.1f} deg), '
            f'turn_angular_z='
            f'{self.get_effective_turn_angular_z():.3f} rad/s'
        )

    def log_settings(self):
        self.node.get_logger().info(
            f'Persistent shared settings: {self.get_settings_text()}'
        )

    def command_for_action(self, action_name: str):
        action = str(action_name).strip().lower()

        if action == 'move':
            cmd = self.make_twist(self.forward_speed_mps, 0.0)
            label = f'FORWARD {self.forward_speed_mps:.2f}'
        elif action == 'left':
            turn_speed = self.get_effective_turn_speed()
            angular_z = self.get_effective_turn_angular_z()
            cmd = self.make_twist(turn_speed, angular_z)
            label = (
                f'LEFT v={turn_speed:.2f} '
                f'sa={self.steering_angle_rad:.2f}'
            )
        elif action == 'right':
            turn_speed = self.get_effective_turn_speed()
            angular_z = self.get_effective_turn_angular_z()
            cmd = self.make_twist(turn_speed, -angular_z)
            label = (
                f'RIGHT v={turn_speed:.2f} '
                f'sa={self.steering_angle_rad:.2f}'
            )
        elif action == 'back':
            cmd = self.make_twist(self.reverse_speed_mps, 0.0)
            label = f'BACK {self.reverse_speed_mps:.2f}'
        else:
            raise ValueError(f'Unknown motion action: {action_name}')

        return action, cmd, label

    def set_manual_hold(self):
        self.clear_active_motion()
        self.manual_status = 'STOPPED'
        self.publish_cmd(self.make_stop_twist())
        self.publish_mode('manual_hold')
        self.node.get_logger().info(
            'Manual Mode stopped. Persistent settings were kept.'
        )

    def start_manual_action(self, action_name: str):
        try:
            action, cmd, label = self.command_for_action(action_name)
        except ValueError:
            self.node.get_logger().warn(
                f'Unknown manual action: {action_name}'
            )
            return False

        self.clear_active_motion()
        self.active_manual_action = action
        self.continuous_twist = cmd
        self.manual_status = label
        self.motion_completion_mode = 'manual_hold'

        self.publish_mode('manual_active')
        self.publish_cmd(cmd)
        self.node.get_logger().info(
            f'Manual action: {label}. Holding until next command.'
        )
        return True

    def start_preset(self, preset_name: str):
        return self.start_manual_action(preset_name)

    def start_timed_action(
        self,
        action_name: str,
        duration_sec: float,
        *,
        completion_mode: str,
        label_prefix: str = 'TEMP',
    ) -> bool:
        try:
            action, cmd, label = self.command_for_action(action_name)
        except ValueError:
            self.node.get_logger().warn(
                f'Unknown timed action: {action_name}'
            )
            return False

        duration = max(0.0, float(duration_sec))
        if duration <= self._EPSILON:
            self.node.get_logger().warn(
                f'Timed action "{action}" has zero duration.'
            )
            return False

        self.clear_active_motion()
        self.active_manual_action = action
        self.continuous_twist = cmd
        self.timed_action_end_time = time.monotonic() + duration
        self.motion_completion_mode = str(completion_mode)
        self.manual_status = f'{label_prefix} {label}'

        self.publish_mode('manual_active')
        self.publish_cmd(cmd)
        self.node.get_logger().info(
            f'Started {label_prefix.lower()} action "{action}" '
            f'for {duration:.2f}s; completion={completion_mode}.'
        )
        return True

    def start_auto_action(
        self,
        action_name: str,
        *,
        angle_provider: Optional[Callable[[], Optional[float]]] = None,
        target_turn_angle_rad: float = math.pi / 2.0,
    ) -> bool:
        action = str(action_name).strip().lower()

        if action == 'move':
            return self.start_timed_action(
                'move',
                self.preset_move_duration_sec,
                completion_mode='auto_active',
                label_prefix='AUTO',
            )

        if action == 'back':
            return self.start_timed_action(
                'back',
                self.preset_back_duration_sec,
                completion_mode='auto_active',
                label_prefix='AUTO',
            )

        if action in ['left', 'right']:
            return self.start_angle_turn(
                action,
                target_turn_angle_rad,
                angle_provider=angle_provider,
                completion_mode='auto_active',
            )

        self.node.get_logger().warn(
            f'Unknown Auto temporary action: {action_name}'
        )
        return False

    def start_angle_turn(
        self,
        direction: str,
        target_angle_rad: float,
        *,
        angle_provider: Optional[Callable[[], Optional[float]]],
        completion_mode: str,
    ) -> bool:
        action = str(direction).strip().lower()
        if action not in ['left', 'right']:
            self.node.get_logger().warn(
                f'Unknown turn direction: {direction}'
            )
            return False

        try:
            _, cmd, label = self.command_for_action(action)
        except ValueError:
            return False

        angular_z = abs(float(cmd.angular.z))
        if angular_z <= self._EPSILON:
            self.node.get_logger().warn(
                'Cannot start angle turn: effective angular.z is zero. '
                'Check ts#/tsauto and sa#/sad#.'
            )
            return False

        target = abs(float(target_angle_rad))
        if target <= self._EPSILON:
            self.node.get_logger().warn(
                'Cannot start angle turn: target angle is zero.'
            )
            return False

        start_angle = angle_provider() if angle_provider is not None else None
        fallback_duration = target / angular_z

        self.clear_active_motion()
        self.active_manual_action = action
        self.continuous_twist = cmd
        self.motion_completion_mode = str(completion_mode)
        self.manual_status = (
            f'AUTO {label} target={math.degrees(target):.0f}deg'
        )
        self.angle_provider = angle_provider
        self.angle_start_rad = start_angle
        self.angle_target_rad = target
        self.angle_fallback_end_time = (
            None
            if start_angle is not None
            else time.monotonic() + fallback_duration
        )

        self.publish_mode('manual_active')
        self.publish_cmd(cmd)

        if start_angle is None:
            self.node.get_logger().warn(
                'Yaw feedback is unavailable. The 90-degree turn is using '
                f'an open-loop fallback of {fallback_duration:.2f}s.'
            )
        else:
            self.node.get_logger().info(
                'Started feedback-based '
                f'{math.degrees(target):.1f}-degree {action} turn.'
            )
        return True

    def start_distance(
        self,
        distance_m: float,
        *,
        distance_provider: Callable[[], Optional[float]],
        completion_mode: str,
    ) -> bool:
        signed_target = float(distance_m)
        target = abs(signed_target)

        if target <= self._EPSILON:
            self.node.get_logger().warn(
                'Distance command must be non-zero.'
            )
            return False

        start_distance = distance_provider()
        if start_distance is None:
            self.node.get_logger().warn(
                'Distance feedback is unavailable or stale. '
                'The distance command was not started.'
            )
            return False

        action = 'move' if signed_target > 0.0 else 'back'
        try:
            _, cmd, label = self.command_for_action(action)
        except ValueError:
            return False

        if abs(float(cmd.linear.x)) <= self._EPSILON:
            self.node.get_logger().warn(
                'Distance command cannot start because the selected '
                'forward/reverse speed is zero.'
            )
            return False

        self.clear_active_motion()
        self.active_manual_action = action
        self.continuous_twist = cmd
        self.motion_completion_mode = str(completion_mode)
        self.manual_status = f'DISTANCE {label} target={target:.2f}m'
        self.distance_provider = distance_provider
        self.distance_start_m = float(start_distance)
        self.distance_target_m = target

        self.publish_mode('manual_active')
        self.publish_cmd(cmd)
        self.node.get_logger().info(
            f'Started distance action: target={target:.3f}m, '
            f'direction={action}, completion={completion_mode}.'
        )
        return True

    def set_linear_speed(self, speed_mps: float):
        value = self.clamp(
            speed_mps,
            -self.max_abs_speed_mps,
            self.max_abs_speed_mps,
        )

        if value >= 0.0:
            self.forward_speed_mps = value
            setting_name = 'forward'
        else:
            self.reverse_speed_mps = value
            setting_name = 'reverse'

        self.refresh_active_action_for_settings()
        self.node.get_logger().info(
            f'Persistent {setting_name} speed set to {value:.3f} m/s. '
            'The selected mode was not changed.'
        )
        self.log_settings()

    def start_speed(self, speed_mps: float):
        """Compatibility method that saves the speed and starts the matching Manual action."""
        self.set_linear_speed(speed_mps)
        if speed_mps > self._EPSILON:
            return self.start_manual_action('move')
        if speed_mps < -self._EPSILON:
            return self.start_manual_action('back')
        self.set_manual_hold()
        return True

    def set_turn_speed(self, speed_mps: float):
        self.turn_speed_mps = self.clamp(
            abs(float(speed_mps)),
            0.0,
            self.max_abs_speed_mps,
        )
        self.turn_speed_follows_forward = False
        self.refresh_active_action_for_settings()

        self.node.get_logger().info(
            f'Persistent turn speed set to '
            f'{self.turn_speed_mps:.3f} m/s. '
            'The selected mode was not changed.'
        )
        self.log_settings()

    def set_turn_speed_follow(self):
        self.turn_speed_follows_forward = True
        self.refresh_active_action_for_settings()

        self.node.get_logger().info(
            'Turn speed now follows the persistent forward speed. '
            'The selected mode was not changed.'
        )
        self.log_settings()

    def set_steering_angle(self, steering_angle_rad: float):
        self.steering_angle_rad = self.clamp(
            abs(float(steering_angle_rad)),
            0.0,
            self.max_steering_angle_rad,
        )
        self.refresh_active_action_for_settings()

        self.node.get_logger().info(
            f'Persistent steering angle set to '
            f'{self.steering_angle_rad:.3f} rad '
            f'({math.degrees(self.steering_angle_rad):.1f} deg). '
            'The selected mode was not changed.'
        )
        self.log_settings()

    def refresh_active_action_for_settings(self):
        if self.active_manual_action not in [
            'move',
            'left',
            'right',
            'back',
        ]:
            return
        if self.continuous_twist is None:
            return

        try:
            _, cmd, label = self.command_for_action(
                self.active_manual_action
            )
        except ValueError:
            return

        self.continuous_twist = cmd
        if self.distance_target_m is not None:
            self.manual_status = (
                f'DISTANCE {label} target='
                f'{self.distance_target_m:.2f}m'
            )
        elif self.angle_target_rad is not None:
            self.manual_status = (
                f'AUTO {label} target='
                f'{math.degrees(self.angle_target_rad):.0f}deg'
            )
        elif self.timed_action_end_time is None:
            self.manual_status = label

        self.publish_cmd(cmd)

    def get_avoidance_settings_text(self) -> str:
        return (
            f'distance={self.avoid_forward_distance_m:.3f} m, '
            f'turn_time={self.avoid_turn_duration_sec:.3f} s'
        )

    def set_avoidance_distance(self, distance_m: float) -> bool:
        value = abs(float(distance_m))
        if value <= self._EPSILON:
            self.node.get_logger().warn(
                'Avoidance-demo distance must be greater than zero.'
            )
            return False

        self.avoid_forward_distance_m = value
        self.node.get_logger().info(
            f'Avoidance-demo forward distance set to {value:.3f} m.'
        )
        return True

    def set_avoidance_turn_duration(self, duration_sec: float) -> bool:
        value = float(duration_sec)
        if value <= self._EPSILON:
            self.node.get_logger().warn(
                'Avoidance-demo turn time must be greater than zero.'
            )
            return False

        self.avoid_turn_duration_sec = value
        self.node.get_logger().info(
            f'Avoidance-demo turn time set to {value:.3f} s.'
        )
        return True

    def start_avoidance_demo(
        self,
        direction: str,
        *,
        completion_mode: str = 'manual_hold',
    ) -> bool:
        turn_direction = str(direction).strip().lower()
        if turn_direction not in ['left', 'right']:
            self.node.get_logger().warn(
                f'Unknown avoidance-demo direction: {direction}'
            )
            return False

        forward_speed = abs(float(self.forward_speed_mps))
        if forward_speed <= self._EPSILON:
            self.node.get_logger().warn(
                'Avoidance demo cannot start because v# is zero.'
            )
            return False

        turn_speed = abs(float(self.get_effective_turn_speed()))
        turn_angular_z = abs(float(self.get_effective_turn_angular_z()))
        if turn_speed <= self._EPSILON or turn_angular_z <= self._EPSILON:
            self.node.get_logger().warn(
                'Avoidance demo cannot turn. Check ts#/tsauto and sa#/sad#.'
            )
            return False

        forward_duration = self.avoid_forward_distance_m / forward_speed
        signed_angular_z = (
            turn_angular_z
            if turn_direction == 'left'
            else -turn_angular_z
        )
        label = 'AVOID-L' if turn_direction == 'left' else 'AVOID-R'

        segments = [
            MotionSegment(
                linear_x=forward_speed,
                angular_z=0.0,
                duration_sec=forward_duration,
            ),
            MotionSegment(
                linear_x=turn_speed,
                angular_z=signed_angular_z,
                duration_sec=self.avoid_turn_duration_sec,
            ),
        ]

        self.node.get_logger().info(
            f'Starting {label}: approximately '
            f'{self.avoid_forward_distance_m:.2f}m forward '
            f'({forward_duration:.2f}s at {forward_speed:.2f}m/s), then '
            f'{self.avoid_turn_duration_sec:.2f}s {turn_direction} turn, '
            f'then continuous forward.'
        )
        return self.start_profile(
            segments,
            label,
            completion_mode=completion_mode,
            completion_action='move',
        )

    def start_trajectory(
        self,
        trajectory_name: str,
        *,
        completion_mode: str = 'manual_hold',
    ) -> bool:
        turn_speed = self.get_effective_turn_speed()
        angular_z = self.get_effective_turn_angular_z()

        if trajectory_name == 'lc':
            segments = motion_profiles.left_circle(
                turn_speed,
                angular_z,
            )
            label = 'LC'
        elif trajectory_name == 'rc':
            segments = motion_profiles.right_circle(
                turn_speed,
                angular_z,
            )
            label = 'RC'
        elif trajectory_name == 'f8':
            segments = motion_profiles.figure_eight(
                turn_speed,
                angular_z,
            )
            label = 'F8'
        else:
            self.node.get_logger().warn(
                f'Unknown trajectory: {trajectory_name}'
            )
            return False

        return self.start_profile(
            segments,
            label,
            completion_mode=completion_mode,
        )

    def start_profile(
        self,
        segments: List[MotionSegment],
        label: str,
        *,
        completion_mode: str = 'manual_hold',
        completion_action: Optional[str] = None,
    ) -> bool:
        self.clear_active_motion()

        valid_segments = [
            seg for seg in segments
            if seg.duration_sec > 0.0
        ]
        if not valid_segments:
            self.node.get_logger().warn(
                f'Profile "{label}" has no valid segments. '
                f'Check turn speed and steering angle.'
            )
            if completion_mode == 'manual_hold':
                self.set_manual_hold()
            else:
                self.publish_cmd(self.make_stop_twist())
                self.publish_mode(completion_mode)
            return False

        self.profile_segments = valid_segments
        self.profile_start_time = time.monotonic()
        self.profile_total_duration = sum(
            seg.duration_sec for seg in valid_segments
        )
        self.motion_completion_mode = str(completion_mode)
        self.profile_completion_action = (
            str(completion_action).strip().lower()
            if completion_action is not None
            else None
        )
        self.manual_status = f'PROFILE {label}'

        self.publish_mode('manual_active')
        first = valid_segments[0]
        self.publish_cmd(
            self.make_twist(first.linear_x, first.angular_z)
        )

        self.node.get_logger().info(
            f'Started profile "{label}" for '
            f'{self.profile_total_duration:.2f}s '
            f'using {self.get_settings_text()}; '
            f'completion={completion_mode}, '
            f'completion_action={self.profile_completion_action}.'
        )
        return True

    def finish_active_motion(self, reason: str):
        completion_mode = self.motion_completion_mode
        self.publish_cmd(self.make_stop_twist())
        self.clear_active_motion()
        self.manual_status = 'STOPPED'
        self.publish_mode(completion_mode)
        self.node.get_logger().info(
            f'{reason} Persistent settings were kept. '
            f'Next mode={completion_mode}.'
        )

    def cancel(self, return_mode: str = 'manual_hold'):
        self.publish_cmd(self.make_stop_twist())
        self.clear_active_motion()
        self.manual_status = 'STOPPED'
        self.publish_mode(return_mode)
        self.node.get_logger().info(
            f'Cancelled motion. Persistent settings were kept. '
            f'Next mode={return_mode}.'
        )

    def emergency_stop(self):
        self.clear_active_motion()
        self.manual_status = 'STOPPED'
        self.publish_cmd(self.make_stop_twist())
        self.publish_mode('stop')
        self.node.get_logger().warn(
            'Stop Mode/disarm requested. '
            'Persistent settings remain in memory.'
        )

    def get_profile_segment_for_elapsed(
        self,
        elapsed_sec: float,
    ) -> Optional[MotionSegment]:
        cursor = 0.0
        for segment in self.profile_segments:
            cursor += segment.duration_sec
            if elapsed_sec <= cursor:
                return segment
        return None

    def update(self):
        now = time.monotonic()

        if self.distance_target_m is not None:
            current_distance = (
                self.distance_provider()
                if self.distance_provider is not None
                else None
            )
            if current_distance is None:
                self.finish_active_motion(
                    'Distance feedback was lost; motion stopped.'
                )
                return

            travelled = abs(
                float(current_distance)
                - float(self.distance_start_m)
            )
            if travelled >= self.distance_target_m:
                self.finish_active_motion(
                    f'Distance target reached '
                    f'({travelled:.3f}m).'
                )
                return

            self.publish_mode('manual_active')
            self.publish_cmd(self.continuous_twist)
            return

        if self.angle_target_rad is not None:
            if self.angle_start_rad is not None:
                current_angle = (
                    self.angle_provider()
                    if self.angle_provider is not None
                    else None
                )
                if current_angle is None:
                    self.finish_active_motion(
                        'Yaw feedback was lost; angle turn stopped.'
                    )
                    return

                turned = abs(
                    float(current_angle)
                    - float(self.angle_start_rad)
                )
                if turned >= self.angle_target_rad:
                    self.finish_active_motion(
                        f'Angle target reached '
                        f'({math.degrees(turned):.1f}deg).'
                    )
                    return
            elif (
                self.angle_fallback_end_time is not None
                and now >= self.angle_fallback_end_time
            ):
                self.finish_active_motion(
                    'Open-loop angle-turn fallback completed.'
                )
                return

            self.publish_mode('manual_active')
            self.publish_cmd(self.continuous_twist)
            return

        if self.timed_action_end_time is not None:
            if now >= self.timed_action_end_time:
                self.finish_active_motion(
                    'Temporary action completed.'
                )
                return

            self.publish_mode('manual_active')
            self.publish_cmd(self.continuous_twist)
            return

        if self.continuous_twist is not None:
            self.publish_mode('manual_active')
            self.publish_cmd(self.continuous_twist)
            return

        if self.profile_start_time is None or not self.profile_segments:
            return

        elapsed = now - self.profile_start_time
        segment = self.get_profile_segment_for_elapsed(elapsed)

        if segment is None:
            completion_action = self.profile_completion_action
            if completion_action is not None:
                self.node.get_logger().info(
                    f'Profile finished. Continuing with '
                    f'"{completion_action}" until the next command.'
                )
                self.start_manual_action(completion_action)
            else:
                self.finish_active_motion('Profile finished.')
            return

        self.publish_mode('manual_active')
        self.publish_cmd(
            self.make_twist(
                segment.linear_x,
                segment.angular_z,
            )
        )
