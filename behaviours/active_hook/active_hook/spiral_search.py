#!/usr/bin/python
"""
Spiral search action server.

Simplified design: the vehicle just spins in place (constant, full-send
yaw command, no control loop -- same as before, nothing to wrap or blow
up) while holding pitch and roll level with plain P control. There is no
pitch sweep any more, so the roll/pitch/yaw coupling problem that came
from sweeping pitch during a yaw spin is gone entirely.

The search ends as soon as YOLO reports a confident, repeated sighting of
the target class (default "sam") -- see the detection callback below. As
a safety net in case nothing is ever found, the search also gives up
after max_turns full rotations (measured from real, accumulated yaw) or
after `timeout` seconds, whichever comes first.

Detection safety margin: a single-frame detection is not trusted on its
own (one bad frame could trigger a false stop). We require
`detection_confirm_count` consecutive detection-array messages that each
contain the target class above `detection_min_score` before treating the
target as found.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Vector3
from yolo_msgs.msg import DetectionArray

from smarc_action_base.gentler_action_server import GentlerActionServer
from active_hook_msgs.msg import Topics as ActiveHookTopics


def wrap_to_180(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


class SpiralSearchAction():
    def __init__(self, node: Node):
        self._node = node

        self._node.declare_parameter("yaw_rate", -1.0)   # constant, full-send yaw command; flip sign if it goes the wrong way
        self._node.declare_parameter("level_pitch", 0.0)   # pitch target to hold while spinning, degrees
        self._node.declare_parameter("pitch_kp", 1.0 / 15.0)
        self._node.declare_parameter("roll_kp", 1.0 / 5.0)    # very strong -- saturates by ~5 deg error, roll target is fixed at 0 so aggressive is safe here
        self._node.declare_parameter("max_turns", 2.0)     # give up (fail) after this many full rotations with no confirmed detection
        self._node.declare_parameter("timeout", 120.0)   # safety cutoff, seconds

        self._node.declare_parameter("detection_topic", "yolo/detections")
        self._node.declare_parameter("target_class", "sam")
        self._node.declare_parameter("detection_min_score", 0.5)
        self._node.declare_parameter("detection_confirm_count", 3)   # consecutive qualifying frames required before we trust the detection

        self._node.declare_parameter("loop_frequency", 20.0)

        self._yaw_rate = self._node.get_parameter("yaw_rate").value
        self._level_pitch = self._node.get_parameter("level_pitch").value
        self._pitch_kp = self._node.get_parameter("pitch_kp").value
        self._roll_kp = self._node.get_parameter("roll_kp").value
        self._default_max_turns = self._node.get_parameter("max_turns").value
        self._timeout = self._node.get_parameter("timeout").value

        detection_topic = self._node.get_parameter("detection_topic").value
        self._default_target_class = self._node.get_parameter("target_class").value
        self._detection_min_score = self._node.get_parameter("detection_min_score").value
        self._detection_confirm_count = self._node.get_parameter("detection_confirm_count").value

        loop_freq = self._node.get_parameter("loop_frequency").value

        self._latest_attitude: Vector3 | None = None
        self._node.create_subscription(
            Vector3, ActiveHookTopics.ATTITUDE_TOPIC, self._attitude_callback, 10
        )

        self._node.create_subscription(
            DetectionArray, detection_topic, self._detection_callback, 10
        )

        self._cmd_vel_pub = self._node.create_publisher(
            Twist, ActiveHookTopics.AUTONOMY_CMD_VEL_TOPIC, 10
        )

        # goal-scoped state
        self._max_turns = self._default_max_turns
        self._target_class = self._default_target_class
        self._start_time = None

        self._prev_raw_yaw: float | None = None
        self._yaw_accum: float = 0.0

        self._confirm_streak: int = 0
        self._target_confirmed: bool = False

        # kept only for _give_feedback
        self._last_e_pitch = 0.0

        self._as = GentlerActionServer(
            node, "spiral_search",
            self._on_goal_received, self._on_cancel_received,
            self._prepare_loop, self._loop_inner, self._give_feedback,
            loop_frequency=loop_freq,
        )

    def _attitude_callback(self, msg: Vector3) -> None:
        self._latest_attitude = msg

    def _detection_callback(self, msg: DetectionArray) -> None:
        # Does this frame contain a confident sighting of the target class?
        seen_this_frame = any(
            det.class_name == self._target_class and det.score >= self._detection_min_score
            for det in msg.detections
        )

        if seen_this_frame:
            self._confirm_streak += 1
        else:
            self._confirm_streak = 0

        if self._confirm_streak >= self._detection_confirm_count:
            self._target_confirmed = True

    def _on_goal_received(self, goal_request: dict) -> bool:
        if self._latest_attitude is None:
            self._node.get_logger().error("No attitude data received yet, rejecting goal")
            return False

        self._max_turns = float(goal_request.get('max_turns', self._default_max_turns))
        self._target_class = str(goal_request.get('target_class', self._default_target_class))
        return True

    def _on_cancel_received(self) -> bool:
        self._cmd_vel_pub.publish(Twist())
        return True

    def _prepare_loop(self) -> None:
        self._start_time = self._node.get_clock().now()
        self._prev_raw_yaw = self._latest_attitude.z
        self._yaw_accum = 0.0
        self._confirm_streak = 0
        self._target_confirmed = False

    def _loop_inner(self) -> bool | None:
        if self._latest_attitude is None:
            return None

        elapsed = (self._node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self._timeout:
            self._node.get_logger().error(
                f"Spiral search timed out after {elapsed:.1f}s without a confirmed "
                f"'{self._target_class}' detection."
            )
            self._cmd_vel_pub.publish(Twist())
            return False

        # --- did we get a confirmed sighting? stop immediately ---
        if self._target_confirmed:
            self._node.get_logger().info(
                f"'{self._target_class}' confirmed ({self._detection_confirm_count} "
                f"consecutive frames) -- ending search."
            )
            self._cmd_vel_pub.publish(Twist())
            return True

        # --- measure how far we've actually spun (unwrap the raw sensor
        # reading, which only ever reports (-180, 180]) ---
        current_yaw = self._latest_attitude.z
        delta = current_yaw - self._prev_raw_yaw
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        self._yaw_accum += delta
        self._prev_raw_yaw = current_yaw

        # --- safety net: give up if we've spun all the way around max_turns
        # times and still have nothing confirmed ---
        if abs(self._yaw_accum) >= abs(self._max_turns) * 360.0:
            self._node.get_logger().error(
                f"Completed {self._max_turns:.1f} turn(s) without finding "
                f"'{self._target_class}' -- giving up."
            )
            self._cmd_vel_pub.publish(Twist())
            return False

        # --- hold pitch/roll level while spinning ---
        current_pitch = self._latest_attitude.y
        current_roll = self._latest_attitude.x

        e_pitch = wrap_to_180(self._level_pitch - current_pitch)
        e_roll = wrap_to_180(0.0 - current_roll)
        self._last_e_pitch = e_pitch

        u_pitch = max(-1.0, min(1.0, self._pitch_kp * e_pitch))
        u_roll = max(-1.0, min(1.0, self._roll_kp * e_roll))

        twist = Twist()
        twist.angular.x = u_roll
        twist.angular.y = u_pitch
        twist.angular.z = self._yaw_rate   # constant, full-send, no control loop at all
        self._cmd_vel_pub.publish(twist)

        return None

    def _give_feedback(self) -> str:
        total_yaw_needed = abs(self._max_turns) * 360.0
        return (
            f"spin {abs(self._yaw_accum):.1f}/{total_yaw_needed:.1f} deg | "
            f"pitch error {self._last_e_pitch:.1f} | "
            f"detection streak {self._confirm_streak}/{self._detection_confirm_count}"
        )


def main():
    rclpy.init()
    node = Node("spiral_search_action_server")
    SpiralSearchAction(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
