"""ROS 2 node for visual servo precision landing.

Subscribes to target observation and computes velocity commands
for precise landing on the detected marker.
"""
from __future__ import annotations

import time
from collections.abc import Callable

import rclpy
from ed_uav_interfaces.msg import TargetObservation
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ed_uav_perception.visual_servo import (
    LandingPhase,
    VelocityCommand,
    VisualServoConfig,
    VisualServoController,
)


class VisualServoNode(Node):
    """ROS 2 node for visual servo precision landing."""
    
    def __init__(
        self,
        *,
        steady_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__("visual_servo_node")
        self._steady_clock = steady_clock
        
        # Declare parameters
        self.declare_parameter("target_topic", "/d_task/target_observation")
        self.declare_parameter("velocity_topic", "/cmd_vel_stamped")
        self.declare_parameter("approach_kp_xy", 0.3)
        self.declare_parameter("approach_kp_z", 0.2)
        self.declare_parameter("descent_kp_xy", 0.5)
        self.declare_parameter("descent_kp_z", 0.3)
        self.declare_parameter("final_kp_xy", 0.8)
        self.declare_parameter("final_kp_z", 0.4)
        self.declare_parameter("touchdown_kp_xy", 1.0)
        self.declare_parameter("touchdown_kp_z", 0.5)
        self.declare_parameter("position_tolerance_m", 0.02)
        self.declare_parameter("stable_time_sec", 0.5)
        self.declare_parameter("enabled", True)
        
        # Create config from parameters
        config = VisualServoConfig(
            approach_kp_xy=self.get_parameter("approach_kp_xy").value,
            approach_kp_z=self.get_parameter("approach_kp_z").value,
            descent_kp_xy=self.get_parameter("descent_kp_xy").value,
            descent_kp_z=self.get_parameter("descent_kp_z").value,
            final_kp_xy=self.get_parameter("final_kp_xy").value,
            final_kp_z=self.get_parameter("final_kp_z").value,
            touchdown_kp_xy=self.get_parameter("touchdown_kp_xy").value,
            touchdown_kp_z=self.get_parameter("touchdown_kp_z").value,
            position_tolerance_m=self.get_parameter("position_tolerance_m").value,
            stable_time_sec=self.get_parameter("stable_time_sec").value,
        )
        
        self._controller = VisualServoController(config)
        self._enabled = self.get_parameter("enabled").value
        self._last_target_time = 0.0
        self._target_timeout_sec = 0.5

        # Camera mounting yaw offsets (rotation about optical axis).
        # Standard optical frame: image-top = forward.
        #   narrow: image-top → right  ⇒ -π/2
        #   wide:   image-top → left   ⇒ +π/2
        self._camera_yaw_offsets: dict[str, float] = {
            "camera_narrow_optical_frame": -math.pi / 2,
            "camera_wide_optical_frame": math.pi / 2,
        }
        
        # QoS for target observation (best effort, sensor data)
        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # QoS for velocity command (reliable)
        velocity_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Subscribe to target observation
        target_topic = self.get_parameter("target_topic").value
        self._target_sub = self.create_subscription(
            TargetObservation,
            target_topic,
            self._on_target,
            target_qos,
        )
        
        # Publish velocity commands
        velocity_topic = self.get_parameter("velocity_topic").value
        self._velocity_pub = self.create_publisher(
            TwistStamped,
            velocity_topic,
            velocity_qos,
        )
        
        # Status timer
        self._status_timer = self.create_timer(1.0, self._publish_status)
        
        self.get_logger().info(
            f"VisualServoNode started: "
            f"sub={target_topic}, pub={velocity_topic}, enabled={self._enabled}"
        )
    
    def _on_target(self, msg: TargetObservation) -> None:
        """Handle incoming target observation."""
        if not self._enabled:
            return
        
        # Check if target is valid
        if not msg.valid or msg.status != TargetObservation.STATUS_VALID:
            self.get_logger().debug("Target observation invalid, skipping")
            return
        
        # Extract position from pose (in camera frame)
        position = msg.pose.pose.position
        target_x = float(position.x)  # Right
        target_y = float(position.y)  # Down
        target_z = float(position.z)  # Forward
        
        # Determine camera mounting yaw offset from frame_id
        yaw_offset = self._camera_yaw_offsets.get(msg.frame_id, 0.0)
        
        # Compute velocity command
        now = self._steady_clock()
        command = self._controller.compute_command(
            target_x_m=target_x,
            target_y_m=target_y,
            target_z_m=target_z,
            current_timestamp_sec=now,
            camera_yaw_offset_rad=yaw_offset,
        )
        
        # Publish velocity command
        self._publish_velocity(command)
        
        # Update last target time
        self._last_target_time = now
        
        # Log phase changes
        if command.converged:
            self.get_logger().info(
                f"Landing converged: phase={command.phase.value}, "
                f"vx={command.vx_m_s:.3f}, vy={command.vy_m_s:.3f}, vz={command.vz_m_s:.3f}"
            )
    
    def _publish_velocity(self, command: VelocityCommand) -> None:
        """Publish velocity command as TwistStamped."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Body frame velocities
        msg.twist.linear.x = command.vx_m_s  # Forward
        msg.twist.linear.y = command.vy_m_s  # Left
        msg.twist.linear.z = command.vz_m_s  # Up
        msg.twist.angular.z = command.yaw_rate_rad_s
        
        self._velocity_pub.publish(msg)
    
    def _publish_status(self) -> None:
        """Publish status information."""
        now = self._steady_clock()
        target_age = now - self._last_target_time if self._last_target_time > 0 else float("inf")
        
        if target_age > self._target_timeout_sec:
            if self._enabled:
                self.get_logger().warn(
                    f"Target lost for {target_age:.1f}s, stopping corrections"
                )
                # Publish zero velocity
                self._publish_velocity(VelocityCommand(
                    vx_m_s=0.0,
                    vy_m_s=0.0,
                    vz_m_s=0.0,
                    yaw_rate_rad_s=0.0,
                    phase=LandingPhase.APPROACH,
                    converged=False,
                ))
        else:
            phase = self._controller.current_phase
            is_stable = self._controller.is_stable
            self.get_logger().info(
                f"Visual servo: phase={phase.value}, stable={is_stable}, "
                f"target_age={target_age:.2f}s"
            )
    
    def enable(self) -> None:
        """Enable visual servo control."""
        self._enabled = True
        self._controller.reset()
        self.get_logger().info("Visual servo enabled")
    
    def disable(self) -> None:
        """Disable visual servo control."""
        self._enabled = False
        self.get_logger().info("Visual servo disabled")
    
    @property
    def is_stable(self) -> bool:
        """Check if the controller has converged."""
        return self._controller.is_stable


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
