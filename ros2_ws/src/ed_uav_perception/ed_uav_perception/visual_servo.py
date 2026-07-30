"""Visual servo controller for precision landing using target observation.

Implements a visual servoing loop that uses the detected marker pose
to compute velocity corrections for precise landing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np


class LandingPhase(str, Enum):
    """Landing phases with different control gains."""
    APPROACH = "approach"      # > 2m, coarse positioning
    DESCENT = "descent"        # 0.5-2m, medium precision
    FINAL = "final"            # < 0.5m, high precision
    TOUCHDOWN = "touchdown"    # < 0.1m, minimal corrections


@dataclass(frozen=True, slots=True)
class VisualServoConfig:
    """Configuration for visual servo controller."""
    # Approach phase (> 2m)
    approach_kp_xy: float = 0.3
    approach_kp_z: float = 0.2
    approach_max_vel_xy: float = 0.5
    approach_max_vel_z: float = 0.3
    
    # Descent phase (0.5-2m)
    descent_kp_xy: float = 0.5
    descent_kp_z: float = 0.3
    descent_max_vel_xy: float = 0.3
    descent_max_vel_z: float = 0.2
    
    # Final phase (< 0.5m)
    final_kp_xy: float = 0.8
    final_kp_z: float = 0.4
    final_max_vel_xy: float = 0.15
    final_max_vel_z: float = 0.1
    
    # Touchdown phase (< 0.1m)
    touchdown_kp_xy: float = 1.0
    touchdown_kp_z: float = 0.5
    touchdown_max_vel_xy: float = 0.05
    touchdown_max_vel_z: float = 0.05
    
    # Derivative gains (for damping)
    kd_xy: float = 0.1
    kd_z: float = 0.05
    
    # Position thresholds
    approach_threshold_m: float = 2.0
    descent_threshold_m: float = 0.5
    final_threshold_m: float = 0.1
    
    # Convergence criteria
    position_tolerance_m: float = 0.02
    velocity_tolerance_m_s: float = 0.01
    stable_time_sec: float = 0.5
    
    # Safety limits
    max_tilt_rad: float = 0.15  # ~8.5 degrees
    max_yaw_rate_rad_s: float = 0.5


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    """Velocity command in body frame."""
    vx_m_s: float  # Forward (positive = forward)
    vy_m_s: float  # Right (positive = right)
    vz_m_s: float  # Down (positive = down)
    yaw_rate_rad_s: float  # Yaw rate (positive = clockwise)
    phase: LandingPhase
    converged: bool


@dataclass(frozen=True, slots=True)
class ServoState:
    """Internal state of the visual servo controller."""
    last_error_x: float = 0.0
    last_error_y: float = 0.0
    last_error_z: float = 0.0
    last_timestamp_sec: float = 0.0
    stable_start_sec: float = 0.0
    is_stable: bool = False


class VisualServoController:
    """Visual servo controller for precision landing.
    
    Uses a PD controller with phase-dependent gains to achieve
    precise landing on the target marker.
    
    Coordinate frame: camera/optical frame
    - X: right
    - Y: down
    - Z: forward (into scene)
    
    The controller outputs body-frame velocity commands.
    """
    
    def __init__(self, config: VisualServoConfig | None = None):
        self._config = config or VisualServoConfig()
        self._state = ServoState()
    
    def reset(self) -> None:
        """Reset controller state."""
        self._state = ServoState()
    
    def compute_command(
        self,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        current_timestamp_sec: float,
    ) -> VelocityCommand:
        """Compute velocity command to move toward target.
        
        Args:
            target_x_m: Target X position in camera frame (right, meters)
            target_y_m: Target Y position in camera frame (down, meters)
            target_z_m: Target Z position in camera frame (forward, meters)
            current_timestamp_sec: Current timestamp (monotonic seconds)
            
        Returns:
            VelocityCommand in body frame
        """
        config = self._config
        
        # Compute position error (in camera frame)
        # Camera frame: X=right, Y=down, Z=forward
        # We want to move toward the target, so error = target - current
        # But since we're computing velocity to reduce error, we use error directly
        error_x = target_x_m  # Lateral error (right positive)
        error_y = target_y_m  # Vertical error (down positive)
        error_z = target_z_m  # Depth error (forward positive)
        
        # Compute distance for phase determination
        distance_m = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # Determine landing phase
        if distance_m > config.approach_threshold_m:
            phase = LandingPhase.APPROACH
            kp_xy = config.approach_kp_xy
            kp_z = config.approach_kp_z
            max_vel_xy = config.approach_max_vel_xy
            max_vel_z = config.approach_max_vel_z
        elif distance_m > config.descent_threshold_m:
            phase = LandingPhase.DESCENT
            kp_xy = config.descent_kp_xy
            kp_z = config.descent_kp_z
            max_vel_xy = config.descent_max_vel_xy
            max_vel_z = config.descent_max_vel_z
        elif distance_m > config.final_threshold_m:
            phase = LandingPhase.FINAL
            kp_xy = config.final_kp_xy
            kp_z = config.final_kp_z
            max_vel_xy = config.final_max_vel_xy
            max_vel_z = config.final_max_vel_z
        else:
            phase = LandingPhase.TOUCHDOWN
            kp_xy = config.touchdown_kp_xy
            kp_z = config.touchdown_kp_z
            max_vel_xy = config.touchdown_max_vel_xy
            max_vel_z = config.touchdown_max_vel_z
        
        # Compute dt for derivative term
        dt = current_timestamp_sec - self._state.last_timestamp_sec
        if dt <= 0:
            dt = 0.02  # Default 50Hz
        
        # Compute derivative terms (for damping)
        if self._state.last_timestamp_sec > 0:
            d_error_x = (error_x - self._state.last_error_x) / dt
            d_error_y = (error_y - self._state.last_error_y) / dt
            d_error_z = (error_z - self._state.last_error_z) / dt
        else:
            d_error_x = 0.0
            d_error_y = 0.0
            d_error_z = 0.0
        
        # PD controller (in camera frame)
        # Camera frame: X=right, Y=down, Z=forward
        vx_camera = kp_xy * error_x + config.kd_xy * d_error_x
        vy_camera = kp_xy * error_y + config.kd_xy * d_error_y
        vz_camera = kp_z * error_z + config.kd_z * d_error_z
        
        # Transform from camera frame to body frame
        # Camera: X=right, Y=down, Z=forward
        # Body: X=forward, Y=left, Z=up
        # So: body_x = camera_z, body_y = -camera_x, body_z = -camera_y
        vx_body = vz_camera   # Forward
        vy_body = -vx_camera  # Left (negative of right)
        vz_body = -vy_camera  # Up (negative of down)
        
        # Apply velocity limits
        vel_xy = math.sqrt(vx_body**2 + vy_body**2)
        if vel_xy > max_vel_xy:
            scale = max_vel_xy / vel_xy
            vx_body *= scale
            vy_body *= scale
        
        vz_body = max(-max_vel_z, min(max_vel_z, vz_body))
        
        # Check convergence
        converged = (
            abs(error_x) < config.position_tolerance_m
            and abs(error_y) < config.position_tolerance_m
            and abs(error_z) < config.position_tolerance_m
            and abs(d_error_x) < config.velocity_tolerance_m_s
            and abs(d_error_y) < config.velocity_tolerance_m_s
            and abs(d_error_z) < config.velocity_tolerance_m_s
        )
        
        if converged:
            if not self._state.is_stable:
                stable_start = current_timestamp_sec
            else:
                stable_start = self._state.stable_start_sec
            is_stable = (current_timestamp_sec - stable_start) >= config.stable_time_sec
        else:
            stable_start = 0.0
            is_stable = False
        
        # Update state
        self._state = ServoState(
            last_error_x=error_x,
            last_error_y=error_y,
            last_error_z=error_z,
            last_timestamp_sec=current_timestamp_sec,
            stable_start_sec=stable_start,
            is_stable=is_stable,
        )
        
        return VelocityCommand(
            vx_m_s=float(vx_body),
            vy_m_s=float(vy_body),
            vz_m_s=float(vz_body),
            yaw_rate_rad_s=0.0,  # No yaw correction for now
            phase=phase,
            converged=is_stable,
        )
    
    @property
    def is_stable(self) -> bool:
        """Check if the controller has converged and is stable."""
        return self._state.is_stable
    
    @property
    def current_phase(self) -> LandingPhase:
        """Get the current landing phase based on last error."""
        config = self._config
        error = math.sqrt(
            self._state.last_error_x**2
            + self._state.last_error_y**2
            + self._state.last_error_z**2
        )
        if error > config.approach_threshold_m:
            return LandingPhase.APPROACH
        elif error > config.descent_threshold_m:
            return LandingPhase.DESCENT
        elif error > config.final_threshold_m:
            return LandingPhase.FINAL
        else:
            return LandingPhase.TOUCHDOWN
