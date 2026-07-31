"""6-DOF constant-velocity Kalman filter for target tracking.

State vector: [x, y, z, vx, vy, vz]ᵀ
Measurement:  [x, y, z]ᵀ  (from PnP pose estimation)

Provides three key capabilities over a simple EMA:
  1. **Smoothing** — fuses noisy PnP measurements via the Kalman gain.
  2. **Prediction** — continues to output an estimated position during
     frame drops or detection failures.
  3. **Delay compensation** — the predicted state can be advanced to a
     future timestamp to compensate for pipeline latency.
"""

from __future__ import annotations

import math

import numpy as np


class KalmanTracker:
    """Constant-velocity Kalman filter for 3-D position tracking.

    Parameters
    ----------
    process_noise_pos : float
        Process noise standard deviation for position (m / √s).
        Higher → tracker responds faster to real motion.
    process_noise_vel : float
        Process noise standard deviation for velocity (m / s² / √s).
        Higher → tracker trusts measurements more vs. model.
    initial_covariance_diag : float
        Initial diagonal value for the state covariance matrix.
    max_predict_age_sec : float
        Maximum time to coast on prediction alone before the tracker
        is considered *stale* and ``is_fresh`` returns ``False``.
    """

    def __init__(
        self,
        process_noise_pos: float = 0.05,
        process_noise_vel: float = 0.3,
        initial_covariance_diag: float = 1.0,
        max_predict_age_sec: float = 0.5,
    ) -> None:
        self._q_pos = process_noise_pos
        self._q_vel = process_noise_vel
        self._max_predict_age = max_predict_age_sec

        # State and covariance (lazy init on first update)
        self._x: np.ndarray | None = None  # 6×1
        self._P: np.ndarray | None = None  # 6×6
        self._init_P_diag = initial_covariance_diag

        # Measurement matrix (constant)
        self._H = np.zeros((3, 6), dtype=np.float64)
        self._H[0, 0] = 1.0
        self._H[1, 1] = 1.0
        self._H[2, 2] = 1.0

        # Timestamps
        self._last_update_sec: float = 0.0
        self._last_predict_sec: float = 0.0

    # ── Public API ──────────────────────────────────────────────────────

    @property
    def is_initialized(self) -> bool:
        """Whether the filter has received at least one measurement."""
        return self._x is not None

    @property
    def is_fresh(self) -> bool:
        """Whether the tracker output is still trustworthy.

        Returns ``True`` when the last *update* (not just prediction) was
        within ``max_predict_age_sec``.
        """
        if not self.is_initialized:
            return False
        age = self._last_predict_sec - self._last_update_sec
        return age <= self._max_predict_age

    @property
    def position(self) -> np.ndarray:
        """Current estimated position [x, y, z] (3-vector)."""
        if self._x is None:
            return np.zeros(3, dtype=np.float64)
        return self._x[:3, 0].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Current estimated velocity [vx, vy, vz] (3-vector)."""
        if self._x is None:
            return np.zeros(3, dtype=np.float64)
        return self._x[3:, 0].copy()

    @property
    def position_uncertainty(self) -> float:
        """RMS position uncertainty (metres) from the covariance diagonal."""
        if self._P is None:
            return float("inf")
        return float(math.sqrt(np.mean(np.diag(self._P)[:3])))

    @property
    def last_update_sec(self) -> float:
        """Monotonic timestamp of the last measurement update."""
        return self._last_update_sec

    def predict(self, now_sec: float) -> np.ndarray:
        """Advance the state to *now_sec* and return the predicted position.

        Does **not** change the filter state (calling ``predict`` repeatedly
        without ``update`` will not drift the covariance indefinitely).
        Call ``update`` to commit the prediction.
        """
        if self._x is None:
            return np.zeros(3, dtype=np.float64)

        dt = now_sec - self._last_predict_sec
        if dt <= 0.0:
            return self.position

        F = self._make_F(dt)
        x_pred = F @ self._x
        self._last_predict_sec = now_sec
        return x_pred[:3, 0].copy()

    def update(
        self,
        measurement: np.ndarray,
        measurement_noise_px: float,
        now_sec: float,
        focal_length_px: float = 800.0,
        depth_m: float = 1.0,
    ) -> np.ndarray:
        """Incorporate a new PnP measurement and return the filtered position.

        Parameters
        ----------
        measurement : (3,) array
            Measured position [x, y, z] in metres (camera frame).
        measurement_noise_px : float
            Reprojection RMS error from PnP solver (pixels).
        now_sec : float
            Monotonic timestamp of the measurement.
        focal_length_px : float
            Camera focal length in pixels (used to convert pixel noise
            to metre noise).
        depth_m : float
            Approximate depth of the target (for noise scaling).

        Returns
        -------
        position : (3,) array
            Filtered position estimate.
        """
        measurement = np.asarray(measurement, dtype=np.float64).reshape(3)

        if self._x is None:
            self._initialize(measurement, now_sec)
            return self.position

        # ── Predict step ────────────────────────────────────────────────
        dt = now_sec - self._last_update_sec
        if dt <= 0.0:
            dt = 0.001  # guard against zero dt
        self._last_predict_sec = now_sec

        F = self._make_F(dt)
        Q = self._make_Q(dt)

        x_pred = F @ self._x
        P_pred = F @ self._P @ F.T + Q

        # ── Update step ─────────────────────────────────────────────────
        # Measurement noise: convert pixel reprojection error to metre
        # noise using the pinhole model: σ_m ≈ σ_px × depth / focal
        sigma_m = max(1e-4, measurement_noise_px * depth_m / focal_length_px)
        R = np.diag([sigma_m**2, sigma_m**2, sigma_m**2])

        z = measurement.reshape(3, 1)
        y = z - self._H @ x_pred            # innovation
        S = self._H @ P_pred @ self._H.T + R  # innovation covariance
        K = P_pred @ self._H.T @ np.linalg.inv(S)  # Kalman gain

        self._x = x_pred + K @ y
        I_KH = np.eye(6) - K @ self._H
        # Joseph form for numerical stability
        self._P = I_KH @ P_pred @ I_KH.T + K @ R @ K.T

        self._last_update_sec = now_sec
        return self.position

    def reset(self) -> None:
        """Clear all state (e.g. after a long tracking gap)."""
        self._x = None
        self._P = None
        self._last_update_sec = 0.0
        self._last_predict_sec = 0.0

    # ── Internals ───────────────────────────────────────────────────────

    def _initialize(self, measurement: np.ndarray, now_sec: float) -> None:
        """Set the initial state from the first measurement (zero velocity)."""
        self._x = np.zeros((6, 1), dtype=np.float64)
        self._x[:3, 0] = measurement
        self._P = np.eye(6, dtype=np.float64) * self._init_P_diag
        self._last_update_sec = now_sec
        self._last_predict_sec = now_sec

    @staticmethod
    def _make_F(dt: float) -> np.ndarray:
        """Constant-velocity state transition matrix."""
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    def _make_Q(self, dt: float) -> np.ndarray:
        """Discrete-time process noise covariance.

        Uses the piecewise-white-noise acceleration model:
          Q_pos = q_p²·dt³/3 + q_v²·dt⁵/20
          Q_pos_vel = q_p²·dt²/2 + q_v²·dt⁴/8
          Q_vel = q_p²·dt + q_v²·dt³/3
        Simplified here to the standard block form for a single axis.
        """
        qp = self._q_pos
        qv = self._q_vel
        dt2 = dt * dt
        dt3 = dt2 * dt

        Q = np.zeros((6, 6), dtype=np.float64)
        for i in range(3):
            # Position–position
            Q[i, i] = qp**2 * dt3 / 3.0 + qv**2 * dt3 * dt2 / 20.0
            # Position–velocity (and vice versa)
            Q[i, i + 3] = qp**2 * dt2 / 2.0 + qv**2 * dt2 * dt2 / 8.0
            Q[i + 3, i] = Q[i, i + 3]
            # Velocity–velocity
            Q[i + 3, i + 3] = qp**2 * dt + qv**2 * dt3 / 3.0
        return Q
