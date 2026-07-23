"""Deterministic synthetic sensors and kinematic odometry for offline replay."""

from __future__ import annotations

import hashlib
import struct
from dataclasses import dataclass
from typing_extensions import assert_never

from .model import Stream


@dataclass(frozen=True, slots=True)
class Point3:
    """One SI point in the synthetic lidar frame."""

    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True, slots=True)
class PointCloudFixture:
    """A tiny finite point cloud with a source acquisition stamp."""

    frame_id: str
    acquisition_time_ns: int
    points: tuple[Point3, ...]

    @property
    def digest(self) -> str:
        """Return a stable payload digest without serializing host-specific metadata."""
        payload = b"".join(struct.pack("<fff", point.x_m, point.y_m, point.z_m) for point in self.points)
        return hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True, slots=True)
class ImuFixture:
    """One timestamped IMU sample in SI units."""

    frame_id: str
    acquisition_time_ns: int
    linear_acceleration_mps2: tuple[float, float, float]
    angular_velocity_radps: tuple[float, float, float]

    @property
    def digest(self) -> str:
        """Return a stable digest of the IMU vectors."""
        values = self.linear_acceleration_mps2 + self.angular_velocity_radps
        return hashlib.sha256(struct.pack("<ffffff", *values)).hexdigest()


@dataclass(frozen=True, slots=True)
class ImageFixture:
    """A small deterministic grayscale frame with an acquisition stamp."""

    frame_id: str
    acquisition_time_ns: int
    width: int
    height: int
    data: bytes

    @property
    def digest(self) -> str:
        """Return a stable image payload digest."""
        return hashlib.sha256(self.data).hexdigest()


@dataclass(frozen=True, slots=True)
class OdomFixture:
    """A deterministic SI/ENU odometry sample."""

    frame_id: str
    child_frame_id: str
    acquisition_time_ns: int
    x_m: float
    y_m: float
    linear_x_mps: float
    linear_y_mps: float

    @property
    def digest(self) -> str:
        """Return a stable digest of kinematic state."""
        payload = struct.pack("<ffff", self.x_m, self.y_m, self.linear_x_mps, self.linear_y_mps)
        return hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True, slots=True)
class SyntheticSample:
    """One generic source sample for deterministic event construction."""

    stream: Stream
    sequence: int
    acquisition_time_ns: int
    frame_id: str
    payload_sha256: str


@dataclass(frozen=True, slots=True)
class DeterministicSensors:
    """Stateless synthetic sensor factory keyed only by seed, tick, and virtual time."""

    seed: int
    tick_duration_ns: int

    def point_cloud(self, tick: int, acquisition_time_ns: int) -> PointCloudFixture:
        """Build a small source-timestamped cloud in the frozen lidar frame."""
        offset = (self.seed % 11) * 0.001 + tick * 0.0025
        return PointCloudFixture(
            frame_id="lidar_link",
            acquisition_time_ns=acquisition_time_ns,
            points=(
                Point3(offset, 0.0, 0.25),
                Point3(offset + 0.1, 0.05, 0.3),
                Point3(offset - 0.05, -0.1, 0.2),
                Point3(offset + 0.02, 0.12, 0.4),
            ),
        )

    def imu(self, tick: int, acquisition_time_ns: int) -> ImuFixture:
        """Build a finite SI IMU sample synchronized to the virtual clock."""
        yaw_rate = ((self.seed + tick) % 7) * 0.001
        return ImuFixture(
            frame_id="lidar_link",
            acquisition_time_ns=acquisition_time_ns,
            linear_acceleration_mps2=(0.0, 0.0, 9.80665),
            angular_velocity_radps=(0.0, 0.0, yaw_rate),
        )

    def image(self, stream: Stream, tick: int, acquisition_time_ns: int) -> ImageFixture:
        """Build a deterministic narrow or wide grayscale image payload."""
        match stream:
            case Stream.NARROW_IMAGE:
                frame_id, width, height = "camera_narrow_optical_frame", 8, 6
            case Stream.WIDE_IMAGE:
                frame_id, width, height = "camera_wide_optical_frame", 10, 6
            case unreachable:
                assert_never(unreachable)
        seed_bytes = f"{self.seed}:{stream.value}:{tick}".encode("ascii")
        digest = hashlib.sha256(seed_bytes).digest()
        pixel_count = width * height
        return ImageFixture(
            frame_id=frame_id,
            acquisition_time_ns=acquisition_time_ns,
            width=width,
            height=height,
            data=(digest * ((pixel_count // len(digest)) + 1))[:pixel_count],
        )

    def odom(self, tick: int, acquisition_time_ns: int) -> OdomFixture:
        """Build a constant-velocity SI/ENU kinematic odometry sample."""
        elapsed_s = tick * self.tick_duration_ns / 1_000_000_000
        return OdomFixture(
            frame_id="odom",
            child_frame_id="base_link",
            acquisition_time_ns=acquisition_time_ns,
            x_m=0.10 * elapsed_s,
            y_m=0.02 * elapsed_s,
            linear_x_mps=0.10,
            linear_y_mps=0.02,
        )

    def sample(self, stream: Stream, tick: int, acquisition_time_ns: int) -> SyntheticSample:
        """Construct one source-neutral sample from its concrete synthetic payload."""
        match stream:
            case Stream.LIDAR_POINTS:
                fixture = self.point_cloud(tick, acquisition_time_ns)
                return SyntheticSample(stream, tick, fixture.acquisition_time_ns, fixture.frame_id, fixture.digest)
            case Stream.LIDAR_IMU:
                fixture = self.imu(tick, acquisition_time_ns)
                return SyntheticSample(stream, tick, fixture.acquisition_time_ns, fixture.frame_id, fixture.digest)
            case Stream.NARROW_IMAGE | Stream.WIDE_IMAGE:
                fixture = self.image(stream, tick, acquisition_time_ns)
                return SyntheticSample(stream, tick, fixture.acquisition_time_ns, fixture.frame_id, fixture.digest)
            case Stream.ODOM:
                fixture = self.odom(tick, acquisition_time_ns)
                return SyntheticSample(stream, tick, fixture.acquisition_time_ns, fixture.frame_id, fixture.digest)
            case unreachable:
                assert_never(unreachable)
