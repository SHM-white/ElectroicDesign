"""Exact local surfaces for deterministic Livox and PointCloud2 tests."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Header:
    stamp_ns: int
    frame_id: str


@dataclass(frozen=True, slots=True)
class LivoxPoint:
    offset_time: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int


@dataclass(frozen=True, slots=True)
class LivoxCustomMsg:
    header: Header
    timebase: int
    point_num: int
    lidar_id: int
    rsvd: tuple[int, int, int]
    points: tuple[LivoxPoint, ...]


@dataclass(frozen=True, slots=True)
class Imu:
    header: Header


@dataclass(frozen=True, slots=True)
class PointCloud2:
    header: Header
    fields: tuple[str, ...]
    point_times_ns: tuple[int, ...]
