"""Deterministic planar geometry checks for field profile topology."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Protocol


EPSILON = 1e-9


class HasPlanarCoordinate(Protocol):
    """A value with a planar SI coordinate."""

    x_m: float
    y_m: float


def polygons_touch_or_overlap(
    first: Sequence[HasPlanarCoordinate], second: Sequence[HasPlanarCoordinate]
) -> bool:
    """Return whether two polygon interiors or boundaries intersect."""
    for first_start, first_end in _edges(first):
        for second_start, second_end in _edges(second):
            if segments_intersect(first_start, first_end, second_start, second_end):
                return True
    return point_in_polygon(first[0], second) or point_in_polygon(second[0], first)


def polygon_self_intersects(vertices: Sequence[HasPlanarCoordinate]) -> bool:
    """Return whether nonadjacent polygon edges intersect."""
    edge_count = len(vertices)
    for first_index, (first_start, first_end) in enumerate(_edges(vertices)):
        for second_index, (second_start, second_end) in enumerate(_edges(vertices)):
            if first_index >= second_index:
                continue
            if (first_index - second_index) % edge_count in (1, edge_count - 1):
                continue
            if segments_intersect(first_start, first_end, second_start, second_end):
                return True
    return False


def polygon_strictly_contains(
    outer: Sequence[HasPlanarCoordinate], inner: Sequence[HasPlanarCoordinate]
) -> bool:
    """Return whether all of the inner polygon is strictly within the outer polygon."""
    for vertex in inner:
        if point_on_polygon_boundary(vertex, outer) or not point_in_polygon(vertex, outer):
            return False
    for outer_start, outer_end in _edges(outer):
        for inner_start, inner_end in _edges(inner):
            if segments_intersect(outer_start, outer_end, inner_start, inner_end):
                return False
    return True


def polygon_has_area(vertices: Sequence[HasPlanarCoordinate]) -> bool:
    """Return whether a polygon has nonzero signed area."""
    twice_area = sum(
        start.x_m * end.y_m - end.x_m * start.y_m for start, end in _edges(vertices)
    )
    return not math.isclose(twice_area, 0.0, abs_tol=EPSILON)


def segments_intersect(
    first_start: HasPlanarCoordinate,
    first_end: HasPlanarCoordinate,
    second_start: HasPlanarCoordinate,
    second_end: HasPlanarCoordinate,
) -> bool:
    """Return whether closed line segments have any common point."""
    first_orientation = _orientation(first_start, first_end, second_start)
    second_orientation = _orientation(first_start, first_end, second_end)
    third_orientation = _orientation(second_start, second_end, first_start)
    fourth_orientation = _orientation(second_start, second_end, first_end)
    if _opposite(first_orientation, second_orientation) and _opposite(
        third_orientation, fourth_orientation
    ):
        return True
    return (
        math.isclose(first_orientation, 0.0, abs_tol=EPSILON)
        and _point_on_segment(second_start, first_start, first_end)
    ) or (
        math.isclose(second_orientation, 0.0, abs_tol=EPSILON)
        and _point_on_segment(second_end, first_start, first_end)
    ) or (
        math.isclose(third_orientation, 0.0, abs_tol=EPSILON)
        and _point_on_segment(first_start, second_start, second_end)
    ) or (
        math.isclose(fourth_orientation, 0.0, abs_tol=EPSILON)
        and _point_on_segment(first_end, second_start, second_end)
    )


def has_nonparallel_segments(segments: Sequence[tuple[HasPlanarCoordinate, HasPlanarCoordinate]]) -> bool:
    """Return whether boundary constraints can observe planar position and heading."""
    for first_index, (first_start, first_end) in enumerate(segments):
        first_x = first_end.x_m - first_start.x_m
        first_y = first_end.y_m - first_start.y_m
        for second_start, second_end in segments[first_index + 1 :]:
            second_x = second_end.x_m - second_start.x_m
            second_y = second_end.y_m - second_start.y_m
            cross_product = first_x * second_y - first_y * second_x
            length_product = math.hypot(first_x, first_y) * math.hypot(second_x, second_y)
            if abs(cross_product) > EPSILON * length_product:
                return True
    return False


def point_in_polygon(point: HasPlanarCoordinate, vertices: Sequence[HasPlanarCoordinate]) -> bool:
    """Return whether a point lies strictly in a polygon interior."""
    intersections = False
    for start, end in _edges(vertices):
        crosses_horizontal_ray = (start.y_m > point.y_m) != (end.y_m > point.y_m)
        if crosses_horizontal_ray:
            ray_x = (end.x_m - start.x_m) * (point.y_m - start.y_m) / (end.y_m - start.y_m)
            if point.x_m < start.x_m + ray_x:
                intersections = not intersections
    return intersections


def point_on_polygon_boundary(
    point: HasPlanarCoordinate, vertices: Sequence[HasPlanarCoordinate]
) -> bool:
    """Return whether a point lies on a polygon boundary."""
    return any(_point_on_segment(point, start, end) for start, end in _edges(vertices))


def _edges(
    vertices: Sequence[HasPlanarCoordinate],
) -> tuple[tuple[HasPlanarCoordinate, HasPlanarCoordinate], ...]:
    return tuple(zip(vertices, (*vertices[1:], vertices[0]), strict=True))


def _orientation(
    start: HasPlanarCoordinate, end: HasPlanarCoordinate, point: HasPlanarCoordinate
) -> float:
    return (end.x_m - start.x_m) * (point.y_m - start.y_m) - (end.y_m - start.y_m) * (
        point.x_m - start.x_m
    )


def _opposite(first: float, second: float) -> bool:
    return (first > EPSILON and second < -EPSILON) or (first < -EPSILON and second > EPSILON)


def _point_on_segment(
    point: HasPlanarCoordinate, start: HasPlanarCoordinate, end: HasPlanarCoordinate
) -> bool:
    return math.isclose(_orientation(start, end, point), 0.0, abs_tol=EPSILON) and (
        min(start.x_m, end.x_m) - EPSILON <= point.x_m <= max(start.x_m, end.x_m) + EPSILON
        and min(start.y_m, end.y_m) - EPSILON <= point.y_m <= max(start.y_m, end.y_m) + EPSILON
    )
