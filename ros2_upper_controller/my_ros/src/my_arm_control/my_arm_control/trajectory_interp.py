"Trajectory helpers for the arm controller."

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass
class Segment:
    "A normalized motion segment."

    positions: list[float]
    duration_s: float


def duration_to_seconds(duration_msg) -> float:
    return float(duration_msg.sec) + float(duration_msg.nanosec) / 1e9


def normalize_trajectory(msg, joint_names: Iterable[str], default_segment_s: float) -> list[Segment]:
    configured_names = list(joint_names)
    if not msg.points:
        raise ValueError('trajectory has no points')

    if msg.joint_names and list(msg.joint_names) != configured_names:
        raise ValueError('joint_names must match configured arm joint order exactly')

    segments: list[Segment] = []
    previous_time = 0.0
    for point in msg.points:
        positions = list(point.positions)
        if len(positions) != len(configured_names):
            raise ValueError('point positions length does not match joint_names')

        time_from_start_s = duration_to_seconds(point.time_from_start)
        if time_from_start_s <= previous_time:
            time_from_start_s = previous_time + default_segment_s

        segments.append(Segment(positions=positions, duration_s=time_from_start_s - previous_time))
        previous_time = time_from_start_s

    return segments


def interpolate_segment(start_positions: Iterable[float], target_positions: Iterable[float], duration_s: float, interpolation_step_s: float) -> tuple[list[list[float]], float]:
    start = list(start_positions)
    target = list(target_positions)
    if len(start) != len(target):
        raise ValueError('start and target lengths do not match')

    duration_s = max(duration_s, 1e-3)
    interpolation_step_s = max(interpolation_step_s, 1e-3)
    steps = max(1, int(math.ceil(duration_s / interpolation_step_s)))
    dt = duration_s / float(steps)

    frames: list[list[float]] = []
    for index in range(1, steps + 1):
        ratio = float(index) / float(steps)
        frames.append([
            start_value + (target_value - start_value) * ratio
            for start_value, target_value in zip(start, target)
        ])

    return frames, dt
