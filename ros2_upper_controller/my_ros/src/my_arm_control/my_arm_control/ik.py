"Six-axis arm inverse kinematics loaded from config/ik.yaml."

from __future__ import annotations

import math
import os
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
import yaml


class Arm6IK:
    def __init__(self, config: dict):
        self.config = dict(config)
        self.input_unit_scale = float(self.config.get("input_unit_scale", 100.0))
        self.bottom_radius = float(self.config["bottom_radius"])
        self.base_height = float(self.config["base_height"])
        self.upper_arm_length = float(self.config["upper_arm_length"])
        self.forearm_length = float(self.config["forearm_length"])
        self.tool_length = float(self.config["tool_length"])
        self.y_min, self.y_max = [float(v) for v in self.config.get("y_limits", [3.0, 20.0])]
        self.default_wrist_roll_deg = float(self.config.get("default_wrist_roll_deg", 0.0))
        self.default_gripper_deg = float(self.config.get("default_gripper_deg", 20.0))
        self.joint_limits_rad = [[float(v) for v in pair] for pair in self.config.get("joint_limits_rad", [])]
        self.joint_output = list(self.config.get("joint_output", []))
        scan = self.config.get("pose_scan_deg", [0, 180, 1])
        self.scan_start = int(scan[0])
        self.scan_stop = int(scan[1])
        self.scan_step = max(1, int(scan[2]))

    @property
    def joint_names(self) -> list[str]:
        return list(self.config.get("joint_names", []))

    @property
    def home_pose_rad(self) -> list[float]:
        return [float(v) for v in self.config.get("home_pose_rad", [])]

    @property
    def probe_pose_rad(self) -> list[float]:
        return [float(v) for v in self.config.get("probe_pose_rad", self.home_pose_rad)]

    def solve_radians(
        self,
        x: float,
        y: float,
        z: float,
        wrist_roll_deg: float | None = None,
        gripper_deg: float | None = None,
    ) -> list[float] | None:
        degrees = self.solve_degrees(x, y, z, wrist_roll_deg, gripper_deg)
        if degrees is None:
            return None
        radians = [math.radians(value) for value in degrees]
        if self.joint_limits_rad and not self.within_limits(radians):
            return None
        return radians

    def solve_degrees(
        self,
        x: float,
        y: float,
        z: float,
        wrist_roll_deg: float | None = None,
        gripper_deg: float | None = None,
    ) -> list[float] | None:
        target_x = float(x) * self.input_unit_scale
        target_y = float(y) * self.input_unit_scale
        target_z = float(z) * self.input_unit_scale
        base_solution = self._solve_4dof_cm(target_x, target_y, target_z)
        if base_solution is None:
            return None

        values = {
            "j1": base_solution[0],
            "j2": base_solution[1],
            "j3": base_solution[2],
            "j4": base_solution[3],
            "wrist_roll": self.default_wrist_roll_deg if wrist_roll_deg is None else float(wrist_roll_deg),
            "gripper": self.default_gripper_deg if gripper_deg is None else float(gripper_deg),
        }
        if not self.joint_output:
            return [values["j1"], values["j2"], values["j3"], values["j4"], values["wrist_roll"], values["gripper"]]

        output = []
        for item in self.joint_output:
            source = str(item.get("source"))
            direction = float(item.get("direction", 1.0))
            offset_deg = float(item.get("offset_deg", 0.0))
            output.append(direction * values[source] + offset_deg)
        return output

    def within_limits(self, joints_rad: Iterable[float]) -> bool:
        joints = list(joints_rad)
        if len(joints) != len(self.joint_limits_rad):
            return False
        for value, limits in zip(joints, self.joint_limits_rad):
            lower, upper = limits
            if value < lower or value > upper:
                return False
        return True

    def _solve_4dof_cm(self, target_x: float, target_y: float, target_z: float) -> list[float] | None:
        target_y = max(self.y_min, min(self.y_max, target_y))
        if abs(target_x) < 1e-6:
            j1 = 90.0
        else:
            j1 = 90.0 - math.degrees(math.atan(target_x / (target_y + self.bottom_radius)))

        arm_len = math.sqrt((target_y + self.bottom_radius) ** 2 + target_x ** 2)
        valid_solutions = []
        for pose_deg in range(self.scan_start, self.scan_stop + 1, self.scan_step):
            pose_rad = math.radians(float(pose_deg))
            wrist_l = arm_len - self.tool_length * math.sin(pose_rad)
            wrist_h = target_z - self.tool_length * math.cos(pose_rad) - self.base_height
            cos_j3 = (
                wrist_l * wrist_l + wrist_h * wrist_h
                - self.upper_arm_length * self.upper_arm_length
                - self.forearm_length * self.forearm_length
            ) / (2.0 * self.upper_arm_length * self.forearm_length)
            if cos_j3 < -1.0 or cos_j3 > 1.0:
                continue

            sin_j3 = math.sqrt(max(0.0, 1.0 - cos_j3 * cos_j3))
            j3 = math.degrees(math.atan2(sin_j3, cos_j3))
            k2 = self.forearm_length * math.sin(math.radians(j3))
            k1 = self.upper_arm_length + self.forearm_length * math.cos(math.radians(j3))
            denom = k1 * k1 + k2 * k2
            if denom <= 1e-6:
                continue

            cos_j2 = (k2 * wrist_l + k1 * wrist_h) / denom
            if cos_j2 < -1.0 or cos_j2 > 1.0:
                continue

            sin_j2 = math.sqrt(max(0.0, 1.0 - cos_j2 * cos_j2))
            j2 = math.degrees(math.atan2(sin_j2, cos_j2))
            j4 = float(pose_deg) - j2 - j3
            if 0.0 <= j2 <= 180.0 and 0.0 <= j3 <= 180.0 and -90.0 <= j4 <= 90.0:
                valid_solutions.append([j1, j2, j3, j4])

        if not valid_solutions:
            return None
        return valid_solutions[(len(valid_solutions) - 1) // 2]


def default_ik_config_path() -> str:
    return os.path.join(get_package_share_directory("my_arm_control"), "config", "ik.yaml")


def load_ik_config(path: str | None = None, key: str = "ik") -> dict:
    config_path = path or default_ik_config_path()
    with open(config_path, "r", encoding="utf-8") as file_obj:
        data = yaml.safe_load(file_obj) or {}
    config = data.get(key)
    if config is None:
        raise KeyError(f"IK config key {key!r} not found in {config_path}")
    return config


def load_arm_ik(path: str | None = None, key: str = "ik") -> Arm6IK:
    return Arm6IK(load_ik_config(path=path, key=key))


__all__ = ["Arm6IK", "default_ik_config_path", "load_arm_ik", "load_ik_config"]
