from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("bi_omx_follower")
@dataclass
class BiOMXFollowerConfig(RobotConfig):
    # 각 팔의 OpenRB-150 USB 포트
    left_arm_port: str
    right_arm_port: str

    left_arm_disable_torque_on_disconnect: bool = True
    left_arm_max_relative_target: float | dict[str, float] | None = None
    left_arm_use_degrees: bool = False

    right_arm_disable_torque_on_disconnect: bool = True
    right_arm_max_relative_target: float | dict[str, float] | None = None
    right_arm_use_degrees: bool = False

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
