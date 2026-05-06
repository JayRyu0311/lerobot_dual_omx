from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_omx_leader")
@dataclass
class BiOMXLeaderConfig(TeleoperatorConfig):
    left_arm_port: str
    right_arm_port: str

    gripper_open_pos: float = 50.0
