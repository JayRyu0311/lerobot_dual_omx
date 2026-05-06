from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("omx_leader")
@dataclass
class OMXLeaderConfig(TeleoperatorConfig):
    # USB 포트 (OpenRB-150 연결, 예: /dev/ttyUSB0 or /dev/ttyACM0)
    port: str

    # gripper를 current-based position mode로 열어두는 목표값 (0~100)
    gripper_open_pos: float = 50.0
