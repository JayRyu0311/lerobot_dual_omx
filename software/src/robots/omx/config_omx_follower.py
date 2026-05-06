from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("omx_follower")
@dataclass
class OMXFollowerConfig(RobotConfig):
    # USB 포트 (OpenRB-150 연결, 예: /dev/ttyUSB0 or /dev/ttyACM0)
    port: str

    disable_torque_on_disconnect: bool = True

    # 안전을 위해 한 스텝에 이동 가능한 최대 상대 목표값 제한
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    use_degrees: bool = False
