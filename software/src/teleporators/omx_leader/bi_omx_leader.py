import logging
from functools import cached_property

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.teleoperators.teleoperator import Teleoperator

from .omx_leader import OMXLeader
from .config_omx_leader import OMXLeaderConfig
from .config_bi_omx_leader import BiOMXLeaderConfig

logger = logging.getLogger(__name__)


class BiOMXLeader(Teleoperator):
    """
    Dual OMX 5+1 DOF Leader Arms (왼팔 + 오른팔)

    각 팔은 독립적인 OpenRB-150 컨트롤러와 USB 포트를 사용합니다.
    관절 이름에 'left_' / 'right_' 접두사가 붙습니다.
    """

    config_class = BiOMXLeaderConfig
    name = "bi_omx_leader"

    def __init__(self, config: BiOMXLeaderConfig):
        super().__init__(config)
        self.config = config

        self.left_arm = OMXLeader(
            OMXLeaderConfig(
                id=f"{config.id}_left" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=config.left_arm_port,
                gripper_open_pos=config.gripper_open_pos,
            )
        )
        self.right_arm = OMXLeader(
            OMXLeaderConfig(
                id=f"{config.id}_right" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=config.right_arm_port,
                gripper_open_pos=config.gripper_open_pos,
            )
        )

    @cached_property
    def action_features(self) -> dict[str, type]:
        return (
            {f"left_{m}.pos": float for m in self.left_arm.bus.motors}
            | {f"right_{m}.pos": float for m in self.right_arm.bus.motors}
        )

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        print("=== 왼팔 리더 연결 ===")
        self.left_arm.connect(calibrate)
        print("=== 오른팔 리더 연결 ===")
        self.right_arm.connect(calibrate)
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        print("=== 왼팔 리더 캘리브레이션 ===")
        self.left_arm.calibrate()
        print("=== 오른팔 리더 캘리브레이션 ===")
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        print("=== 왼팔 리더 모터 설정 ===")
        self.left_arm.setup_motors()
        print("=== 오른팔 리더 모터 설정 ===")
        self.right_arm.setup_motors()

    def get_action(self) -> dict[str, float]:
        action: dict[str, float] = {}
        action.update({f"left_{k}": v for k, v in self.left_arm.get_action().items()})
        action.update({f"right_{k}": v for k, v in self.right_arm.get_action().items()})
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        logger.info(f"{self} disconnected.")
