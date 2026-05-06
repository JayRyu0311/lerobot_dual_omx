import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots.robot import Robot

from .omx_follower import OMXFollower
from .config_omx_follower import OMXFollowerConfig
from .config_bi_omx_follower import BiOMXFollowerConfig

logger = logging.getLogger(__name__)


class BiOMXFollower(Robot):
    """
    Dual OMX 5+1 DOF Follower Arms (왼팔 + 오른팔)

    각 팔은 독립적인 OpenRB-150 컨트롤러와 USB 포트를 사용합니다.
    관절 이름에 'left_' / 'right_' 접두사가 붙습니다.
    """

    config_class = BiOMXFollowerConfig
    name = "bi_omx_follower"

    def __init__(self, config: BiOMXFollowerConfig):
        super().__init__(config)
        self.config = config

        self.left_arm = OMXFollower(
            OMXFollowerConfig(
                id=f"{config.id}_left" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=config.left_arm_port,
                disable_torque_on_disconnect=config.left_arm_disable_torque_on_disconnect,
                max_relative_target=config.left_arm_max_relative_target,
                use_degrees=config.left_arm_use_degrees,
                cameras={},
            )
        )
        self.right_arm = OMXFollower(
            OMXFollowerConfig(
                id=f"{config.id}_right" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=config.right_arm_port,
                disable_torque_on_disconnect=config.right_arm_disable_torque_on_disconnect,
                max_relative_target=config.right_arm_max_relative_target,
                use_degrees=config.right_arm_use_degrees,
                cameras={},
            )
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return (
            {f"left_{m}.pos": float for m in self.left_arm.bus.motors}
            | {f"right_{m}.pos": float for m in self.right_arm.bus.motors}
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.left_arm.bus.is_connected
            and self.right_arm.bus.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)
        for cam in self.cameras.values():
            cam.connect()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        print("=== 왼팔 캘리브레이션 ===")
        self.left_arm.calibrate()
        print("=== 오른팔 캘리브레이션 ===")
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        print("=== 왼팔 모터 설정 ===")
        self.left_arm.setup_motors()
        print("=== 오른팔 모터 설정 ===")
        self.right_arm.setup_motors()

    def get_observation(self) -> dict[str, Any]:
        obs: dict[str, Any] = {}
        obs.update({f"left_{k}": v for k, v in self.left_arm.get_observation().items()})
        obs.update({f"right_{k}": v for k, v in self.right_arm.get_observation().items()})
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        left_action  = {k.removeprefix("left_"):  v for k, v in action.items() if k.startswith("left_")}
        right_action = {k.removeprefix("right_"): v for k, v in action.items() if k.startswith("right_")}

        sent_left  = self.left_arm.send_action(left_action)
        sent_right = self.right_arm.send_action(right_action)

        return (
            {f"left_{k}": v for k, v in sent_left.items()}
            | {f"right_{k}": v for k, v in sent_right.items()}
        )

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
