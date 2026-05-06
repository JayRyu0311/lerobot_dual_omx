import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
from lerobot.robots.robot import Robot

from lerobot.robots.utils import ensure_safe_goal_position
from .config_omx_follower import OMXFollowerConfig

logger = logging.getLogger(__name__)


class OMXFollower(Robot):
    """
    OpenManipulator-X (OMX) 5+1 DOF Follower Arm

    모터 구성 (OpenRB-150 컨트롤러):
      ID 11: XL430-W250-T  — shoulder_pan
      ID 12: XL430-W250-T  — shoulder_lift
      ID 13: XL430-W250-T  — elbow_flex
      ID 14: XL330-M288-T  — wrist_flex
      ID 15: XL330-M288-T  — wrist_roll
      ID 16: XL330-M288-T  — gripper
    """

    config_class = OMXFollowerConfig
    name = "omx_follower"

    def __init__(self, config: OMXFollowerConfig):
        super().__init__(config)
        self.config = config
        norm = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        self.bus = DynamixelMotorsBus(
            port=config.port,
            motors={
                "shoulder_pan":  Motor(11, "xl430-w250", norm),
                "shoulder_lift": Motor(12, "xl430-w250", norm),
                "elbow_flex":    Motor(13, "xl430-w250", norm),
                "wrist_flex":    Motor(14, "xl330-m288", norm),
                "wrist_roll":    Motor(15, "xl330-m288", norm),
                "gripper":       Motor(16, "xl330-m288", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

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
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info("캘리브레이션 파일 없음 또는 불일치 — 캘리브레이션을 시작합니다.")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        self.bus.disable_torque()
        if self.calibration:
            user_input = input(
                f"저장된 캘리브레이션 파일을 사용하려면 ENTER, 새로 캘리브레이션하려면 'c' 입력: "
            )
            if user_input.strip().lower() != "c":
                logger.info("저장된 캘리브레이션을 모터에 씁니다.")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\n{self} 캘리브레이션 시작")
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        input("OMX follower를 가동 범위 중간 위치로 이동한 후 ENTER를 누르세요...")
        homing_offsets = self.bus.set_half_turn_homings()

        # shoulder_pan, wrist_roll은 360° 이상 회전 가능 → 전체 범위 고정
        full_turn_motors = ["shoulder_pan", "wrist_roll"]
        limited_motors = [m for m in self.bus.motors if m not in full_turn_motors]

        print(
            f"gripper를 제외한 모든 관절을 순서대로 전체 가동 범위로 움직이세요.\n"
            "완료되면 ENTER를 누르세요..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(limited_motors)
        for motor in full_turn_motors:
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        self.calibration = {
            motor: MotorCalibration(
                id=info.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
            for motor, info in self.bus.motors.items()
        }
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"캘리브레이션 저장: {self.calibration_fpath}")

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                if motor != "gripper":
                    # 360° 이상 회전을 허용하는 extended position mode
                    self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)
            # gripper는 current-based position mode로 과부하 방지
            self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"'{motor}' 모터만 컨트롤러에 연결하고 ENTER를 누르세요.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' 모터 ID → {self.bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        obs = self.bus.sync_read("Present_Position")
        obs = {f"{k}.pos": v for k, v in obs.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal = {k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")}

        if self.config.max_relative_target is not None:
            present = self.bus.sync_read("Present_Position")
            goal = ensure_safe_goal_position(
                {k: (g, present[k]) for k, g in goal.items()},
                self.config.max_relative_target,
            )

        self.bus.sync_write("Goal_Position", goal)
        return {f"{k}.pos": v for k, v in goal.items()}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
