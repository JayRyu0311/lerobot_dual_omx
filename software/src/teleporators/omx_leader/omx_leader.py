import logging
import time

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_omx_leader import OMXLeaderConfig

logger = logging.getLogger(__name__)


class OMXLeader(Teleoperator):
    """
    OpenManipulator-X (OMX) 5+1 DOF Leader Arm

    사람이 직접 잡고 움직이는 리더 암. 토크를 비활성화하여 자유롭게 움직일 수 있습니다.
    gripper만 current-based position mode로 토크를 활성화하여 스프링 역할을 합니다.

    모터 구성 (OpenRB-150 컨트롤러):
      ID 1: XL330-M288-T  — shoulder_pan
      ID 2: XL330-M288-T  — shoulder_lift
      ID 3: XL330-M288-T  — elbow_flex
      ID 4: XL330-M288-T  — wrist_flex
      ID 5: XL330-M288-T  — wrist_roll
      ID 6: XL330-M077-T  — gripper
    """

    config_class = OMXLeaderConfig
    name = "omx_leader"

    def __init__(self, config: OMXLeaderConfig):
        super().__init__(config)
        self.config = config

        self.bus = DynamixelMotorsBus(
            port=config.port,
            motors={
                "shoulder_pan":  Motor(1, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "elbow_flex":    Motor(3, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "wrist_flex":    Motor(4, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "wrist_roll":    Motor(5, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "gripper":       Motor(6, "xl330-m077", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info("캘리브레이션 파일 없음 또는 불일치 — 캘리브레이션을 시작합니다.")
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        self.bus.disable_torque()
        if self.calibration:
            user_input = input(
                f"저장된 캘리브레이션을 사용하려면 ENTER, 새로 캘리브레이션하려면 'c' 입력: "
            )
            if user_input.strip().lower() != "c":
                logger.info("저장된 캘리브레이션을 모터에 씁니다.")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\n{self} 캘리브레이션 시작")
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        input("OMX leader를 가동 범위 중간 위치로 이동한 후 ENTER를 누르세요...")
        homing_offsets = self.bus.set_half_turn_homings()

        full_turn_motors = ["shoulder_pan", "wrist_roll"]
        limited_motors = [m for m in self.bus.motors if m not in full_turn_motors]

        print(
            "gripper를 제외한 모든 관절을 순서대로 전체 가동 범위로 움직이세요.\n"
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
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            if motor != "gripper":
                # 리더 암은 토크 해제 — 사람이 자유롭게 움직임
                self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        # gripper만 current-based position mode → 손가락으로 누르면 열리고 놓으면 닫힘
        self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)
        self.bus.enable_torque("gripper")
        if self.is_calibrated:
            self.bus.write("Goal_Position", "gripper", self.config.gripper_open_pos)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"'{motor}' 모터만 컨트롤러에 연결하고 ENTER를 누르세요.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' 모터 ID → {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # 현재 force feedback 미구현
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
