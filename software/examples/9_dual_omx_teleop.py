#!/usr/bin/env python3
"""
Dual OMX Teleoperation — Leader → Follower

사용법:
  단일 팔 (single arm):
    python 9_dual_omx_teleop.py --mode single \
        --follower-port /dev/ttyUSB0 \
        --leader-port   /dev/ttyUSB1

  양팔 (dual arm):
    python 9_dual_omx_teleop.py --mode dual \
        --left-follower-port  /dev/ttyUSB0 \
        --right-follower-port /dev/ttyUSB1 \
        --left-leader-port    /dev/ttyUSB2 \
        --right-leader-port   /dev/ttyUSB3

  포트 확인: ls /dev/ttyUSB* /dev/ttyACM*
  종료: Ctrl+C
"""

import argparse
import logging
import sys
import time
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# 프로젝트 루트를 sys.path에 추가
ROOT = Path(__file__).resolve().parents[1]  # software/
sys.path.insert(0, str(ROOT))

from src.robots.omx import OMXFollower, OMXFollowerConfig, BiOMXFollower, BiOMXFollowerConfig
from src.teleporators.omx_leader import OMXLeader, OMXLeaderConfig, BiOMXLeader, BiOMXLeaderConfig


def busy_wait(dt: float) -> None:
    """정밀 대기 (sleep + spin)"""
    end = time.perf_counter() + dt
    while time.perf_counter() < end:
        pass


def teleop_loop_single(
    leader: OMXLeader,
    follower: OMXFollower,
    fps: int = 60,
) -> None:
    """단일 팔 teleop 루프"""
    period = 1.0 / fps
    logger.info(f"Single-arm teleop 시작 ({fps} Hz). 종료: Ctrl+C")

    while True:
        loop_start = time.perf_counter()

        action = leader.get_action()
        follower.send_action(action)

        dt = time.perf_counter() - loop_start
        busy_wait(max(0.0, period - dt))
        actual_fps = 1.0 / (time.perf_counter() - loop_start)
        print(f"\r{actual_fps:.1f} Hz  ", end="", flush=True)


def teleop_loop_dual(
    leader: BiOMXLeader,
    follower: BiOMXFollower,
    fps: int = 60,
) -> None:
    """양팔 teleop 루프"""
    period = 1.0 / fps
    logger.info(f"Dual-arm teleop 시작 ({fps} Hz). 종료: Ctrl+C")

    while True:
        loop_start = time.perf_counter()

        action = leader.get_action()
        follower.send_action(action)

        dt = time.perf_counter() - loop_start
        busy_wait(max(0.0, period - dt))
        actual_fps = 1.0 / (time.perf_counter() - loop_start)
        print(f"\r{actual_fps:.1f} Hz  ", end="", flush=True)


def run_single(args: argparse.Namespace) -> None:
    follower_cfg = OMXFollowerConfig(
        port=args.follower_port,
        max_relative_target=args.max_relative_target,
    )
    leader_cfg = OMXLeaderConfig(
        port=args.leader_port,
        gripper_open_pos=args.gripper_open_pos,
    )

    follower = OMXFollower(follower_cfg)
    leader   = OMXLeader(leader_cfg)

    logger.info("=== Follower 연결 중 ===")
    follower.connect(calibrate=not args.skip_calibration)
    logger.info("=== Leader 연결 중 ===")
    leader.connect(calibrate=not args.skip_calibration)

    try:
        teleop_loop_single(leader, follower, fps=args.fps)
    except KeyboardInterrupt:
        print()
        logger.info("종료 요청")
    finally:
        logger.info("연결 해제 중...")
        leader.disconnect()
        follower.disconnect()
        logger.info("완료.")


def run_dual(args: argparse.Namespace) -> None:
    follower_cfg = BiOMXFollowerConfig(
        left_arm_port=args.left_follower_port,
        right_arm_port=args.right_follower_port,
        left_arm_max_relative_target=args.max_relative_target,
        right_arm_max_relative_target=args.max_relative_target,
    )
    leader_cfg = BiOMXLeaderConfig(
        left_arm_port=args.left_leader_port,
        right_arm_port=args.right_leader_port,
        gripper_open_pos=args.gripper_open_pos,
    )

    follower = BiOMXFollower(follower_cfg)
    leader   = BiOMXLeader(leader_cfg)

    logger.info("=== Follower 양팔 연결 중 ===")
    follower.connect(calibrate=not args.skip_calibration)
    logger.info("=== Leader 양팔 연결 중 ===")
    leader.connect(calibrate=not args.skip_calibration)

    try:
        teleop_loop_dual(leader, follower, fps=args.fps)
    except KeyboardInterrupt:
        print()
        logger.info("종료 요청")
    finally:
        logger.info("연결 해제 중...")
        leader.disconnect()
        follower.disconnect()
        logger.info("완료.")


def main() -> None:
    parser = argparse.ArgumentParser(description="OMX Dual-Arm Teleoperation")
    parser.add_argument("--mode", choices=["single", "dual"], default="dual",
                        help="단일 팔(single) 또는 양팔(dual) 모드")

    # single arm
    parser.add_argument("--follower-port", default="/dev/ttyUSB0",
                        help="[single] follower OpenRB-150 포트")
    parser.add_argument("--leader-port", default="/dev/ttyUSB1",
                        help="[single] leader OpenRB-150 포트")

    # dual arm
    parser.add_argument("--left-follower-port",  default="/dev/ttyUSB0")
    parser.add_argument("--right-follower-port", default="/dev/ttyUSB1")
    parser.add_argument("--left-leader-port",    default="/dev/ttyUSB2")
    parser.add_argument("--right-leader-port",   default="/dev/ttyUSB3")

    # 공통
    parser.add_argument("--fps", type=int, default=60, help="제어 루프 주파수")
    parser.add_argument("--gripper-open-pos", type=float, default=50.0,
                        help="leader gripper 기본 열림 위치 (0~100)")
    parser.add_argument("--max-relative-target", type=float, default=None,
                        help="follower 안전 제한: 한 스텝 최대 이동값 (None=무제한)")
    parser.add_argument("--skip-calibration", action="store_true",
                        help="캘리브레이션 건너뜀 (저장된 파일 자동 사용)")

    args = parser.parse_args()

    if args.mode == "single":
        run_single(args)
    else:
        run_dual(args)


if __name__ == "__main__":
    main()
