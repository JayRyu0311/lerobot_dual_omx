#!/usr/bin/env python3
"""
OMX Dual-Arm Dataset Recording — Leader → Follower + 카메라

사용법:
  단일 팔 recording:
    python 10_dual_omx_record.py --mode single \
        --follower-port /dev/ttyUSB0 \
        --leader-port   /dev/ttyUSB1 \
        --repo-id <hf_username>/<dataset_name> \
        --task  "Pick up the cube"

  양팔 recording:
    python 10_dual_omx_record.py --mode dual \
        --left-follower-port  /dev/ttyUSB0 \
        --right-follower-port /dev/ttyUSB1 \
        --left-leader-port    /dev/ttyUSB2 \
        --right-leader-port   /dev/ttyUSB3 \
        --repo-id <hf_username>/<dataset_name> \
        --task  "Hand over the cube"

키보드 단축키 (recording 중):
  →  (오른쪽 화살표) : 현재 에피소드 저장하고 다음으로
  ←  (왼쪽 화살표)  : 현재 에피소드 버리고 다시 녹화
  ESC                : 녹화 종료 (저장된 에피소드는 유지)
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# ── lerobot 풀 환경이 필요 (torch 포함) ───────────────────────────────────────
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.utils.control_utils import init_keyboard_listener, is_headless
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import log_say

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[1]  # software/
sys.path.insert(0, str(ROOT))

from src.robots.omx import OMXFollower, OMXFollowerConfig, BiOMXFollower, BiOMXFollowerConfig
from src.teleporators.omx_leader import OMXLeader, OMXLeaderConfig, BiOMXLeader, BiOMXLeaderConfig


# ─────────────────────────────────────────────────────────────────────────────
# Record loop
# ─────────────────────────────────────────────────────────────────────────────

def record_loop(
    robot,
    teleop,
    dataset,
    events: dict,
    fps: int,
    control_time_s: float,
    task: str,
    display_data: bool = False,
):
    """한 에피소드(또는 reset 구간)를 녹화합니다."""
    period = 1.0 / fps
    timestamp = 0.0
    start = time.perf_counter()

    while timestamp < control_time_s:
        loop_start = time.perf_counter()

        # 사용자가 → 를 누르면 조기 종료
        if events["exit_early"]:
            events["exit_early"] = False
            break

        # 관찰값 수집
        obs = robot.get_observation()

        # 리더에서 액션 수집
        action = teleop.get_action()

        # 로봇에 명령 전송
        sent_action = robot.send_action(action)

        # 데이터셋에 프레임 추가
        if dataset is not None:
            obs_frame    = build_dataset_frame(dataset.features, obs,         prefix="observation")
            action_frame = build_dataset_frame(dataset.features, sent_action, prefix="action")
            dataset.add_frame({**obs_frame, **action_frame, "task": task})

        # FPS 유지
        dt = time.perf_counter() - loop_start
        busy_wait(max(0.0, period - dt))
        timestamp = time.perf_counter() - start


# ─────────────────────────────────────────────────────────────────────────────
# 메인 recording 루틴
# ─────────────────────────────────────────────────────────────────────────────

def run_record(args: argparse.Namespace) -> None:
    # ── 로봇 / 리더 생성 ──────────────────────────────────────────────────────
    if args.mode == "single":
        follower = OMXFollower(OMXFollowerConfig(
            port=args.follower_port,
            cameras=_parse_cameras(args),
        ))
        leader = OMXLeader(OMXLeaderConfig(
            port=args.leader_port,
            gripper_open_pos=args.gripper_open_pos,
        ))
        robot_type = "omx_follower"
    else:
        follower = BiOMXFollower(BiOMXFollowerConfig(
            left_arm_port=args.left_follower_port,
            right_arm_port=args.right_follower_port,
            cameras=_parse_cameras(args),
        ))
        leader = BiOMXLeader(BiOMXLeaderConfig(
            left_arm_port=args.left_leader_port,
            right_arm_port=args.right_leader_port,
            gripper_open_pos=args.gripper_open_pos,
        ))
        robot_type = "bi_omx_follower"

    # ── 연결 ──────────────────────────────────────────────────────────────────
    logger.info("=== Follower 연결 중 ===")
    follower.connect(calibrate=not args.skip_calibration)
    logger.info("=== Leader 연결 중 ===")
    leader.connect(calibrate=not args.skip_calibration)

    # ── 데이터셋 피처 빌드 ────────────────────────────────────────────────────
    action_features = hw_to_dataset_features(follower.action_features,      "action",      args.video)
    obs_features    = hw_to_dataset_features(follower.observation_features, "observation", args.video)
    dataset_features = {**action_features, **obs_features}

    # ── 데이터셋 생성 / 재개 ──────────────────────────────────────────────────
    n_cams = len(follower.cameras) if hasattr(follower, "cameras") else 0
    if args.resume:
        dataset = LeRobotDataset(
            args.repo_id,
            root=args.dataset_root or None,
        )
        if n_cams > 0:
            dataset.start_image_writer(
                num_processes=args.num_image_writer_processes,
                num_threads=args.num_image_writer_threads_per_camera * n_cams,
            )
    else:
        dataset = LeRobotDataset.create(
            args.repo_id,
            args.fps,
            root=args.dataset_root or None,
            robot_type=robot_type,
            features=dataset_features,
            use_videos=args.video,
            image_writer_processes=args.num_image_writer_processes,
            image_writer_threads=args.num_image_writer_threads_per_camera * max(n_cams, 1),
        )

    # ── 키보드 리스너 ─────────────────────────────────────────────────────────
    listener, events = init_keyboard_listener()
    logger.info(
        "키보드 단축키: [→] 에피소드 저장  [←] 다시 녹화  [ESC] 종료"
    )

    # ── Recording 루프 ────────────────────────────────────────────────────────
    try:
        with VideoEncodingManager(dataset):
            recorded = 0
            while recorded < args.num_episodes and not events["stop_recording"]:
                ep_idx = dataset.num_episodes
                log_say(f"Recording episode {ep_idx}", args.play_sounds)
                logger.info(f"▶ 에피소드 {ep_idx} 녹화 시작 ({args.episode_time_s}초)")

                record_loop(
                    robot=follower,
                    teleop=leader,
                    dataset=dataset,
                    events=events,
                    fps=args.fps,
                    control_time_s=args.episode_time_s,
                    task=args.task,
                    display_data=args.display_data,
                )

                # ── 다시 녹화 요청 ────────────────────────────────────────
                if events["rerecord_episode"]:
                    log_say("Re-record episode", args.play_sounds)
                    logger.info("← 에피소드 버림, 다시 녹화합니다.")
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                # ── 에피소드 저장 ─────────────────────────────────────────
                log_say("Saving episode", args.play_sounds, blocking=True)
                dataset.save_episode()
                recorded += 1
                logger.info(f"✓ 에피소드 {ep_idx} 저장 완료 ({recorded}/{args.num_episodes})")

                # ── reset 구간 (마지막 에피소드 제외) ────────────────────
                if recorded < args.num_episodes and not events["stop_recording"]:
                    log_say("Reset the environment", args.play_sounds)
                    logger.info(f"환경 리셋 ({args.reset_time_s}초) — 로봇을 초기 위치로 되돌리세요.")
                    record_loop(
                        robot=follower,
                        teleop=leader,
                        dataset=None,      # reset 구간은 저장 안 함
                        events=events,
                        fps=args.fps,
                        control_time_s=args.reset_time_s,
                        task=args.task,
                    )

    except KeyboardInterrupt:
        logger.info("Ctrl+C — 녹화 중단")
    finally:
        log_say("Stop recording", args.play_sounds, blocking=True)
        leader.disconnect()
        follower.disconnect()
        if not is_headless() and listener is not None:
            listener.stop()

    # ── HuggingFace Hub 업로드 ────────────────────────────────────────────────
    if args.push_to_hub and recorded > 0:
        logger.info(f"HuggingFace Hub에 업로드 중: {args.repo_id}")
        dataset.push_to_hub(private=args.private)
        logger.info("업로드 완료.")

    logger.info(f"총 {recorded}개 에피소드 저장. 데이터셋: {args.repo_id}")


# ─────────────────────────────────────────────────────────────────────────────
# 카메라 파싱 헬퍼
# ─────────────────────────────────────────────────────────────────────────────

def _parse_cameras(args: argparse.Namespace) -> dict:
    """
    --cameras 인자를 파싱합니다.
    형식: "name:/dev/video0:640:480:30,name2:..."
    """
    if not args.cameras:
        return {}

    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

    cameras = {}
    for entry in args.cameras.split(","):
        parts = entry.strip().split(":")
        if len(parts) != 5:
            raise ValueError(
                f"카메라 형식 오류: '{entry}'\n"
                "올바른 형식: 이름:/dev/videoX:너비:높이:fps"
            )
        name, path, w, h, fps = parts
        cameras[name.strip()] = OpenCVCameraConfig(
            index_or_path=path.strip(),
            width=int(w),
            height=int(h),
            fps=int(fps),
        )
    return cameras


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="OMX Dual-Arm Dataset Recording",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # 모드
    parser.add_argument("--mode", choices=["single", "dual"], default="dual")

    # 포트 (single)
    parser.add_argument("--follower-port", default="/dev/ttyUSB0")
    parser.add_argument("--leader-port",   default="/dev/ttyUSB1")

    # 포트 (dual)
    parser.add_argument("--left-follower-port",  default="/dev/ttyUSB0")
    parser.add_argument("--right-follower-port", default="/dev/ttyUSB1")
    parser.add_argument("--left-leader-port",    default="/dev/ttyUSB2")
    parser.add_argument("--right-leader-port",   default="/dev/ttyUSB3")

    # 데이터셋
    parser.add_argument("--repo-id", required=True,
                        help="HuggingFace 데이터셋 이름 (예: my_user/omx-pick-cube)")
    parser.add_argument("--task", required=True,
                        help="태스크 설명 (예: 'Pick up the red cube')")
    parser.add_argument("--dataset-root", default=None,
                        help="로컬 저장 경로 (기본: ~/.cache/huggingface/lerobot)")
    parser.add_argument("--num-episodes", type=int, default=50)
    parser.add_argument("--episode-time-s", type=float, default=60.0,
                        help="에피소드 녹화 시간 (초)")
    parser.add_argument("--reset-time-s", type=float, default=30.0,
                        help="환경 리셋 시간 (초)")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--video", action="store_true", default=True,
                        help="프레임을 비디오로 인코딩")
    parser.add_argument("--push-to-hub", action="store_true", default=False)
    parser.add_argument("--private", action="store_true", default=False)
    parser.add_argument("--resume", action="store_true", default=False,
                        help="기존 데이터셋에 이어서 녹화")
    parser.add_argument("--display-data", action="store_true", default=False,
                        help="Rerun으로 실시간 시각화")
    parser.add_argument("--play-sounds", action="store_true", default=False,
                        help="음성 안내 활성화")

    # 카메라
    parser.add_argument(
        "--cameras", default=None,
        help=(
            "카메라 목록 (쉼표 구분): 이름:/dev/videoX:너비:높이:fps\n"
            "예: head:/dev/video0:640:480:30,left_wrist:/dev/video2:640:480:30"
        ),
    )
    parser.add_argument("--num-image-writer-processes",       type=int, default=0)
    parser.add_argument("--num-image-writer-threads-per-camera", type=int, default=4)

    # 기타
    parser.add_argument("--gripper-open-pos", type=float, default=50.0)
    parser.add_argument("--skip-calibration", action="store_true",
                        help="저장된 캘리브레이션 파일 자동 사용")

    args = parser.parse_args()
    run_record(args)


if __name__ == "__main__":
    main()
