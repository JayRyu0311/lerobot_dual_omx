# OMX Dual-Arm — Setup, Teleoperation, Dataset Recording

OpenManipulator-X (OMX) 5+1 DOF 두 개를 사용하는 **Dual-Arm** 구성의
설치·조작·데이터 수집 가이드입니다.

---

## 목차

1. [하드웨어 구성](#1-하드웨어-구성)
2. [Python 가상환경 설정](#2-python-가상환경-설정)
3. [USB 포트 설정](#3-usb-포트-설정)
4. [모터 ID 확인](#4-모터-id-확인)
5. [캘리브레이션](#5-캘리브레이션)
6. [Teleoperation (원격 조종)](#6-teleoperation-원격-조종)
7. [Dataset Recording](#7-dataset-recording)
8. [파일 구조](#8-파일-구조)
9. [트러블슈팅](#9-트러블슈팅)

---

## 1. 하드웨어 구성

### Follower Arm (2× 필요)

| 관절 | 모터 모델 | ID |
|------|-----------|-----|
| shoulder_pan | XL430-W250-T | 11 |
| shoulder_lift | XL430-W250-T | 12 |
| elbow_flex | XL430-W250-T | 13 |
| wrist_flex | XL330-M288-T | 14 |
| wrist_roll | XL330-M288-T | 15 |
| gripper | XL330-M288-T | 16 |

컨트롤러: **OpenRB-150** (USB-C → PC)

### Leader Arm (2× 필요)

| 관절 | 모터 모델 | ID |
|------|-----------|-----|
| shoulder_pan | XL330-M288-T | 1 |
| shoulder_lift | XL330-M288-T | 2 |
| elbow_flex | XL330-M288-T | 3 |
| wrist_flex | XL330-M288-T | 4 |
| wrist_roll | XL330-M288-T | 5 |
| gripper | XL330-M077-T | 6 |

컨트롤러: **OpenRB-150** (USB-C → PC)

> **연결 방식**: 각 OMX 팔에 OpenRB-150 1개 → USB-C 케이블로 PC에 연결.  
> Dual arm 구성 시 총 USB 포트 4개 필요 (follower 2 + leader 2).

---

## 2. Python 가상환경 설정

### 2-1. conda 환경 생성

```bash
# Python 3.12 (권장)
conda create -n omx python=3.12 -y

# 또는 Python 3.13 (현재 시스템과 동일)
# conda create -n omx python=3.13 -y

conda activate omx
```

```bash
# Python 3.12 (권장)
python3 -m venv ~/venv/xlerobot
source ~/venv/xlerobot/bin/activate

# 또는 Python 3.13 (현재 시스템과 동일)
# conda create -n omx python=3.13 -y
```

> Python 3.12 / 3.13 모두 동작이 확인되었습니다.  
> lerobot의 `Requires-Python: >=3.10` 조건을 만족합니다.

### 2-2. PyTorch 설치

> **Teleoperation만 할 경우에도 PyTorch가 필요합니다.**  
> lerobot 0.3.4는 모터 제어 모듈(`lerobot.motors`)이 내부적으로  
> `lerobot.utils.utils`를 import하고, 이 파일이 모듈 최상단에서  
> `import torch`를 실행합니다. 기능상으론 불필요하지만 생략할 수 없습니다.  
> Recording은 `LeRobotDataset`·`torchvision` 등에서 실제로 사용합니다.

```bash
# CUDA 13.0 (현재 시스템)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130

# CPU only (GPU 없는 환경 / 테스트용, 설치 용량이 훨씬 작음 ~300 MB)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### 2-3. LeRobot 설치

```bash
pip install lerobot
```

설치 확인:

```bash
python -c "import lerobot; print(lerobot.__version__)"
# 0.3.4
```

### 2-4. Dynamixel SDK 확인

LeRobot 설치 시 자동으로 포함됩니다.

```bash
pip show dynamixel-sdk
# Name: dynamixel-sdk
# Version: 4.0.3
```

### 2-5. 이 저장소 클론 (이미 있다면 생략)

```bash
git clone https://github.com/jae0311/XLeRobot.git
cd XLeRobot
```

### 2-6. 환경 요약

```
conda activate omx
# 설치된 패키지:
#   python       3.12 또는 3.13
#   torch        (CUDA 13.0 또는 CPU)
#   torchvision
#   lerobot      >= 0.3.4
#   dynamixel-sdk >= 4.0.0
```

---

## 3. USB 포트 설정

### 3-1. 포트 확인

OpenRB-150을 PC에 연결한 뒤:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
# /dev/ttyACM0  /dev/ttyACM1  /dev/ttyACM2  /dev/ttyACM3
```

### 3-2. 권한 설정 (최초 1회)

```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인하면 영구 적용
```

임시 적용 (재부팅 전까지):

```bash
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3
```

### 3-3. 포트와 팔 매핑 확인

케이블을 하나씩 꽂아가며 포트 번호를 기록하세요.

```bash
# 케이블 연결 시 실시간으로 포트가 잡히는지 확인
dmesg | tail -5
```

권장 매핑 예시:

| 포트 | 역할 |
|------|------|
| `/dev/ttyACM0` | 왼팔 Follower |
| `/dev/ttyACM1` | 오른팔 Follower |
| `/dev/ttyACM2` | 왼팔 Leader |
| `/dev/ttyACM3` | 오른팔 Leader |

---

## 4. 모터 ID 확인

Dynamixel Wizard 2.0으로 모터 ID를 확인·변경할 수 있습니다.

```bash
# Dynamixel Wizard 2.0 다운로드
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/
```

또는 Python으로 스캔:

```python
from lerobot.motors.dynamixel import DynamixelMotorsBus

bus = DynamixelMotorsBus(port="/dev/ttyACM0", motors={})
bus.connect()
# 1~253 범위의 ID를 스캔합니다
```

> **Follower**: ID 11–16  
> **Leader**: ID 1–6  
> 두 버스(포트)는 완전히 독립적이므로 같은 ID가 달리 팔에 있어도 충돌 없음.

---

## 5. 캘리브레이션

캘리브레이션은 **처음 실행 시 자동으로 진행**됩니다.  
완료된 파일은 `~/.cache/huggingface/lerobot/calibration/` 하위에 저장되며,  
이후 실행 시 저장 파일을 재사용합니다.

### 캘리브레이션 절차 (자동 안내)

```
1. [ENTER] → 중간 위치 homing
   OMX를 대략 가동 범위 중간 위치(직립)에 놓고 ENTER

2. 각 관절을 전체 가동 범위로 천천히 움직이기
   shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, gripper 순서

3. [ENTER] → 범위 기록 완료
```

### 저장 위치

```
~/.cache/huggingface/lerobot/calibration/
├── robots/
│   ├── omx_follower/
│   │   ├── <id>_left.json
│   │   └── <id>_right.json
└── teleoperators/
    └── omx_leader/
        ├── <id>_left.json
        └── <id>_right.json
```

### 강제 재캘리브레이션

실행 시 `'c'` 를 입력하면 새로 캘리브레이션합니다.

---

## 6. Teleoperation (원격 조종)

`software/examples/` 디렉토리에서 실행합니다.

```bash
conda activate omx
cd /path/to/XLeRobot/software/examples
```

### 6-1. 양팔 Teleoperation (권장)

```bash
python 9_dual_omx_teleop.py \
    --mode dual \
    --left-follower-port  /dev/ttyACM0 \
    --right-follower-port /dev/ttyACM1 \
    --left-leader-port    /dev/ttyACM2 \
    --right-leader-port   /dev/ttyACM3
```

### 6-2. 단일 팔 Teleoperation

```bash
python 9_dual_omx_teleop.py \
    --mode single \
    --follower-port /dev/ttyACM0 \
    --leader-port   /dev/ttyACM1
```

### 6-3. 옵션 설명

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--fps` | `60` | 제어 루프 주파수 |
| `--gripper-open-pos` | `50.0` | Leader gripper 기본 열림 위치 (0–100) |
| `--max-relative-target` | `None` | Follower 안전 제한: 한 스텝 최대 이동값 |
| `--skip-calibration` | `False` | 저장된 캘리브레이션 자동 사용 (빠른 시작) |

### 6-4. 조작 방법

- **Leader 팔을 손으로 잡고 움직이면** Follower 팔이 동일하게 따라옵니다.
- Gripper: Leader의 gripper를 손가락으로 누르면 닫히고, 놓으면 스프링처럼 열립니다.
- 종료: `Ctrl+C`

---

## 7. Dataset Recording

LeRobot 표준 형식으로 데이터셋을 로컬에 저장하고,  
선택적으로 HuggingFace Hub에 업로드할 수 있습니다.

```bash
conda activate omx
cd /path/to/XLeRobot/software/examples
```

### 7-1. 양팔 Recording (카메라 없음)

```bash
python 10_dual_omx_record.py \
    --mode dual \
    --left-follower-port  /dev/ttyACM0 \
    --right-follower-port /dev/ttyACM1 \
    --left-leader-port    /dev/ttyACM2 \
    --right-leader-port   /dev/ttyACM3 \
    --repo-id  your_hf_username/omx-pick-cube \
    --task     "Pick up the red cube" \
    --num-episodes 50 \
    --episode-time-s 60 \
    --reset-time-s   30
```

### 7-2. 카메라 추가

`--cameras` 옵션으로 여러 카메라를 추가할 수 있습니다.

```
형식: 이름:/dev/videoX:너비:높이:fps  (쉼표로 구분)
```

```bash
python 10_dual_omx_record.py \
    --mode dual \
    --left-follower-port  /dev/ttyACM0 \
    --right-follower-port /dev/ttyACM1 \
    --left-leader-port    /dev/ttyACM2 \
    --right-leader-port   /dev/ttyACM3 \
    --repo-id  your_hf_username/omx-pick-cube \
    --task     "Pick up the red cube" \
    --num-episodes 50 \
    --cameras "head:/dev/video0:640:480:30,left_wrist:/dev/video2:640:480:30,right_wrist:/dev/video4:640:480:30"
```

카메라 인덱스 확인:

```bash
ls /dev/video*
# 또는
v4l2-ctl --list-devices
```

### 7-3. 단일 팔 Recording

```bash
python 10_dual_omx_record.py \
    --mode single \
    --follower-port /dev/ttyACM0 \
    --leader-port   /dev/ttyACM1 \
    --repo-id  your_hf_username/omx-single-task \
    --task     "Grasp the cup"
```

### 7-4. 키보드 단축키 (Recording 중)

| 키 | 동작 |
|----|------|
| `→` 오른쪽 화살표 | 현재 에피소드 **저장** 후 다음 에피소드 시작 |
| `←` 왼쪽 화살표 | 현재 에피소드 **버리고** 다시 녹화 |
| `ESC` | 녹화 **종료** (저장된 에피소드는 유지) |

### 7-5. 전체 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--repo-id` | *필수* | HuggingFace 데이터셋 이름 (`user/name`) |
| `--task` | *필수* | 태스크 설명 문자열 |
| `--num-episodes` | `50` | 녹화할 에피소드 수 |
| `--episode-time-s` | `60.0` | 에피소드 당 녹화 시간 (초) |
| `--reset-time-s` | `30.0` | 에피소드 사이 리셋 시간 (초) |
| `--fps` | `30` | 녹화 프레임 레이트 |
| `--cameras` | `None` | 카메라 목록 (위 형식 참조) |
| `--dataset-root` | `~/.cache/...` | 로컬 저장 경로 |
| `--push-to-hub` | `False` | 완료 후 HF Hub 업로드 |
| `--private` | `False` | Hub 비공개 저장소 |
| `--resume` | `False` | 기존 데이터셋에 이어서 녹화 |
| `--skip-calibration` | `False` | 저장된 캘리브레이션 자동 사용 |

### 7-6. HuggingFace Hub 업로드

녹화 종료 후 자동 업로드:

```bash
python 10_dual_omx_record.py ... --push-to-hub
```

수동 업로드 (녹화 완료 후):

```bash
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('your_hf_username/omx-pick-cube')
ds.push_to_hub()
"
```

HuggingFace 로그인 (최초 1회):

```bash
huggingface-cli login
```

### 7-7. 저장된 데이터셋 확인

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset("your_hf_username/omx-pick-cube")
print(ds)
print(f"에피소드 수: {ds.num_episodes}")
print(f"총 프레임:  {ds.num_frames}")
print(f"피처:       {list(ds.features.keys())}")
```

---

## 8. 파일 구조

```
software/
├── src/
│   ├── robots/
│   │   └── omx/
│   │       ├── __init__.py
│   │       ├── config_omx_follower.py      # 단일 follower 설정
│   │       ├── omx_follower.py             # 단일 follower 로봇 클래스
│   │       ├── config_bi_omx_follower.py   # 양팔 follower 설정
│   │       └── bi_omx_follower.py          # 양팔 follower 로봇 클래스
│   └── teleporators/
│       └── omx_leader/
│           ├── __init__.py
│           ├── config_omx_leader.py        # 단일 leader 설정
│           ├── omx_leader.py               # 단일 leader 클래스
│           ├── config_bi_omx_leader.py     # 양팔 leader 설정
│           └── bi_omx_leader.py            # 양팔 leader 클래스
└── examples/
    ├── 9_dual_omx_teleop.py                # Teleoperation 실행 스크립트
    └── 10_dual_omx_record.py              # Dataset recording 실행 스크립트
```

---

## 9. 트러블슈팅

### 포트를 찾을 수 없음

```
SerialException: [Errno 2] No such file or directory: '/dev/ttyACM0'
```

- USB 케이블 연결 확인
- `ls /dev/ttyACM*` 로 실제 포트 번호 재확인
- OpenRB-150 전원 공급 확인 (USB 버스 파워 한계 주의 → 별도 12V 공급 권장)

### 권한 오류

```
SerialException: [Errno 13] Permission denied: '/dev/ttyACM0'
```

```bash
sudo chmod 666 /dev/ttyACM0
# 또는 영구 설정
sudo usermod -a -G dialout $USER
```

### 모터가 응답하지 않음

1. Baud rate 확인: LeRobot은 기본 **1,000,000 bps** 사용
2. Dynamixel Wizard로 모터 ID와 baud rate 확인
3. 버스에 모터가 1개 이상 연결되어 있는지 확인
4. OpenRB-150 펌웨어 최신 버전 업데이트

### 캘리브레이션 파일 초기화

```bash
rm -rf ~/.cache/huggingface/lerobot/calibration/robots/omx_follower/
rm -rf ~/.cache/huggingface/lerobot/calibration/teleoperators/omx_leader/
```

다음 실행 시 새로 캘리브레이션 진행됩니다.

### Follower가 Leader를 제대로 따라가지 않음

- `--max-relative-target 5.0` 으로 안전 제한 설정 후 테스트
- 캘리브레이션을 새로 진행 (Leader와 Follower가 동일한 기준 위치에서 수행)
- `--fps` 를 낮춰서 (예: `30`) 통신 안정성 확보

### Recording 중 FPS가 불안정함

- `--num-image-writer-threads-per-camera 4` (카메라당 스레드 증가)
- `--num-image-writer-processes 1` (서브프로세스 사용)
- 카메라 해상도/fps 낮추기 (`320:240:15`)
