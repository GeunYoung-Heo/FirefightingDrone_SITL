# Firefighting Drone SITL

S550 헥사콥터를 PX4 SITL + Gazebo Classic 환경에서 시뮬레이션하기 위한 작업 공간입니다. 향후 소방 페이로드/미션 시뮬을 위한 베이스 환경으로 사용합니다.

---

## 1. 개요

### 1.1 목표
- DJI F550/S550 계열 hexacopter를 PX4 펌웨어와 함께 Gazebo Classic에서 비행 시뮬레이션
- 향후 단계: 소방 페이로드/미션 시뮬, MAVSDK/ROS 2 자동 제어 등

### 1.2 확정된 스택

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) |
| 아키텍처 | x86_64 |
| PX4-Autopilot | v1.14.4 (Gazebo Classic을 1급 지원하는 마지막 라인) |
| 시뮬레이터 | Gazebo Classic 11.10.2 |
| 빌드 도구 | gcc/g++ 11.4.0, cmake 3.22.1, ninja, ccache |
| Python (PX4 빌드용) | 시스템 Python 3.10 (NOT conda) |
| GCS | QGroundControl AppImage |

### 1.3 디렉토리 구조

본 저장소는 어느 경로에 두든 동작합니다. 본문에서는 예시로 `~/Firefighting_Drone/SITL/`을 사용합니다.

```
~/Firefighting_Drone/SITL/                # 프로젝트 루트 (이 README가 있는 곳)
├── README.md                # 이 파일
├── run_sitl.sh              # SITL 실행 래퍼 (스크립트 위치 기준으로 PX4-Autopilot 자동 인식)
├── apply_overlay.sh         # overlay/ 의 파일을 PX4-Autopilot/ 으로 복사하는 헬퍼
├── .gitignore               # PX4-Autopilot/ 등 제외
│
├── overlay/                 # ← S550 커스터마이징을 PX4 트리 구조 그대로 미러링
│   ├── Tools/simulation/gazebo-classic/sitl_gazebo-classic/
│   │   ├── models/s550/
│   │   │   ├── s550.sdf.jinja              # 짐벌/레그 fixed, 외란 plugin 등록
│   │   │   ├── model.config
│   │   │   └── meshes/                     # typhoon mesh 9개 자기완결 사본
│   │   └── worlds/
│   │       └── s550.world                  # 단순 회색 ground (어두운 asphalt 대체)
│   ├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   │   ├── 4501_gazebo-classic_s550        # 신규 airframe
│   │   ├── 4501_gazebo-classic_s550.post
│   │   └── CMakeLists.txt                  # s550 항목 추가된 수정본
│   └── src/modules/simulation/simulator_mavlink/
│       └── sitl_targets_gazebo-classic.cmake   # s550 항목 추가된 수정본
│
├── tools/                   # 외란 테스트 + 데이터 기록 + 시각화 도구 (7~8장 참조)
│   ├── disturbance_profiles.json # 외란 타임라인 정의 (외란별 mean/stddev/frame/τ)
│   ├── apply_disturbance.py # CLI — JSON 프로파일을 GzString으로 트리거 publish
│   ├── record_experiment.py # 외란 트리거 + ULog 스냅샷 + metadata 저장 (8장)
│   ├── plot_experiment.py   # 실험 디렉토리 → 3×3 분석 plot (궤적/위치/자세, 8.5장)
│   ├── data/                # 실험 데이터 (gitignore — .gitignore와 README.md만 추적)
│   │   ├── .gitignore
│   │   └── README.md        # 데이터 형식·시간축 정렬 가이드
│   └── disturbance_plugin/
│       ├── disturbance_plugin.cc           # Gazebo Classic ModelPlugin (force 적용 + 화살표 spawn/track)
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── build/                          # cmake/make 산출물 (gitignore)
│       └── disturbance_arrow_assets/
│           ├── model.config                # Gazebo model:// URI 인식용
│           ├── model.sdf                   # placeholder
│           └── meshes/cone.stl             # 화살표 끝 cone mesh (16 segments)
│
└── PX4-Autopilot/           # PX4 v1.14.4 체크아웃 (3.2절에서 clone, .gitignore로 제외)
    ├── Tools/setup/ubuntu.sh
    ├── Tools/simulation/gazebo-classic/
    │   ├── sitl_run.sh
    │   ├── setup_gazebo.bash
    │   └── sitl_gazebo-classic/
    │       ├── CMakeLists.txt           # jinja → sdf 자동 변환 (file GLOB)
    │       └── models/
    │           ├── iris/                # 기본 quad
    │           ├── typhoon_h480/        # 기본 hexa
    │           └── s550/                # 3.6 apply_overlay.sh 실행 시 생성됨
    ├── ROMFS/px4fmu_common/init.d-posix/airframes/
    │   ├── 4501_gazebo-classic_s550        # overlay 적용 후 등장
    │   ├── 4501_gazebo-classic_s550.post   # overlay 적용 후 등장
    │   ├── 6011_gazebo-classic_typhoon_h480
    │   └── CMakeLists.txt                  # overlay 적용 후 s550 항목 포함
    ├── src/modules/simulation/simulator_mavlink/
    │   └── sitl_targets_gazebo-classic.cmake  # overlay 적용 후 s550 항목 포함
    └── build/px4_sitl_default/                # 빌드 산출물
```

본 저장소(GitHub)에는 `PX4-Autopilot/` 자체는 포함되지 않습니다 (몇 GB 규모 + PX4 측 저장소가 별도). 팀원은 README 3장 절차로 PX4를 받은 뒤 `apply_overlay.sh`로 S550 커스터마이징을 입혀 사용합니다.

### 1.4 Quick Start (이미 셋업된 시스템에서)

```bash
cd ~/Firefighting_Drone/SITL

# 1) S550 SITL 띄우기 (자동: 환경 정리 → 좀비 청소 → 빌드 → 실행)
./run_sitl.sh s550

# 2) 다른 터미널에서 QGroundControl 실행
~/Apps/QGroundControl-x86_64.AppImage    # (경로는 본인 환경에 맞게)
```

QGC가 자동으로 UDP 14550으로 PX4 SITL에 연결됩니다. pxh 프롬프트에서 `commander takeoff` 또는 QGC GUI의 Takeoff 슬라이드로 비행 시험 가능.

**처음 셋업하는 경우** 2~3장을 먼저 진행해주세요. 핵심 절차 요약:
1. 2장의 호스트 요건 확인
2. **3.1 이 저장소 clone + 시스템 도구 설치** — `git clone https://github.com/GeunYoung-Heo/FirefightingDrone_SITL.git SITL` 후 `cd SITL`
3. 3.2~3.4 셋업 (PX4 clone, ubuntu.sh, Gazebo Classic 복구)
4. 3.5 QGroundControl AppImage 설치
5. **3.6 `./apply_overlay.sh` 실행** ← S550 커스터마이징 적용
6. **7.4 disturbance_plugin 빌드** ← 외란 테스트 도구 (선택; 안 빌드해도 비행은 정상, 부팅 시 무해한 plugin load 에러 1줄만 출력)
7. `./run_sitl.sh s550` 실행

**외란 테스트 — 호버 중인 SITL에 JSON 프로파일 외란 인가:**
```bash
./tools/apply_disturbance.py    # tools/disturbance_profiles.json 타임라인 트리거
```
외란 내용은 `tools/disturbance_profiles.json` 을 편집해서 정의합니다 (외란별 평균·표준편차·frame·시작/지속 시간). 외란마다 색깔 있는 화살표가 드론에 부착되어 평균 force 방향을 보여주고, PX4 위치 제어기의 외란 거부 거동을 관찰할 수 있습니다. 자세한 내용은 7장 참조.

---

## 2. 사전 요구사항

### 2.1 호스트 요건

- **Ubuntu 22.04 LTS (x86_64)** — Jammy. 다른 버전(20.04, 24.04)이나 ARM은 검증 안 됨.
- 디스크 여유: **10 GB 이상** (PX4 + 서브모듈 + 빌드 산출물)
- 메모리: 8 GB 이상 권장 (빌드 시 4 GB+ 사용)
- GUI 가능한 데스크탑 환경 (Gazebo Classic GUI 창 표시용)
  - 로컬 모니터 권장. SSH 사용 시 X11 forwarding (`ssh -X`) 필요
- GPU 드라이버: 정상 동작하는 OpenGL (NVIDIA, AMD, Intel 무관)

### 2.2 Python 환경 주의

PX4 v1.14는 시스템 Python 3.10 기준으로 검증되어 있습니다. 만약 **conda(miniforge3/anaconda)의 base 환경이 자동 활성화**되어 Python 3.11+ 가 잡혀 있으면 PX4 코드 생성 단계에서 문제가 생길 수 있습니다.

확인:
```bash
which python3            # /usr/bin/python3 이어야 함
python3 --version        # 3.10.x 이어야 함
```

`/home/<user>/miniforge3/...` 등 conda 경로가 나오면 셋업 전 비활성화:
```bash
conda deactivate                                # 일회성
conda config --set auto_activate_base false     # 영구
```

> `run_sitl.sh`는 매 실행 시 자체적으로 conda를 비활성화하므로, 빌드 시점만 신경 쓰면 됩니다.

### 2.3 ROS 2 환경 주의

호스트에 ROS 2 (Humble, Foxy 등) 가 설치되어 있고 `~/.bashrc`에서 자동 소싱되는 경우 — 새 Gazebo Sim용 환경변수(`GAZEBO_MODEL_PATH`, `GZ_SIM_RESOURCE_PATH`, `AMENT_PREFIX_PATH` 등)가 채워져 **PX4 Gazebo Classic 빌드를 깨뜨릴 수 있습니다.**

`run_sitl.sh`가 매 실행 시 이런 환경변수를 unset하므로 **실행 시점은 자동 처리**되지만, **빌드 의존성 설치(`ubuntu.sh`) 시점에는 충돌 가능성이 있어** 가급적 깨끗한 쉘에서 진행하길 권장합니다.

---

## 3. 환경 셋업

> 처음 한 번만 수행하는 단계입니다. 이미 완료된 경우 `4. SITL 실행`으로 넘어가세요.

### 3.1 시스템 사전 도구 설치 + 이 저장소 클론

먼저 git 등 기본 도구를 설치합니다:

```bash
sudo apt update
sudo apt install -y git curl wget ca-certificates lsb-release gnupg \
                    python3 python3-pip python3-venv python3-dev
```

확인:
```bash
which python3            # /usr/bin/python3
python3 --version        # Python 3.10.x
which gcc cmake git
```

그 다음 이 저장소(S550 SITL 작업공간)를 원하는 위치에 clone합니다. 아래 예시는 `~/Firefighting_Drone/` 아래에 받는 경우입니다 (경로는 자유롭게 바꿔도 됩니다):

```bash
# 받을 상위 폴더를 만들고 그 안으로 이동
mkdir -p ~/Firefighting_Drone
cd ~/Firefighting_Drone

# 저장소 clone (HTTPS — GitHub 로그인이나 SSH 키 설정 없이 받을 수 있음)
# 마지막 인자 'SITL' 은 만들어질 폴더 이름. README 전체가 이 이름을 가정합니다.
git clone https://github.com/GeunYoung-Heo/FirefightingDrone_SITL.git SITL

# 저장소 루트로 이동 — 이후 모든 명령은 이 폴더 기준으로 실행합니다.
cd SITL
```

clone이 끝나면 `~/Firefighting_Drone/SITL/` 폴더가 생기고, 그 안에 이 `README.md`, `run_sitl.sh`, `overlay/`, `tools/` 등이 들어 있습니다. 이 폴더가 본문에서 말하는 **"저장소 루트"** 입니다. 다음 절(§3.2)의 PX4-Autopilot은 바로 이 폴더 안에 받게 됩니다.

> SSH 키를 이미 설정해 둔 경우엔 `git clone git@github.com:GeunYoung-Heo/FirefightingDrone_SITL.git SITL` 도 됩니다. 잘 모르겠으면 위의 HTTPS 방식을 쓰면 됩니다.

### 3.2 PX4-Autopilot v1.14.4 클론

이 저장소의 루트에서 (예: `~/Firefighting_Drone/SITL/`):

```bash
# 프로젝트 루트로 이동 (run_sitl.sh이 있는 디렉토리)
cd ~/Firefighting_Drone/SITL

# (1) 메인 리포지토리 클론
git clone https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot

# (2) v1.14.4 태그 체크아웃
git checkout v1.14.4

# (3) 서브모듈 받기 (5~15분, 약 1~2GB)
git submodule update --init --recursive
```

> 참고: `src/drivers/uavcan/libuavcan`의 nested submodule 재귀 적용에서 `fatal: ... 하위 모듈에 재귀적으로 적용이 실패했습니다` 메시지가 나올 수 있는데, SITL에는 영향 없습니다 (실기체 UAVCAN 페리퍼럴용 모듈).

검증:
```bash
git describe --tags                            # v1.14.4
git submodule status | awk '{print $1, $2}'    # '-' 없어야 함

# 핵심 디렉토리 존재 확인
ls Tools/simulation/gazebo-classic/sitl_gazebo-classic/
ls Tools/setup/ubuntu.sh
ls ROMFS/px4fmu_common/init.d-posix/airframes/ | grep typhoon
```

### 3.3 PX4 빌드 의존성 설치 (`ubuntu.sh`)

```bash
# PX4-Autopilot 디렉토리 안에서
bash ./Tools/setup/ubuntu.sh
```

이 스크립트가 자동으로 설치하는 것:
- ninja, ccache, exiftool 등 빌드 도구
- libeigen3-dev, libopencv-dev, protobuf-compiler 등 Gazebo 플러그인 빌드용 헤더
- empy, jinja2, kconfiglib, jsonschema 등 PX4 코드 생성용 Python 패키지 (`pip3 install --user`)
- gcc-arm-none-eabi (NuttX 크로스컴파일러; SITL에는 불필요하지만 향후 실기체 작업에 유용)

> **주의 — Gazebo 패키지 함정 (Issue #1 참고):** 이 스크립트는 Jammy에서 **gz-garden** (새 Gazebo Sim)을 설치하며, 그 과정에서 충돌 회피로 **gazebo-classic 런타임 바이너리(`/usr/bin/gazebo`)가 자동 제거**됩니다. 해결은 3.4절 참조.

**외란 플러그인 추가 의존성:** `ubuntu.sh`는 외란 플러그인 빌드(§7.4)에 필요한 `libjsoncpp-dev`를 설치하지 않습니다. 보통 OpenCV/Gazebo 의존성으로 함께 딸려오지만, 누락 시 §7.4의 `cmake`가 실패하므로 외란 실험을 할 거라면 미리 설치해 두면 안전합니다:
```bash
sudo apt install -y libjsoncpp-dev
```

검증:
```bash
which ninja ccache exiftool
arm-none-eabi-gcc --version  # 없어도 SITL은 OK; 재로그인 후에 PATH 잡힘

# Python 의존성 (import name 기준)
python3 - <<'PY'
import importlib
for pip, mod in [("empy","em"),("jinja2","jinja2"),("kconfiglib","kconfiglib"),
                 ("jsonschema","jsonschema"),("numpy","numpy"),("pyros-genmsg","genmsg"),
                 ("toml","toml"),("pyyaml","yaml"),("future","future"),
                 ("packaging","packaging"),("pyserial","serial")]:
    try:
        importlib.import_module(mod); print(f"  OK    {pip}")
    except ImportError:
        print(f"  MISS  {pip}")
PY
```

### 3.4 Gazebo Classic 복구 (필요시)

`ubuntu.sh` 실행 직후라면 거의 확실히 필요합니다 (Issue #1 참조).

```bash
sudo apt install gazebo gazebo-common gazebo-plugin-base libgazebo-dev libgazebo11
```

apt가 `gz-garden`과 관련 `libgz-*` 패키지들을 제거하겠다고 묻는데, **그대로 `y`** 로 진행하면 됩니다 (Classic과 새 Gazebo가 충돌하므로 둘 중 하나만 설치 가능).

검증:
```bash
which gazebo gzserver gzclient   # 모두 /usr/bin/ 에 있어야 함
gazebo --version                  # 11.10.2
```

### 3.5 QGroundControl 설치

[공식 사이트](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)에서 AppImage 다운로드 → 실행 권한 부여 → 어느 경로든 보관:

```bash
mkdir -p ~/Apps
# 다운로드한 파일을 ~/Apps/QGroundControl-x86_64.AppImage 로 이동
chmod +x ~/Apps/QGroundControl-x86_64.AppImage
```

설치 검증 (5초 후 자동 종료):
```bash
timeout 5 ~/Apps/QGroundControl-x86_64.AppImage 2>&1 | head -5
```

GUI 창이 잠깐이라도 뜨면 정상.

### 3.6 S550 Overlay 적용

본 저장소의 `overlay/` 디렉토리에는 PX4-Autopilot 트리 구조를 미러링한 S550 커스터마이징 파일들이 들어 있습니다. 다음 한 줄로 PX4-Autopilot/ 안의 해당 경로에 복사됩니다:

```bash
cd ~/Firefighting_Drone/SITL
chmod +x apply_overlay.sh
./apply_overlay.sh
```

수행 내용:
- `overlay/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/s550/*` → PX4 트리의 대응 경로 (신규 모델 디렉토리)
- `overlay/ROMFS/px4fmu_common/init.d-posix/airframes/4501_gazebo-classic_s550{,.post}` → 신규 airframe 파일
- `overlay/ROMFS/.../airframes/CMakeLists.txt` → PX4 원본 덮어씀 (s550 등록된 버전)
- `overlay/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake` → PX4 원본 덮어씀 (s550 등록된 버전)

스크립트는 현재 PX4-Autopilot의 git 태그가 `v1.14.4` 인지 확인 후 진행합니다.

> 이미 한 번 빌드한 적이 있는 경우 새 모델 인식을 위해 stamp 정리가 필요합니다 (Issue #4 참조):
> ```bash
> rm -rf PX4-Autopilot/build/px4_sitl_default/external/Stamp/sitl_gazebo-classic
> ```

---

## 4. SITL 실행

### 4.1 환경 오염 이슈 (배경 설명)

호스트에 ROS 2 자동 소싱이 켜져 있거나 conda base가 활성화된 상태로 새 터미널을 열면, 다음 환경변수가 채워져 PX4 Gazebo Classic SITL을 깨뜨릴 수 있습니다:

- `GAZEBO_MODEL_PATH` → ROS 2 워크스페이스 경로로 채워짐 → PX4 모델 못 찾음
- `GZ_SIM_RESOURCE_PATH`, `IGN_GAZEBO_*` 등 → 새 Gazebo Sim용 변수
- `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH` → ROS 2 빌드 환경
- `ROS_DISTRO=humble` 등
- conda 활성화 시 `python3`가 시스템 3.10이 아닌 conda Python으로 잡힘

`run_sitl.sh`는 매 실행 시 이러한 환경을 정리한 뒤 SITL을 띄우므로, **실제로 신경 쓸 필요는 없습니다.** 다만 디버깅 시 배경 지식으로 참고.

### 4.2 권장 실행 방법 — `run_sitl.sh` 래퍼

[run_sitl.sh](run_sitl.sh)는 4.1절의 환경 정리와 SITL 실행을 한 번에 처리하는 래퍼 스크립트입니다.

**스크립트가 하는 일**

1. **conda 비활성화** — `(base)` 환경이 켜져 있으면 빠져나와 시스템 Python 3.10이 사용되도록 보장
2. **ROS 2 / 새 Gazebo Sim 환경변수 제거** — 현재 셸 한정으로 unset
3. **Free camera 모드 활성화** — `PX4_NO_FOLLOW_MODE=1` export. 기본 follow 플러그인이 마우스 입력을 가로채 zoom 등을 막는 문제 회피. 필요시 GUI에서 모델 우클릭 → Follow 가능 (5.5 참조)
4. **좀비 프로세스 청소** — `gzserver` / `gzclient` / `px4` 강제 종료
5. **`make px4_sitl gazebo-classic_<model>` 실행** (PX4-Autopilot 디렉토리는 스크립트 위치 기준 자동 인식)

**최초 1회 — 실행 권한 부여**

```bash
chmod +x ~/Firefighting_Drone/SITL/run_sitl.sh
```

**사용법**

```bash
cd ~/Firefighting_Drone/SITL

./run_sitl.sh                # iris (default, quad)
./run_sitl.sh typhoon_h480   # hexacopter baseline
./run_sitl.sh s550           # S550 (이 프로젝트의 메인 타깃)
```

인자로 넘긴 모델명은 PX4 빌드 타깃 접미사로 매핑됩니다 (`gazebo-classic_<model>`). 등록되지 않은 모델명을 넣으면 ninja가 타깃을 못 찾고 즉시 실패합니다.

> 스크립트는 자기 위치(`$(dirname $0)`) 옆의 `PX4-Autopilot/`를 찾으므로, 저장소를 어디에 두든 그대로 동작합니다.

### 4.3 정상 부팅 흐름

```
[빌드 단계가 처음이면 5~15분, 이후는 ccache 덕분에 수 초]
SITL ARGS
sitl_bin: .../bin/px4
model: s550
GAZEBO_PLUGIN_PATH /home/.../tools/disturbance_plugin/build:.../build_gazebo-classic
GAZEBO_MODEL_PATH  /home/.../tools/disturbance_plugin:.../sitl_gazebo-classic/models
empty world, default world s550.world for model found      <- s550.world 자동 선택 (단순 회색 ground)
Using: .../models/s550/s550.sdf
SITL COMMAND: ...
px4 starting.
INFO  [px4] startup script: /bin/sh etc/init.d-posix/rcS 0
INFO  [init] found model autostart file as SYS_AUTOSTART=4501   <- S550 airframe 로드
...
INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.       <- 결정적
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
...
INFO  [px4] Startup script returned successfully
pxh> INFO  [tone_alarm] home set
INFO  [commander] Ready for takeoff!
```

핵심 체크포인트:
- `SYS_AUTOSTART=4501` (S550) / `=6011` (typhoon_h480) / `=10015` (iris) — 의도한 airframe이 잡혔는지
- `Simulator connected on TCP port 4560.` — Gazebo의 mavlink_interface 플러그인이 PX4에 접속한 신호
- `Ready for takeoff!` — EKF 융합/사전점검 완료, 이륙 가능

---

## 5. pxh 명령

`pxh>` 프롬프트는 PX4 펌웨어의 쉘입니다. 시뮬레이션 중에 명령을 직접 입력해 비행을 제어할 수 있습니다. QGC GUI로도 동일한 작업이 가능합니다.

### 5.1 기본 비행 시퀀스

```text
pxh> commander takeoff      # 자동 arm 후 약 2.5m 이륙
pxh> commander land         # 천천히 착륙
pxh> shutdown               # PX4 종료 (Gazebo도 같이 닫힘)
```

> `commander takeoff`가 `Preflight Fail: ekf2 missing data`로 거절될 수 있습니다. 부팅 후 20~30초 대기하면 EKF가 GPS를 융합하므로 다시 시도하시면 됩니다.

### 5.2 상태 진단

```text
pxh> commander status       # 비행 모드, 헬스 체크, arming 상태, 기체 type (헥사는 13)
pxh> sensors status         # 센서 헬스
pxh> ekf2 status            # EKF 융합 상태
pxh> listener vehicle_local_position    # 로컬 위치 uORB 토픽 1회 출력
pxh> listener actuator_motors -n 1      # 6개 모터 출력값 (헥사 검증용)
```

### 5.3 모드 전환

```text
pxh> commander mode auto:loiter      # 호버
pxh> commander mode auto:mission     # 미션 모드 (미션이 로드된 경우)
pxh> commander mode manual           # 수동
pxh> commander arm
pxh> commander disarm
```

### 5.4 비정상 종료 후 청소

`pxh>`가 깜빡거리지만 입력이 안 먹히거나, Ctrl+C로 강제 종료한 뒤에는 `gzserver`/`gzclient`가 좀비로 남을 수 있습니다. 다음 실행 전에 청소:

```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f "build/px4_sitl_default/bin/px4"
```

`run_sitl.sh` 래퍼는 시작 시 이 청소를 자동으로 수행합니다.

### 5.5 Gazebo Classic 카메라 조작

`run_sitl.sh`는 follow 카메라 플러그인을 비활성화하므로 모든 표준 마우스 컨트롤이 동작합니다.

| 동작 | 마우스 |
|---|---|
| Zoom in/out | 스크롤 휠 (커서를 3D 뷰포트 위에 둔 상태로) |
| Zoom in/out (대안) | 우클릭 + 드래그 (위/아래) |
| Pan (평행 이동) | 좌클릭 + 드래그 |
| Orbit (회전) | 중클릭(휠 클릭) + 드래그 |
| 객체 선택 | Shift + 좌클릭 |
| 일시정지/재생 | Spacebar (정지 중에도 카메라 조작 가능) |

**드론 자동 추적이 필요한 경우** (예: 빠른 기동 관찰):
- Gazebo 좌측 패널의 World 트리에서 `s550` (또는 다른 모델) **우클릭 → Follow** 선택
- 카메라가 드론을 자동으로 따라다님

> **주의:** Gazebo Classic 11.10의 follow 모드는 한 번 켜면 같은 우클릭 메뉴로 깨끗하게 해제가 안 됩니다 (zoom/pan 차단, orbit만 허용). 해제하려면 SITL을 재시작하거나 follow 없이 마우스로 직접 추적하는 게 실용적입니다.

> ⚠️ **`Ctrl + R` 주의:** Gazebo의 `Ctrl + R`은 **카메라 리셋이 아니라 "Reset Models"** — 월드의 모든 모델을 초기 자세로 리셋합니다. 비행 중 누르면 드론이 spawn 위치로 순간이동하니 사용 금지.

---

## 6. S550 (Hexarotor) 커스텀 모델

PX4 v1.14 + Gazebo Classic 조합에서 즉시 빌드되는 hexarotor는 `typhoon_h480`(Yuneec Typhoon H, 360° 짐벌 카메라 장착)이 유일합니다. 헥사-X 60° 모터 배치가 S550(DJI F550)과 동일하므로 이를 베이스로 분기해 S550용 SITL 모델을 만들었습니다.

### 6.1 설계 결정

| 항목 | 선택 | 이유 |
|---|---|---|
| Visual mesh | typhoon_h480 STL 9개 자기완결 사본 | 외형 차이는 추후 보강. 비행 동특성 검증 우선 |
| AUW | 2.2 kg (typhoon 2.02 kg에서 +9%) | 향후 소방 페이로드(소화액 탱크 등) 마진 |
| Wheelbase | 550mm (motor-to-motor, S550 표준) | DJI F550 사양 |
| `SYS_AUTOSTART` | `4501` | 4XXX 멀티콥터 영역의 비어있는 슬롯, gz_*(4001~4006) 후속 |
| `MAV_TYPE` | 13 (Hexarotor X) | typhoon과 동일 (헥사-X 배치) |
| 로터 정규화 위치·KM | typhoon_h480 값 그대로 사용 | 60° 등간격 헥사-X는 정규화 좌표가 동일 |

### 6.2 추가/수정된 파일

**신규 파일 (모두 typhoon_h480에서 분기·수정):**
- `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/s550/s550.sdf.jinja`
- `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/s550/model.config`
- `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/s550/meshes/*.stl` (9개)
- `ROMFS/px4fmu_common/init.d-posix/airframes/4501_gazebo-classic_s550`
- `ROMFS/px4fmu_common/init.d-posix/airframes/4501_gazebo-classic_s550.post`

**기존 파일 편집 (PX4 빌드 시스템에 S550 등록):**
- `src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake` — `set(models ...)` 리스트에 `s550` 한 줄 추가 (rover와 standard_vtol 사이, 알파벳 순)
- `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` — `4501_gazebo-classic_s550` 및 `.post` 두 줄 추가 (4006_gz_px4vision과 6011_typhoon_h480 사이)

> 이 변경사항은 PX4-Autopilot 서브모듈 트리 안에 있으므로, 본 저장소에서는 git tracking이 안 됩니다. 팀원에게 배포하려면 이 README의 셋업 절차를 따라 PX4를 직접 clone 후 동일하게 적용하는 패턴이 됩니다. 향후 patch 파일로 묶어 자동 적용하는 방법도 고려할 수 있습니다.

### 6.3 S550 airframe 파일 핵심 내용

`4501_gazebo-classic_s550`의 본질은:
- `rc.mc_defaults` 상속 (멀티콥터 기본값)
- 6개 모터 PWM_MAIN_FUNC1~6 = 101~106 (모터 출력만)
- 헥사-X 6개 로터 정규화 좌표 + KM (typhoon과 동일)
- PID 게인은 typhoon 기준 그대로 (현장 튜닝 가능)
- **typhoon에 있던 짐벌(PWM_MAIN_FUNC7~9), 랜딩기어 서보(PWM_MAIN_FUNC10/11), 카메라 트리거(TRIG_*), 마운트(MNT_*) 파라미터는 모두 제거**

### 6.4 S550 SDF 차이 — 동특성 정리 + 시각 노이즈 제거

typhoon SDF를 그대로 분기하면 비행·착륙이 불안정합니다 (Issue #5 참조). S550에서는 다음 작업으로 정리했습니다.

**(a) 짐벌/레그 강체화**
- **짐벌 4-link 계층의 revolute joint들을 `fixed`로** (cgo3_mount_joint, cgo3_vertical_arm_joint, cgo3_horizontal_arm_joint, cgo3_camera_joint)
- **양쪽 랜딩기어 joint도 `fixed`로** (left_leg_joint, right_leg_joint)
- `mavlink_interface` plugin의 짐벌·레그 channel 5개 제거 (gimbal_roll/pitch/yaw, left_leg, right_leg)
- `gimbal_controller` plugin 블록 통째로 제거
- 짐벌 link 4개의 parent를 모두 `base_link` 직속으로 평탄화 (chain compliance 누적 방지)
- camera_link의 sphere collision 제거 (spawn 시 지면 깊이 침투 → 솔버 불안정)

**(b) 미사용 센서·플러그인 제거 (시각 노이즈 정리)**
- `cgo3_camera_link` 안의 **camera 센서 + camera_imu 센서 통째 제거** — Gazebo의 파란색 FOV 피라미드 시각화와 QGC 카메라 탭/비디오 스트림 모두 사라짐
- `model://sonar` include + `sonar_joint` 제거 — sonar의 하얀 광선(detection ray) 시각화 제거 (고도는 GPS+baro로 충분)
- `GstCameraPlugin`과 `CameraManagerPlugin`은 camera 센서 안에 있던 자식이라 함께 제거됨

결과:
- 시각적으로 짐벌 mesh는 그대로 보이지만 카메라 FOV 시각 효과·sonar 광선 없음
- 동역학적으로 짐벌·레그가 base_link와 강체로 결합 → 비행 중 잔진동 없음
- QGC도 카메라 관련 UI 표시 없이 깔끔 (헥사 비행 정보만)

### 6.5 알려진 한계 (Known Limitations) — 추후 해결 예정

#### (a) 지상 정지 시 본체 미세 진동

스폰 직후·착륙 직후 본체와 짐벌이 **미세하게 떨립니다**. 비행 중에는 사라집니다.

원인:
- 다리 cylinder의 collision 바닥이 월드 z = -0.007 m로 **지면에 7mm 침투**
- contact stiffness `kp=1e+8`이 매우 단단해 작은 침투에도 큰 spring-damper 반발력
- ODE 솔버가 지속적으로 침투 해소를 시도하며 미세 oscillation 생성
- typhoon은 다리가 revolute joint + PID 제어라 자동 흡수, 우리는 fixed라 흡수 불가

**현재 영향:**
- 기능적으로 무관 — 비행, takeoff, land, MAVLink, EKF 모두 정상 동작
- 시각적 노이즈만 잔존

**추후 해결 방안:**
1. 빠른 꼼수: `s550.sdf.jinja`의 model `<pose>0 0 0.26 ...>` 을 `0.28`로 변경 → 다리가 지면 위 13mm로 떠서 자연 안착
2. 정공법: 다리 link/collision pose를 재계산해 cylinder 바닥이 정확히 z=0에 닿게 맞춤
3. 본격: 실제 S550 사양에 맞춘 mesh와 collision 재작성

#### (b) Visual mesh가 S550이 아닌 typhoon_h480

의도된 단순화입니다. 실제 S550 외형(중심부 원판 + 6개 분리된 arm + 다리)이 필요하면 mesh 교체 작업이 별도로 필요합니다.

#### (c) PID 게인이 typhoon 기준

S550에 맞춘 in-flight 튜닝이 필요합니다 (현 게인으로도 비행은 가능).

#### (d) QGC Actuator Testing이 SITL에서 사실상 무용

SDF의 `<zero_position_disarmed>0</zero_position_disarmed>`가 disarmed 상태에서 motor speed를 0으로 클램프하기 때문에, QGC의 Actuator Testing 슬라이더를 올려도 SITL Gazebo에서 프로펠러가 회전하지 않습니다. 실기 하드웨어에서는 정상 동작하는 기능이고, SITL에서는 그냥 `commander takeoff` 또는 QGC Takeoff 버튼으로 비행 확인하면 됩니다.

---

## 7. 외란 테스트 도구 (Disturbance Testing)

S550 위치 제어기의 외란 거부 강건성을 평가하기 위한 도구. **JSON 프로파일**로 외란 타임라인을 정의하면, 플러그인이 호버 중인 드론에 OU(Ornstein-Uhlenbeck) 확률 force/torque를 인가하고 외란별 화살표로 시각화한다.

### 7.1 구성 요소

| 컴포넌트 | 역할 |
|---|---|
| `tools/disturbance_profiles.json` | 외란 타임라인 정의 파일. 외란별 평균·표준편차·상관시간·frame·시작시각·지속시간 |
| `tools/disturbance_plugin/disturbance_plugin.cc` | Gazebo Classic ModelPlugin. JSON(GzString) 수신 → 타임라인 재생: 매 physics step OU 외란 적용 + 외란별 화살표 spawn/track/despawn |
| `tools/disturbance_plugin/disturbance_arrow_assets/` | 화살표 끝(cone) mesh 자원: `meshes/cone.stl` (16 segments) + `model.config` (Gazebo `model://` URI 인식용) |
| `tools/apply_disturbance.py` | CLI. JSON 프로파일을 읽어 GzString 1회 publish 로 플러그인 타임라인을 트리거. 외란 이벤트를 TSV 로 기록 |
| `overlay/.../worlds/s550.world` | s550 전용 world. `empty.world` 의 어두운 asphalt_plane 제거, 단순한 회색 평면으로 대체 |

### 7.2 왜 plugin인가 (설계 근거)

Gazebo Classic의 표준 `<link>/wrench` 토픽은 publish 1회당 1 physics step(약 1ms) 만 force가 적용된다. `gz topic -p`를 외부에서 루프로 돌리면 호출당 약 400ms 오버헤드가 있어 효과적 duty cycle이 0.6% 수준에 그쳐 외란 강도가 무의미하다. 플러그인 내부에서 매 step 외란을 계산·적용하면 단 한 번의 트리거 publish로 충분하다. OU 노이즈도 매 step 적분이 필요하므로 플러그인 내부 처리가 필수다.

화살표 시각화도 플러그인 내부에서 처리한다:
- 외란 활성화 시 `~/factory`로 cone-tipped arrow 모델 spawn (외란마다 1개, 색상 구분)
- 매 physics step `Model::SetWorldPose`로 드론 world pose에 맞춰 갱신 → 드론이 움직이면 화살표도 따라감
- 외란 비활성화 시 `~/request` (entity_delete cmd)로 despawn

> **참고:** `~/visual` 토픽으로 Visual 메시지를 publish해 기존 visual의 scale/pose를 동적으로 갱신하는 방법은 Gazebo Classic 11에서 ModelPlugin 발신 시 일관되게 동작하지 않는다 (Scene이 id 기반 lookup을 함). 별도 모델 spawn + SetWorldPose 방식이 안정적.

### 7.3 외란 프로파일 JSON

`tools/disturbance_profiles.json` 이 외란 타임라인을 정의한다. 각 외란은 OU 과정으로, **평균(mean)** 주변을 **표준편차(stddev)** 폭으로, **상관시간(correlation_time, τ)** 의 시간 척도로 변동하는 force/torque다.

```jsonc
{
  "disturbances": [
    {
      "name": "side_gust",          // 식별 이름 (화살표 라벨 / TSV 행 이름)
      "frame": "global",            // "body"(FLU) 또는 "global"(ENU) — §7.7 참조
      "start_time": 5.0,            // 트리거 0점 기준 시작 시각 (s)
      "duration": 8.0,              // 지속 시간 (s). 활성 구간 = [start, start+duration]
      "force":  { "mean": [0,3,0], "stddev": [0.5,1.0,0.3] },  // N
      "torque": { "mean": [0,0,0], "stddev": [0,0,0] },        // N·m
      "offset": [0.0, 0.0, 0.0],    // force 인가점 (m, 항상 body FLU)
      "correlation_time": 1.5       // OU τ (s). 작을수록 white, 클수록 느린 변동
    }
  ]
}
```

- **OU 정확 이산화**: `α = exp(−dt/τ)`, `β = √(1−α²)`, `F[k+1] = mean + (F[k]−mean)·α + stddev·β·N(0,1)`. τ→0 이면 white noise, 모든 τ>0 에서 안정. `stddev` 는 정상상태 표준편차.
- **타임라인 0점** = 플러그인이 트리거 메시지를 받은 sim time. 외란은 각자 `[start_time, start_time+duration]` 구간에 활성화되며, 겹치면 합산된다.
- **다중 외란**: 배열에 여러 개 정의 가능. 예시 파일은 측풍(global)·분사 반발력(body)·하강기류(global) 3종이 일부 시간 겹치도록 구성.
- 파일 상단의 `_schema` / `_frame_convention` 주석에 전체 필드 설명이 있다.

### 7.4 빌드

```bash
cd ~/Firefighting_Drone/SITL/tools/disturbance_plugin
rm -rf build && mkdir build && cd build
cmake ..
make -j$(nproc)
```

산출물: `build/libdisturbance_plugin.so`. `run_sitl.sh`가 자동으로 `GAZEBO_PLUGIN_PATH`에 prepend.

> 의존성: `libgazebo-dev` (3.4절에서 설치됨) + `libjsoncpp-dev` (§3.3에서 미리 설치 권장). jsoncpp 미설치 시 `cmake ..` 가 `pkg_check_modules(JSONCPP REQUIRED jsoncpp)` 에서 실패한다 → `sudo apt install libjsoncpp-dev`.
> 플러그인 소스/CMake 를 바꿨으므로 기존 `build/` 는 지우고 새로 빌드할 것.

> **빌드는 선택입니다.** 빌드하지 않고 `./run_sitl.sh s550`을 실행해도 드론 spawn·비행은 정상입니다 — Gazebo Classic은 SDF가 참조하는 `libdisturbance_plugin.so`를 못 찾으면 `[Err] Failed to load plugin libdisturbance_plugin.so` 를 한 줄 출력한 뒤 그 plugin만 skip합니다. 외란 실험을 할 때만 이 절을 수행하면 됩니다.

### 7.5 SDF 등록 (이미 overlay에 반영됨)

`s550.sdf.jinja`의 `</model>` 직전에 다음 블록이 있다:

```xml
<plugin name="disturbance" filename="libdisturbance_plugin.so">
  <link_name>base_link</link_name>
</plugin>
```

선택 SDF 파라미터:

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `<link_name>` | `base_link` | force/torque를 적용할 link |
| `<topic_name>` | `/gazebo/<world>/<model>/disturbance_json` | JSON(GzString) 수신 토픽 |
| `<enable_arrow>` | `true` | 화살표 시각화 on/off |
| `<arrow_scale_factor>` | `0.05` | 길이/N — 외란의 **평균 force** 크기 기준. 예: 평균 30N → 1.5m |
| `<arrow_max_length>` | `2.0` | 길이 cap (m) |
| `<arrow_radius>` | `0.03` | shaft 반경 (m). cone head 반경은 shaft × 3 |

### 7.6 사용법

(SITL 부팅 + `pxh> commander takeoff` + EKF 안정화 30초 대기 후, 새 터미널에서)

```bash
cd ~/Firefighting_Drone/SITL

# 기본 프로파일(tools/disturbance_profiles.json) 트리거 — 타임라인 끝까지 블록
./tools/apply_disturbance.py

# 다른 프로파일 + 외란 이벤트를 TSV 로 기록
./tools/apply_disturbance.py --profiles my_profile.json --log /tmp/disturbance.tsv

# 트리거만 하고 즉시 종료 (타임라인은 sim 안에서 계속 진행)
./tools/apply_disturbance.py --no-wait
```

외란 내용은 명령행 인자가 아니라 **JSON 파일을 편집**해서 바꾼다 (§7.3). 실험 단위 기록·분석은 §8 참조.

### 7.7 좌표계 규약 (Frame Convention)

외란 JSON의 각 외란은 `frame` 필드로 `body` 또는 `global` 중 하나를 지정한다. **JSON에 적는 force/torque/offset 벡터는 모두 아래 규약을 따른다.**

| `frame` | 좌표계 | 축 정의 | 용도 예시 |
|---|---|---|---|
| `global` | Gazebo 월드 고정 **ENU** | X = 동(East), Y = 북(North), **Z = 위(Up)** | 돌풍, 하강기류 — 항상 월드 기준 한 방향 |
| `body` | 드론 동체 고정 **FLU** | X = 전방(Forward), Y = 좌(Left), **Z = 상(Up)** | 분사 반발력 — 드론이 기울면 함께 기움 |

- `global`: 중력이 −Z 이므로 **하강기류(downdraft)는 force Z가 음수**, 상승기류는 양수.
- `body`: **전방 분사의 반발력은 force X가 음수**. 드론이 롤/피치하면 외란 방향도 같이 회전한다.
- PX4 펌웨어 내부는 NED를 쓰지만, 이 외란 도구는 Gazebo Link API(`AddForce`/`AddLinkForce`) 위에서 동작하므로 **JSON 입력은 위 ENU/FLU 규약**을 따른다. 혼동 주의.

**`offset` 은 예외 — `frame` 과 무관하게 항상 body(FLU) 좌표.** offset은 "기체 어디에 힘이 작용하는가"라는 물리적 위치(예: 분사 노즐 위치)이므로, `frame` 이 `global` 이어도 base_link 기준으로 해석한다. `frame` 은 force/torque의 *방향*만 결정한다.

플러그인은 `global` 외란을 `AddForceAtWorldPosition`/`AddTorque`, `body` 외란을 `AddLinkForce`/`AddRelativeTorque`로 인가한다. 화살표는 두 경우 모두 force를 world frame으로 변환해 표시하므로 실제 미는 방향을 정확히 보여준다.

### 7.8 화살표 표시 규칙

- 외란마다 **자기 화살표 1개**. 활성 구간 동안만 표시되고, 끝나면 despawn.
- 화살표 방향 = 그 외란의 **평균 force 방향** (OU 순간값이 아니라 mean). `body` 외란은 드론 자세에 따라 회전, `global` 외란은 월드 고정 방향.
- 화살표 길이 = `min(|평균 force| × arrow_scale_factor, arrow_max_length)`.
- 색상은 6색 팔레트를 외란 인덱스 순서로 순환 — `plot_experiment.py` 의 외란 구간 음영 색과 동일하므로, 시뮬 화면의 화살표와 분석 plot 을 색으로 매칭할 수 있다.
- 평균 force 가 0 인 외란(순수 noise 또는 torque 전용)은 화살표를 그리지 않는다.

### 7.9 화살표 모양 커스터마이즈

`disturbance_plugin.cc`의 `SpawnArrow()` 함수 내부에서 조정 가능:
- `shaftLen = length * 0.8` / `headLen = length * 0.2` — shaft/head 비율
- `headR = arrowRadius * 3.0` — head 굵기 (shaft 대비 배수)
- `kArrowColors` 팔레트 — 외란별 색상

수정 후 `make -j$(nproc)` 재빌드 + SITL 재기동으로 반영.

### 7.10 화살표 구조

```
┌─────────────────────────────────────┐
│   shaft (cylinder)        head      │
│  radius=0.03m            (cone STL) │
│   length=80% of L      length=20%  │
│                          radius=3x  │
│                                     │
│  ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━▶  │
│  ↑                                  │
│  drone CoG (또는 offset 지점)
└─────────────────────────────────────┘
```

`length = min(|평균 force| × 0.05, 2.0) m` — 평균 외력 크기에 비례. 위 그림의 ━ 부분이 shaft cylinder, ▶ 부분이 cone STL mesh.

---

## 8. 실험 데이터 기록 (Logging)

외란 응답을 분석하기 위해 비행 데이터를 디스크에 저장하고(8.1~8.4), `plot_experiment.py`로 시각화(8.5)하는 단계. 더 깊은 정량 분석은 외부 도구로 확장한다.

### 8.1 PX4 ULog — 이미 자동

PX4 SITL은 부팅과 동시에 ULog 기록을 시작한다 (`INFO [logger] Start file log`). 위치:
```
PX4-Autopilot/build/px4_sitl_default/rootfs/log/<날짜>/<시각>.ulg
```

이 파일에 거의 모든 uORB 토픽이 시간순으로 들어간다. 외란 분석에 쓰이는 핵심 토픽:

| 토픽 | 의미 |
|---|---|
| `vehicle_local_position` | EKF 위치/속도 추정 (NED) |
| `vehicle_local_position_groundtruth` | **Gazebo 실측** 위치/속도 (s550 SDF의 groundtruth_plugin 덕분) |
| `vehicle_attitude` / `vehicle_attitude_groundtruth` | 자세 (quaternion), 추정 vs 실측 |
| `vehicle_attitude_setpoint`, `vehicle_local_position_setpoint` | 제어기 setpoint |
| `vehicle_angular_velocity`, `sensor_combined` | 각속도, IMU raw |
| `actuator_motors`, `actuator_outputs` | 6개 모터 출력 |
| `ekf2_innovations` | EKF residual (추정 성능) |

> **추정 vs 실측 비교**: `*_groundtruth` 토픽 덕분에 EKF 추정이 실제와 얼마나 차이나는지 직접 평가 가능. 제어기 성능 분석의 핵심.

ULog가 기록하는 토픽 범위는 `SDLOG_PROFILE` 파라미터로 조정 가능하지만, 기본값으로 외란 분석엔 충분하다. 더 높은 rate가 필요하면 `pxh> param set SDLOG_PROFILE 8` (high rate) 후 재기록.

### 8.2 한계 — 외란 인가 시점은 ULog에 없음

`apply_disturbance.py`가 언제·어떤 force를 인가했는지는 PX4가 모르므로 ULog에 안 들어간다. 이를 별도 TSV로 기록하고, 분석 시 시간축을 정렬한다 (`tools/data/README.md` 참조).

### 8.3 `record_experiment.py` — 실험 단위 스냅샷

[tools/record_experiment.py](tools/record_experiment.py)는 외란 트리거 + ULog 스냅샷 + 메타데이터 저장을 한 번에 처리한다.

**전제**: SITL이 떠 있고 (`./run_sitl.sh s550`) 드론이 호버 중 (`pxh> commander takeoff` + EKF 안정화 30초).

```bash
cd ~/Firefighting_Drone/SITL

# 기본 프로파일(tools/disturbance_profiles.json)로 실험
./tools/record_experiment.py --name baseline_gust

# 다른 프로파일 + 메모 + settle 길게
./tools/record_experiment.py --name spray_test --profiles my_profile.json \
    --settle 10 --notes "분사 반발력 + 측풍 동시 인가, 비대칭 응답 확인"
```

수행 동작:
1. `tools/data/<timestamp>_<name>/` 디렉토리 생성
2. 사용한 프로파일 JSON 을 `disturbance_profiles.json` 으로 사본 저장
3. `apply_disturbance.py` 호출 (외란 타임라인 트리거 + `disturbance.tsv` 기록, 타임라인 종료까지 블록)
4. `--settle` 초 대기 (응답이 ULog에 다 들어가도록)
5. PX4 `rootfs/log/` 의 최신 `.ulg` → `flight.ulg` 로 복사
6. `metadata.json` 저장 (외란 목록, controller 라벨, 메모 등)

### 8.4 출력 구조

```
tools/data/20260515T153021_baseline_gust/
├── flight.ulg                 # ULog 스냅샷 (SITL 세션 시작 ~ 복사 시점)
├── disturbance.tsv            # 외란 이벤트 (외란당 1행: name/frame/start/end/τ/mean/stddev/offset)
├── disturbance_profiles.json  # 사용한 외란 프로파일 사본
└── metadata.json              # 실험 컨텍스트
```

`tools/data/` 는 `.gitignore` 처리 — 데이터는 git에 안 올라가고 각자 관리. 형식 상세는 [tools/data/README.md](tools/data/README.md).

### 8.5 분석 — `plot_experiment.py`

[tools/plot_experiment.py](tools/plot_experiment.py)는 실험 디렉토리 하나를 입력으로 받아 3×3 plot 을 그린다.

```bash
./tools/plot_experiment.py tools/data/20260515T153021_baseline_gust
./tools/plot_experiment.py tools/data/<exp> --source groundtruth --no-show
```

- **Row 1 궤적**: top / front / side view, 시간 흐름을 무지개 색으로. 6개 축 scale 통일.
- **Row 2 위치 / Row 3 자세**: x/y/alt, roll/pitch/yaw vs 시간. ground truth(solid) + estimate(dashed).
- **다중 외란**: `disturbance.tsv` 의 모든 외란을 각각 색깔 음영 구간으로 표시 (색상은 시뮬 화면 화살표 팔레트와 동일 순서). 시간축 0점 = 첫 외란 시작.
- `disturbance.tsv`(unix time) ↔ `flight.ulg`(PX4 boot µs) 정렬은 `vehicle_gps_position.time_utc_usec` 로 자동 처리.
- 출력 `analysis.png` 를 실험 디렉토리에 저장. 전제: `pip3 install --user pyulog`.

> 더 깊은 분석(회복 시간·RMS·제어기 비교 등)은 PlotJuggler / pyulog / flight_review 등 외부 도구로 `tools/data/<실험>/` 을 입력 삼아 자유롭게 확장할 수 있다.

---

## 9. Troubleshooting (시행착오 로그)

셋업·개발 과정에서 실제 발생한 문제와 해결 내역입니다. 동일한 함정에 다시 빠지지 않기 위해 기록합니다.

### Issue #1 — `ubuntu.sh`가 Jammy에서 gz-garden을 깔며 Classic을 밀어냄

**증상**
- `bash Tools/setup/ubuntu.sh` 정상 완료
- 그러나 이어서 `make px4_sitl gazebo-classic` 빌드가 마지막 실행 단계에서 실패:
  ```
  SITL ARGS
  model: iris
  You need to have gazebo simulator installed!
  FAILED: src/modules/simulation/simulator_mavlink/CMakeFiles/gazebo-classic_iris
  ```
- 그런데 셋업 전에는 `which gazebo` → `/usr/bin/gazebo` 였음

**원인**
- `Tools/setup/ubuntu.sh` 232줄에서 Ubuntu Jammy에 대해 `gazebo_packages="gz-garden"`으로 분기 → OSRF의 **새 Gazebo Sim (gz-garden, gz-sim7, libgz-*)** 을 설치
- apt가 gz-garden 설치 중 충돌 회피로 **Classic 런타임 바이너리(`gazebo` 패키지)** 를 자동 제거
- 그러나 헤더/라이브러리(`libgazebo-dev`, `libgazebo11`)는 남아있어 컴파일은 성공, 실행 시점에서 `sitl_run.sh`가 `command -v gazebo`를 못 찾고 죽음

**해결**
```bash
sudo apt install gazebo gazebo-common gazebo-plugin-base libgazebo-dev libgazebo11
```
- apt가 gz-garden 및 libgz-* 44개를 제거하겠다고 묻는데 그대로 `y`로 진행
- Classic 11.10.2 재설치 후 `which gazebo` → `/usr/bin/gazebo` 복구

**교훈**
- `ubuntu.sh`를 재실행하거나 새 머신에 셋업할 때 항상 이 함정을 의심해야 합니다.
- PX4 v1.14는 Classic→Sim 전환기에 있어 setup 스크립트의 디폴트가 빌드 타깃(`gazebo-classic_*`)과 어긋난 케이스.

### Issue #2 — ROS 2 자동 소싱에 의한 `GAZEBO_*_PATH` 오염

**증상**
- `gazebo` 재설치 후 `make px4_sitl gazebo-classic` 실행 → 빌드/실행은 시작되었으나:
  ```
  INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
  [무한 대기]
  ```
- Gazebo 창은 떴고 iris도 보였으나, PX4와의 MAVLink 연결이 안 됨

**원인**
- 호스트의 `~/.bashrc`에서 ROS 2 (Humble 등)와 워크스페이스가 자동 소싱되며 다음 환경변수가 채워짐:
  ```
  GAZEBO_MODEL_PATH=<ROS 2 워크스페이스 경로>:...
  GAZEBO_PLUGIN_PATH=    (empty)
  AMENT_PREFIX_PATH=<ROS 2 install 경로>
  ROS_DISTRO=humble
  ```
- `sitl_run.sh`는 내부적으로 `source setup_gazebo.bash`로 PX4 경로를 설정하지만, 기존 환경의 오염이 우선하면 작동 실패
- `libgazebo_mavlink_interface.so` (PX4 빌드 산출물의 핵심 플러그인)을 Gazebo가 못 찾아 로드 실패 → 4560 포트로 PX4에 연결을 시도하지 않음 → PX4 무한 대기

**해결**
- 깨끗한 셸에서 ROS 2/conda 환경변수 제거 후 실행:
  ```bash
  conda deactivate 2>/dev/null
  unset GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH \
        GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH \
        IGN_GAZEBO_SYSTEM_PLUGIN_PATH IGN_GAZEBO_RESOURCE_PATH \
        AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
        ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
        PYTHONPATH LD_LIBRARY_PATH
  make px4_sitl gazebo-classic
  ```
- 영구 해결: `run_sitl.sh` 래퍼 스크립트가 매 실행 시 자동으로 이 정리 작업 수행

**교훈**
- ROS 2 환경이 셋업된 머신에서 PX4 Gazebo Classic SITL을 돌릴 땐 항상 환경 오염을 의심해야 합니다.
- 빌드 시점(`ubuntu.sh` 실행 등)에서도 깨끗한 쉘이 권장됩니다.

### Issue #3 — heredoc 안에서 make 실행 시 `pxh>` 입력 안 됨

**증상**
- 깨끗한 환경에서 PX4 SITL이 정상 부팅 (`pxh>` 까지 도달, `Ready for takeoff!` 출력)
- 그러나 `pxh>` 프롬프트에 키를 눌러도 명령이 입력되지 않음 (커서만 깜빡)

**원인**
- heredoc(`<<'EOF' ... EOF`)은 bash의 stdin으로 heredoc 내용을 공급하며, EOF에 도달하면 bash의 stdin이 닫혀버림
- bash가 그 안에서 호출한 `make` → `sitl_run.sh` → `px4` 프로세스가 stdin을 상속받았는데 이미 닫혀있어, `pxh>`가 키 입력을 읽을 수 없음

**해결**
- heredoc 대신 일반 스크립트 파일 또는 현재 터미널에서 직접 실행
- 본 프로젝트의 `run_sitl.sh`는 일반 스크립트이므로 영향 없음

**교훈**
- 인터랙티브 쉘(`pxh>`)이 필요한 명령은 heredoc 안에 넣지 말 것.

### Issue #4 — 새 모델 추가 후 `ninja: no work to do.` (sitl_gazebo-classic 재구성 누락)

**증상**
- `models/<new_model>/` 디렉토리 + `.sdf.jinja` 추가 + sitl_targets에 모델명 등록 후 `make px4_sitl gazebo-classic_<new_model>` 실행
- 그런데 SDF가 자동 생성되지 않고:
  ```
  [0/X] Performing build step for 'sitl_gazebo-classic'
  ninja: no work to do.
  ...
  Model <new_model> not found in model path: ...
  ```

**원인**
- `Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt`의 jinja → sdf 변환은 `file(GLOB_RECURSE)` (CONFIGURE_DEPENDS 없음) — CMake configure 시점에만 jinja 파일 목록을 캐싱
- `sitl_gazebo-classic`은 PX4의 ExternalProject로 래핑되어 있어, 한 번 configure된 후엔 강제 트리거 없이는 재configure 안 됨
- 새 모델 추가 후 ExternalProject가 build step만 시도 → 캐시된 jinja 목록엔 새 모델 없음 → 아무 일도 안 함

**해결**
```bash
rm -rf <PX4_ROOT>/build/px4_sitl_default/external/Stamp/sitl_gazebo-classic
```
그 후 다시 빌드. ExternalProject가 처음부터 mkdir → configure → build 단계를 다시 밟으면서 새 jinja도 발견.

**교훈**
- 새 모델을 추가하거나 기존 SDF/jinja를 편집할 때마다 위 stamp 디렉토리 정리 필요.
- Stamp 위치는 `build/<config>/external/Stamp/<external_project_name>/`. PX4의 `sitl_gazebo-classic-prefix/...` 아님 (처음에 헷갈렸음).

### Issue #5 — typhoon_h480 베이스 SDF의 짐벌/랜딩기어 동특성

**증상**
S550 SDF를 typhoon_h480에서 분기(이름만 치환) 후 빌드:
- 착륙이 불안정 (틸트, 바운스)
- 호버 중 짐벌 카메라 시점이 본체보다 더 크게 흔들림

**원인 1 — 랜딩기어 swing-on-arming**
- typhoon의 landing leg는 revolute joint + PID 제어
- PWM_MAIN_FUNC10/11을 airframe에서 제거하면 PX4 입력이 0
- 그러나 `mavlink_interface` 채널 공식 `pos = zero_armed + (input + offset) * scaling` (offset=1, scaling=0.5)에 따라 **PID가 다리를 0.5 rad(반쯤 접힘)로 끌고감**
- 비행 중 다리가 비스듬한 각도, 착륙 시 cylinder가 한쪽으로 접지하며 틸트

**원인 2 — 짐벌 펜듈럼**
- 짐벌 hierarchy(mount → vertical_arm → horizontal_arm → camera) 총 ~0.4 kg
- `gimbal_controller_plugin` + `mavlink_interface`의 gimbal_* 채널이 양쪽 PID로 동시 제어 → 충돌
- 짐벌이 base_link에서 펜듈럼처럼 흔들리며 자세 제어에 잡음 추가

**1차 해결 — joint를 fixed로 + 채널/플러그인 제거**
- 6개 joint를 `type='revolute'` → `type='fixed'` (cgo3_*_joint 4개 + left_leg_joint + right_leg_joint)
- `mavlink_interface` plugin에서 channel 5개 제거 (gimbal_roll/pitch/yaw, left_leg, right_leg)
- `gimbal_controller` plugin 블록 통째로 제거
- 결과: 다리는 강체로 고정, 착륙 안정성 회복. 짐벌도 채널 충돌 해소.

**원인 3 — 짐벌 4단 chained fixed joint의 누적 솔버 컴플라이언스**
- joint를 fixed로 해도 Gazebo Classic ODE는 fixed joint를 6-DOF 제약(작은 CFM 컴플라이언스)으로 구현
- 4단 chain은 컴플라이언스가 누적되어 끝(camera)에서 잔진동
- 게다가 camera는 base_link 기준 z=-0.586m로 긴 모멘트 암 → 본체 미세 회전이 카메라에서 크게 증폭

**2차 해결 — 짐벌 4-link를 base_link 직속 평탄화**
- 4개 `cgo3_*_joint` 모두 parent를 `base_link`로 변경, pose를 누적값으로 재계산
- 단일 fixed joint × 4 (병렬) 구조 → chain compliance 누적 없음

**원인 4 — `cgo3_camera_collision` sphere가 지면에 깊이 침투**
- camera_link의 sphere collision (r=0.035, kp=1e8)이 spawn 시 base_link 기준 z=-0.586
- 월드 z = 0.26(model pose) + (-0.586) = -0.326 (지면 36cm 아래!)
- ODE 솔버가 격렬히 해소 시도하며 본체 전체에 진동 전파

**3차 해결 — camera sphere collision 제거**
- 시각 메쉬는 유지, collision 블록만 삭제
- 카메라는 시뮬에서 물리적으로 부딪힐 일이 없으므로 collision은 불필요

**교훈**
- typhoon에서 모델을 분기할 땐 visual 메쉬만 재활용한다는 의도에 맞춰 dynamics 부속(채널, 플러그인, 컨트롤러)도 통째로 제거할 것.
- 사용하지 않을 actuator의 joint는 fixed + 채널·플러그인 제거가 통일된 패턴.
- chain depth는 ODE 솔버 컴플라이언스를 누적시키므로 가능하면 base_link 직속 평탄화.
- visual-only로 쓸 collision은 제거.

### 그 외 무해한 메시지 (안 고쳐도 됨)

다음 메시지들은 SITL 동작에 영향을 주지 않습니다:

- `etc/init.d-posix/rcS: 39: [: Illegal number:` — rcS 스크립트의 빈 환경변수 비교 경고
- `[Err] [InsertModelWidget.cc:403] Missing model.config for model "..."` — Gazebo Insert 패널이 GAZEBO_MODEL_PATH의 ROS 2 경로를 스캔하다 model.config 못 찾음. GUI 노이즈일 뿐 (`run_sitl.sh`로 실행하면 사라짐)
- `[Wrn] [Event.cc:61] Warning: Deleting a connection right after creation` — Gazebo 내부 코드 경고
- `WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data` — 부팅 직후 EKF가 GPS 융합 중. 20~30초 대기하면 사라짐
- `INFO [commander] LED: open /dev/led0 failed (22)` — SITL이라 LED 디바이스 없음. 정상
- `src/drivers/uavcan/libuavcan` 서브모듈의 재귀 적용 실패 — UAVCAN은 SITL과 무관

---

## 10. 다음 단계

- [x] Step 1 — 시스템 의존성 확인 및 설치
- [x] Step 2 — PX4-Autopilot v1.14.4 클론 + 서브모듈
- [x] Step 3 — `ubuntu.sh`로 빌드 의존성 설치 + Gazebo Classic 복구
- [x] Step 4 — iris(quad)로 SITL Sanity Check
- [x] Step 5 — `typhoon_h480`(hexa) 베이스로 SITL 빌드/이륙 검증
- [x] Step 6 — S550 커스텀 airframe + SDF 모델 작성
- [x] Step 7 — S550 타깃 빌드 + 이륙 검증 (※ 지상 미세 진동은 6.5 (a)로 보류)
- [x] Step 8 — QGroundControl AppImage 연결 + GUI 비행 검증 (MAVLink UDP 14550 자동 연결)
- [x] Step 9 — **외란 입력 도구 Phase 1** — disturbance_plugin 빌드, `apply_disturbance.py` 단일 publish + duration → plugin이 매 physics step force 적용
- [x] Step 10 — **외란 입력 도구 Phase 2** — cone-tipped 화살표, 드론에 부착되어 매 step 추적 (factory spawn + SetWorldPose)
- [x] Step 11 — `s550.world` 추가 (단순 회색 ground)
- [x] Step 12 — **실험 데이터 기록** — `record_experiment.py` (ULog 스냅샷 + 외란 TSV + metadata.json), `tools/data/` 구조 (8장)
- [x] Step 13 — **로그 시각화** — `plot_experiment.py` 3×3 plot (궤적/위치/자세, 외란 구간 음영)
- [x] Step 14 — **JSON 외란 프로파일** — `disturbance_profiles.json` + 플러그인 전면 개편: OU 확률 외란, body/global frame, 타임라인 트리거, 다중 외란, 외란별 화살표 (7장)
- [ ] Step 15 — **베이스라인 측정** — default PX4의 외란 응답 정량화 (회복 시간·RMS 등)
- [ ] Step 16 — **외란 보상 제어기** — DOB / 적분기 튜닝 / ADRC 등 (별도 세션, 접근 미정)
- [ ] Step 17 — 통합 시나리오 스크립트 (takeoff→호버→외란→로그→착륙 자동화)
- [ ] (이후) S550 지상 진동 해소, mesh S550 사양으로 교체, 소방 페이로드/센서 추가, ROS 2 px4_ros_com 브리지 등

---

## 11. 참고 자료

- PX4 v1.14 공식 문서: https://docs.px4.io/v1.14/
- PX4 SITL Gazebo Classic: https://docs.px4.io/v1.14/en/sim_gazebo_classic/
- Gazebo Classic: https://classic.gazebosim.org/
- Gazebo Classic Plugin Tutorial: https://classic.gazebosim.org/tutorials?cat=write_plugin
- Gazebo Wrench msg / Link::AddLinkForce API: https://gazebosim.org/api/gazebo/11/classgazebo_1_1physics_1_1Link.html
- DJI F550/S550 hardware reference: https://www.dji.com/flame-wheel-arf
- QGroundControl: https://docs.qgroundcontrol.com/master/en/
