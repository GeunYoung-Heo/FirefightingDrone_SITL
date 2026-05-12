#!/usr/bin/env bash
# PX4 SITL + Gazebo Classic launcher wrapper
#
# 이 스크립트는 매 SITL 세션마다 필요한 환경 정리(conda 비활성화 + ROS 2
# 자동 소싱에 의한 GAZEBO_*_PATH / AMENT_PREFIX_PATH 등 오염 제거 + 좀비
# gzserver/gzclient/px4 프로세스 청소)를 수행한 뒤 `make px4_sitl
# gazebo-classic_<model>` 을 실행한다.
#
# 사용법:
#   ./run_sitl.sh                    # iris (default, quad)
#   ./run_sitl.sh typhoon_h480       # hexacopter
#   ./run_sitl.sh s550               # S550 (Step 6 이후 airframe/SDF 등록 후)
#
# 자세한 배경은 README.md 의 "4. SITL 실행" 절 참조.

set -e

# (1) conda base 환경 빠져나오기 (PX4 v1.14는 시스템 Python 3.10 기준 검증)
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    eval "$(conda shell.bash hook 2>/dev/null)" || true
    conda deactivate 2>/dev/null || true
fi

# (2) ROS 2 / 새 Gazebo Sim 환경변수 제거
#     ~/.bashrc 에서 /opt/ros/humble/setup.bash 또는 ROS 2 워크스페이스의
#     local_setup.bash 가 소싱되면 아래 변수들이 채워지며 Classic 빌드를
#     깨뜨린다. 현재 셸 한정으로만 unset.
unset GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH \
      GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH \
      IGN_GAZEBO_SYSTEM_PLUGIN_PATH IGN_GAZEBO_RESOURCE_PATH \
      AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
      PYTHONPATH

# (3) Gazebo 카메라를 free 모드로 시작
#     기본값으로 sitl_run.sh는 libgazebo_user_camera_plugin.so 를 띄워
#     카메라가 드론을 자동 추적한다. 이 플러그인이 마우스 입력을 가로채
#     스크롤 휠 zoom 등 일반 Gazebo 컨트롤이 동작하지 않는다.
#     필요시 Gazebo GUI의 World 트리에서 모델 우클릭 → Follow 로 추적 활성 가능.
export PX4_NO_FOLLOW_MODE=1

# (4) 이전 세션의 좀비 프로세스 청소
#     sitl_run.sh 는 `pkill -x gazebo` 만 수행하므로 gzserver/gzclient 가
#     남아 다음 실행 시 드론이 중복 생성되는 현상이 흔히 발생한다.
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true

# (5) SITL 실행
#     PX4_ROOT는 이 스크립트와 같은 디렉토리에 있는 PX4-Autopilot으로 해석.
#     저장소를 어느 경로에 두든 자동으로 동작 (팀원 PC 호환).
MODEL="${1:-iris}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${SCRIPT_DIR}/PX4-Autopilot"

if [[ ! -d "$PX4_ROOT" ]]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_ROOT"
    echo "       Make sure run_sitl.sh is placed next to the PX4-Autopilot directory."
    exit 1
fi

cd "$PX4_ROOT"

echo "================================================"
echo "  PX4 SITL launching"
echo "    model     : ${MODEL}"
echo "    target    : gazebo-classic_${MODEL}"
echo "    PX4_ROOT  : ${PX4_ROOT}"
echo "================================================"

exec make px4_sitl "gazebo-classic_${MODEL}"
