#!/usr/bin/env bash
# Apply S550 overlay to the PX4-Autopilot checkout.
#
# 본 스크립트는 overlay/ 디렉토리의 파일들을 PX4-Autopilot/ 안의 대응 경로로
# 복사한다. README의 절차에 따라 PX4-Autopilot을 먼저 clone + checkout(v1.14.4)
# 한 뒤 실행하면 된다.
#
# 사용법:
#   ./apply_overlay.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_DIR="${SCRIPT_DIR}/overlay"
PX4_ROOT="${SCRIPT_DIR}/PX4-Autopilot"

if [[ ! -d "$OVERLAY_DIR" ]]; then
    echo "ERROR: overlay/ not found at $OVERLAY_DIR"
    exit 1
fi

if [[ ! -d "$PX4_ROOT" ]]; then
    echo "ERROR: PX4-Autopilot/ not found at $PX4_ROOT"
    echo "       Clone PX4-Autopilot first (see README §3.2)."
    exit 1
fi

# v1.14.4 태그가 체크아웃되어 있는지 가벼운 확인
CURRENT_TAG="$(cd "$PX4_ROOT" && git describe --tags 2>/dev/null || true)"
if [[ "$CURRENT_TAG" != "v1.14.4" ]]; then
    echo "WARNING: PX4-Autopilot 현재 태그가 v1.14.4가 아닙니다 (현재: ${CURRENT_TAG:-unknown})."
    echo "         README §3.2 절차에 따라 v1.14.4로 체크아웃하길 권장합니다."
    echo "         그래도 계속 진행하려면 5초 후 시작합니다. 중단하려면 Ctrl+C."
    sleep 5
fi

echo "================================================"
echo "  Applying S550 overlay"
echo "    OVERLAY  : ${OVERLAY_DIR}"
echo "    PX4_ROOT : ${PX4_ROOT}"
echo "================================================"

# overlay/. 의 . 은 hidden 포함 모든 컨텐츠를 의미. tar 대신 cp -r 으로 깔끔히.
cp -rv "${OVERLAY_DIR}/." "${PX4_ROOT}/"

echo ""
echo "Overlay 적용 완료."
echo ""
echo "참고: 이미 한 번 빌드한 적이 있다면, 새 모델을 인식시키기 위해"
echo "sitl_gazebo-classic 서브프로젝트의 stamp 디렉토리를 정리해야 합니다:"
echo "  rm -rf ${PX4_ROOT}/build/px4_sitl_default/external/Stamp/sitl_gazebo-classic"
echo ""
echo "이제 './run_sitl.sh s550' 으로 SITL을 띄울 수 있습니다."
