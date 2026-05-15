#!/usr/bin/env python3
"""Run a JSON-defined disturbance experiment and snapshot all artifacts.

이 스크립트는 사용자가 이미 SITL 을 띄우고 (`./run_sitl.sh s550`) 드론을 호버
상태로 만든 뒤 실행한다. 외란 프로파일(JSON)을 트리거 → 타임라인 종료까지 대기
→ settle → PX4 ULog + 외란 이벤트 TSV + 실험 메타데이터 + 프로파일 사본을
한 디렉토리에 묶어 저장한다.

저장 위치: `tools/data/<YYYYMMDD>T<HHMMSS>_<name>/`
  - flight.ulg                 : 현재 SITL 세션의 ULog 스냅샷
  - disturbance.tsv            : apply_disturbance.py 출력 (외란당 1행)
  - disturbance_profiles.json  : 사용한 외란 프로파일 사본
  - metadata.json              : 실험 컨텍스트

사용 예:
  # 기본 프로파일로 실험
  ./record_experiment.py --name baseline_gust

  # 다른 프로파일 + 메모 + settle 길게
  ./record_experiment.py --name spray_test --profiles my_profile.json \\
      --settle 10 --notes "분사 반발력 + 측풍 동시 인가, 비대칭 응답 확인"
"""
import argparse
import json
import pathlib
import shutil
import subprocess
import sys
import time
from datetime import datetime


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PX4_LOG_ROOT = REPO_ROOT / "PX4-Autopilot" / "build" / "px4_sitl_default" / "rootfs" / "log"
DATA_ROOT = REPO_ROOT / "tools" / "data"
APPLY_SCRIPT = pathlib.Path(__file__).resolve().parent / "apply_disturbance.py"
DEFAULT_PROFILE = REPO_ROOT / "tools" / "disturbance_profiles.json"


def find_latest_ulog():
    """Return the .ulg file with the most-recent mtime under PX4_LOG_ROOT."""
    if not PX4_LOG_ROOT.exists():
        sys.exit(f"PX4 log root not found: {PX4_LOG_ROOT}\n"
                 f"SITL을 한 번이라도 띄운 적이 없습니다.")
    ulgs = list(PX4_LOG_ROOT.rglob("*.ulg"))
    if not ulgs:
        sys.exit(f"No .ulg files found under {PX4_LOG_ROOT}.\n"
                 f"SITL이 현재 떠 있고 PX4 logger가 동작 중인지 확인.")
    return max(ulgs, key=lambda p: p.stat().st_mtime)


def load_profile(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except FileNotFoundError:
        sys.exit(f"프로파일 파일 없음: {path}")
    except json.JSONDecodeError as e:
        sys.exit(f"JSON 파싱 실패 ({path}): {e}")
    dists = data.get("disturbances")
    if not isinstance(dists, list) or not dists:
        sys.exit(f"'disturbances' 배열이 비어 있거나 없음: {path}")
    return dists


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--name', required=True,
                   help='실험 식별 이름. 디렉토리명에 포함 (예: baseline_gust)')
    p.add_argument('--notes', default='', help='metadata.json 의 자유 메모 필드')
    p.add_argument('--profiles', default=str(DEFAULT_PROFILE),
                   help=f'외란 프로파일 JSON 경로 (default: {DEFAULT_PROFILE})')
    p.add_argument('--settle', type=float, default=5.0,
                   help='타임라인 종료 후 ULog snapshot 까지의 추가 대기 (s).')
    p.add_argument('--model', default='s550', help='Gazebo 모델명 (default: s550)')
    p.add_argument('--world', default='default', help='Gazebo 월드명 (default: default)')
    p.add_argument('--controller', default='default_PX4_v1.14.4',
                   help='metadata.json 의 controller 라벨 (수정 제어기 실험 시 변경)')
    args = p.parse_args()

    profile_path = pathlib.Path(args.profiles).resolve()
    dists = load_profile(profile_path)

    # 실험 디렉토리 생성
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    safe_name = args.name.replace('/', '_').replace(' ', '_')
    exp_dir = DATA_ROOT / f"{ts}_{safe_name}"
    exp_dir.mkdir(parents=True, exist_ok=True)

    tsv_path = exp_dir / "disturbance.tsv"
    metadata_path = exp_dir / "metadata.json"
    profile_dst = exp_dir / "disturbance_profiles.json"
    ulg_dst = exp_dir / "flight.ulg"

    # 프로파일 사본 저장 (실험 재현용)
    shutil.copy2(profile_path, profile_dst)

    # ---- 1) 외란 트리거 (apply_disturbance.py 호출, 타임라인 종료까지 블록) ----
    cmd = [
        str(APPLY_SCRIPT),
        '--profiles', str(profile_path),
        '--model', args.model,
        '--world', args.world,
        '--log', str(tsv_path),
    ]
    print(f"[record] experiment dir: {exp_dir}")
    print(f"[record] invoking: {' '.join(cmd)}")
    sys.stdout.flush()

    t0_unix = time.time()
    rc = subprocess.run(cmd).returncode
    t1_unix = time.time()
    if rc != 0:
        print(f"[record] WARN: apply_disturbance.py exited with code {rc}",
              file=sys.stderr)

    # ---- 2) settle wait (응답이 ULog에 다 들어가도록) ----
    if args.settle > 0:
        print(f"[record] settling {args.settle:.1f}s for response to finish in ULog ...",
              flush=True)
        time.sleep(args.settle)

    # ---- 3) 최신 ULog 복사 ----
    ulg_src = find_latest_ulog()
    print(f"[record] copying ULog: {ulg_src.relative_to(REPO_ROOT)} → "
          f"{ulg_dst.relative_to(REPO_ROOT)}")
    shutil.copy2(ulg_src, ulg_dst)
    ulg_size_kb = ulg_dst.stat().st_size / 1024.0

    # ---- 4) metadata.json ----
    metadata = {
        "experiment_name": args.name,
        "timestamp": ts,
        "wall_time_start_unix": t0_unix,
        "wall_time_end_unix": t1_unix,
        "px4_log_source": str(ulg_src.relative_to(REPO_ROOT)),
        "model": args.model,
        "controller": args.controller,
        "profile_file": "disturbance_profiles.json",
        "disturbances": dists,
        "settle_s": args.settle,
        "notes": args.notes,
    }
    with metadata_path.open('w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    print(f"[record] saved:")
    print(f"  {tsv_path.relative_to(REPO_ROOT)}")
    print(f"  {profile_dst.relative_to(REPO_ROOT)}")
    print(f"  {ulg_dst.relative_to(REPO_ROOT)}   ({ulg_size_kb:.1f} KB)")
    print(f"  {metadata_path.relative_to(REPO_ROOT)}")
    print(f"[record] done. 분석은 plot_experiment.py 에 위 디렉토리를 입력으로.")


if __name__ == '__main__':
    main()
