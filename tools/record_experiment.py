#!/usr/bin/env python3
"""Run a disturbance experiment and snapshot all artifacts into tools/data/.

이 스크립트는 사용자가 이미 SITL을 띄우고 (`./run_sitl.sh s550`) 드론을 호버
상태로 만든 뒤 실행한다. 외란 인가 → 잠시 settle → PX4 ULog 와 외란 이벤트
TSV 와 실험 메타데이터를 한 디렉토리에 묶어 저장한다.

저장 위치: `tools/data/<YYYYMMDD>T<HHMMSS>_<name>/`
  - flight.ulg       : 현재 SITL 세션의 ULog 스냅샷
  - disturbance.tsv  : apply_disturbance.py 출력
  - metadata.json    : 실험 컨텍스트

사용 예:
  # 1초 펄스, X 30N
  ./record_experiment.py --name step_30N_x --force "30 0 0" --duration 1.0

  # 메모 + offset + 회복 대기 길게
  ./record_experiment.py --name pulse_off_y --force "20 0 0" \\
      --offset "0 0.2 0" --duration 0.5 --settle 10 \\
      --notes "CoG에서 Y +20cm offset, 비대칭 토크 응답 확인"

  # Sustained (Ctrl+C 로 외란 종료 후 자동 ULog 스냅샷)
  ./record_experiment.py --name wind_5N --force "5 0 0" --sustained --settle 5
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


def parse_vec3_safe(s):
    try:
        return [float(v) for v in s.split()]
    except Exception:
        return None


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--name', required=True,
                   help='실험 식별 이름. 디렉토리명에 포함 (예: step_30N_x)')
    p.add_argument('--notes', default='', help='metadata.json 의 자유 메모 필드')
    p.add_argument('--force', default='0 0 0', help='Body-frame force "Fx Fy Fz" (N)')
    p.add_argument('--torque', default='0 0 0', help='Body-frame torque "Tx Ty Tz" (N·m)')
    p.add_argument('--offset', default='0 0 0',
                   help='Force 인가점 offset "x y z" (m, base_link frame)')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--duration', type=float, default=1.0,
                   help='Pulse duration (s) — 기본 1.0')
    g.add_argument('--sustained', action='store_true',
                   help='Sustained 외란. Ctrl+C로 외란 종료 후 자동 settle+snapshot.')
    p.add_argument('--settle', type=float, default=5.0,
                   help='외란 종료 후 ULog snapshot 까지의 추가 대기 (s). 응답이 ULog에 다 들어가도록.')
    p.add_argument('--model', default='s550', help='Gazebo 모델명 (default: s550)')
    p.add_argument('--controller', default='default_PX4_v1.14.4',
                   help='metadata.json 의 controller 라벨 (수정 제어기 실험 시 적절히 변경)')
    args = p.parse_args()

    # 실험 디렉토리 생성
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    safe_name = args.name.replace('/', '_').replace(' ', '_')
    exp_dir = DATA_ROOT / f"{ts}_{safe_name}"
    exp_dir.mkdir(parents=True, exist_ok=True)

    tsv_path = exp_dir / "disturbance.tsv"
    metadata_path = exp_dir / "metadata.json"
    ulg_dst = exp_dir / "flight.ulg"

    # ---- 1) 외란 인가 (apply_disturbance.py 호출) ----
    cmd = [
        str(APPLY_SCRIPT),
        '--force', args.force,
        '--torque', args.torque,
        '--offset', args.offset,
        '--model', args.model,
        '--log', str(tsv_path),
    ]
    if args.sustained:
        cmd.append('--sustained')
    else:
        cmd.extend(['--duration', str(args.duration)])

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
        "disturbance": {
            "type": "sustained" if args.sustained else "pulse",
            "force_N": parse_vec3_safe(args.force),
            "torque_Nm": parse_vec3_safe(args.torque),
            "offset_m": parse_vec3_safe(args.offset),
            "duration_s": None if args.sustained else args.duration,
        },
        "settle_s": args.settle,
        "notes": args.notes,
    }
    with metadata_path.open('w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    print(f"[record] saved:")
    print(f"  {tsv_path.relative_to(REPO_ROOT)}")
    print(f"  {ulg_dst.relative_to(REPO_ROOT)}   ({ulg_size_kb:.1f} KB)")
    print(f"  {metadata_path.relative_to(REPO_ROOT)}")
    print(f"[record] done. 분석은 별도 코드에서 위 디렉토리를 입력으로.")


if __name__ == '__main__':
    main()
