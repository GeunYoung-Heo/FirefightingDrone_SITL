#!/usr/bin/env python3
"""Apply step force disturbance to S550 in Gazebo Classic via disturbance_plugin.

플러그인이 매 physics step에서 link에 force/torque를 적용하고, force가 0이
아닐 때 빨간 화살표 모델을 spawn해서 드론에 부착시킨다. 본 스크립트는
시작 시 Wrench 메시지 1회 publish + duration 동안 sleep + 종료 시 zero
Wrench 1회 publish 만 한다.

전제:
  - disturbance_plugin이 빌드되어 있고 (`tools/disturbance_plugin/build/`)
  - s550.sdf.jinja에 `<plugin ... filename="libdisturbance_plugin.so">` 블록 포함
  - SITL이 호버 중

사용 예:
  # 1초 펄스, X축 양방향 30N (드론에 화살표 따라옴)
  ./apply_disturbance.py --force "30 0 0" --duration 1.0

  # 외란 인가점 offset (자연스러운 토크 동반 + 화살표가 offset 지점에서 출발)
  ./apply_disturbance.py --force "20 0 0" --offset "0 0.2 0" --duration 0.5

  # Sustained — Ctrl+C 로 중단 시 자동 정리
  ./apply_disturbance.py --force "10 0 0" --sustained

  # 명시적 외력 해제
  ./apply_disturbance.py --stop

  # 외란 timestamp 기록 (추후 ULog와 동기 분석용)
  ./apply_disturbance.py --force "30 0 0" --duration 1.0 \\
      --log ~/Firefighting_Drone/SITL/tools/disturbance_log.tsv

화살표 표시·길이 등은 s550.sdf.jinja의 `<plugin>` 블록에서 조정한다:
  <enable_arrow>true</enable_arrow>
  <arrow_scale_factor>0.05</arrow_scale_factor>  <!-- m per N -->
  <arrow_max_length>2.0</arrow_max_length>
  <arrow_radius>0.03</arrow_radius>
"""
import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime


ZEROS = [0.0, 0.0, 0.0]


def parse_vec3(s, name):
    parts = s.split()
    if len(parts) != 3:
        sys.exit(f"--{name}: 3개의 숫자가 공백으로 구분되어야 함 (받은 값: {s!r})")
    try:
        return [float(p) for p in parts]
    except ValueError as e:
        sys.exit(f"--{name}: 숫자 변환 실패 ({e})")


def make_wrench_text(force, torque, offset):
    return (
        f"force {{ x: {force[0]} y: {force[1]} z: {force[2]} }}\n"
        f"torque {{ x: {torque[0]} y: {torque[1]} z: {torque[2]} }}\n"
        f"force_offset {{ x: {offset[0]} y: {offset[1]} z: {offset[2]} }}\n"
    )


def publish_wrench(topic, force, torque, offset):
    msg_text = make_wrench_text(force, torque, offset)
    fd, msg_file = tempfile.mkstemp(suffix='.wrench.msg', prefix='disturbance_')
    with os.fdopen(fd, 'w') as f:
        f.write(msg_text)
    try:
        subprocess.run(
            ['gz', 'topic', '-p', topic, '-f', msg_file],
            check=False,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    finally:
        try:
            os.unlink(msg_file)
        except OSError:
            pass


def main():
    p = argparse.ArgumentParser(
        description="Apply step force disturbance via disturbance_plugin",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('--force', default='0 0 0',
                   help='Force vector "Fx Fy Fz" in N')
    p.add_argument('--torque', default='0 0 0',
                   help='Torque vector "Tx Ty Tz" in N·m')
    p.add_argument('--offset', default='0 0 0',
                   help='Force offset "x y z" in m (base_link frame; default: CoG)')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--duration', type=float, default=1.0,
                   help='Pulse duration in seconds (default: 1.0)')
    g.add_argument('--sustained', action='store_true',
                   help='Keep force applied until Ctrl+C')
    g.add_argument('--stop', action='store_true',
                   help='Publish zero wrench to clear active disturbance')
    p.add_argument('--model', default='s550', help='Gazebo model name (default: s550)')
    p.add_argument('--log', default=None,
                   help='Append TSV record to file (start_unix, end_unix, force, torque, offset)')
    args = p.parse_args()

    topic = f"/gazebo/default/{args.model}/disturbance_cmd"

    if args.stop:
        t = datetime.now().isoformat(timespec='milliseconds')
        print(f"[{t}] DISTURBANCE STOP (publish zero wrench to {topic})")
        publish_wrench(topic, ZEROS, ZEROS, ZEROS)
        return

    force = parse_vec3(args.force, 'force')
    torque = parse_vec3(args.torque, 'torque')
    offset = parse_vec3(args.offset, 'offset')

    if all(v == 0 for v in force + torque):
        sys.exit("force와 torque가 둘 다 0입니다. 외력 해제용이라면 --stop 사용.")

    interrupted = {'val': False}
    def _handler(_sig, _frame):
        interrupted['val'] = True
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)

    t0_unix = time.time()
    t0_iso = datetime.now().isoformat(timespec='milliseconds')
    print(f"[{t0_iso}] DISTURBANCE START")
    print(f"  topic   : {topic}")
    print(f"  force   : {force} N")
    print(f"  torque  : {torque} N·m")
    print(f"  offset  : {offset} m (base_link frame)")
    print(f"  mode    : {'sustained (Ctrl+C to stop)' if args.sustained else f'pulse {args.duration:.2f} s'}",
          flush=True)

    # Start: 1 publish (플러그인이 force 적용 + 화살표 spawn)
    publish_wrench(topic, force, torque, offset)

    if args.sustained:
        while not interrupted['val']:
            time.sleep(0.1)
    else:
        end_mono = time.monotonic() + args.duration
        while not interrupted['val'] and time.monotonic() < end_mono:
            time.sleep(min(0.05, max(0.001, end_mono - time.monotonic())))

    # Stop: zero wrench (플러그인이 force 중단 + 화살표 despawn)
    publish_wrench(topic, ZEROS, ZEROS, ZEROS)

    t1_unix = time.time()
    t1_iso = datetime.now().isoformat(timespec='milliseconds')
    elapsed = t1_unix - t0_unix
    print(f"[{t1_iso}] DISTURBANCE END (actual duration: {elapsed:.2f}s)")

    if args.log:
        with open(args.log, 'a') as f:
            f.write(
                f"{t0_unix:.6f}\t{t1_unix:.6f}\t"
                f"{','.join(f'{v:.4f}' for v in force)}\t"
                f"{','.join(f'{v:.4f}' for v in torque)}\t"
                f"{','.join(f'{v:.4f}' for v in offset)}\n"
            )


if __name__ == '__main__':
    main()
