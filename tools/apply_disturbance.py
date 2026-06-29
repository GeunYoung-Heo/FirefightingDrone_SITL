#!/usr/bin/env python3
"""Trigger a JSON-defined disturbance timeline on the S550 disturbance_plugin.

외란의 내용은 더 이상 명령행 인자가 아니라 **JSON 프로파일 파일**로 정의한다
(기본: `tools/disturbance_profiles.json`). 이 스크립트는 그 JSON을 읽어
GzString 메시지로 disturbance_plugin 에 1회 publish 하고, 플러그인이 내부
타임라인에 따라 OU 외란을 인가한다.

타임라인 0점 = 플러그인이 트리거 메시지를 받은 sim time. 각 외란은
[start_time, start_time + duration] 동안 활성화된다.

전제:
  - disturbance_plugin 이 빌드되어 있고 (`tools/disturbance_plugin/build/`)
  - s550.sdf.jinja 에 `<plugin ... filename="libdisturbance_plugin.so">` 포함
  - SITL 이 호버 중

사용 예:
  # 기본 프로파일 트리거 (타임라인이 끝날 때까지 블록)
  ./apply_disturbance.py

  # 다른 프로파일 + 외란 이벤트를 TSV 로 기록
  ./apply_disturbance.py --profiles my_profile.json --log disturbance.tsv

  # 트리거만 하고 즉시 종료 (대기 안 함)
  ./apply_disturbance.py --no-wait

JSON 스키마는 `tools/disturbance_profiles.json` 의 `_schema` / `_frame_convention`
주석 참조. 좌표계 규약은 README §7.6 참조.
"""
import argparse
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import time
from datetime import datetime


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_PROFILE = REPO_ROOT / "tools" / "disturbance_profiles.json"

TSV_HEADER = (
    "# name\tframe\tstart_unix\tend_unix\ttau\t"
    "force_mean\tforce_stddev\ttorque_mean\ttorque_stddev\toffset\n"
)


def load_profile(path):
    """Load + minimally validate the disturbance profile JSON."""
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

    for i, d in enumerate(dists):
        name = d.get("name", f"dist_{i}")
        frame = d.get("frame", "body")
        if frame not in ("body", "global"):
            sys.exit(f"[{name}] frame 은 'body' 또는 'global' 이어야 함 (받음: {frame!r})")
        for key in ("start_time", "duration", "correlation_time"):
            if key not in d:
                sys.exit(f"[{name}] 필수 필드 누락: {key}")
        for key in ("force", "torque"):
            blk = d.get(key, {})
            if "mean" not in blk or "stddev" not in blk:
                sys.exit(f"[{name}] {key}.mean / {key}.stddev 둘 다 필요")
    return dists


def proto_escape(s):
    """Escape a string for protobuf text-format string literal."""
    return s.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')


def publish_profile(topic, dists):
    """Publish {"disturbances": [...]} as a GzString message via `gz topic -p`."""
    payload = json.dumps({"disturbances": dists}, ensure_ascii=True)
    msg_text = f'data: "{proto_escape(payload)}"\n'

    fd, msg_file = tempfile.mkstemp(suffix='.gzstring.msg', prefix='disturbance_')
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


def publish_mavlink_ff(endpoint, dists):
    """Send first disturbance to PX4 as MAVLink DEBUG_FLOAT_ARRAY (name="DIST").

    mc_rate_control 이 debug_array uORB 를 subscribe 해서 ff 보상에 사용한다.
    Payload (8 float, FRD 좌표):
      [0..2] force_body  [N]
      [3..5] torque_body [N.m]
      [6]    start_offset_s  (트리거 0점 기준 시작 시각)
      [7]    duration_s

    JSON 의 body FLU → PX4 body FRD 변환을 여기서 처리: Y, Z 부호 반전.
    첫 외란만 송신. frame=global 이면 스킵 (ff 는 body 만 지원).
    """
    try:
        from pymavlink.dialects.v20 import common as mavlink2
    except ImportError:
        print("[apply] pymavlink 미설치 — FF broadcast 스킵 "
              "(pip3 install --user pymavlink)")
        return

    d = dists[0]
    name = d.get('name', 'dist_0')
    frame = d.get('frame', 'body')
    if frame != 'body':
        print(f"[apply] FF skip: '{name}' frame={frame} "
              f"(ff 는 body frame 외란만 지원)")
        return

    fx, fy, fz = (float(v) for v in d['force']['mean'])
    tx, ty, tz = (float(v) for v in d['torque']['mean'])
    # FLU (apply_disturbance / Gazebo) → FRD (PX4 펌웨어)
    fy, fz = -fy, -fz
    ty, tz = -ty, -tz

    payload = [fx, fy, fz, tx, ty, tz,
               float(d['start_time']), float(d['duration'])]
    payload += [0.0] * (58 - len(payload))   # debug_array.ARRAY_SIZE = 58

    # MAVLink2 DEBUG_FLOAT_ARRAY 메시지를 packing 후 raw UDP 로 송신
    mav = mavlink2.MAVLink(file=None, srcSystem=255, srcComponent=190)
    msg = mavlink2.MAVLink_debug_float_array_message(
        int(time.time() * 1e6),  # time_usec
        b'DIST',                 # name (bytes, 10 char max)
        0,                       # array_id
        payload,
    )
    buf = msg.pack(mav)

    # endpoint parse — 'udpout:host:port' 또는 'udp:host:port'
    import socket
    parts = endpoint.split(':')
    if parts and parts[0] in ('udpout', 'udp', 'udpin'):
        parts = parts[1:]
    if len(parts) != 2:
        print(f"[apply] FF skip: invalid --ff-endpoint {endpoint!r} "
              f"(expected udpout:host:port)")
        return
    host, port = parts[0], int(parts[1])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(buf, (host, port))

    print(f"  ff bcast: name={name} FRD F=[{fx:.2f},{fy:.2f},{fz:.2f}]N "
          f"τ=[{tx:.3f},{ty:.3f},{tz:.3f}]Nm "
          f"t=[{d['start_time']:.1f},{d['start_time']+d['duration']:.1f}]s "
          f"→ {endpoint}")


def vec_str(v):
    return ",".join(f"{float(x):.4f}" for x in v)


def write_tsv(path, send_unix, dists):
    """Append one row per disturbance. start/end_unix = send_unix + timeline offset."""
    path = pathlib.Path(path)
    need_header = not path.exists() or path.stat().st_size == 0
    with path.open('a', encoding='utf-8') as f:
        if need_header:
            f.write(TSV_HEADER)
        for i, d in enumerate(dists):
            name = d.get("name", f"dist_{i}")
            frame = d.get("frame", "body")
            start_unix = send_unix + float(d["start_time"])
            end_unix = start_unix + float(d["duration"])
            f.write(
                f"{name}\t{frame}\t{start_unix:.6f}\t{end_unix:.6f}\t"
                f"{float(d['correlation_time']):.4f}\t"
                f"{vec_str(d['force']['mean'])}\t{vec_str(d['force']['stddev'])}\t"
                f"{vec_str(d['torque']['mean'])}\t{vec_str(d['torque']['stddev'])}\t"
                f"{vec_str(d.get('offset', [0, 0, 0]))}\n"
            )


def main():
    p = argparse.ArgumentParser(
        description="Trigger a JSON-defined disturbance timeline via disturbance_plugin",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('--profiles', default=str(DEFAULT_PROFILE),
                   help=f'외란 프로파일 JSON 경로 (default: {DEFAULT_PROFILE})')
    p.add_argument('--model', default='s550', help='Gazebo 모델명 (default: s550)')
    p.add_argument('--world', default='default', help='Gazebo 월드명 (default: default)')
    p.add_argument('--log', default=None,
                   help='외란 이벤트를 TSV 로 append (한 외란당 1행)')
    p.add_argument('--no-wait', action='store_true',
                   help='트리거만 하고 즉시 종료 (타임라인 종료 대기 안 함)')
    p.add_argument('--no-ff-broadcast', action='store_true',
                   help='PX4 에 MAVLink FF 메시지 안 보냄 (기본은 송신)')
    p.add_argument('--ff-endpoint', default='udpout:localhost:14580',
                   help='PX4 SITL MAVLink endpoint (default: udpout:localhost:14580)')
    args = p.parse_args()

    dists = load_profile(args.profiles)
    topic = f"/gazebo/{args.world}/{args.model}/disturbance_json"
    timeline_end = max(float(d["start_time"]) + float(d["duration"]) for d in dists)

    t0_iso = datetime.now().isoformat(timespec='milliseconds')
    print(f"[{t0_iso}] DISTURBANCE TRIGGER")
    print(f"  profile : {args.profiles}")
    print(f"  topic   : {topic}")
    print(f"  count   : {len(dists)} disturbance(s), timeline 0~{timeline_end:.1f}s")
    for i, d in enumerate(dists):
        name = d.get("name", f"dist_{i}")
        s = float(d["start_time"])
        e = s + float(d["duration"])
        print(f"    - {name:<18} {d.get('frame', 'body'):<6} "
              f"t=[{s:.1f},{e:.1f}]s  tau={float(d['correlation_time']):.2f}s  "
              f"F_mean={d['force']['mean']}")
    sys.stdout.flush()

    send_unix = time.time()
    publish_profile(topic, dists)
    if not args.no_ff_broadcast:
        publish_mavlink_ff(args.ff_endpoint, dists)

    if args.log:
        write_tsv(args.log, send_unix, dists)
        print(f"  logged  : {args.log}")

    if args.no_wait:
        print(f"[{datetime.now().isoformat(timespec='milliseconds')}] "
              f"triggered (--no-wait, 타임라인은 백그라운드에서 진행)")
        return

    print(f"  waiting : {timeline_end:.1f}s for timeline to finish ...", flush=True)
    try:
        time.sleep(timeline_end)
    except KeyboardInterrupt:
        print("\n  interrupted — 플러그인 타임라인은 sim time 기준이라 계속 진행될 수 있음")
        return
    t1_iso = datetime.now().isoformat(timespec='milliseconds')
    print(f"[{t1_iso}] DISTURBANCE TIMELINE DONE")


if __name__ == '__main__':
    main()
