#!/usr/bin/env python3
"""Visualize a disturbance experiment recorded by record_experiment.py.

입력: 실험 디렉토리 (tools/data/<exp>/) — flight.ulg + disturbance.tsv + metadata.json

3x3 plot:
  Row 1 [궤적]  top(E-N)    front(E-Alt)   side(N-Alt)   — 무지개 색상 (시간 흐름)
  Row 2 [위치]  x vs t      y vs t          alt vs t      — 외란 시점 점선
  Row 3 [자세]  roll vs t   pitch vs t      yaw vs t      — 외란 시점 점선

- 시간창: [외란 시작 -3s, 외란 종료 +3s]
- 시간축: 외란 시작 = t=0 (상대 시간)
- 좌표: PX4 NED 기준. x=North, y=East, alt=-z(위가 양수). 자세는 도(°).
- source 'both'(기본): Row1 궤적은 ground truth, Row2/3는 실측(solid)+추정(dashed) 오버레이

사용 예:
  ./plot_experiment.py tools/data/20260514T134734_test_30N_x
  ./plot_experiment.py tools/data/<exp> --source groundtruth --no-show
  ./plot_experiment.py tools/data/<exp> --cmap rainbow --event 0

전제: pyulog 설치됨 (pip3 install --user pyulog)
"""
import argparse
import json
import math
import pathlib
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("pyulog 미설치. 다음으로 설치: pip3 install --user pyulog")

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.ticker import MultipleLocator


PRE_S = 3.0    # 외란 시작 전 표시 구간 (s)
POST_S = 3.0   # 외란 종료 후 표시 구간 (s)


# ---------- ULog helpers ----------

def get_topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d
    return None


def build_time_mapping(ulog):
    """unix epoch seconds -> ULog timestamp(microsec) 선형 매핑.

    vehicle_gps_position 의 (timestamp, time_utc_usec) 한 쌍으로 offset 산출.
    실패 시 None.
    """
    gps = get_topic(ulog, 'vehicle_gps_position')
    if gps is None:
        return None
    ts = gps.data.get('timestamp')
    utc = gps.data.get('time_utc_usec')
    if ts is None or utc is None:
        return None
    valid = np.flatnonzero(np.asarray(utc) > 0)
    if len(valid) == 0:
        return None
    i = int(valid[0])
    ulog_us0 = float(ts[i])
    unix0 = float(utc[i]) / 1e6
    return lambda unix_t: ulog_us0 + (unix_t - unix0) * 1e6


def quat_to_euler_deg(q):
    """PX4 Hamilton quaternion array [N,4]=(w,x,y,z) -> roll,pitch,yaw (deg).

    aerospace 3-2-1 (yaw-pitch-roll). yaw/roll 는 unwrap 으로 연속화.
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roll = np.unwrap(roll)
    yaw = np.unwrap(yaw)
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def extract_pos(ulog, groundtruth):
    name = 'vehicle_local_position_groundtruth' if groundtruth else 'vehicle_local_position'
    d = get_topic(ulog, name)
    if d is None:
        return None
    t = np.asarray(d.data['timestamp'], dtype=float)
    x = np.asarray(d.data['x'], dtype=float)   # North
    y = np.asarray(d.data['y'], dtype=float)   # East
    z = np.asarray(d.data['z'], dtype=float)   # Down (NED)
    return {'t': t, 'x': x, 'y': y, 'z': z, 'alt': -z}


def extract_att(ulog, groundtruth):
    name = 'vehicle_attitude_groundtruth' if groundtruth else 'vehicle_attitude'
    d = get_topic(ulog, name)
    if d is None:
        return None
    t = np.asarray(d.data['timestamp'], dtype=float)
    q = np.column_stack([
        np.asarray(d.data['q[0]'], dtype=float),
        np.asarray(d.data['q[1]'], dtype=float),
        np.asarray(d.data['q[2]'], dtype=float),
        np.asarray(d.data['q[3]'], dtype=float),
    ])
    roll, pitch, yaw = quat_to_euler_deg(q)
    return {'t': t, 'roll': roll, 'pitch': pitch, 'yaw': yaw}


def clip_window(d, win_start, win_end):
    """dict의 'timestamp'(='t') 기준으로 [win_start, win_end] 구간만 남김."""
    if d is None:
        return None
    m = (d['t'] >= win_start) & (d['t'] <= win_end)
    return {k: (v[m] if isinstance(v, np.ndarray) else v) for k, v in d.items()}


# ---------- plotting ----------

def nice_tick_interval(span, target_ticks=7):
    """span 을 target_ticks 개로 나눈 '보기 좋은' 눈금 간격.

    1 / 2 / 2.5 / 5 / 10 × 10^n 중 하나를 반환.
    """
    if not math.isfinite(span) or span <= 0:
        return 1.0
    raw = span / target_ticks
    mag = 10.0 ** math.floor(math.log10(raw))
    norm = raw / mag
    if norm <= 1.0:
        nice = 1.0
    elif norm <= 2.0:
        nice = 2.0
    elif norm <= 2.5:
        nice = 2.5
    elif norm <= 5.0:
        nice = 5.0
    else:
        nice = 10.0
    return nice * mag


def unify_trajectory_axes(axes_list, margin=0.10, target_ticks=7):
    """3개 trajectory plot의 6개 축(3 plot × x,y)을 모두 같은 scale로 통일.

    - 각 축의 데이터 중심은 그대로 두고, 범위 길이(span)를 모두 '최대 span'으로
      맞춤 → 같은 거리가 6개 plot에서 같은 크기로 보임 → 어느 축 방향 위치변화가
      큰지 시각적으로 직접 비교 가능.
    - 동일 span 이므로 동일한 nice tick 간격도 함께 적용.
    - plot_trajectory 가 set_aspect('equal', adjustable='box') 를 쓰므로,
      span 통일 + equal aspect → 6개 축의 화면상 scale(단위/픽셀)이 일치.

    반환: (common_span, tick_interval) 또는 데이터 없으면 None
    """
    infos = []   # (ax, 'x'|'y', center)
    spans = []
    for ax in axes_list:
        dl = ax.dataLim
        if math.isfinite(dl.width) and dl.width > 0:
            infos.append((ax, 'x', (dl.x0 + dl.x1) / 2.0))
            spans.append(dl.width)
        if math.isfinite(dl.height) and dl.height > 0:
            infos.append((ax, 'y', (dl.y0 + dl.y1) / 2.0))
            spans.append(dl.height)
    if not spans:
        return None

    common = max(spans) * (1.0 + margin)
    half = common / 2.0
    tick_iv = nice_tick_interval(common, target_ticks=target_ticks)

    for ax, axis, center in infos:
        if axis == 'x':
            ax.set_xlim(center - half, center + half)
            ax.xaxis.set_major_locator(MultipleLocator(tick_iv))
        else:
            ax.set_ylim(center - half, center + half)
            ax.yaxis.set_major_locator(MultipleLocator(tick_iv))
    return common, tick_iv


def plot_trajectory(ax, xs, ys, t_rel, cmap, xlabel, ylabel, title):
    """xs vs ys 궤적을 t_rel 로 색칠. LineCollection 반환 (colorbar용)."""
    if xs is None or len(xs) < 2:
        ax.text(0.5, 0.5, 'no data in window', transform=ax.transAxes,
                ha='center', va='center', color='gray')
        ax.set_title(title)
        return None
    points = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segs = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(t_rel.min(), t_rel.max())
    lc = LineCollection(segs, cmap=cmap, norm=norm)
    lc.set_array(t_rel[:-1])
    lc.set_linewidth(2.0)
    ax.add_collection(lc)
    ax.scatter([xs[0]], [ys[0]], c='lime', s=45, zorder=5,
               edgecolors='k', label='start')
    ax.scatter([xs[-1]], [ys[-1]], c='red', s=45, zorder=5,
               edgecolors='k', label='end')
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    # 범위(xlim/ylim)와 눈금은 unify_trajectory_axes 에서 일괄 설정.
    # adjustable='box': span을 직접 지정하므로 box를 줄여 equal aspect를 맞춤
    #   → 6개 축 span이 같고 + equal aspect 이면 화면 scale이 모두 동일해진다.
    ax.set_aspect('equal', adjustable='box')
    ax.legend(fontsize=8, loc='best')
    return lc


def plot_timeseries(ax, gt, est, key, t0_ulog, dist_start_rel, dist_end_rel,
                    ylabel, title):
    """시계열 plot. gt(solid) / est(dashed). 외란 구간 점선·음영."""
    plotted = False
    if gt is not None and len(gt['t']) > 0:
        t_rel = (gt['t'] - t0_ulog) / 1e6
        ax.plot(t_rel, gt[key], '-', color='C0', linewidth=1.6,
                label='ground truth')
        plotted = True
    if est is not None and len(est['t']) > 0:
        t_rel = (est['t'] - t0_ulog) / 1e6
        ax.plot(t_rel, est[key], '--', color='C3', linewidth=1.1,
                alpha=0.85, label='estimate')
        plotted = True

    # 외란 인가/해제 시점 표시
    ax.axvline(dist_start_rel, color='k', linestyle=':', linewidth=1.3)
    ax.axvline(dist_end_rel, color='k', linestyle=':', linewidth=1.3)
    ax.axvspan(dist_start_rel, dist_end_rel, color='gray', alpha=0.12)

    ax.set_xlabel('time since disturbance start [s]')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    if plotted:
        ax.legend(fontsize=8, loc='best')
    else:
        ax.text(0.5, 0.5, 'no data in window', transform=ax.transAxes,
                ha='center', va='center', color='gray')


# ---------- disturbance.tsv ----------

def parse_disturbance_tsv(path, event_index):
    lines = [l for l in path.read_text().splitlines() if l.strip()]
    if not lines:
        sys.exit(f"{path} 가 비어있음 — 외란 이벤트 기록 없음")
    if event_index < 0 or event_index >= len(lines):
        sys.exit(f"--event {event_index} 범위 초과 (disturbance.tsv 총 {len(lines)} 행)")
    if len(lines) > 1:
        print(f"[plot] disturbance.tsv 에 {len(lines)} 개 이벤트 — "
              f"{event_index} 번째 사용 (--event 로 변경)")
    parts = lines[event_index].split('\t')
    return {
        'start_unix': float(parts[0]),
        'end_unix': float(parts[1]),
        'force': parts[2] if len(parts) > 2 else '?',
        'torque': parts[3] if len(parts) > 3 else '?',
        'offset': parts[4] if len(parts) > 4 else '?',
    }


# ---------- main ----------

def main():
    p = argparse.ArgumentParser(
        description="Visualize a disturbance experiment (3x3 plot).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('exp_dir', help='실험 디렉토리 (flight.ulg / disturbance.tsv / metadata.json 포함)')
    p.add_argument('--source', choices=['both', 'groundtruth', 'estimate'],
                   default='both', help='plot 대상 데이터 (default: both)')
    p.add_argument('--event', type=int, default=0,
                   help='disturbance.tsv 의 이벤트 행 인덱스 (default: 0)')
    p.add_argument('--cmap', default='turbo',
                   help='궤적 colormap (default: turbo). rainbow/jet/viridis 등 가능')
    p.add_argument('--save', action='store_true',
                   help='PNG 저장 (지정 안 해도 기본 저장됨; --no-save 로 끄기)')
    p.add_argument('--no-save', action='store_true', help='PNG 저장 안 함')
    p.add_argument('--no-show', action='store_true', help='대화창 표시 안 함')
    args = p.parse_args()

    exp_dir = pathlib.Path(args.exp_dir).resolve()
    ulg_path = exp_dir / 'flight.ulg'
    tsv_path = exp_dir / 'disturbance.tsv'
    meta_path = exp_dir / 'metadata.json'

    for path in (ulg_path, tsv_path):
        if not path.exists():
            sys.exit(f"필수 파일 없음: {path}")

    # headless 면 Agg 백엔드
    do_show = not args.no_show
    if not do_show:
        matplotlib.use('Agg')

    # --- load ---
    ulog = ULog(str(ulg_path))
    dist = parse_disturbance_tsv(tsv_path, args.event)
    meta = {}
    if meta_path.exists():
        try:
            meta = json.loads(meta_path.read_text())
        except Exception as e:
            print(f"[plot] metadata.json 파싱 실패 (무시): {e}")

    # --- time mapping ---
    to_ulog = build_time_mapping(ulog)
    if to_ulog is None:
        sys.exit("ULog 에서 GPS UTC 시간을 못 찾음 — 외란 시점을 ULog 시간축에 정렬 불가.\n"
                 "vehicle_gps_position 토픽이 로깅되는지 확인 필요.")

    t_dist_start = to_ulog(dist['start_unix'])
    t_dist_end = to_ulog(dist['end_unix'])
    win_start = t_dist_start - PRE_S * 1e6
    win_end = t_dist_end + POST_S * 1e6

    # ULog 데이터 범위와 sanity check
    u0, u1 = ulog.start_timestamp, ulog.last_timestamp
    if t_dist_start < u0 or t_dist_end > u1:
        print(f"[plot] WARN: 외란 시점이 ULog 범위 밖일 수 있음 "
              f"(외란 {t_dist_start/1e6:.1f}~{t_dist_end/1e6:.1f}s vs "
              f"ULog {u0/1e6:.1f}~{u1/1e6:.1f}s). GPS 시간 매핑 확인 권장.")

    dist_start_rel = 0.0
    dist_end_rel = (t_dist_end - t_dist_start) / 1e6

    # --- extract & clip ---
    pos_gt = extract_pos(ulog, groundtruth=True)
    pos_est = extract_pos(ulog, groundtruth=False)
    att_gt = extract_att(ulog, groundtruth=True)
    att_est = extract_att(ulog, groundtruth=False)

    if args.source == 'groundtruth':
        pos_est = att_est = None
    elif args.source == 'estimate':
        pos_gt = att_gt = None

    pos_gt = clip_window(pos_gt, win_start, win_end)
    pos_est = clip_window(pos_est, win_start, win_end)
    att_gt = clip_window(att_gt, win_start, win_end)
    att_est = clip_window(att_est, win_start, win_end)

    if pos_gt is None and pos_est is None:
        sys.exit("위치 데이터 없음 (vehicle_local_position[_groundtruth] 토픽 부재)")

    # Row1 궤적용 source: ground truth 우선, 없으면 estimate
    traj = pos_gt if pos_gt is not None else pos_est
    traj_label = 'ground truth' if pos_gt is not None else 'estimate'
    if traj is None or len(traj['t']) < 2:
        print("[plot] WARN: 시간창 내 궤적 데이터가 거의 없음")

    # --- figure ---
    # Row 1(궤적)은 set_aspect('equal','box')로 square가 된다. cell이 가로로 길면
    # square plot이 cell 폭을 못 채워 plot 사이 간격이 벌어진다.
    # → height_ratios로 Row 1 cell을 더 높게 잡아 square가 폭을 꽉 채우게 함.
    # constrained_layout: colorbar + box-aspect square axes 조합을 tight_layout
    #   보다 잘 배치 (suptitle 간격도 자동).
    fig, axes = plt.subplots(3, 3, figsize=(16, 14.5),
                             gridspec_kw={'height_ratios': [1.35, 1.0, 1.0]},
                             constrained_layout=True)

    # suptitle
    force = meta.get('disturbance', {}).get('force_N', dist['force'])
    dur = meta.get('disturbance', {}).get('duration_s')
    exp_name = meta.get('experiment_name', exp_dir.name)
    ctrl = meta.get('controller', '?')
    suptitle = (f"{exp_name}  |  force={force} N  "
                f"|  duration={dur if dur is not None else 'sustained'} s  "
                f"|  controller={ctrl}")
    fig.suptitle(suptitle, fontsize=13, fontweight='bold')

    # ----- Row 1: 궤적 (rainbow) -----
    if traj is not None and len(traj['t']) >= 2:
        t_rel_traj = (traj['t'] - t_dist_start) / 1e6
        N, E, Alt = traj['x'], traj['y'], traj['alt']
        lc1 = plot_trajectory(axes[0, 0], E, N, t_rel_traj, args.cmap,
                              'East [m]', 'North [m]', f'Top view ({traj_label})')
        plot_trajectory(axes[0, 1], E, Alt, t_rel_traj, args.cmap,
                        'East [m]', 'Altitude (-z) [m]', f'Front view ({traj_label})')
        plot_trajectory(axes[0, 2], N, Alt, t_rel_traj, args.cmap,
                        'North [m]', 'Altitude (-z) [m]', f'Side view ({traj_label})')
        if lc1 is not None:
            # colorbar를 Row 1 세 축 전체에 붙임 (axes[0,2]에만 붙이면
            # 그 plot만 colorbar 자리만큼 작아진다). 셋이 동일하게 줄어듦.
            cb = fig.colorbar(lc1, ax=list(axes[0]), fraction=0.02, pad=0.02)
            cb.set_label('time since disturbance start [s]')
        # Row 1 의 6개 축을 동일 scale 로 통일.
        # 범위 길이(span)를 최대값으로 맞춰 — 어느 축 방향 변위가 큰지 직접 비교 가능.
        result = unify_trajectory_axes([axes[0, 0], axes[0, 1], axes[0, 2]],
                                       target_ticks=7)
        if result is not None:
            common_span, tick_iv = result
            print(f"[plot] Row 1 축 scale 통일: span {common_span:.2f} m, "
                  f"tick {tick_iv} m (6축 모두 동일)")
    else:
        for j in range(3):
            axes[0, j].text(0.5, 0.5, 'no trajectory data', ha='center',
                            va='center', transform=axes[0, j].transAxes, color='gray')

    # ----- Row 2: x, y, alt vs time -----
    plot_timeseries(axes[1, 0], pos_gt, pos_est, 'x', t_dist_start,
                    dist_start_rel, dist_end_rel, 'x — North [m]', 'X position')
    plot_timeseries(axes[1, 1], pos_gt, pos_est, 'y', t_dist_start,
                    dist_start_rel, dist_end_rel, 'y — East [m]', 'Y position')
    plot_timeseries(axes[1, 2], pos_gt, pos_est, 'alt', t_dist_start,
                    dist_start_rel, dist_end_rel, 'altitude (-z) [m]', 'Z position (altitude)')

    # ----- Row 3: roll, pitch, yaw vs time -----
    plot_timeseries(axes[2, 0], att_gt, att_est, 'roll', t_dist_start,
                    dist_start_rel, dist_end_rel, 'roll [deg]', 'Roll')
    plot_timeseries(axes[2, 1], att_gt, att_est, 'pitch', t_dist_start,
                    dist_start_rel, dist_end_rel, 'pitch [deg]', 'Pitch')
    plot_timeseries(axes[2, 2], att_gt, att_est, 'yaw', t_dist_start,
                    dist_start_rel, dist_end_rel, 'yaw [deg]', 'Yaw')

    # layout은 constrained_layout (plt.subplots에서 활성화) 가 자동 처리.

    # --- output ---
    if not args.no_save:
        png_path = exp_dir / 'analysis.png'
        fig.savefig(png_path, dpi=130)
        print(f"[plot] saved: {png_path}")

    if do_show:
        try:
            plt.show()
        except Exception as e:
            print(f"[plot] 대화창 표시 실패 (무시): {e}")


if __name__ == '__main__':
    main()
