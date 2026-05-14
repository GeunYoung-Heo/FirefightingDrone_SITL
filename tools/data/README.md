# Experiment Data

`tools/record_experiment.py` 가 만든 실험 단위 데이터 스냅샷을 모으는 디렉토리.

본 디렉토리 자체는 `.gitignore` 처리되어 있어, 데이터 파일은 git에 추적되지 않는다. 각자 로컬에서 관리하거나 외부 공유 수단(파일 서버, S3, Git LFS 등)을 사용한다.

## 구조

```
data/
├── README.md
├── .gitignore
└── <YYYYMMDD>T<HHMMSS>_<experiment_name>/
    ├── flight.ulg         # PX4 ULog 스냅샷 (실험 종료 시점까지의 SITL 세션)
    ├── disturbance.tsv    # apply_disturbance.py 출력. 외란 이벤트 1줄/회
    └── metadata.json      # 실험 컨텍스트 (이름, 외란 파라미터, controller 버전, 메모)
```

## 파일 형식

### `flight.ulg`
PX4 ULog v1 binary. `pyulog` 또는 `PlotJuggler` 로 읽음. SITL 세션 시작부터 record_experiment.py가 복사한 시점까지의 모든 uORB 토픽 데이터 포함. 핵심 토픽:
- `vehicle_local_position` — EKF 위치/속도 추정 (NED)
- `vehicle_local_position_groundtruth` — Gazebo 실측 위치/속도
- `vehicle_attitude` / `vehicle_attitude_groundtruth` — 자세 (quaternion)
- `vehicle_attitude_setpoint`, `vehicle_local_position_setpoint` — 제어기 setpoint
- `vehicle_angular_velocity`, `sensor_combined` — 각속도/IMU
- `actuator_motors`, `actuator_outputs` — 6개 모터 출력
- `ekf2_innovations` — EKF residual

### `disturbance.tsv`
탭 구분 텍스트. 한 줄 = 외란 인가 한 회. 컬럼:

| 컬럼 | 의미 |
|---|---|
| `start_unix` | 외란 시작 unix epoch (float, 초) |
| `end_unix` | 외란 종료 unix epoch (float, 초) |
| `force` | "Fx,Fy,Fz" (N, body frame) |
| `torque` | "Tx,Ty,Tz" (N·m, body frame) |
| `offset` | "x,y,z" (m, body frame, force 인가점) |

### `metadata.json`
```json
{
  "experiment_name": "step_30N_x",
  "timestamp": "20260514T153021",
  "wall_time_start_unix": 1747...,
  "wall_time_end_unix":   1747...,
  "px4_log_source": "PX4-Autopilot/build/.../rootfs/log/2026-05-14/15_30_15.ulg",
  "model": "s550",
  "controller": "default_PX4_v1.14.4",
  "disturbance": {
    "type": "pulse",
    "force_N": [30.0, 0.0, 0.0],
    "torque_Nm": [0.0, 0.0, 0.0],
    "offset_m": [0.0, 0.0, 0.0],
    "duration_s": 1.0
  },
  "notes": "Baseline. 약 5m 호버 후 X 방향 30N 1초 펄스."
}
```

## 분석 사이드 — 시간축 정렬

`disturbance.tsv` 는 **wall-clock unix time**, `flight.ulg` 는 **PX4 boot microsec**. 정렬 절차:

1. ULog 헤더의 첫 `vehicle_gps_position.time_utc_usec` 또는 `log_message`의 GPS-fix UTC 추출 → 그 시점의 ULog 마이크로초도 같이 추출 → `ulog_us0 ↔ utc0_unix` 매핑 확립
2. 외란 이벤트의 `start_unix` 를 ULog 마이크로초로 변환:
   `event_us = ulog_us0 + (start_unix - utc0_unix) * 1e6`
3. ULog 데이터 시계열에 외란 시점 마커 오버레이

> Note: SITL의 시뮬 GPS 시간은 실제 unix time과 다를 수 있다 (보통 unix 시각으로 설정되긴 함). 정확한 매핑이 안 되면 metadata.json 의 `wall_time_start_unix` 와 ULog 의 `vehicle_gps_position.time_utc_usec` 차이를 보정 오프셋으로 사용.
