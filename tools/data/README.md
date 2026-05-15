# Experiment Data

`tools/record_experiment.py` 가 만든 실험 단위 데이터 스냅샷을 모으는 디렉토리.

본 디렉토리 자체는 `.gitignore` 처리되어 있어, 데이터 파일은 git에 추적되지 않는다. 각자 로컬에서 관리하거나 외부 공유 수단(파일 서버, S3, Git LFS 등)을 사용한다.

## 구조

```
data/
├── README.md
├── .gitignore
└── <YYYYMMDD>T<HHMMSS>_<experiment_name>/
    ├── flight.ulg                # PX4 ULog 스냅샷 (실험 종료 시점까지의 SITL 세션)
    ├── disturbance.tsv           # apply_disturbance.py 출력. 외란 1개당 1줄
    ├── disturbance_profiles.json # 사용한 외란 프로파일 JSON 사본 (재현용)
    └── metadata.json             # 실험 컨텍스트 (이름, 외란 목록, controller 버전, 메모)
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
탭 구분 텍스트. `#` 로 시작하는 첫 줄은 헤더. **한 줄 = 외란 1개** (JSON 프로파일의 `disturbances` 배열 항목 1개에 대응). 컬럼:

| 컬럼 | 의미 |
|---|---|
| `name` | 외란 이름 (JSON `name`) |
| `frame` | `body` 또는 `global` |
| `start_unix` | 외란 시작 unix epoch (float, 초) = 트리거 송신 시각 + `start_time` |
| `end_unix` | 외란 종료 unix epoch (float, 초) = `start_unix` + `duration` |
| `tau` | OU 상관시간 (s) |
| `force_mean` | "x,y,z" (N) — OU 평균 |
| `force_stddev` | "x,y,z" (N) — OU 정상상태 표준편차 |
| `torque_mean` | "x,y,z" (N·m) |
| `torque_stddev` | "x,y,z" (N·m) |
| `offset` | "x,y,z" (m, body frame, force 인가점) |

> `start_unix` 은 트리거 메시지 송신 시각 기준이다. 플러그인의 타임라인 0점(메시지 수신 시각)과는 sub-ms 수준 latency 차이가 있으나 SITL lockstep 에서는 무시 가능.

### `disturbance_profiles.json`
실험에 사용한 외란 프로파일 JSON 의 사본. 스키마는 `tools/disturbance_profiles.json` 의 `_schema` / `_frame_convention` 주석 참조.

### `metadata.json`
```json
{
  "experiment_name": "baseline_gust",
  "timestamp": "20260514T153021",
  "wall_time_start_unix": 1747...,
  "wall_time_end_unix":   1747...,
  "px4_log_source": "PX4-Autopilot/build/.../rootfs/log/2026-05-14/15_30_15.ulg",
  "model": "s550",
  "controller": "default_PX4_v1.14.4",
  "profile_file": "disturbance_profiles.json",
  "disturbances": [ ... 사용한 프로파일의 disturbances 배열 그대로 ... ],
  "settle_s": 5.0,
  "notes": "측풍 + 분사 반발력 동시 인가."
}
```

## 분석 사이드 — 시간축 정렬

`disturbance.tsv` 는 **wall-clock unix time**, `flight.ulg` 는 **PX4 boot microsec**. 정렬 절차:

1. ULog 헤더의 첫 `vehicle_gps_position.time_utc_usec` 또는 `log_message`의 GPS-fix UTC 추출 → 그 시점의 ULog 마이크로초도 같이 추출 → `ulog_us0 ↔ utc0_unix` 매핑 확립
2. 외란 이벤트의 `start_unix` 를 ULog 마이크로초로 변환:
   `event_us = ulog_us0 + (start_unix - utc0_unix) * 1e6`
3. ULog 데이터 시계열에 외란 시점 마커 오버레이

> Note: SITL의 시뮬 GPS 시간은 실제 unix time과 다를 수 있다 (보통 unix 시각으로 설정되긴 함). 정확한 매핑이 안 되면 metadata.json 의 `wall_time_start_unix` 와 ULog 의 `vehicle_gps_position.time_utc_usec` 차이를 보정 오프셋으로 사용.
