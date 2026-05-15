# disturbance_plugin

Gazebo Classic 11 ModelPlugin. JSON 외란 프로파일(GzString)을 받아 외란 타임라인을 재생한다 — 매 physics step OU(Ornstein-Uhlenbeck) 확률 force/torque를 link에 적용하고, 외란별 화살표를 spawn/track/despawn 한다.

전체 설명은 저장소 루트 `README.md` §7 참조. 이 파일은 빌드/검증만 다룬다.

## 왜 필요한가

Gazebo Classic의 표준 `<link>/wrench` 토픽은 publish 1번당 한 simulation step(약 1ms)만 force가 적용된다. `gz topic -p`를 루프로 돌리는 방식은 호출당 약 400ms 오버헤드 때문에 효과적 duty cycle이 ~0.6%에 그쳐 무의미하다. 또한 OU 노이즈는 매 step 적분이 필요하다. 따라서 외란 계산·적용을 플러그인 내부에서 처리하고, 외부에서는 JSON 프로파일을 1회 트리거하기만 한다.

## 빌드

```bash
cd ~/Firefighting_Drone/SITL/tools/disturbance_plugin
rm -rf build && mkdir build && cd build
cmake ..
make -j$(nproc)
```

산출물: `build/libdisturbance_plugin.so`. `run_sitl.sh`가 이 디렉토리를 자동으로 `GAZEBO_PLUGIN_PATH`에 추가한다.

의존성: `libgazebo-dev` + `libjsoncpp-dev`. jsoncpp 미설치 시 `cmake ..`가 `pkg_check_modules(JSONCPP REQUIRED jsoncpp)`에서 실패한다 → `sudo apt install libjsoncpp-dev`.

> 소스/CMake를 바꿨으면 기존 `build/`는 지우고 새로 빌드할 것 (`rm -rf build`).

## SDF 등록 (이미 overlay에 반영됨)

`overlay/Tools/.../models/s550/s550.sdf.jinja`의 `</model>` 직전:

```xml
<plugin name="disturbance" filename="libdisturbance_plugin.so">
  <link_name>base_link</link_name>
  <!-- topic_name optional; default: /gazebo/<world>/<model>/disturbance_json -->
</plugin>
```

`./apply_overlay.sh`로 PX4-Autopilot 트리에 적용. 이미 빌드한 적이 있다면 stamp 정리도 필요:
```bash
rm -rf PX4-Autopilot/build/px4_sitl_default/external/Stamp/sitl_gazebo-classic
```

## 동작 검증

SITL 부팅 시 콘솔에서 다음 로그가 보여야 한다:
```
[DisturbancePlugin] loaded: model=s550, link=base_link, topic=/gazebo/default/s550/disturbance_json, arrow=on
```

토픽 노출 확인:
```bash
gz topic -l | grep disturbance_json
```

## 사용

`tools/apply_disturbance.py`가 `tools/disturbance_profiles.json`을 읽어 트리거한다:
```bash
./tools/apply_disturbance.py
```

외란 트리거 시 콘솔에 프로파일 로드 로그(`profile loaded: N disturbance(s)`)가 보인다.
