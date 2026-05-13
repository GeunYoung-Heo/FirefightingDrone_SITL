# disturbance_plugin

Gazebo Classic 11 ModelPlugin. 외란용 force/torque를 Wrench 메시지로 받아 매 physics step 적용한다.

## 왜 필요한가

Gazebo Classic의 표준 `<link>/wrench` 토픽은 publish 1번당 한 simulation step(약 1ms)만 force가 적용된다. step force 외란을 위해 `gz topic -p`를 루프로 돌리는 방식은 호출당 약 400ms 오버헤드 때문에 효과적 duty cycle이 ~0.6%에 그쳐 드론에 거의 영향을 주지 못한다.

이 플러그인은 **마지막에 받은 wrench 값을 매 step 자동으로 link에 적용**한다. 사용자는 시작과 종료 두 번만 publish하면 된다.

## 빌드

```bash
cd ~/Firefighting_Drone/SITL/tools/disturbance_plugin
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

산출물: `build/libdisturbance_plugin.so`

빌드 후 `run_sitl.sh`가 이 디렉토리를 자동으로 `GAZEBO_PLUGIN_PATH`에 추가하므로 PX4 build 디렉토리로 별도 복사할 필요가 없다.

## SDF 등록 (이미 overlay에 반영됨)

`overlay/Tools/.../models/s550/s550.sdf.jinja`의 `</model>` 직전에 다음 블록이 들어가 있다:

```xml
<plugin name="disturbance" filename="libdisturbance_plugin.so">
  <link_name>base_link</link_name>
  <!-- topic_name optional; default: /gazebo/default/<model>/disturbance_cmd -->
</plugin>
```

`./apply_overlay.sh`로 PX4-Autopilot 트리에 적용. 이미 빌드한 적이 있다면 stamp 정리도 필요:
```bash
rm -rf PX4-Autopilot/build/px4_sitl_default/external/Stamp/sitl_gazebo-classic
```

## 동작 검증

SITL 부팅 시 콘솔에서 다음 로그가 보여야 한다:
```
[DisturbancePlugin] loaded: model=s550, link=base_link, topic=/gazebo/default/s550/disturbance_cmd
```

토픽 노출 확인:
```bash
gz topic -l | grep disturbance_cmd
```

## 사용

`tools/apply_disturbance.py` 참고. 가장 간단한 호출:
```bash
./tools/apply_disturbance.py --force "30 0 0" --duration 1.0
```
