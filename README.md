# Physical AI Automatic Car

Jetson Orin Nano 기반 ROS 2 워크스페이스입니다. 수동 조종, 스테레오 인지, YOLO 웹 뷰, 안전 정지, 연구용 통합 실행을 한 저장소에서 다룹니다.

## 한눈에 보기

- 목표: Jetson Orin Nano + ROS 2 기반 소형 자율주행 플랫폼입니다.
- 인지: 스테레오 카메라, rectification, depth, UFLD 차선 인식, YOLO 객체 인식을 사용합니다.
- 판단/제어: `autonomous_driver_node`가 자율주행 명령을 만들고, `safety_supervisor_node`가 depth/객체 위험 상황에서 override를 겁니다.
- 실행: Level 2는 차선 추종 + 정지, Level 3은 정적 장애물 회피, Level 4는 접근 속도까지 반영한 동적 회피입니다.
- 검증: 실제 주행 전 바퀴를 띄운 상태에서 토픽, 카메라, 조향/스로틀, estop 동작을 먼저 확인해야 합니다.

이 저장소의 알고리즘은 상용차 개발 방식에서 쓰는 구조를 참고해 방어적으로 작성했지만, 상용차 장착 가능 수준의 인증 소프트웨어는 아닙니다. 실제 상용차 수준이 되려면 중복 센서, 기능안전 분석, SOTIF, HIL/SIL, 폐쇄 코스 검증, 하드웨어 독립 estop, 장시간 데이터 검증이 별도로 필요합니다.

필요 작업 backlog는 `docs/github_issues.md`에 GitHub Issue로 옮기기 쉬운 형태로 정리했습니다.

## 구성

- `src/jetcar_base`: 차량 하드웨어 제어, 키보드 조종, 웹 조종
- `src/jetcar_control`: drive mode 관리, control mux
- `src/jetcar_perception`: 스테레오 카메라, rectification, depth, lane/object detection, YOLO 웹 뷰
- `src/jetcar_decision`: safety supervisor, autonomous driver
- `src/jetcar_research`: 통합 실험 launch와 연구 프로파일

## 설치와 빌드

필수 패키지:

```bash
sudo apt update
sudo apt install -y python3-smbus python3-opencv python3-flask python3-numpy
python3 -m pip install --user -r requirements-jetson.txt
```

주의:

```bash
python3 -m pip install --user "numpy==1.26.4"
```

Jetson 기본 `torch`는 현재 `numpy 2.x`와 ABI가 맞지 않아 `yolo_web_node`가 모델 로드 중 실패할 수 있습니다. `ultralytics` 재설치 후에도 `numpy`가 다시 올라가지 않도록 `requirements-jetson.txt` 기준으로 설치하세요.

빌드:

```bash
cd /home/kown/jetcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

새 터미널마다 다시:

```bash
cd /home/kown/jetcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

확인:

```bash
ros2 pkg list | grep jetcar_perception
```

## 자주 쓰는 실행

하드웨어만:

```bash
ros2 launch jetcar_base hardware_only.launch.py
```

키보드 수동 조종:

```bash
ros2 launch jetcar_base manual_keyboard.launch.py
```

웹 수동 조종:

```bash
ros2 launch jetcar_base manual_web.launch.py
```

브라우저: `http://localhost:5000`

노트북/다른 PC에서 접속할 때는 `localhost`를 쓰면 노트북 자기 자신으로 접속합니다. Jetson에서 IP를 확인한 뒤 노트북 브라우저에서 Jetson IP로 접속하세요.

```bash
hostname -I
```

예시:

```text
http://JETSON_IP:5000
http://JETSON_IP:8081
```

제어 스택만:

```bash
ros2 launch jetcar_control control_stack.launch.py
```

## Perception Launch

아래 perception launch는 공통으로 웹 뷰와 기본 control stack을 함께 올립니다. 웹 대시보드는 `http://localhost:8081`입니다.

스테레오 카메라:

```bash
ros2 launch jetcar_perception stereo_camera.launch.py
```

스테레오 보정:

```bash
ros2 launch jetcar_perception stereo_rectification.launch.py
```

스테레오 깊이:

```bash
ros2 launch jetcar_perception stereo_depth.launch.py
```

차선 인지:

```bash
ros2 launch jetcar_perception lane_detection.launch.py
```

기본 차선 인식 backend는 UFLD입니다. C170 전방 카메라가 `/sensors/c170/image_raw`로 발행되고, `lane_detection_node`가 이 토픽을 UFLD 입력으로 사용합니다. 기존 ROS 토픽(`/perception/lane/ready`, `/perception/lane/offset_m`, `/perception/lane/heading_error_deg`, `/perception/lane/confidence`, `/perception/lane/suggested_steering`, `/perception/lane/status`)은 그대로 발행하므로 decision/control 스택은 변경하지 않아도 됩니다.

UFLD 모델은 저장소에 포함하지 않습니다. TorchScript 또는 ONNX로 export한 UFLD v1 모델을 Jetson에 두고 `src/jetcar_perception/config/lane_detection.yaml`의 `model_path`를 맞춥니다.

```yaml
detector_backend: ufld
model_path: /home/kown/jetcar_ws/models/ufld_culane.torchscript.pt
ufld_runtime: auto
ufld_model_type: culane
ufld_input_width: 800
ufld_input_height: 288
ufld_griding_num: 200
ufld_lane_indices: [1, 2]
```

기본값은 CULane 계열 UFLD 모델 기준입니다. TuSimple 모델을 쓰면 `ufld_model_type`, `ufld_griding_num`, `ufld_row_anchors`를 모델 export 설정에 맞춰 바꿔야 합니다. 모델이 없거나 로드에 실패하면 안전하게 `cv` backend로 내려가며, 실제 UFLD가 동작 중인지는 아래 상태에서 `backend=ufld`, `runtime=torchscript` 또는 `runtime=onnx`로 확인합니다.

```bash
ros2 topic echo /perception/lane/status
```

객체 인지:

```bash
ros2 launch jetcar_perception object_detection.launch.py
```

`object_detection.launch.py`는 현재 C170 웹 카메라, 스테레오 카메라, rectification, 객체검출, 차선인식, YOLO 웹 뷰를 함께 띄웁니다. 객체검출도 기본적으로 `/sensors/c170/image_raw`를 사용합니다.

## YOLO Web View

기본 설정은 `src/jetcar_perception/config/yolo_web_stereo.yaml`입니다. 웹 화면은 C170을 상단 메인 패널로 크게 보여주고, 하단에 stereo left/right를 보조 패널로 배치합니다. C170은 UFLD 차선 인식과 객체 인식용이고, stereo left/right는 차선 보조 인식과 depth/distance 용도입니다.

모델 파일은 둘 중 하나를 준비하면 됩니다.

```text
/home/kown/jetcar_ws/models/yolov8n_int8.engine
/home/kown/jetcar_ws/models/yolov8n.engine
/home/kown/jetcar_ws/models/yolov8n.pt
```

실행:

```bash
ros2 launch jetcar_perception yolo_web.launch.py
```

브라우저: `http://localhost:8081`

원격 Jetson 접속 예시:

```bash
ssh -L 8081:localhost:8081 kown@JETCAR_IP
```

## AI Intervention

`AI_INTERVENTION` 모드에서는 사람이 감지되면 자동 감속 및 정지 경로가 동작합니다. `yolo_web.launch.py`와 perception launch에서 이 동작을 바로 사용할 수 있습니다.

보는 토픽:

```bash
ros2 topic echo /perception/detections/person_detected
ros2 topic echo /system/estop_cmd
ros2 topic echo /vehicle/emergency_stop
```

## 스테레오 파이프라인 점검

스테레오 카메라:

```bash
ros2 topic hz /sensors/stereo/left/image_raw
ros2 topic hz /sensors/stereo/right/image_raw
ros2 topic echo /sensors/stereo/status
```

Rectification:

```bash
ros2 topic hz /sensors/stereo/left/image_rect
ros2 topic hz /sensors/stereo/right/image_rect
ros2 topic echo /sensors/stereo/rectified/ready
```

Depth:

```bash
ros2 topic echo /perception/depth/status
ros2 topic echo /perception/depth/min_distance_m
```

## Decision / Research

## Autonomous Levels

아래 launch는 기본으로 `AUTONOMOUS` 모드에서 시작합니다. 웹 대시보드는 공통으로 `http://JETSON_IP:8081`이며, 대시보드에서 수동 조작을 하면 `AI_INTERVENTION` 모드로 전환되어 사람 조종 위에 depth 안전 개입이 유지됩니다.

Level 2: 차선 인식 기반 저속 차선 추종 + depth 기반 전방 장애물 정지

```bash
ros2 launch jetcar_perception level2_depth_stop.launch.py
```

Level 3: depth 기반 정적 장애물 회피. 좌/중/우 depth sector와 closest offset으로 회피 방향을 정합니다.

```bash
ros2 launch jetcar_perception level3_static_avoid.launch.py
```

Level 4: depth 변화량으로 접근 속도를 추정해 동적 장애물에 더 일찍 반응합니다.

```bash
ros2 launch jetcar_perception level4_dynamic_avoid.launch.py
```

## 자율주행 알고리즘

현재 자율주행 판단은 `src/jetcar_decision/jetcar_decision/autonomous_driver_node.py`에 있습니다. 입력은 UFLD 차선 인식, stereo depth, YOLO 객체 위험 신호이며 출력은 `/input/autonomy/throttle`, `/input/autonomy/steering`입니다. 최종 차량 명령은 `control_mux_node`가 drive mode와 safety override를 보고 선택합니다.

상태기계:

- `DISABLED`: 자율주행 비활성화. throttle/steering을 0으로 유지합니다.
- `STARTUP_HOLD`: 실행 직후 안정화 대기. 카메라와 depth가 올라오기 전에 움직이지 않습니다.
- `FAULT_STOP`: 필수 입력이 stale이거나 depth가 ready가 아닐 때 정지합니다.
- `OBJECT_STOP`: 사람 또는 객체 위험 신호가 fresh하게 들어오면 정지합니다.
- `HAZARD_STOP`: 전방 거리가 `hazard_stop_distance_m` 이하이거나 가까운 거리에서 closing speed가 큰 경우 정지합니다.
- `LANE_LOSS_STOP`: `require_lane=true`인데 차선 입력이 없으면 정지합니다.
- `DEGRADED`: 차선이 없지만 `require_lane=false`이면 직진 저속 crawl만 허용합니다.
- `CAUTION`: 전방 거리, 중앙 sector, closing speed, 낮은 차선 confidence 때문에 감속합니다.
- `CRUISE`: depth/lane/object 입력이 fresh하고 위험이 없을 때 정상 저속 주행합니다.

정책:

- 입력 watchdog: depth, lane, object 토픽이 `input_timeout_sec`보다 오래되면 stale로 보고 속도를 낮추거나 정지합니다.
- 거리 envelope: `hazard_stop_distance_m` 이하는 정지, `clear_distance_m`까지는 선형 감속, 충분히 멀 때만 cruise throttle을 허용합니다.
- 접근 속도: depth 최소 거리 변화량으로 closing speed를 추정하고 빠르게 가까워지는 물체가 있으면 조기 감속/정지합니다.
- 차선 confidence: UFLD 차선 confidence가 낮으면 조향은 제한하고 throttle을 줄입니다.
- 곡률 감속: 조향 명령이 클수록 throttle을 자동으로 낮춥니다.
- 명령 제한: throttle과 steering은 한 주기당 변화량을 제한해 급격한 입력을 막습니다.
- safety override: `safety_supervisor_node`는 depth/person hazard가 있으면 autonomous 명령보다 우선하는 AI override 또는 estop을 발행합니다.

상태 확인:

```bash
ros2 topic echo /autonomy/status
ros2 topic echo /system/safety_override_reason
ros2 topic echo /system/selected_control_source
```

튜닝 시작점:

- 더 보수적으로 멈추기: `hazard_stop_distance_m`, `slow_distance_m`, `center_blocked_distance_m`를 키웁니다.
- 차선 없을 때 움직이지 않기: `require_lane: true`로 바꿉니다.
- 조향이 흔들릴 때: `lane_follow_gain`, `max_steering_step`, `curve_slow_steering`을 낮춥니다.
- 너무 늦게 감속할 때: `closing_speed_slow_mps`, `closing_speed_stop_mps`를 낮춥니다.

Depth 설정 확인:

```bash
ros2 topic echo /perception/depth/status
ros2 topic echo /perception/lane/status
ros2 topic echo /perception/depth/closest_offset
ros2 topic echo /system/safety_override_reason
ros2 topic echo /system/selected_control_source
```

Safety supervisor만:

```bash
ros2 launch jetcar_decision ai_intervention.launch.py
```

Autonomous driver만:

```bash
ros2 launch jetcar_decision autonomous_driver.launch.py
```

전체 연구 스택:

```bash
ros2 launch jetcar_research full_autonomous.launch.py
```

이 런치는 현재 스테레오 카메라, rectification, depth, 객체검출, 차선인식, safety supervisor, autonomous driver, YOLO 웹 뷰를 함께 실행합니다.

연구 비교 launch:

```bash
ros2 launch jetcar_research research_compare.launch.py
```

자율주행 상태 확인:

```bash
ros2 topic echo /autonomy/status
ros2 topic echo /system/selected_control_source
ros2 topic echo /vehicle/state
```

자율주행 활성화:

```bash
ros2 topic pub --once /system/autonomy_enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /system/drive_mode_cmd std_msgs/msg/String "{data: AUTONOMOUS}"
```

즉시 수동 복귀:

```bash
ros2 topic pub --once /system/autonomy_enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /system/drive_mode_cmd std_msgs/msg/String "{data: MANUAL}"
ros2 topic pub --once /system/estop_cmd std_msgs/msg/Bool "{data: true}"
```

## 유틸리티 스크립트

루트 스크립트는 ROS 2 launch가 아니라 Python으로 직접 실행합니다.

```bash
python3 test_servo_direct.py
python3 servo_hold_test.py
python3 servo_calibrate_raw.py
python3 servo_manual_step.py
```

## 주요 설정 파일

- `src/jetcar_base/config/vehicle_hw.yaml`: I2C bus, PCA9685, servo/motor 제한값
- `src/jetcar_base/config/manual_web.yaml`: 웹 조종 포트와 입력 step
- `src/jetcar_control/config/control.yaml`: drive mode와 throttle 제한
- `src/jetcar_perception/config/stereo_camera.yaml`: 스테레오 카메라 입력, publish 해상도, fps
- `src/jetcar_perception/config/stereo_rectification.yaml`: rectification 파라미터
- `src/jetcar_perception/config/stereo_depth.yaml`: depth 추정 파라미터
- `src/jetcar_perception/config/lane_detection.yaml`: UFLD 차선 인식 모델, row anchor, 조향 보조 파라미터
- `src/jetcar_perception/config/object_detection.yaml`: YOLO detection 설정
- `src/jetcar_perception/config/yolo_web_stereo.yaml`: 8081 C170 + stereo 3카메라 웹 뷰 설정
- `src/jetcar_decision/config/*.yaml`: safety, autonomy 관련 파라미터
- `src/jetcar_research/config/research_profiles.yaml`: 연구 프로파일

## 안전

- 실제 주행 전에는 반드시 바퀴를 띄운 상태에서 먼저 테스트합니다.
- 하드웨어 배선이 다르면 `vehicle_hw.yaml`부터 확인해야 합니다.
- 웹 조종과 자율주행은 실제 모터 출력을 만들 수 있으므로 `estop` 토픽 절차를 먼저 확인하십시오.

## 개발

```bash
colcon build --symlink-install
python3 -m compileall src
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
