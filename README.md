# Physical AI Automatic Car

Jetson Orin Nano 기반 ROS 2 워크스페이스입니다. 수동 조종, 스테레오 인지, YOLO 웹 뷰, 안전 정지, 연구용 통합 실행을 한 저장소에서 다룹니다.

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

객체 인지:

```bash
ros2 launch jetcar_perception object_detection.launch.py
```

`object_detection.launch.py`는 현재 스테레오 카메라, rectification, 객체검출, 차선인식, YOLO 웹 뷰를 함께 띄웁니다.

## YOLO Web View

기본 설정은 `src/jetcar_perception/config/yolo_web_stereo.yaml`입니다. 웹 화면은 좌우 스테레오 영상을 나란히 보여주고, YOLO 박스는 왼쪽 영상 기준으로 표시합니다.

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

Level 2: depth 기반 저속 직진 + 전방 장애물 정지

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

Depth 설정 확인:

```bash
ros2 topic echo /perception/depth/status
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
- `src/jetcar_perception/config/object_detection.yaml`: YOLO detection 설정
- `src/jetcar_perception/config/yolo_web_stereo.yaml`: 8081 스테레오 웹 뷰 설정
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
