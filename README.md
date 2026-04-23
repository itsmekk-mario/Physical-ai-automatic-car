# Physical AI Automatic Car

Jetson Orin Nano 기반 ROS 2 워크스페이스입니다. 저장소는 수동 조종, 하드웨어 제어, 인지, 안전 판단, 연구용 통합 실행까지 한 워크스페이스 안에서 다룹니다.

## Workspace Layout

- `src/jetcar_base`: 차량 하드웨어 제어, 키보드 조종, 웹 조종
- `src/jetcar_control`: 수동/AI/자율주행 입력 중재와 드라이브 모드 관리
- `src/jetcar_perception`: 스테레오 카메라, 차선, 깊이, 객체 인지, YOLO 웹 뷰
- `src/jetcar_decision`: 안전 감독과 자율주행 명령 생성
- `src/jetcar_research`: 통합 실험 프로파일과 풀스택 실행
- 루트 Python 스크립트: PCA9685/서보 직접 점검 및 캘리브레이션

## Prerequisites

- Ubuntu on Jetson
- ROS 2 Python workspace capable of running `colcon`
- Python packages and system packages used by the nodes:

```bash
sudo apt update
sudo apt install -y python3-smbus python3-opencv python3-flask python3-numpy
pip install smbus2 ultralytics
```

`ultralytics`는 YOLO 관련 기능에서 필요합니다. 하드웨어 제어는 PCA9685가 I2C bus `7`에 연결되어 있다는 현재 설정을 기준으로 합니다.

## Build

```bash
cd /home/kown/jetcar_ws
colcon build --symlink-install
source install/setup.bash
```

매 새 터미널마다 아래는 다시 실행해야 합니다.

```bash
cd /home/kown/jetcar_ws
source install/setup.bash
```

## How To Run

### 1. Hardware Only

차량 하드웨어 노드만 실행합니다.

```bash
ros2 launch jetcar_base hardware_only.launch.py
```

### 2. Manual Keyboard Control

키보드 입력과 제어 mux까지 포함한 수동 주행 스택입니다.

```bash
ros2 launch jetcar_base manual_keyboard.launch.py
```

### 3. Manual Web Control

웹 조종 UI를 엽니다. 기본 포트는 `5000`입니다.

```bash
ros2 launch jetcar_base manual_web.launch.py
```

원격 Jetson에 SSH 접속 중이면 포트 포워딩 후 브라우저에서 엽니다.

```bash
ssh -L 5000:localhost:5000 kown@JETCAR_IP
```

브라우저: `http://localhost:5000`

### 4. Control Stack Only

드라이브 모드 관리자와 제어 mux만 분리 실행합니다.

```bash
ros2 launch jetcar_control control_stack.launch.py
```

### 5. Perception Pipelines

스테레오 카메라만:

```bash
ros2 launch jetcar_perception stereo_camera.launch.py
```

실행 후 브라우저: `http://localhost:8081`

스테레오 보정:

```bash
ros2 launch jetcar_perception stereo_rectification.launch.py
```

실행 후 브라우저: `http://localhost:8081`

스테레오 깊이:

```bash
ros2 launch jetcar_perception stereo_depth.launch.py
```

실행 후 브라우저: `http://localhost:8081`

차선 인지:

```bash
ros2 launch jetcar_perception lane_detection.launch.py
```

실행 후 브라우저: `http://localhost:8081`

객체 인지:

```bash
ros2 launch jetcar_perception object_detection.launch.py
```

위 다섯 개 perception launch는 모두 차량 하드웨어 제어, control mux, 그리고 `8081` 웹 대시보드를 함께 올립니다. 화면 구성은 `yolo_web.launch.py`와 동일한 조종 페이지입니다.

### 6. YOLO Web View

기본 설정은 `src/jetcar_perception/config/yolo_web_stereo.yaml`을 사용하며, 기본 포트는 현재 `8081`입니다.

모델 파일은 아래 둘 중 하나를 준비하면 됩니다.

```text
/home/kown/jetcar_ws/models/yolov8n.engine
/home/kown/jetcar_ws/models/yolov8n.pt
```

실행:

```bash
ros2 launch jetcar_perception yolo_web.launch.py
```

원격 접속 예시:

```bash
ssh -L 8081:localhost:8081 kown@JETCAR_IP
```

브라우저: `http://localhost:8081`

### 7. Decision Nodes

안전 감독만:

```bash
ros2 launch jetcar_decision ai_intervention.launch.py
```

자율주행 드라이버만:

```bash
ros2 launch jetcar_decision autonomous_driver.launch.py
```

### 8. Full Autonomous Research Stack

전체 스택 실행:

```bash
ros2 launch jetcar_research full_autonomous.launch.py
```

실행 직후 모터가 돌지 않도록 바퀴를 띄운 상태에서 먼저 아래 토픽을 확인하는 것이 안전합니다.

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

즉시 수동 안전 상태로 복귀:

```bash
ros2 topic pub --once /system/autonomy_enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /system/drive_mode_cmd std_msgs/msg/String "{data: MANUAL}"
ros2 topic pub --once /system/estop_cmd std_msgs/msg/Bool "{data: true}"
```

### 9. Research Profile Node

```bash
ros2 launch jetcar_research research_compare.launch.py
```

## Direct Utility Scripts

루트의 스크립트들은 ROS 2 launch가 아니라 Python으로 직접 실행합니다. 모두 PCA9685와 I2C bus `7` 접근이 가능해야 합니다.

서보 직접 테스트:

```bash
python3 test_servo_direct.py
```

서보 고정 위치 점검:

```bash
python3 servo_hold_test.py
```

서보 raw tick 캘리브레이션:

```bash
python3 servo_calibrate_raw.py
```

서보 수동 미세 조정:

```bash
python3 servo_manual_step.py
```

## Configuration Files

- `src/jetcar_base/config/vehicle_hw.yaml`: I2C bus, PCA9685 채널, 서보/모터 제한값
- `src/jetcar_base/config/manual_web.yaml`: 웹 제어 포트와 입력 step
- `src/jetcar_control/config/control.yaml`: 기본 드라이브 모드와 throttle 제한
- `src/jetcar_perception/config/*.yaml`: 카메라, 차선, 객체 인지, YOLO 관련 설정
- `src/jetcar_decision/config/*.yaml`: 안전 정지 거리, 자율주행 gain과 throttle
- `src/jetcar_research/config/research_profiles.yaml`: 연구 프로파일 설정

## Development

```bash
colcon build --symlink-install
python3 -m compileall src
```

ROS 2 실행 환경과 실제 Jetson 하드웨어가 없으면 런타임 검증은 제한됩니다. 변경 시에는 최소한 해당 launch 파일과 설정 파일이 서로 맞는지 함께 확인하는 것을 권장합니다.

## Safety Notes

- 실제 바닥 주행 전에는 반드시 바퀴를 띄운 상태에서 테스트합니다.
- 하드웨어 기본 설정은 실차에 맞춰져 있으므로 다른 배선에서는 `vehicle_hw.yaml`부터 조정해야 합니다.
- 웹 제어와 자율주행은 모두 실제 모터 출력을 만들 수 있으므로 비상 정지 토픽 사용 절차를 먼저 확인하십시오.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
