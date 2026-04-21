# Contributing

## Before You Start

- 이 저장소는 ROS 2 Python 워크스페이스입니다.
- 하드웨어 관련 변경은 실차 안전에 직접 영향을 줄 수 있습니다.
- unrelated 변경은 한 PR에 섞지 않는 것을 원칙으로 합니다.

## Development Setup

```bash
cd /home/kown/jetcar_ws
colcon build --symlink-install
source install/setup.bash
python3 -m compileall src
```

필요 시 런타임 의존성:

```bash
sudo apt install -y python3-smbus python3-opencv python3-flask python3-numpy
pip install smbus2 ultralytics
```

## Change Guidelines

- launch 파일을 바꾸면 관련된 config 파일도 같이 검토합니다.
- 하드웨어 기본값을 바꾸면 이유와 실측 근거를 PR 설명에 적습니다.
- 새 노드나 토픽을 추가하면 README에 실행 방법과 토픽 목적을 반영합니다.
- 안전 관련 동작 변경은 실패 시 동작과 비상 정지 경로를 반드시 설명합니다.

## Testing

가능한 최소 검증:

```bash
colcon build --symlink-install
python3 -m compileall src
```

하드웨어가 있는 경우에는 영향을 받는 launch 파일을 실제 장비에서 검증하십시오.

예시:

```bash
ros2 launch jetcar_base hardware_only.launch.py
ros2 launch jetcar_base manual_web.launch.py
ros2 launch jetcar_perception yolo_web.launch.py
```

## Pull Requests

- 변경 목적을 명확히 적습니다.
- 설정값 변경은 전/후 값과 이유를 적습니다.
- UI 또는 스트리밍 변경은 가능하면 스크린샷을 첨부합니다.
- 하드웨어 검증 여부를 명시합니다.

## Commit Style

강제 규칙은 아니지만 한 커밋은 한 주제에 집중시키는 편이 리뷰에 유리합니다.
