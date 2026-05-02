# GitHub Issue Backlog

현재 환경에는 인증된 GitHub CLI가 없어 원격 GitHub Issue를 직접 생성하지 못했습니다. 아래 항목은 그대로 GitHub Issues에 복사해 등록할 수 있도록 제목, 목적, 완료 조건 중심으로 정리했습니다.

## Issue 1: UFLD 모델 export 및 Jetson 배포

Labels: `perception`, `lane`, `model`

UFLD v1 차선 인식 모델을 TorchScript 또는 ONNX로 export하고 Jetson의 `lane_detection.yaml` 경로에 배포한다.

Acceptance criteria:
- `model_path`에 실제 모델 파일이 존재한다.
- `/perception/lane/status`에 `backend=ufld`와 `runtime=torchscript` 또는 `runtime=onnx`가 표시된다.
- 폐쇄 환경 영상 10분 이상에서 차선 ready drop, offset jump, steering jump를 기록한다.

## Issue 2: 카메라 캘리브레이션 및 depth 신뢰도 검증

Labels: `perception`, `depth`, `calibration`

스테레오 카메라 intrinsic/extrinsic, baseline, rectification을 실제 장착 상태 기준으로 재검증한다.

Acceptance criteria:
- 정지 장애물 거리 측정 오차를 0.5m, 1.0m, 1.5m, 2.0m에서 기록한다.
- `/perception/depth/status`의 `ready=true` 유지율을 테스트 로그로 남긴다.
- 좌/중/우 sector distance가 실제 장애물 위치와 일치하는지 확인한다.

## Issue 3: 자율주행 상태기계 closed-course 검증

Labels: `autonomy`, `validation`, `safety`

`autonomous_driver_node`의 `DISABLED`, `STARTUP_HOLD`, `FAULT_STOP`, `OBJECT_STOP`, `HAZARD_STOP`, `LANE_LOSS_STOP`, `DEGRADED`, `CAUTION`, `CRUISE` 상태 전이를 폐쇄 코스에서 검증한다.

Acceptance criteria:
- 각 상태를 강제로 재현한 rosbag 또는 테스트 로그가 있다.
- 상태별 throttle/steering 출력이 기대 범위 안에 있다.
- stale depth/lane/object 입력에서 차량 명령이 0으로 떨어진다.

## Issue 4: 조향/스로틀 한계값 실차 튜닝

Labels: `control`, `hardware`, `tuning`

`vehicle_hw.yaml`, `control.yaml`, `autonomous_driver.yaml`의 throttle/steering 한계값과 slew rate를 실제 차량 하드웨어에 맞춘다.

Acceptance criteria:
- 바퀴를 띄운 상태에서 steering center, left/right limit, throttle deadband를 기록한다.
- 폐쇄 코스에서 `max_throttle_step`, `max_brake_step`, `max_steering_step` 튜닝 결과를 남긴다.
- 급격한 steering oscillation 또는 throttle surge가 없는 설정을 README에 반영한다.

## Issue 5: 하드웨어 독립 estop 경로 추가

Labels: `safety`, `hardware`, `estop`

ROS 노드가 멈춰도 모터 전원이 차단되는 물리적 estop 또는 별도 MCU watchdog을 추가한다.

Acceptance criteria:
- ROS 프로세스 kill, Jetson freeze, 네트워크 단절 상황에서도 모터가 안전 상태로 들어간다.
- estop 복귀 절차가 문서화되어 있다.
- 실제 테스트 영상 또는 로그가 있다.

## Issue 6: rosbag 기반 회귀 테스트 추가

Labels: `testing`, `ci`, `autonomy`

차선, depth, object 토픽을 담은 rosbag으로 autonomy 상태기계와 safety supervisor를 반복 검증한다.

Acceptance criteria:
- 정상 주행, 차선 소실, 전방 장애물, 사람 감지, depth stale 시나리오 bag이 있다.
- bag replay 후 `/autonomy/status`와 `/system/safety_override_reason`을 검사하는 테스트 스크립트가 있다.
- 새 코드 변경 전후 regression 결과를 비교할 수 있다.

## Issue 7: 상용차 수준으로 가기 위한 safety case 작성

Labels: `safety`, `documentation`, `roadmap`

현재 코드는 연구/교육용 소형 플랫폼 수준이다. 상용차 탑재를 목표로 한다면 운영 설계 영역, 위험원 분석, 기능안전, SOTIF, 사이버보안, 검증 계획을 별도 safety case로 정리해야 한다.

Acceptance criteria:
- ODD가 명확하다. 예: 실내/폐쇄 코스/저속/차선 표시 있음/보행자 없음.
- 위험원 목록과 완화 전략이 문서화되어 있다.
- 센서 중복, fault detection, fallback minimal risk maneuver 요구사항이 정의되어 있다.
- 실제 도로 주행 금지 조건과 테스트 승인 절차가 명시되어 있다.

## Issue 8: perception 성능 지표 대시보드 추가

Labels: `observability`, `perception`, `dashboard`

웹 대시보드에서 lane confidence, depth ready, min distance, autonomy state, safety override reason을 한 화면에서 볼 수 있게 한다.

Acceptance criteria:
- `/autonomy/status`, `/perception/lane/status`, `/perception/depth/status`, `/system/safety_override_reason`이 웹 화면에 표시된다.
- stale 상태가 시각적으로 구분된다.
- 주행 테스트 중 로그 저장 버튼 또는 rosbag 녹화 절차가 제공된다.
