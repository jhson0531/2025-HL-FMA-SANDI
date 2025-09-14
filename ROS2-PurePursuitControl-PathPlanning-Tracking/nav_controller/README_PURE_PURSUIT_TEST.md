# Pure Pursuit 테스트 환경

이 디렉토리는 UTM Pure Pursuit 제어 알고리즘을 테스트하기 위한 환경을 제공합니다.

## 파일 구성

### 테스트 파일들
- `utm_pure_pursuit_test.py`: 원본 utm_pure_pursuit.py를 복사하여 경로 발행 기능을 추가한 테스트용 파일
- `utm_test_publisher.py`: 고정된 UTM 좌표 (329999.864145, 4123213.562485)를 발행하는 테스트용 노드
- `pure_pursuit_visualizer.py`: waypoint, 보간된 경로, cmd_vel을 실시간으로 시각화하는 프로그램

### Launch 파일
- `launch/pure_pursuit_test.launch.py`: 모든 테스트 노드를 한번에 실행하는 launch 파일

## 실행 방법

### 1. 패키지 빌드
```bash
cd /home/jh/ros2_workspace
colcon build --packages-select nav_controller
source install/setup.bash
```

### 2. 테스트 실행

#### 방법 1: Launch 파일 사용 (권장)
```bash
# 시각화 포함하여 실행
ros2 launch nav_controller pure_pursuit_test.launch.py

# 시각화 없이 실행
ros2 launch nav_controller pure_pursuit_test.launch.py use_visualizer:=false
```

#### 방법 2: 개별 노드 실행
```bash
# 터미널 1: UTM 테스트 퍼블리셔
ros2 run nav_controller utm_test_publisher.py

# 터미널 2: Pure Pursuit 테스트 제어기
ros2 run nav_controller utm_pure_pursuit_test.py

# 터미널 3: 시각화 (선택사항)
ros2 run nav_controller pure_pursuit_visualizer.py
```

## 시각화 기능

`pure_pursuit_visualizer.py`는 다음을 실시간으로 시각화합니다:

### 1. 경로 추적 플롯
- **빨간 점**: Waypoint들
- **파란 선**: B-Spline으로 보간된 경로
- **초록 점**: 현재 로봇 위치
- **초록 화살표**: 로봇의 현재 방향
- **주황 원**: Lookahead distance (1.0m)

### 2. 속도 그래프
- **파란 선**: Linear velocity (m/s) 시간 변화

### 3. 각속도 그래프
- **빨간 선**: Angular velocity (rad/s) 시간 변화

### 4. 상태 정보
- 로봇의 현재 위치 (X, Y, Yaw)
- 현재 제어 명령 (Linear/Angular velocity)
- 실행 시간 및 데이터 포인트 수

## 테스트 시나리오

1. **초기화**: UTM 테스트 퍼블리셔가 고정된 좌표를 발행
2. **경로 생성**: Pure Pursuit 노드가 waypoint들을 기반으로 B-Spline 경로 생성
3. **경로 발행**: 생성된 경로를 `/pure_pursuit/path` 토픽으로 발행
4. **제어 실행**: Pure Pursuit 알고리즘이 경로를 추적하며 `cmd_vel` 발행
5. **시각화**: 시각화 노드가 모든 데이터를 실시간으로 표시

## Waypoint 설정

현재 설정된 waypoint들:
- WP1: (329983.719725, 4123210.415129)
- WP2: (329977.396338, 4123210.620894)
- WP3: (329964.442297, 4123208.130461)
- WP4: (329955.384840, 4123213.997123)

waypoint를 변경하려면 `utm_pure_pursuit_test.py`의 `self.waypoints` 리스트를 수정하세요.

## 주의사항

- 시각화 프로그램은 matplotlib을 사용하므로 GUI 환경에서 실행해야 합니다
- 테스트 좌표는 고정되어 있으므로 실제 로봇 움직임은 시뮬레이션입니다
- IMU 보정각도 파일(`imu_calibration_angle.txt`)이 없으면 기본값 0을 사용합니다

## 문제 해결

### 시각화가 나타나지 않는 경우
- GUI 환경에서 실행하고 있는지 확인
- matplotlib이 설치되어 있는지 확인: `pip install matplotlib`

### 경로가 생성되지 않는 경우
- waypoint가 2개 이상 설정되어 있는지 확인
- UTM 테스트 퍼블리셔가 정상적으로 실행되고 있는지 확인

### 제어 명령이 발행되지 않는 경우
- odometry 데이터가 정상적으로 수신되고 있는지 확인
- 경로 생성이 완료되었는지 로그 확인
