# UTM Pure Pursuit 제어기

이 코드는 UTM 좌표계를 사용하여 로봇이 waypoint를 따라가는 Pure Pursuit 알고리즘을 구현합니다.

## 주요 기능

- **UTM 좌표 기반**: UTM 좌표계를 사용한 정확한 위치 제어
- **Pure Pursuit 알고리즘**: 부드러운 경로 추적
- **테스트 모드**: 실제 하드웨어 없이도 시뮬레이션 가능
- **실시간 제어**: 10Hz 주기로 cmd_vel 명령 발행
- **Waypoint 관리**: 동적으로 waypoint 추가/제거 가능

## 파일 구조

```
nav_controller/
├── nav_controller/
│   ├── utm_pure_pursuit.py    # 메인 Pure Pursuit 제어기
│   └── control.py             # 기존 제어기 (참고용)
├── setup.py                   # 빌드 설정 (업데이트됨)
└── package.xml               # 패키지 메타데이터
```

## 설정된 Waypoint

- **초기 위치**: UTM X: 330034.491650, UTM Y: 4123117.956354
- **Waypoint 1**: UTM X: 330030.501943, UTM Y: 4123118.148129
- **Waypoint 2**: UTM X: 330024.601050, UTM Y: 4123120.376601

## 주요 파라미터

```python
lookahead_distance = 2.0      # 전방 주시 거리 (미터)
max_speed = 0.5              # 최대 선속도 (m/s)
min_speed = 0.1              # 최소 선속도 (m/s)
max_angular_speed = 1.0      # 최대 각속도 (rad/s)
waypoint_tolerance = 1.0     # waypoint 도달 판정 거리 (미터)
```

## 사용 방법

### 1. 패키지 빌드

```bash
cd ~/ros2_workspace
colcon build --packages-select nav_controller
source install/setup.bash
```

### 2. 테스트 모드 실행

```bash
# 직접 실행
ros2 run nav_controller utm_pure_pursuit

# 또는 테스트 스크립트 사용
python3 test_utm_pure_pursuit.py
```

### 3. 실제 하드웨어 연결

코드에서 `test_mode = False`로 설정하고 다시 빌드:

```python
# utm_pure_pursuit.py 파일에서
self.test_mode = False  # True → False로 변경
```

그 후 실행:
```bash
ros2 run nav_controller utm_pure_pursuit
```

## 토픽 인터페이스

### 구독 토픽
- `/odometry/global` (nav_msgs/Odometry): 로봇의 UTM 좌표 위치 정보
- `goal_pose` (geometry_msgs/PoseStamped): 새로운 목표점 설정 (선택사항)

### 발행 토픽
- `cmd_vel` (geometry_msgs/Twist): 로봇 제어 명령

## 테스트 모드 vs 실제 모드

### 테스트 모드 (`test_mode = True`)
- 실제 토픽 구독 없이 시뮬레이션
- 로봇이 waypoint 방향으로 자동 이동
- 디버깅 및 알고리즘 검증에 유용

### 실제 모드 (`test_mode = False`)
- `/odometry/global` 토픽에서 실제 위치 정보 수신
- 실제 로봇 하드웨어와 연동
- 실제 환경에서 waypoint 추적

## 코드 수정 가이드

### 1. Waypoint 추가/수정

```python
# utm_pure_pursuit.py의 __init__ 메서드에서
self.waypoints = [
    (330030.501943, 4123118.148129),  # 첫 번째 waypoint
    (330024.601050, 4123120.376601),  # 두 번째 waypoint
    (330020.000000, 4123125.000000),  # 새로운 waypoint 추가
]
```

### 2. 제어 파라미터 조정

```python
# 더 빠른 이동을 원한다면
self.lookahead_distance = 3.0  # 2.0 → 3.0
self.max_speed = 0.8          # 0.5 → 0.8

# 더 정밀한 제어를 원한다면
self.waypoint_tolerance = 0.5  # 1.0 → 0.5
```

### 3. 초기 위치 변경

```python
# utm_pure_pursuit.py의 __init__ 메서드에서
self.initial_x = 330034.491650  # 새로운 UTM X 좌표
self.initial_y = 4123117.956354  # 새로운 UTM Y 좌표
```

## 디버깅

### 로그 확인
```bash
# ROS2 로그 확인
ros2 topic echo /rosout

# 특정 노드 로그 확인
ros2 node list
ros2 node info /utm_pure_pursuit
```

### 토픽 모니터링
```bash
# cmd_vel 토픽 모니터링
ros2 topic echo /cmd_vel

# odometry 토픽 모니터링 (실제 모드에서)
ros2 topic echo /odometry/global
```

## 문제 해결

### 1. 패키지 빌드 오류
```bash
# 의존성 확인
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
rm -rf build/ install/ log/
colcon build --packages-select nav_controller
```

### 2. 토픽 연결 오류
- `test_mode` 설정 확인
- 토픽 이름 확인: `ros2 topic list`
- 토픽 타입 확인: `ros2 topic info /odometry/global`

### 3. 제어 성능 문제
- `lookahead_distance` 조정
- `max_speed`, `min_speed` 조정
- `waypoint_tolerance` 조정

## 확장 가능성

1. **동적 Waypoint**: 런타임에 waypoint 추가/제거
2. **장애물 회피**: 동적 장애물 감지 및 회피
3. **속도 프로파일**: 상황별 적응적 속도 제어
4. **경로 스무딩**: B-Spline 등을 이용한 경로 최적화
5. **다중 로봇**: 여러 로봇의 협조 제어

## 라이선스

이 코드는 기존 control.py를 참고하여 작성되었으며, 동일한 라이선스를 따릅니다.
