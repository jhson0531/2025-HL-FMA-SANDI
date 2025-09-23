# Lidar Perception Package

이 패키지는 ROS2 기반 라이다 센서를 사용한 장애물 감지 시스템입니다.

## 기능

- 라이다 센서 데이터 수집 및 처리
- 설정 가능한 각도 및 거리 범위 내에서 장애물 감지
- 실시간 디버깅 출력 (감지된 장애물의 각도와 거리)
- 안정적인 감지를 위한 연속 감지 확인

## 노드 구성

1. **lidar_publisher_node**: 실제 라이다 센서에서 데이터를 읽어와서 `lidar_raw` 토픽으로 발행
2. **lidar_processor_node**: `lidar_raw` 토픽을 구독하여 데이터를 처리한 후 `lidar_processed` 토픽으로 발행
3. **lidar_obstacle_detector_node**: `lidar_processed` 토픽을 구독하여 장애물을 감지하고 `lidar_obstacle_info` 토픽으로 발행

## 사용법

### 방법 1: 개별 노드 실행 (3개 터미널)

**터미널 1: 라이다 센서 실행**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

**터미널 2: 라이다 데이터 처리 노드**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

**터미널 3: 장애물 감지 노드 (기본 설정)**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 run lidar_perception_pkg lidar_obstacle_detector_node
```

**터미널 3: 장애물 감지 노드 (커스텀 설정)**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 run lidar_perception_pkg lidar_obstacle_detector_node --ros-args -p start_angle:=0 -p end_angle:=90 -p range_min:=0.3 -p range_max:=3.0
```

### 방법 2: Launch 파일 사용

**터미널 1: 라이다 센서 실행**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

**터미널 2: 라이다 처리 및 장애물 감지 노드 실행**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 launch lidar_perception_pkg lidar_perception_launch.py
```

**커스텀 파라미터로 실행:**
```bash
cd /home/woong/lidar_ws
source install/setup.bash
ros2 launch lidar_perception_pkg lidar_perception_launch.py start_angle:=0 end_angle:=90 range_min:=0.3 range_max:=3.0
```

## 파라미터 설정

- `start_angle`: 감지할 각도 범위의 시작값 (0-359도, 기본값: 0)
- `end_angle`: 감지할 각도 범위의 끝값 (0-359도, 기본값: 30)
- `range_min`: 감지할 거리 범위의 최소값 (미터, 기본값: 0.5)
- `range_max`: 감지할 거리 범위의 최대값 (미터, 기본값: 2.0)
- `consec_count`: 연속 감지 횟수 (안정성 확인용, 기본값: 5)

## 토픽

- `/lidar_raw`: 원시 라이다 데이터 (LaserScan)
- `/lidar_processed`: 처리된 라이다 데이터 (LaserScan)
- `/lidar_obstacle_info`: 장애물 감지 정보 (Bool)

## 디버깅 출력 예시

```
[INFO] [lidar_obstacle_detector_node]: 🚨 장애물 감지: 15.0도, 1.25m
[INFO] [lidar_obstacle_detector_node]: 🚨 장애물 감지: 16.0도, 1.30m
[INFO] [lidar_obstacle_detector_node]: 감지 범위: 0~30도, 0.5~2.0m | 최종 감지 결과: True
```

## 각도 범위 설정 예시

- **전방 감지**: `start_angle=0, end_angle=30` (0도~30도)
- **후방 감지**: `start_angle=180, end_angle=210` (180도~210도)
- **좌측 감지**: `start_angle=270, end_angle=300` (270도~300도)
- **우측 감지**: `start_angle=90, end_angle=120` (90도~120도)
- **전방 좌측**: `start_angle=315, end_angle=45` (315도~45도, 0도를 포함)

## 빌드

```bash
cd /home/woong/lidar_ws
colcon build --packages-select lidar_perception_pkg
source install/setup.bash
```
