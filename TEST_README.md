# UTM Pure Pursuit 테스트 가이드

## 개요
직선-곡선-후진 구간이 포함된 경로 추적 시스템을 테스트하기 위한 코드입니다.

## 파일 구성
- `utm_pure_pursuit_test.py`: 가상 GPS/IMU 데이터를 발행하는 Pure Pursuit 제어 노드
- `point_plot_test.py`: 구간별 경로를 시각화하는 플롯 노드

## 테스트 실행 방법

### 1. 터미널 1: Pure Pursuit 테스트 노드 실행
```bash
cd /home/jh/ros2_workspace/src
python3 utm_pure_pursuit_test.py
```

### 2. 터미널 2: 시각화 노드 실행
```bash
cd /home/jh/ros2_workspace/src
python3 point_plot_test.py
```

## 구간 설정
현재 설정된 구간:
```python
segment_config = [[16, 's'], [20, 'c'], [25, 'b'], [40, 'c']]
```
- 0-16: 직선 구간 (s = straight)
- 17-20: 곡선 구간 (c = curve)  
- 21-25: 후진 구간 (b = back/reverse)
- 26-40: 곡선 구간 (c = curve)

## 시각화 색상
- **파란색 점**: 직선 구간 (Straight)
- **빨간색 점**: 곡선 구간 (Curve)
- **보라색 점**: 후진 구간 (Reverse)
- **노란색 점**: 미분류 구간 (Unclassified)

## 테스트 모드 특징
1. **가상 GPS/IMU**: 실제 센서 없이 가상 데이터 발행
2. **자동 경로 추적**: waypoint 파일에서 경로를 자동으로 로드하여 추적
3. **구간별 시각화**: 각 구간을 다른 색상으로 표시
4. **실시간 업데이트**: 100Hz로 경로와 구간 정보 업데이트

## 로그 확인
터미널에서 다음과 같은 로그를 확인할 수 있습니다:
- 🗺️ 경로 생성 정보
- 🔄 구간 전환 알림
- 🧭 현재 주행 구간 정보
- 📡 ROS2 토픽 발행 정보

## 문제 해결
1. **파일을 찾을 수 없음**: `utm_coordinates.txt` 파일이 `/home/jh/ros2_workspace/src/` 경로에 있는지 확인
2. **ROS2 연결 오류**: ROS2 환경이 올바르게 설정되어 있는지 확인
3. **시각화가 안됨**: matplotlib이 설치되어 있는지 확인 (`pip install matplotlib`)

## 종료 방법
- `Ctrl+C`로 각 터미널에서 프로그램 종료
- 시각화 창을 닫으면 자동으로 정리됨
