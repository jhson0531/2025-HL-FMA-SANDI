#!/usr/bin/env python3
"""
경로 발행 테스트용 point_plot.py
실제 로봇 위치 발행을 비활성화하고 경로 시각화에만 집중합니다.
"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import io
import math
import threading
import re
import time
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from sensor_msgs.msg import NavSatFix
    from geometry_msgs.msg import PoseArray
    from std_msgs.msg import String
    import utm
    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False

# =============================================================================
# 설정 섹션: 여기서 waypoint 파일 경로를 수정하세요
# =============================================================================

# 입력 waypoint 파일 경로
INPUT_WAYPOINTS_FILE = 'remapped_utmcoordinates.txt'  # 시각화할 waypoint 파일

# =============================================================================

# 한글 폰트 설정
matplotlib.rcParams['font.family'] = 'Malgun Gothic'
# 마이너스 부호 깨짐 방지
matplotlib.rcParams['axes.unicode_minus'] = False

# ROS2에서 수신한 데이터 (스레드간 공유)
_robot_lock = threading.Lock()

# ROS2 종료 신호용
_shutdown_event = threading.Event()

# 보간된 경로를 위한 변수들
_interpolated_path_x = []
_interpolated_path_y = []
_interpolated_path_received = False

# 구간 정보를 위한 변수들
_path_segments = []
_path_length = 0

class _PoseListener(Node):
    def __init__(self):
        super().__init__('waypoint_pose_listener_test')
        # 실제 로봇 위치는 구독하지 않고 경로 데이터만 구독
        self.create_subscription(PoseArray, '/interpolated_path', self._interpolated_path_cb, 10)
        self.create_subscription(String, '/path_segments', self._path_segments_cb, 10)
        # 100 Hz 타이머로 플롯 업데이트
        self.create_timer(0.01, self._on_timer)

    def _interpolated_path_cb(self, msg: PoseArray):
        """보간된 경로 데이터 처리"""
        global _interpolated_path_x, _interpolated_path_y, _interpolated_path_received
        
        with _robot_lock:
            _interpolated_path_x = []
            _interpolated_path_y = []
            
            for pose in msg.poses:
                _interpolated_path_x.append(pose.position.x)
                _interpolated_path_y.append(pose.position.y)
            
            _interpolated_path_received = True
            self.get_logger().info(f"📡 보간된 경로 수신: {len(_interpolated_path_x)}개 점")

    def _path_segments_cb(self, msg: String):
        """구간 정보 데이터 처리"""
        global _path_segments, _path_length
        
        try:
            import json
            segments_data = json.loads(msg.data)
            _path_segments = segments_data.get('segments', [])
            _path_length = segments_data.get('path_length', 0)
            self.get_logger().info(f"📡 구간 정보 수신: {len(_path_segments)}개 구간")
            
            # 구간별 상세 정보 출력 (version 정보 포함)
            for i, seg in enumerate(_path_segments):
                version = seg.get('version', 1)
                self.get_logger().info(f"   구간 {i+1}: {seg['type']}{version} (waypoint {seg['waypoint_start']}-{seg['waypoint_end']}, "
                                     f"경로 {seg['start_index']}-{seg['end_index']}, 거리: {seg['segment_distance']:.2f}m)")
        except Exception as e:
            self.get_logger().warn(f"구간 정보 파싱 오류: {e}")

    def _on_timer(self):
        # 종료 신호 확인
        if _shutdown_event.is_set():
            raise KeyboardInterrupt("Shutdown requested")
        # 주기적으로 오버레이 업데이트
        _update_path_overlay()

# --- 수정된 부분 시작 ---
# data_string 변수를 삭제하고, 파일에서 직접 데이터를 로드합니다.
input_filename = INPUT_WAYPOINTS_FILE

def _detect_delimiter_and_cols(path):
    delim = None
    num_cols = None
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            if ',' in s:
                delim = ','
                tokens = [t for t in s.split(',') if t.strip() != '']
            else:
                tokens = s.split()
            num_cols = len(tokens)
            break
    if num_cols is None:
        raise ValueError('유효한 데이터 라인이 없습니다.')
    return delim, num_cols

try:
    delimiter, col_count = _detect_delimiter_and_cols(input_filename)
    if col_count < 2:
        raise ValueError('최소 2개 컬럼(x,y)이 필요합니다.')
    coords = np.loadtxt(input_filename, usecols=(0, 1), delimiter=delimiter)
    x_coords = coords[:, 0]
    y_coords = coords[:, 1]
    print(f"'{input_filename}' 파일에서 {len(x_coords)}개의 웨이포인트를 성공적으로 불러왔습니다.")
except FileNotFoundError:
    print(f"오류: '{input_filename}' 파일을 찾을 수 없습니다.")
    print("스크립트와 같은 경로에 파일이 있는지 확인하세요.")
    exit() # 파일이 없으면 프로그램 종료
except Exception as e:
    print(f"파일을 읽는 중 오류가 발생했습니다: {e}")
    exit() # 그 외 오류 발생 시 종료
# --- 수정된 부분 끝 ---

# 2. Matplotlib으로 그래프 설정
fig, ax = plt.subplots(figsize=(12, 10))
sc = ax.scatter(x_coords, y_coords, c=np.arange(len(x_coords)), cmap='viridis', picker=True, pickradius=5)

# 첫 번째 점(index 0)과 마지막 점을 강조 표시
ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start (Index 0)')
ax.plot(x_coords[-1], y_coords[-1], 'rs', markersize=10, label=f'End (Index {len(x_coords)-1})')

# 3. 주석(Annotation) 설정
annot = ax.annotate("", xy=(0,0), xytext=(20,20),
                    textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="yellow", alpha=0.7),
                    arrowprops=dict(arrowstyle="->"))
annot.set_visible(False)

# 4. 마우스 호버(hover) 및 클릭/패닝 이벤트 처리 함수 정의
is_panning = False
_pan_start = None          # (xdata, ydata)
_pan_start_disp = None     # (x, y) in pixels
_pan_xlim0 = None
_pan_ylim0 = None
_pan_dragged = False
_pan_threshold_px = 5

def update_annot(ind):
    """포인트 주석의 위치와 텍스트를 업데이트"""
    point_index = ind["ind"][0]
    pos = sc.get_offsets()[point_index]
    annot.xy = pos
    annot.set_text(f"Index: {point_index}\nX: {pos[0]:.2f}\nY: {pos[1]:.2f}")

def on_scroll(event):
    """마우스 휠로 확대/축소"""
    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return
    cur_xlim = ax.get_xlim()
    cur_ylim = ax.get_ylim()
    xdata, ydata = event.xdata, event.ydata
    zoom_factor = 1.2
    if event.button == 'up':
        scale = 1.0 / zoom_factor
    elif event.button == 'down':
        scale = zoom_factor
    else:
        return
    new_w = (cur_xlim[1] - cur_xlim[0]) * scale
    new_h = (cur_ylim[1] - cur_ylim[0]) * scale
    relx = (xdata - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0] + 1e-12)
    rely = (ydata - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0] + 1e-12)
    ax.set_xlim(xdata - new_w * relx, xdata + new_w * (1 - relx))
    ax.set_ylim(ydata - new_h * rely, ydata + new_h * (1 - rely))
    fig.canvas.draw_idle()

def on_pan_press(event):
    """오른쪽 버튼으로 드래그하여 패닝 시작"""
    global is_panning, _pan_start, _pan_start_disp, _pan_xlim0, _pan_ylim0, _pan_dragged
    if event.inaxes != ax or event.button not in (1, 3):
        return
    is_panning = True
    _pan_start = (event.xdata, event.ydata)
    _pan_start_disp = (event.x, event.y)
    _pan_xlim0 = ax.get_xlim()
    _pan_ylim0 = ax.get_ylim()
    _pan_dragged = False

def on_pan_release(event):
    """패닝 종료"""
    global is_panning, _pan_start, _pan_start_disp, _pan_xlim0, _pan_ylim0, _pan_dragged
    if event.button not in (1, 3):
        return
    # 좌클릭 드래그가 아니고 이동이 거의 없었다면 클릭 처리 (점 말풍선)
    if event.button == 1 and not _pan_dragged and event.inaxes == ax:
        contains, ind = sc.contains(event)
        if contains:
            update_annot(ind)
            annot.set_visible(True)
            fig.canvas.draw_idle()
        else:
            if annot.get_visible():
                annot.set_visible(False)
                fig.canvas.draw_idle()
    is_panning = False
    _pan_start = None
    _pan_start_disp = None
    _pan_xlim0 = None
    _pan_ylim0 = None
    _pan_dragged = False

def on_pan_motion(event):
    """패닝 중 마우스 이동 처리"""
    global _pan_dragged
    if not is_panning or event.inaxes != ax or event.xdata is None or event.ydata is None:
        return
    # 드래그 판정 (픽셀 단위)
    if _pan_start_disp is not None:
        dxp = event.x - _pan_start_disp[0]
        dyp = event.y - _pan_start_disp[1]
        if abs(dxp) > _pan_threshold_px or abs(dyp) > _pan_threshold_px:
            _pan_dragged = True
    dx = event.xdata - _pan_start[0]
    dy = event.ydata - _pan_start[1]
    ax.set_xlim(_pan_xlim0[0] - dx, _pan_xlim0[1] - dx)
    ax.set_ylim(_pan_ylim0[0] - dy, _pan_ylim0[1] - dy)
    fig.canvas.draw_idle()

# 5. 이벤트 핸들러를 그래프에 연결
fig.canvas.mpl_connect("scroll_event", on_scroll)
fig.canvas.mpl_connect("button_press_event", on_pan_press)
fig.canvas.mpl_connect("button_release_event", on_pan_release)
fig.canvas.mpl_connect("motion_notify_event", on_pan_motion)

# 6. 그래프 스타일링 및 표시
ax.set_title("UTM Waypoints Visualization (Test Mode - No Robot Position)")
ax.set_xlabel("UTM X (Easting)")
ax.set_ylabel("UTM Y (Northing)")
ax.set_aspect('equal', adjustable='box')
ax.grid(True)
ax.legend()
plt.colorbar(sc, label='Point Order Index')
plt.tight_layout()

# 경로 시각화를 위한 변수들
_interpolated_path_plot = None
_segment_plots = {}  # 동적으로 관리되는 구간별 플롯들

def _update_path_overlay():
    """경로 오버레이 업데이트 (로봇 위치 제외)"""
    global _interpolated_path_plot, _segment_plots
    with _robot_lock:
        interp_x, interp_y = _interpolated_path_x.copy(), _interpolated_path_y.copy()
        interp_received = _interpolated_path_received
        segments = _path_segments.copy()
        path_length = _path_length
    changed = False
    
    # 보간된 경로 표시 (구간별로 구분 - s1-s5, c1-c3, r1-r2)
    if interp_received and len(interp_x) > 0 and len(segments) > 0:
        # 세분화된 구간별로 분리
        s1_x, s1_y = [], []  # 직선1
        s2_x, s2_y = [], []  # 직선2
        s3_x, s3_y = [], []  # 직선3
        s4_x, s4_y = [], []  # 직선4
        s5_x, s5_y = [], []  # 직선5
        c1_x, c1_y = [], []  # 곡선1
        c2_x, c2_y = [], []  # 곡선2
        c3_x, c3_y = [], []  # 곡선3
        r1_x, r1_y = [], []  # 후진1
        r2_x, r2_y = [], []  # 후진2
        unclassified_x, unclassified_y = [], []
        
        # 모든 경로 점을 추적하기 위한 배열
        classified_indices = set()
        
        for segment in segments:
            start_idx = segment['start_index']
            end_idx = segment['end_index']
            segment_type = segment['type']
            version = segment.get('version', 1)  # version 정보 가져오기
            
            # 경로 인덱스가 유효한 범위인지 확인
            if start_idx < len(interp_x) and end_idx < len(interp_x) and start_idx <= end_idx:
                # 유효한 인덱스 범위로 조정
                start_idx = max(0, start_idx)
                end_idx = min(len(interp_x) - 1, end_idx)
                
                segment_x = interp_x[start_idx:end_idx+1]
                segment_y = interp_y[start_idx:end_idx+1]
                
                # 구간 타입과 version에 따라 분류
                if segment_type == 'straight':
                    if version == 1:
                        s1_x.extend(segment_x)
                        s1_y.extend(segment_y)
                    elif version == 2:
                        s2_x.extend(segment_x)
                        s2_y.extend(segment_y)
                    elif version == 3:
                        s3_x.extend(segment_x)
                        s3_y.extend(segment_y)
                    elif version == 4:
                        s4_x.extend(segment_x)
                        s4_y.extend(segment_y)
                    elif version == 5:
                        s5_x.extend(segment_x)
                        s5_y.extend(segment_y)
                elif segment_type == 'curve':
                    if version == 1:
                        c1_x.extend(segment_x)
                        c1_y.extend(segment_y)
                    elif version == 2:
                        c2_x.extend(segment_x)
                        c2_y.extend(segment_y)
                    elif version == 3:
                        c3_x.extend(segment_x)
                        c3_y.extend(segment_y)
                elif segment_type == 'reverse':
                    if version == 1:
                        r1_x.extend(segment_x)
                        r1_y.extend(segment_y)
                    elif version == 2:
                        r2_x.extend(segment_x)
                        r2_y.extend(segment_y)
                
                # 분류된 인덱스 기록
                for i in range(start_idx, end_idx + 1):
                    classified_indices.add(i)
        
        # 분류되지 않은 점들 찾기
        for i in range(len(interp_x)):
            if i not in classified_indices:
                unclassified_x.append(interp_x[i])
                unclassified_y.append(interp_y[i])
        
        # 각 구간별로 표시 (다른 색상과 크기 사용)
        segment_plots = [
            (s1_x, s1_y, 'blue', 8, 'S1 (Straight1)'),
            (s2_x, s2_y, 'lightblue', 8, 'S2 (Straight2)'),
            (s3_x, s3_y, 'cyan', 8, 'S3 (Straight3)'),
            (s4_x, s4_y, 'teal', 8, 'S4 (Straight4)'),
            (s5_x, s5_y, 'darkblue', 8, 'S5 (Straight5)'),
            (c1_x, c1_y, 'red', 10, 'C1 (Curve1)'),
            (c2_x, c2_y, 'orange', 10, 'C2 (Curve2)'),
            (c3_x, c3_y, 'darkred', 10, 'C3 (Curve3)'),
            (r1_x, r1_y, 'purple', 12, 'R1 (Reverse1)'),
            (r2_x, r2_y, 'magenta', 12, 'R2 (Reverse2)')
        ]
        
        # 기존 플롯 변수들 초기화 (동적으로 관리)
        global _segment_plots
        if '_segment_plots' not in globals():
            _segment_plots = {}
        
        for i, (x_data, y_data, color, size, label) in enumerate(segment_plots):
            if len(x_data) > 0:
                plot_key = f'segment_{i}'
                if plot_key not in _segment_plots:
                    _segment_plots[plot_key] = ax.scatter(x_data, y_data, c=color, s=size, alpha=0.8, label=label)
                    changed = True
                else:
                    _segment_plots[plot_key].set_offsets(np.column_stack((x_data, y_data)))
                    changed = True
        
        # 분류되지 않은 구간 표시 (노란색 점)
        if len(unclassified_x) > 0:
            if _interpolated_path_plot is None:
                _interpolated_path_plot = ax.scatter(unclassified_x, unclassified_y, c='yellow', s=8, alpha=0.8, label='Unclassified Segments')
                changed = True
            else:
                _interpolated_path_plot.set_offsets(np.column_stack((unclassified_x, unclassified_y)))
                changed = True
    elif interp_received and len(interp_x) > 0:
        # 구간 정보가 없는 경우 기본 표시
        if _interpolated_path_plot is None:
            _interpolated_path_plot = ax.scatter(interp_x, interp_y, c='orange', s=8, alpha=0.8, label='Interpolated Path')
            changed = True
        else:
            _interpolated_path_plot.set_offsets(np.column_stack((interp_x, interp_y)))
            changed = True
    
    if changed:
        try:
            ax.legend()
        except Exception:
            pass
        fig.canvas.draw_idle()

def _start_ros_thread_if_available():
    if not RCLPY_AVAILABLE:
        print('[INFO] ROS2(rclpy) 미탑재: 경로 오버레이 비활성화')
        return
    def runner():
        try:
            rclpy.init()
            node = _PoseListener()
            exec_ = rclpy.executors.SingleThreadedExecutor()
            exec_.add_node(node)
            print('[INFO] ROS2 경로 오버레이 활성화: 100 Hz 업데이트')
            
            # 논블로킹 방식으로 spin
            while not _shutdown_event.is_set():
                exec_.spin_once(timeout_sec=0.1)
            
            print('[INFO] ROS2 스레드 종료 중...')
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f'[INFO] ROS2 스레드 종료: {e}')
    th = threading.Thread(target=runner, daemon=True)
    th.start()
    return th

# ROS2 스레드 시작
ros_thread = _start_ros_thread_if_available()

def cleanup_and_exit():
    """정리 및 종료 함수"""
    print("경로 시각화 프로그램 종료 중...")
    
    # ROS2 스레드 종료 신호
    _shutdown_event.set()
    
    # ROS2 스레드가 존재하면 종료 대기
    if ros_thread is not None:
        ros_thread.join(timeout=2.0)  # 최대 2초 대기
        if ros_thread.is_alive():
            print("[WARNING] ROS2 스레드가 정상 종료되지 않았습니다")
    
    # matplotlib 종료
    try:
        plt.close('all')
    except Exception:
        pass

try:
    plt.show()
except KeyboardInterrupt:
    cleanup_and_exit()
finally:
    cleanup_and_exit()
