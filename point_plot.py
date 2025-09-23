import matplotlib.pyplot as plt
import matplotlib
from matplotlib.widgets import Button, TextBox
import numpy as np
import io
import math
import threading
import re
import time
import os
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
INPUT_WAYPOINTS_FILE = 'remapped_utmcoordinates_ver1.txt'  # 시각화할 waypoint 파일
# INPUT_WAYPOINTS_FILE = 'waypoints/full_wp_ver1.txt'

# =============================================================================

# 한글 폰트 설정
matplotlib.rcParams['font.family'] = 'Malgun Gothic'
# 마이너스 부호 깨짐 방지
matplotlib.rcParams['axes.unicode_minus'] = False
# ROS2에서 수신한 로봇 상태 (스레드간 공유)
_robot_lock = threading.Lock()
_robot_x = None
_robot_y = None

# ROS2 종료 신호용
_shutdown_event = threading.Event()

# 로봇 경로 추적을 위한 변수들
_robot_path_x = []
_robot_path_y = []
_robot_path_timestamps = []
_path_update_interval = 0.5  # 0.5초마다 경로점 추가
_last_path_update = 0.0

# 보간된 경로를 위한 변수들
_interpolated_path_x = []
_interpolated_path_y = []
_interpolated_path_received = False

# 구간 정보를 위한 변수들
_path_segments = []
_path_length = 0

# IMU/헤딩 오버레이 상태
_imu_yaw_rad = None              # IMU에서 받은 현재 yaw (rad, [-pi, pi])
_heading_ref_set = False         # 기준(0도) 설정 여부
_heading_ref_angle_rad = None    # 초기 진행 방향 (rad)
_heading_quiver = None           # 현재 헤딩 화살표 핸들
_heading_text = None             # 현재 헤딩 텍스트 핸들
_heading_arrow_len = 0.5         # 화살표 길이 (m)

def _normalize_angle_rad(angle):
    """[-pi, pi] 범위로 정규화"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def _yaw_from_quaternion(x, y, z, w):
    """쿼터니언 → yaw(rad)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)



class _PoseListener(Node):
    def __init__(self):
        super().__init__('waypoint_pose_listener')
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self._gps_cb, 10)
        self.create_subscription(PoseArray, '/interpolated_path', self._interpolated_path_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(String, '/path_segments', self._path_segments_cb, 10)
        # 100 Hz 타이머로 플롯 업데이트
        self.create_timer(0.01, self._on_timer)


    def _gps_cb(self, msg: NavSatFix):
        # GPS 데이터 수신 비활성화 - waypoint와 보간된 경로만 표시
        pass
        global _robot_x, _robot_y, _robot_path_x, _robot_path_y, _robot_path_timestamps, _last_path_update, _heading_ref_set, _heading_ref_angle_rad
        
        # GPS 좌표가 유효한지 확인
        if msg.status.status < 0:  # STATUS_NO_FIX
            return
            
        # lat, lon을 UTM으로 변환
        try:
            utm_x, utm_y, zone_num, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
            current_time = time.time()
            
            with _robot_lock:
                _robot_x = float(utm_x)
                _robot_y = float(utm_y)
                
                # 경로점 추가 (일정 간격으로)
                if current_time - _last_path_update >= _path_update_interval:
                    _robot_path_x.append(_robot_x)
                    _robot_path_y.append(_robot_y)
                    _robot_path_timestamps.append(current_time)
                    _last_path_update = current_time
                    
                    # 경로점이 너무 많아지면 오래된 것부터 제거 (최대 1000개)
                    if len(_robot_path_x) > 1000:
                        _robot_path_x.pop(0)
                        _robot_path_y.pop(0)
                        _robot_path_timestamps.pop(0)
                # 초기 진행 방향 기준 설정 (최초 0.3m 이동 시)
                if not _heading_ref_set and len(_robot_path_x) >= 2:
                    x0, y0 = _robot_path_x[0], _robot_path_y[0]
                    dx0 = _robot_x - x0
                    dy0 = _robot_y - y0
                    dist0 = math.hypot(dx0, dy0)
                    if dist0 >= 0.3:
                        _heading_ref_angle_rad = math.atan2(dy0, dx0)
                        _heading_ref_set = True
                        self.get_logger().info(f"[Heading] 기준 각도 설정 완료: {_heading_ref_angle_rad:.3f} rad ({math.degrees(_heading_ref_angle_rad):.1f}°)")
        except Exception as e:
            self.get_logger().warn(f"GPS to UTM 변환 오류: {e}")

    def _imu_cb(self, msg: Imu):
        # IMU 데이터 수신 
        pass
        global _imu_yaw_rad
        try:
            yaw = _yaw_from_quaternion(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
            _imu_yaw_rad = _normalize_angle_rad(yaw)
        except Exception as e:
            self.get_logger().warn(f"IMU yaw 파싱 오류: {e}")

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
            
            # 구간별 상세 정보 출력
            for i, seg in enumerate(_path_segments):
                self.get_logger().info(f"   구간 {i+1}: {seg['type']} (waypoint {seg['waypoint_start']}-{seg['waypoint_end']}, "
                                     f"경로 {seg['start_index']}-{seg['end_index']}, 거리: {seg['segment_distance']:.2f}m)")
        except Exception as e:
            self.get_logger().warn(f"구간 정보 파싱 오류: {e}")

    def _on_timer(self):
        # 종료 신호 확인
        if _shutdown_event.is_set():
            raise KeyboardInterrupt("Shutdown requested")
        # 주기적으로 오버레이 업데이트
        _update_robot_overlay()

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


## 헤딩 오버레이(세그먼트 화살표/회전각) 제거됨

# 3. 주석(Annotation) 설정
annot = ax.annotate("", xy=(0,0), xytext=(20,20),
                    textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="yellow", alpha=0.7),
                    arrowprops=dict(arrowstyle="->"))
annot.set_visible(False)

## 헤딩 오버레이용 애노테이션 제거됨

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

## 헤딩 오버레이 업데이트 함수 제거됨

## 헤딩 오버레이 호버 핸들러 제거됨

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
            # 선택 모드일 경우 선택 토글
            if _select_mode:
                idx = ind["ind"][0]
                if idx in _selected_indices:
                    _selected_indices.remove(idx)
                else:
                    _selected_indices.add(idx)
                _update_selected_overlay()
            # 주석 표시 갱신
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
ax.set_title("UTM Waypoints Visualization (from file)")
ax.set_xlabel("UTM X (Easting)")
ax.set_ylabel("UTM Y (Northing)")
ax.set_aspect('equal', adjustable='box')
ax.grid(True)
ax.legend()
plt.colorbar(sc, label='Point Order Index')
plt.tight_layout()

# 선택 모드/저장 UI 및 선택 오버레이 전역
_select_mode = False
_selected_indices = set()
_selected_plot = None
_btn_select = None
_btn_save = None
_txt_filename = None

def _update_selected_overlay():
    """선택된 waypoint를 강조 표시/업데이트"""
    global _selected_plot
    if len(_selected_indices) == 0:
        # 선택이 없으면 오버레이 비우기
        if _selected_plot is not None:
            try:
                _selected_plot.set_offsets(np.empty((0, 2)))
            except Exception:
                try:
                    _selected_plot.remove()
                except Exception:
                    pass
                _selected_plot = None
        try:
            ax.legend()
        except Exception:
            pass
        fig.canvas.draw_idle()
        return
    sel_x = [x_coords[i] for i in sorted(_selected_indices)]
    sel_y = [y_coords[i] for i in sorted(_selected_indices)]
    if _selected_plot is None:
        _selected_plot = ax.scatter(sel_x, sel_y, s=60, facecolors='none', edgecolors='cyan', linewidths=1.5, label='Selected')
    else:
        _selected_plot.set_offsets(np.column_stack((sel_x, sel_y)))
    try:
        ax.legend()
    except Exception:
        pass
    fig.canvas.draw_idle()

def _on_select_button_clicked(event):
    """선택 모드 토글"""
    global _select_mode
    _select_mode = not _select_mode
    if _select_mode:
        _btn_select.label.set_text('선택 중')
        try:
            _btn_select.color = 'lightgreen'
            _btn_select.hovercolor = 'green'
        except Exception:
            pass
    else:
        _btn_select.label.set_text('선택하기')
        try:
            _btn_select.color = '0.85'
            _btn_select.hovercolor = '0.95'
        except Exception:
            pass
    fig.canvas.draw_idle()

def _on_save_button_clicked(event):
    """선택된 waypoint를 파일로 저장"""
    if len(_selected_indices) == 0:
        print('[INFO] 저장할 선택된 waypoint가 없습니다.')
        return
    # 파일명 가져오기
    filename = _txt_filename.text.strip() if _txt_filename is not None else ''
    if filename == '':
        filename = f"selected_{os.path.basename(input_filename)}"
    # 디렉토리 보장 X: 상대 경로로 저장
    sep = delimiter if delimiter is not None else ' '
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            for i in sorted(_selected_indices):
                x = float(x_coords[i])
                y = float(y_coords[i])
                if sep == ',':
                    f.write(f"{x:.6f},{y:.6f}\n")
                else:
                    f.write(f"{x:.6f} {y:.6f}\n")
        print(f"[INFO] {len(_selected_indices)}개 waypoint를 '{filename}'로 저장했습니다.")
    except Exception as e:
        print(f"[ERROR] 파일 저장 실패: {e}")

def _init_selection_ui():
    """하단에 선택/저장 UI 구성"""
    global _btn_select, _btn_save, _txt_filename
    try:
        btn_ax = fig.add_axes([0.12, 0.01, 0.1, 0.045])
        _btn_select = Button(btn_ax, '선택하기')
        _btn_select.on_clicked(_on_select_button_clicked)
        txt_ax = fig.add_axes([0.25, 0.01, 0.38, 0.045])
        default_name = f"selected_{os.path.basename(input_filename)}"
        _txt_filename = TextBox(txt_ax, '파일명: ', initial=default_name)
        save_ax = fig.add_axes([0.65, 0.01, 0.1, 0.045])
        _btn_save = Button(save_ax, '저장')
        _btn_save.on_clicked(_on_save_button_clicked)
    except Exception as e:
        print(f"[WARNING] UI 초기화 실패: {e}")

_init_selection_ui()

# ROS2로부터 현재 로봇 위치/헤딩을 주기적으로 오버레이 (가능한 경우)
_robot_plot = None
_robot_path_plot = None
_interpolated_path_plot = None
_interpolated_path_straight_plot = None
_interpolated_path_curve_plot = None

def _update_robot_overlay():
    global _robot_plot, _robot_path_plot, _interpolated_path_plot, _interpolated_path_straight_plot, _interpolated_path_curve_plot, _heading_quiver, _heading_text
    with _robot_lock:
        # GPS 관련 변수들
        rx, ry = _robot_x, _robot_y
        path_x, path_y = _robot_path_x.copy(), _robot_path_y.copy()
        interp_x, interp_y = _interpolated_path_x.copy(), _interpolated_path_y.copy()
        interp_received = _interpolated_path_received
        segments = _path_segments.copy()
        path_length = _path_length
        imu_yaw = _imu_yaw_rad
        ref_set = _heading_ref_set
        ref_angle = _heading_ref_angle_rad
    changed = False
    
    # 로봇 경로 표시 
    if len(path_x) > 0:
        if _robot_path_plot is None:
            _robot_path_plot = ax.plot(path_x, path_y, 'b-', linewidth=1, alpha=0.7, label='Robot Path')[0]
            changed = True
        else:
            _robot_path_plot.set_data(path_x, path_y)
            changed = True
    
    # 보간된 경로 표시 (구간별로 구분)
    if interp_received and len(interp_x) > 0 and len(segments) > 0:
        # 직선 구간과 곡선 구간을 분리
        straight_x, straight_y = [], []
        curve_x, curve_y = [], []
        unclassified_x, unclassified_y = [], []
        
        # 모든 경로 점을 추적하기 위한 배열
        classified_indices = set()
        
        for segment in segments:
            start_idx = segment['start_index']
            end_idx = segment['end_index']
            segment_type = segment['type']
            
            # 경로 인덱스가 유효한 범위인지 확인
            if start_idx < len(interp_x) and end_idx < len(interp_x) and start_idx <= end_idx:
                # 유효한 인덱스 범위로 조정
                start_idx = max(0, start_idx)
                end_idx = min(len(interp_x) - 1, end_idx)
                
                if segment_type == 'straight':
                    straight_x.extend(interp_x[start_idx:end_idx+1])
                    straight_y.extend(interp_y[start_idx:end_idx+1])
                else:  # curve
                    curve_x.extend(interp_x[start_idx:end_idx+1])
                    curve_y.extend(interp_y[start_idx:end_idx+1])
                
                # 분류된 인덱스 기록
                for i in range(start_idx, end_idx + 1):
                    classified_indices.add(i)
        
        # 분류되지 않은 점들 찾기
        for i in range(len(interp_x)):
            if i not in classified_indices:
                unclassified_x.append(interp_x[i])
                unclassified_y.append(interp_y[i])
        
        # 직선 구간 표시 (파란색 점)
        if len(straight_x) > 0:
            if _interpolated_path_straight_plot is None:
                _interpolated_path_straight_plot = ax.scatter(straight_x, straight_y, c='blue', s=8, alpha=0.8, label='Straight Segments')
                changed = True
            else:
                _interpolated_path_straight_plot.set_offsets(np.column_stack((straight_x, straight_y)))
                changed = True
        
        # 곡선 구간 표시 (빨간색 점)
        if len(curve_x) > 0:
            if _interpolated_path_curve_plot is None:
                _interpolated_path_curve_plot = ax.scatter(curve_x, curve_y, c='red', s=8, alpha=0.8, label='Curve Segments')
                changed = True
            else:
                _interpolated_path_curve_plot.set_offsets(np.column_stack((curve_x, curve_y)))
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
    
    # 현재 로봇 위치 표시
    if rx is not None and ry is not None:
        if _robot_plot is None:
            _robot_plot = ax.plot(rx, ry, 'bo', markersize=8, label='Robot')[0]
            changed = True
        else:
            _robot_plot.set_data([rx], [ry])
            changed = True
        # 헤딩 오버레이 (초기 0.3m 이동 방향을 0도로, IMU yaw 반영: 좌 +, 우 -)
        if ref_set:
            yaw = imu_yaw if imu_yaw is not None else 0.0
            theta_world = ref_angle + yaw
            dxh = math.cos(theta_world) * _heading_arrow_len
            dyh = math.sin(theta_world) * _heading_arrow_len
            if _heading_quiver is None:
                _heading_quiver = ax.quiver([rx], [ry], [dxh], [dyh], angles='xy', scale_units='xy', scale=1, color='blue', width=0.003, label='Heading')
                changed = True
            else:
                try:
                    _heading_quiver.set_offsets(np.array([[rx, ry]]))
                    _heading_quiver.set_UVC(np.array([dxh]), np.array([dyh]))
                except Exception:
                    # quiver 업데이트가 실패하면 재생성
                    try:
                        _heading_quiver.remove()
                    except Exception:
                        pass
                    _heading_quiver = ax.quiver([rx], [ry], [dxh], [dyh], angles='xy', scale_units='xy', scale=1, color='blue', width=0.003, label='Heading')
                changed = True
            # 텍스트(도 단위, 좌 + / 우 -)
            yaw_deg = math.degrees(yaw)
            text_str = f"Heading {yaw_deg:+.1f}°"
            tx = rx + dxh
            ty = ry + dyh
            if _heading_text is None:
                _heading_text = ax.text(tx, ty, text_str, color='blue', fontsize=9, bbox=dict(boxstyle='round', fc='white', alpha=0.7))
                changed = True
            else:
                _heading_text.set_position((tx, ty))
                _heading_text.set_text(text_str)
                changed = True
    
    if changed:
        try:
            ax.legend()
        except Exception:
            pass
        fig.canvas.draw_idle()

def _start_ros_thread_if_available():
    if not RCLPY_AVAILABLE:
        print('[INFO] ROS2(rclpy) 미탑재: 로봇 오버레이 비활성화')
        return
    def runner():
        try:
            rclpy.init()
            node = _PoseListener()
            exec_ = rclpy.executors.SingleThreadedExecutor()
            exec_.add_node(node)
            print('[INFO] ROS2 오버레이 활성화: 100 Hz 업데이트')
            
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
    print("시각화 프로그램 종료 중...")
    
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