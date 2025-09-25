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
import json
import colorsys
from matplotlib import colors as mcolors
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
    # 안전 스텁: rclpy 미사용 환경에서도 클래스 정의가 진행되도록 함
    class Node:  # type: ignore
        pass
    class NavSatFix:  # type: ignore
        pass
    class PoseArray:  # type: ignore
        pass
    class Imu:  # type: ignore
        pass
    class String:  # type: ignore
        pass
try:
    import contextily as ctx
    CONTEXTILY_AVAILABLE = True
except ImportError:
    CONTEXTILY_AVAILABLE = False

# =============================================================================
# 설정 섹션: 여기서 waypoint 파일 경로를 수정하세요
# =============================================================================

# 입력 waypoint 파일 경로 (다중 지원)
# 단일 파일만 사용하려면 INPUT_WAYPOINTS_FILES = [] 또는 주석 처리하고 INPUT_WAYPOINTS_FILE 사용
#INPUT_WAYPOINTS_FILES = ['full_wp_except_t_ll.txt','selected_ll_2.txt','selected_tl.txt']
#INPUT_WAYPOINTS_FILES = ['full_wp_except_t_ll.txt','selected_ll_1.txt','selected_tr.txt']
INPUT_WAYPOINTS_FILES = []  # 시각화할 waypoint 파일들
INPUT_WAYPOINTS_FILE = 'added_selected_ll_1.txt'  # INPUT_WAYPOINTS_FILES가 비어있을 경우 사용될 단일 파일

# 배경 위성사진을 위한 UTM 존 정보
UTM_ZONE_NUMBER = 52
UTM_ZONE_LETTER = 'S'

# =============================================================================
_OFFSET_CONFIG_FILE = 'map_offset.json'

# Font: use default safe font (no Korean-specific font required)
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
# Avoid unicode minus rendering issues
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
delimiter = None
_file_data = []
_multifile_enabled = False

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

def _load_waypoint_files():
    global _file_data, _multifile_enabled, delimiter, x_coords, y_coords, input_filename
    colors = ['tab:blue','tab:red','tab:green','tab:orange','tab:purple','tab:brown','tab:pink','tab:gray','tab:olive','tab:cyan']
    if isinstance(INPUT_WAYPOINTS_FILES, (list, tuple)) and len(INPUT_WAYPOINTS_FILES) > 0:
        _multifile_enabled = True
        _file_data.clear()
        for i, fname in enumerate(INPUT_WAYPOINTS_FILES):
            try:
                delim, col_count = _detect_delimiter_and_cols(fname)
                if col_count < 2:
                    print(f"경고: '{fname}' 파일에 최소 2개 컬럼(x,y)이 필요합니다.")
                    continue
                coords = np.loadtxt(fname, usecols=(0, 1), delimiter=delim)
                fx = coords[:, 0]
                fy = coords[:, 1]
                _file_data.append({
                    'filename': fname,
                    'x_coords': fx,
                    'y_coords': fy,
                    'color': colors[i % len(colors)],
                    'scatter': None,
                })
                print(f"'{fname}' 파일에서 {len(fx)}개의 웨이포인트를 성공적으로 불러왔습니다.")
            except FileNotFoundError:
                print(f"경고: '{fname}' 파일을 찾을 수 없습니다.")
            except Exception as e:
                print(f"경고: '{fname}' 파일을 읽는 중 오류가 발생했습니다: {e}")
        if len(_file_data) == 0:
            print("오류: 로드된 파일이 없습니다.")
            exit()
        # 첫 번째 파일을 기본 참조로 유지
        input_filename = _file_data[0]['filename']
        x_coords = _file_data[0]['x_coords']
        y_coords = _file_data[0]['y_coords']
    else:
        _multifile_enabled = False
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

_load_waypoint_files()
# --- 수정된 부분 끝 ---



# 2. Matplotlib으로 그래프 설정
fig, ax = plt.subplots(figsize=(12, 10))

if _multifile_enabled:
    # 여러 파일 스캐터 생성
    for fi, info in enumerate(_file_data):
        vx = info['x_coords']
        vy = info['y_coords']
        base_rgb = mcolors.to_rgb(info['color'])
        npts = len(vx)
        # 밝기(명도)를 인덱스가 커질수록 어둡게(진하게). 최소 명도 0.3 보장
        def _grad_color(i):
            r, g, b = base_rgb
            h, l, s = colorsys.rgb_to_hls(r, g, b)
            # l: 0(검정)~1(흰색). 인덱스 증가 시 l 감소
            frac = 0.0 if npts <= 1 else i / (npts - 1)
            l_i = max(0.3, l * (1.0 - 0.7 * frac))
            rr, gg, bb = colorsys.hls_to_rgb(h, l_i, s)
            return (rr, gg, bb, 0.85)
        colors_arr = [_grad_color(i) for i in range(npts)]
        # 같은 파일은 같은 색조, 인덱스 커질수록 진한 그라데이션
        info['scatter'] = ax.scatter(vx, vy, c=colors_arr, s=35, alpha=None, picker=True, pickradius=10, label=f"{os.path.basename(info['filename'])} ({len(vx)})")
        info['__colors_all'] = colors_arr  # 업데이트에 재사용
        if npts > 0:
            ax.plot(vx[0], vy[0], 'go', markersize=7, alpha=0.8)
            ax.plot(vx[-1], vy[-1], 'rs', markersize=7, alpha=0.8)
    # 기본 선택 타겟은 첫 파일로 유지하기 위해 sc를 첫 파일 스캐터로 둠
    sc = _file_data[0]['scatter']
else:
    # 단일 파일 스캐터
    sc = ax.scatter(
        x_coords,
        y_coords,
        c=np.arange(len(x_coords)),
        cmap='viridis',
        picker=True,
        pickradius=8,
    )
    # 첫 번째/마지막 강조
    if len(x_coords) > 0:
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

# 키보드 이동 관련
_map_offset_x = 0.0
_map_offset_y = 0.0
_key_move_step = 0.05 # 한 번 누를 때 이동할 거리 (미터)

# 배경 이미지 이동을 위한 전역 변수
_basemap_image_obj = None
_basemap_original_extent = None
_bg_offset_x = 0.0
_bg_offset_y = 0.0

def _load_map_offset():
    """저장된 지도 오프셋을 파일에서 불러옵니다."""
    global _bg_offset_x, _bg_offset_y
    if os.path.exists(_OFFSET_CONFIG_FILE):
        try:
            with open(_OFFSET_CONFIG_FILE, 'r') as f:
                data = json.load(f)
                _bg_offset_x = data.get('x', 0.0)
                _bg_offset_y = data.get('y', 0.0)
                print(f"[INFO] 불러온 지도 오프셋: x={_bg_offset_x:.3f}, y={_bg_offset_y:.3f}")
        except Exception as e:
            print(f"[WARNING] 지도 오프셋 파일 로드 실패: {e}")

def _save_map_offset():
    """현재 지도 오프셋을 파일에 저장합니다."""
    try:
        with open(_OFFSET_CONFIG_FILE, 'w') as f:
            data = {'x': _bg_offset_x, 'y': _bg_offset_y}
            json.dump(data, f, indent=4)
            print(f"[INFO] 현재 지도 오프셋을 '{_OFFSET_CONFIG_FILE}'에 저장했습니다.")
    except Exception as e:
        print(f"[WARNING] 지도 오프셋 저장 실패: {e}")

# 프로그램 시작 시 오프셋 불러오기
_load_map_offset()

def update_annot(ind):
    """포인트 주석의 위치와 text를 업데이트"""
    point_index = ind["ind"][0]
    pos = sc.get_offsets()[point_index]
    annot.xy = pos
    annot.set_text(f"Index: {point_index}\nX: {pos[0]:.2f}\nY: {pos[1]:.2f}")

def hover(event):
    """마우스 호버 이벤트 처리"""
    if event.inaxes == ax:
        # 다중 파일 모드에서 호버 처리
        if _multifile_enabled:
            for info in _file_data:
                sca = info.get('scatter')
                if sca is None:
                    continue
                try:
                    cont, ind = sca.contains(event)
                    if cont and len(ind.get("ind", [])) > 0:
                        point_index = ind["ind"][0]
                        pos = sca.get_offsets()[point_index]
                        annot.xy = pos
                        filename = os.path.basename(info['filename'])
                        annot.set_text(f"{point_index + 1}th point of {filename}\nX: {pos[0]:.2f}\nY: {pos[1]:.2f}")
                        annot.set_visible(True)
                        fig.canvas.draw_idle()
                        return
                except Exception:
                    continue
        else:
            # 단일 파일 모드
            cont, ind = sc.contains(event)
            if cont and len(ind.get("ind", [])) > 0:
                point_index = ind["ind"][0]
                pos = sc.get_offsets()[point_index]
                annot.xy = pos
                filename = os.path.basename(input_filename)
                annot.set_text(f"{point_index + 1}th point of {filename}\nX: {pos[0]:.2f}\nY: {pos[1]:.2f}")
                annot.set_visible(True)
                fig.canvas.draw_idle()
                return
        
        # 호버된 점이 없으면 말풍선 숨기기
        if annot.get_visible():
            annot.set_visible(False)
            fig.canvas.draw_idle()

def _pick_index_from_event(event, pixel_tol=12):
    """마우스 이벤트로부터 점 인덱스를 안정적으로 획득."""
    if _multifile_enabled:
        try:
            if event.x is None or event.y is None:
                return None
            # 1) contains 우선
            for info in _file_data:
                sca = info.get('scatter')
                if sca is None:
                    continue
                try:
                    contains, ind = sca.contains(event)
                except Exception:
                    contains, ind = (False, {})
                if contains and len(ind.get("ind", [])) > 0:
                    # 가장 앞의 것을 채택
                    local_idx = int(ind["ind"][0])
                    return (info['filename'], local_idx)
            # 2) 최근접 (픽셀 거리)
            best = None
            best_d = float('inf')
            for info in _file_data:
                sca = info.get('scatter')
                if sca is None:
                    continue
                offsets = sca.get_offsets()
                if offsets is None or len(offsets) == 0:
                    continue
                pts = ax.transData.transform(offsets)
                dx = pts[:,0] - event.x
                dy = pts[:,1] - event.y
                d2 = dx*dx + dy*dy
                i = int(np.argmin(d2))
                dist = math.sqrt(float(d2[i]))
                if dist < best_d and dist <= pixel_tol:
                    key = (info['filename'], i)
                    if key not in _deleted_original_indices:
                        best_d = dist
                        best = key
            return best
        except Exception:
            return None
    else:
        # 기존 단일 파일 로직
        try:
            contains, ind = sc.contains(event)
            if contains and len(ind.get("ind", [])) > 0:
                visible_idx = ind["ind"][0]
                if 'visible_indices' in globals() and visible_idx < len(visible_indices):
                    return visible_indices[visible_idx]
                else:
                    return visible_idx
        except Exception:
            pass
        try:
            if event.x is None or event.y is None:
                return None
            offsets = sc.get_offsets()
            if offsets is None or len(offsets) == 0:
                return None
            pts_disp = ax.transData.transform(offsets)
            dx = pts_disp[:, 0] - event.x
            dy = pts_disp[:, 1] - event.y
            d2 = dx * dx + dy * dy
            i = int(np.argmin(d2))
            min_dist = math.sqrt(float(d2[i]))
            if min_dist <= pixel_tol:
                if 'visible_indices' in globals() and i < len(visible_indices):
                    return visible_indices[i]
                else:
                    return i
        except Exception:
            return None
    return None

def _pick_created_index_from_event(event, pixel_tol=8):
    """생성된 점 스캐터에서 인덱스 선택."""
    global _created_sc
    try:
        if _created_sc is not None:
            contains, ind = _created_sc.contains(event)
            if contains and len(ind.get("ind", [])) > 0:
                return ind["ind"][0]
    except Exception:
        pass
    try:
        if _created_sc is None or event.x is None or event.y is None:
            return None
        offsets = _created_sc.get_offsets() if hasattr(_created_sc, 'get_offsets') else None
        if offsets is None or len(offsets) == 0:
            return None
        pts_disp = ax.transData.transform(offsets)
        dx = pts_disp[:, 0] - event.x
        dy = pts_disp[:, 1] - event.y
        d2 = dx * dx + dy * dy
        i = int(np.argmin(d2))
        min_dist = math.sqrt(float(d2[i]))
        if min_dist <= pixel_tol:
            return i
    except Exception:
        return None
    return None

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
    if event.inaxes != ax:
        return
    # 좌/우 버튼 공통으로 시작점은 기록하되, 패닝은 오른쪽 버튼(3)만 허용
    is_panning = (event.button == 3)
    _pan_start = (event.xdata, event.ydata)
    _pan_start_disp = (event.x, event.y)
    _pan_xlim0 = ax.get_xlim()
    _pan_ylim0 = ax.get_ylim()
    _pan_dragged = False

def on_pan_release(event):
    """패닝 종료"""
    global is_panning, _pan_start, _pan_start_disp, _pan_xlim0, _pan_ylim0, _pan_dragged
    global _select_mode, _range_mode, _range_first_index, _selected_indices
    global _created_points_x, _created_points_y, _created_sc, _selected_created_indices, _selection_order, _range_groups

    if event.button not in (1, 3):
        return
    
    left_dragged = False
    if event.button == 1 and _pan_start_disp is not None:
        dxp = event.x - _pan_start_disp[0]
        dyp = event.y - _pan_start_disp[1]
        if abs(dxp) > _pan_threshold_px or abs(dyp) > _pan_threshold_px:
            left_dragged = True

    if event.button == 1 and not left_dragged and event.inaxes == ax:
        if _add_mode:
            # ... (Add mode logic remains the same)
            j = _pick_created_index_from_event(event)
            if j is not None:
                _add_to_history({'type': 'remove_point', 'index': j, 'x': _created_points_x[j], 'y': _created_points_y[j]})
                if j in _selected_created_indices: _selected_created_indices.remove(j)
                new_order = []
                for t, v_key in _selection_order:
                    if t == 'new':
                        v = v_key
                        if v == j: continue
                        elif v > j: new_order.append((t, v - 1))
                        else: new_order.append((t, v))
                    else: new_order.append((t, v_key))
                _selection_order = new_order
                del _created_points_x[j]; del _created_points_y[j]
            else:
                if event.xdata is not None and event.ydata is not None:
                    new_idx = len(_created_points_x)
                    _add_to_history({'type': 'add_point','index': new_idx,'x': float(event.xdata),'y': float(event.ydata)})
                    _created_points_x.append(float(event.xdata))
                    _created_points_y.append(float(event.ydata))
            _update_created_scatter()
        else:
            # Selection logic unified for single and multi-file
            pick_result = _pick_index_from_event(event)
            created_pick_result = _pick_created_index_from_event(event)

            if pick_result is not None:
                # 다중 파일 모드와 단일 파일 모드 모두 처리
                if _multifile_enabled:
                    curr_file, idx = pick_result
                    key = (curr_file, idx)
                else:
                    idx = int(pick_result)
                    key = idx

                if _select_mode:
                    if key in _selected_indices:
                        _add_to_history({'type': 'deselect_orig', 'index': key})
                        _selected_indices.remove(key)
                        _selection_order.remove(('orig', key))
                    else:
                        _add_to_history({'type': 'select_orig', 'index': key})
                        _selected_indices.add(key)
                        _selection_order.append(('orig', key))
                elif _range_mode:
                    if _multifile_enabled:
                        if _range_first_index is None:
                            _range_first_index = key
                        else:
                            if _range_first_index[0] != curr_file:
                                _range_first_index = key # 파일이 다르면 시작점 재설정
                                return

                            i0 = min(_range_first_index[1], idx)
                            i1 = max(_range_first_index[1], idx)
                            range_indices = list(range(i0, i1 + 1))
                            
                            pre_selected = {i for f, i in _selected_indices if f == curr_file}
                            added_indices = []
                            for k in range_indices:
                                range_key = (curr_file, k)
                                if range_key not in _selected_indices:
                                    _selection_order.append(('orig', range_key))
                                    added_indices.append(k)
                                _selected_indices.add(range_key)
                            
                            _add_to_history({'type': 'range_select', 'file': curr_file, 'indices': range_indices, 'added_indices': added_indices})
                            _range_groups.append((curr_file, i0, i1))
                            _range_first_index = None
                    else:
                        # 단일 파일 모드: 인덱스만 사용
                        if _range_first_index is None:
                            _range_first_index = idx
                        else:
                            i0 = min(_range_first_index, idx)
                            i1 = max(_range_first_index, idx)
                            range_indices = list(range(i0, i1 + 1))
                            pre_selected = set(_selected_indices)
                            added_indices = []
                            for k in range_indices:
                                if k not in _selected_indices:
                                    _selection_order.append(('orig', k))
                                    added_indices.append(k)
                                _selected_indices.add(k)
                            _add_to_history({'type': 'range_select', 'file': os.path.basename(input_filename), 'indices': range_indices, 'added_indices': added_indices})
                            _range_groups.append((os.path.basename(input_filename), i0, i1))
                            _range_first_index = None
                
                _update_selected_overlay()
                # Update annotation...
                
            elif created_pick_result is not None:
                idx = created_pick_result
                if _select_mode:
                    if idx in _selected_created_indices:
                        _add_to_history({'type': 'deselect_new', 'index': idx})
                        _selected_created_indices.remove(idx)
                        _selection_order.remove(('new', idx))
                    else:
                        _add_to_history({'type': 'select_new', 'index': idx})
                        _selected_created_indices.add(idx)
                        _selection_order.append(('new', idx))
                _update_selected_overlay()
                # Update annotation...
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
    # 드래그 판정 (픽셀 단위). 임계치 초과 시에만 실제 패닝 수행
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

def on_key_press(event):
    """키보드 이벤트 처리 (화살표 키로 위성사진만 이동)"""
    global _bg_offset_x, _bg_offset_y

    # 이동 단계 설정 (뷰 너비의 0.1%) - 더 미세하게 조정
    move_step = (ax.get_xlim()[1] - ax.get_xlim()[0]) * 0.001

    changed = False
    if event.key == 'up':
        _bg_offset_y += move_step
        changed = True
    elif event.key == 'down':
        _bg_offset_y -= move_step
        changed = True
    elif event.key == 'left':
        _bg_offset_x -= move_step
        changed = True
    elif event.key == 'right':
        _bg_offset_x += move_step
        changed = True

    if changed:
        _apply_map_offset()

def _apply_map_offset():
    """현재 오프셋을 배경 지도에 적용합니다."""
    if _basemap_image_obj is None or _basemap_original_extent is None:
        return
    left, right, bottom, top = _basemap_original_extent
    new_extent = [
        left + _bg_offset_x,
        right + _bg_offset_x,
        bottom + _bg_offset_y,
        top + _bg_offset_y
    ]
    _basemap_image_obj.set_extent(new_extent)
    fig.canvas.draw_idle()

# 5. 이벤트 핸들러를 그래프에 연결
fig.canvas.mpl_connect("scroll_event", on_scroll)
fig.canvas.mpl_connect("button_press_event", on_pan_press)
fig.canvas.mpl_connect("button_release_event", on_pan_release)
fig.canvas.mpl_connect("motion_notify_event", on_pan_motion)
fig.canvas.mpl_connect("motion_notify_event", hover)
fig.canvas.mpl_connect("key_press_event", on_key_press)

# 6. 그래프 스타일링 및 표시
ax.set_title("UTM Waypoints Visualization (Multi-File)")
ax.set_xlabel("UTM X (Easting)")
ax.set_ylabel("UTM Y (Northing)")
ax.set_aspect('equal', adjustable='box')
ax.grid(True, linestyle='--', alpha=0.6) # 그리드 투명도 조절
ax.legend()
if not _multifile_enabled:
    plt.colorbar(sc, label='Point Order Index')

def _setup_view_and_basemap(ax, zone_num, zone_letter):
    """모든 점을 포함하도록 뷰를 설정하고 배경 지도 추가"""
    
    # 1. 모든 점의 경계 계산
    all_x = []
    all_y = []
    
    if _multifile_enabled:
        for info in _file_data:
            all_x.extend(info['x_coords'])
            all_y.extend(info['y_coords'])
    else:
        all_x.extend(x_coords)
        all_y.extend(y_coords)
    
    # 생성된 점들도 경계에 포함
    if '_created_points_x' in globals():
        all_x.extend(_created_points_x)
        all_y.extend(_created_points_y)

    if not all_x or not all_y:
        # 점이 없으면 아무것도 하지 않음
        return

    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    
    # 2. 뷰에 패딩 추가
    x_pad = (max_x - min_x) * 0.1
    y_pad = (max_y - min_y) * 0.1
    
    ax.set_xlim(min_x - x_pad, max_x + x_pad)
    ax.set_ylim(min_y - y_pad, max_y + y_pad)

    # 3. 배경 지도 추가
    if not CONTEXTILY_AVAILABLE:
        print("[INFO] contextily 라이브러리가 설치되지 않아 배경 지도를 표시할 수 없습니다.")
        print("       pip install contextily rasterio pyproj")
        return

    try:
        epsg_code = f"326{zone_num}" if zone_letter.upper() >= 'N' else f"327{zone_num}"
        utm_crs = f"EPSG:{epsg_code}"
        
        ctx.add_basemap(
            ax,
            crs=utm_crs,
            source=ctx.providers.Esri.WorldImagery, # 위성 사진을 기본으로
            zoom=18
        )
        global _basemap_image_obj, _basemap_original_extent
        if ax.images:
            _basemap_image_obj = ax.images[0]
            _basemap_original_extent = _basemap_image_obj.get_extent()
            _apply_map_offset() # 저장된 오프셋 적용
    except Exception as e:
        print(f"[ERROR] 배경 지도 로딩 실패 (Esri): {e}")
        try:
            # Esri 실패 시 Google Satellite로 다시 시도
            print("[INFO] Esri World Imagery 로딩 실패, Google Maps Satellite로 재시도합니다...")
            # Google Maps Satellite URL 직접 사용
            google_satellite_url = 'https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
            ctx.add_basemap(
                ax,
                crs=utm_crs,
                source=google_satellite_url,
                zoom=18
            )
            if ax.images:
                _basemap_image_obj = ax.images[0]
                _basemap_original_extent = _basemap_image_obj.get_extent()
                _apply_map_offset() # 저장된 오프셋 적용
        except Exception as e2:
            print(f"[ERROR] 배경 지도 로딩 실패 (Google): {e2}")

# 배경 지도 추가 및 뷰 설정
_setup_view_and_basemap(ax, UTM_ZONE_NUMBER, UTM_ZONE_LETTER)

plt.tight_layout()

# 선택 모드/저장 UI 및 선택 오버레이 전역
_select_mode = False
_selected_indices = set()
_selected_plot = None
_btn_select = None
_btn_save = None
_txt_filename = None
_btn_range = None
_btn_delete = None
_btn_undo = None
_btn_redo = None
_btn_save_all = None
_range_mode = False
_range_first_index = None
_range_groups = []

# 생성 점 추가/선택 관련 전역
_add_mode = False
_btn_add = None
_created_points_x = []
_created_points_y = []
_created_sc = None
_selected_created_indices = set()
_selected_created_plot = None
# 선택 클릭 순서 (Select 모드에서만 순서를 유지): ('orig', idx) 또는 ('new', idx)
_selection_order = []

# 삭제된 점들을 추적
_deleted_original_indices = set()  # 삭제된 원본 점들의 인덱스

# Undo/Redo 기능을 위한 히스토리
_operation_history = []  # [{'type': 'add_point', 'x': float, 'y': float, 'index': int}, ...]
_redo_history = []      # 취소된 동작들
_max_history_size = 50  # 최대 히스토리 크기

def _get_visible_original_points():
    """삭제되지 않은 원본 점들의 좌표와 인덱스 반환"""
    visible_x = []
    visible_y = []
    visible_indices = []
    for i, (x, y) in enumerate(zip(x_coords, y_coords)):
        if i not in _deleted_original_indices:
            visible_x.append(x)
            visible_y.append(y)
            visible_indices.append(i)
    return np.array(visible_x), np.array(visible_y), visible_indices

def _add_to_history(operation):
    """히스토리에 동작 추가"""
    global _operation_history, _redo_history, _max_history_size
    _operation_history.append(operation)
    _redo_history.clear()  # 새로운 동작 시 redo 히스토리 초기화
    
    # 히스토리 크기 제한
    if len(_operation_history) > _max_history_size:
        _operation_history.pop(0)

def _update_original_scatter():
    """원본 점 스캐터 업데이트 (삭제된 점들 제외)"""
    global sc
    if _multifile_enabled:
        for info in _file_data:
            vx_all = info['x_coords']
            vy_all = info['y_coords']
            npts = len(vx_all)
            # 색상 배열 준비(없으면 생성)
            if '__colors_all' not in info or info['__colors_all'] is None or len(info['__colors_all']) != npts:
                base_rgb = mcolors.to_rgb(info['color'])
                def _grad_color(i):
                    r, g, b = base_rgb
                    h, l, s = colorsys.rgb_to_hls(r, g, b)
                    frac = 0.0 if npts <= 1 else i / (npts - 1)
                    l_i = max(0.3, l * (1.0 - 0.7 * frac))
                    rr, gg, bb = colorsys.hls_to_rgb(h, l_i, s)
                    return (rr, gg, bb, 0.85)
                info['__colors_all'] = [_grad_color(i) for i in range(npts)]
            visible_x, visible_y, visible_colors = [], [], []
            for i, (x, y) in enumerate(zip(vx_all, vy_all)):
                if (info['filename'], i) not in _deleted_original_indices:
                    visible_x.append(x)
                    visible_y.append(y)
                    visible_colors.append(info['__colors_all'][i])
            if info['scatter'] is not None:
                if len(visible_x) > 0:
                    info['scatter'].set_offsets(np.column_stack((visible_x, visible_y)))
                    info['scatter'].set_facecolors(np.array(visible_colors))
                else:
                    info['scatter'].set_offsets(np.empty((0, 2)))
                    info['scatter'].set_facecolors(np.empty((0, 4)))
    else:
        visible_x, visible_y, visible_indices = _get_visible_original_points()
        if sc is not None:
            sc.set_offsets(np.column_stack((visible_x, visible_y)))
            # c 배열도 길이에 맞게 업데이트
            sc.set_array(np.arange(len(visible_x)))

    fig.canvas.draw_idle()

def _update_created_scatter():
    """생성된 점 스캐터 업데이트"""
    global _created_sc
    try:
        if len(_created_points_x) > 0:
            if _created_sc is None:
                _created_sc = ax.scatter(_created_points_x, _created_points_y, c='k', s=30, marker='o', alpha=0.9, label='Created')
            else:
                _created_sc.set_offsets(np.column_stack((_created_points_x, _created_points_y)))
        else:
            if _created_sc is not None:
                try:
                    _created_sc.set_offsets(np.empty((0, 2)))
                except Exception:
                    try:
                        _created_sc.remove()
                    except Exception:
                        pass
                    _created_sc = None
        try:
            ax.legend()
        except Exception:
            pass
        fig.canvas.draw_idle()
    except Exception:
        pass

def _update_selected_overlay(offset_update=False):
    """선택된 waypoint를 강조 표시/업데이트 (원본/생성 분리)"""
    global _selected_plot, _selected_created_plot
    has_orig = len(_selected_indices) > 0
    has_new = len(_selected_created_indices) > 0
    if not has_orig and not has_new:
        if _selected_plot is not None:
            _selected_plot.set_offsets(np.empty((0, 2)))
        if _selected_created_plot is not None:
            _selected_created_plot.set_offsets(np.empty((0, 2)))
        fig.canvas.draw_idle()
        return

    if has_orig:
        sel_x, sel_y = [], []
        if _multifile_enabled:
            for fname, idx in _selected_indices:
                info = next((f for f in _file_data if f['filename'] == fname), None)
                if info and idx < len(info['x_coords']):
                    sel_x.append(info['x_coords'][idx])
                    sel_y.append(info['y_coords'][idx])
        else:
            for idx in _selected_indices:
                if idx < len(x_coords):
                    sel_x.append(x_coords[idx])
                    sel_y.append(y_coords[idx])

        if _selected_plot is None:
            _selected_plot = ax.scatter(sel_x, sel_y, s=60, facecolors='none', edgecolors='cyan', linewidths=1.5, label='Selected (Original)')
        else:
            _selected_plot.set_offsets(np.column_stack((sel_x, sel_y)))
    else:
        if _selected_plot is not None:
            _selected_plot.set_offsets(np.empty((0, 2)))

    if has_new:
        sel_nx = [_created_points_x[i] for i in _selected_created_indices]
        sel_ny = [_created_points_y[i] for i in _selected_created_indices]
        if _selected_created_plot is None:
            _selected_created_plot = ax.scatter(sel_nx, sel_ny, s=60, facecolors='none', edgecolors='magenta', linewidths=1.5, label='Selected (Created)')
        else:
            _selected_created_plot.set_offsets(np.column_stack((sel_nx, sel_ny)))
    else:
        if _selected_created_plot is not None:
            _selected_created_plot.set_offsets(np.empty((0, 2)))

    if not offset_update:
        try:
            ax.legend()
        except Exception:
            pass
    fig.canvas.draw_idle()

def _on_select_button_clicked(event):
    """선택 모드 토글"""
    global _select_mode, _range_mode, _range_first_index, _add_mode, _selected_created_indices, _selection_order, _range_groups
    if _select_mode:
        # 활성 상태에서 재클릭: 전체 선택 초기화 및 모드 해제
        _selected_indices.clear()
        _selected_created_indices.clear()
        _selection_order.clear()
        _update_selected_overlay()
        _select_mode = False
        _btn_select.label.set_text('Select')
        try:
            _btn_select.color = '0.85'
            _btn_select.hovercolor = '0.95'
        except Exception:
            pass
    else:
        # 선택 모드 활성화, 범위 모드 비활성화
        _select_mode = True
        _range_mode = False
        _range_first_index = None
        _add_mode = False
        _btn_select.label.set_text('Selecting')
        try:
            _btn_select.color = 'lightgreen'
            _btn_select.hovercolor = 'green'
            if _btn_range is not None:
                _btn_range.color = '0.85'
                _btn_range.hovercolor = '0.95'
            if _btn_add is not None:
                _btn_add.color = '0.85'
                _btn_add.hovercolor = '0.95'
        except Exception:
            pass
    fig.canvas.draw_idle()

def _on_range_button_clicked(event):
    """범위로 선택 모드 토글"""
    global _range_mode, _select_mode, _range_first_index, _add_mode, _range_groups
    if _range_mode:
        # 활성 상태에서 재클릭: 전체 선택 초기화 및 모드 해제
        _selected_indices.clear()
        _update_selected_overlay()
        _range_mode = False
        _range_first_index = None
        _range_groups.clear()
        _btn_range.label.set_text('Select Range')
        try:
            _btn_range.color = '0.85'
            _btn_range.hovercolor = '0.95'
        except Exception:
            pass
    else:
        # 범위 모드 활성화, 선택 모드 비활성화
        _range_mode = True
        _select_mode = False
        _range_first_index = None
        _add_mode = False
        _btn_range.label.set_text('Selecting Range')
        try:
            _btn_range.color = 'lightgreen'
            _btn_range.hovercolor = 'green'
            if _btn_select is not None:
                _btn_select.color = '0.85'
                _btn_select.hovercolor = '0.95'
            if _btn_add is not None:
                _btn_add.color = '0.85'
                _btn_add.hovercolor = '0.95'
        except Exception:
            pass
    fig.canvas.draw_idle()

def _on_add_button_clicked(event):
    """사용자 지정 점 추가 모드 토글"""
    global _add_mode, _select_mode, _range_mode, _range_first_index
    if _add_mode:
        _add_mode = False
        _btn_add.label.set_text('Add Points')
        try:
            _btn_add.color = '0.85'
            _btn_add.hovercolor = '0.95'
        except Exception:
            pass
    else:
        _add_mode = True
        _select_mode = False
        _range_mode = False
        _range_first_index = None
        _btn_add.label.set_text('Adding')
        try:
            _btn_add.color = 'lightblue'
            _btn_add.hovercolor = 'deepskyblue'
            if _btn_select is not None:
                _btn_select.color = '0.85'
                _btn_select.hovercolor = '0.95'
            if _btn_range is not None:
                _btn_range.color = '0.85'
                _btn_range.hovercolor = '0.95'
        except Exception:
            pass
    fig.canvas.draw_idle()

def _on_delete_button_clicked(event):
    """선택된 점들 삭제"""
    global _selected_indices, _selected_created_indices, _selection_order, _deleted_original_indices, _range_groups

    if len(_selected_indices) == 0 and len(_selected_created_indices) == 0:
        print('[INFO] 삭제할 선택된 점이 없습니다.')
        return

    # 삭제 동작을 히스토리에 기록
    # 원본 점 삭제
    deleted_original_keys = list(_selected_indices)
    _deleted_original_indices.update(deleted_original_keys)
    
    # 생성된 점 삭제
    deleted_created_data = []
    for idx in sorted(list(_selected_created_indices), reverse=True):
        deleted_created_data.append({
            'index': idx,
            'x': _created_points_x[idx],
            'y': _created_points_y[idx]
        })
        del _created_points_x[idx]
        del _created_points_y[idx]

    _add_to_history({
        'type': 'group_delete',
        'deleted_original_keys': deleted_original_keys,
        'deleted_created_data': deleted_created_data
    })
    
    # 선택 초기화
    _selected_indices.clear()
    _selected_created_indices.clear()
    _selection_order = [] # 삭제 후 선택 순서 초기화
    _range_groups = []

    # UI 업데이트
    _update_original_scatter()
    _update_created_scatter()
    _update_selected_overlay()
    
    total_deleted = len(deleted_original_keys) + len(deleted_created_data)
    print(f"[INFO] {total_deleted}개 점을 삭제했습니다.")

def _on_undo_button_clicked(event):
    """마지막 동작 취소"""
    if _undo_last_operation():
        # UI 업데이트
        _update_selected_overlay()
        _update_original_scatter()
        _update_created_scatter()
        print("[INFO] 마지막 동작을 취소했습니다.")
    else:
        print("[INFO] 취소할 동작이 없습니다.")

def _on_redo_button_clicked(event):
    """취소된 동작 되돌리기"""
    if _redo_last_operation():
        # UI 업데이트
        _update_selected_overlay()
        _update_original_scatter()
        _update_created_scatter()
        print("[INFO] 취소된 동작을 되돌렸습니다.")
    else:
        print("[INFO] 되돌릴 동작이 없습니다.")

def _on_save_all_button_clicked(event):
    """모든 점을 파일로 저장 (삭제된 점 제외)"""
    # 파일명 가져오기
    filename = _txt_filename.text.strip() if _txt_filename is not None else ''
    if filename == '':
        filename = f"all_{os.path.basename(input_filename)}"
    
    # 디렉토리 보장 X: 상대 경로로 저장
    sep = delimiter if delimiter is not None else ' '
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            count = 0
            if _multifile_enabled:
                for info in _file_data:
                    f.write(f"# {info['filename']}\n")
                    for i, (x, y) in enumerate(zip(info['x_coords'], info['y_coords'])):
                        # 다중 파일 모드에서는 _deleted_original_indices 사용 안함 (단순화를 위해)
                        if sep == ',':
                            f.write(f"{x:.6f},{y:.6f}\n")
                        else:
                            f.write(f"{x:.6f} {y:.6f}\n")
                        count += 1
                    f.write("\n")
            else:
                # 단일 파일 저장 로직
                for i, (x, y) in enumerate(zip(x_coords, y_coords)):
                    if i not in _deleted_original_indices:
                        if sep == ',':
                            f.write(f"{x:.6f},{y:.6f}\n")
                        else:
                            f.write(f"{x:.6f} {y:.6f}\n")
                        count += 1
                
                # 생성된 점들 저장
                if len(_created_points_x) > 0:
                    f.write("# Created Points\n")
                    for x, y in zip(_created_points_x, _created_points_y):
                        if sep == ',':
                            f.write(f"{x:.6f},{y:.6f}\n")
                        else:
                            f.write(f"{x:.6f} {y:.6f}\n")
                        count += 1
            
            print(f"[INFO] {count}개 waypoint를 '{filename}'로 저장했습니다. (삭제된 점 제외)")
    except Exception as e:
        print(f"[ERROR] 파일 저장 실패: {e}")

def _undo_last_operation():
    """마지막 동작 취소"""
    global _operation_history, _redo_history, _created_points_x, _created_points_y
    global _selected_indices, _selected_created_indices, _selection_order, _deleted_original_indices
    
    if not _operation_history:
        return False
    
    last_op = _operation_history.pop()
    _redo_history.append(last_op)
    
    if last_op['type'] == 'add_point':
        # 점 추가 취소: 해당 인덱스의 점 제거
        idx = last_op['index']
        if idx < len(_created_points_x):
            del _created_points_x[idx]
            del _created_points_y[idx]
            # 선택에서도 제거
            _selected_created_indices.discard(idx)
            # 인덱스 시프트 반영
            new_order = []
            for t, v in _selection_order:
                if t == 'new':
                    if v == idx:
                        continue
                    elif v > idx:
                        new_order.append(('new', v - 1))
                    else:
                        new_order.append((t, v))
                else:
                    new_order.append((t, v))
            _selection_order = new_order
            # 인덱스 재조정
            adjusted_selected = set()
            for i in _selected_created_indices:
                if i > idx:
                    adjusted_selected.add(i - 1)
                elif i < idx:
                    adjusted_selected.add(i)
            _selected_created_indices = adjusted_selected
            
    elif last_op['type'] == 'remove_point':
        # 점 제거 취소: 점 복원
        idx = last_op['index']
        x, y = last_op['x'], last_op['y']
        _created_points_x.insert(idx, x)
        _created_points_y.insert(idx, y)
        # 인덱스 재조정
        adjusted_selected = set()
        for i in _selected_created_indices:
            if i >= idx:
                adjusted_selected.add(i + 1)
            else:
                adjusted_selected.add(i)
        _selected_created_indices = adjusted_selected
        
    elif last_op['type'] == 'delete_original':
        # 원본 점 삭제 취소: 복원
        idx = last_op['index']
        _deleted_original_indices.discard(idx)
        
    elif last_op['type'] == 'delete_created':
        # 생성 점 삭제 취소: 복원
        idx = last_op['index']
        x, y = last_op['x'], last_op['y']
        _created_points_x.insert(idx, x)
        _created_points_y.insert(idx, y)
        # 인덱스 재조정
        adjusted_selected = set()
        for i in _selected_created_indices:
            if i >= idx:
                adjusted_selected.add(i + 1)
            else:
                adjusted_selected.add(i)
        _selected_created_indices = adjusted_selected
        
    elif last_op['type'] == 'group_delete':
        # 그룹 삭제 취소: 모든 삭제된 점들을 한번에 복원
        # 원본 점들 복원
        for key in last_op['deleted_original_keys']:
            _deleted_original_indices.discard(key)
        
        # 생성된 점들 복원 (인덱스 순서대로 삽입)
        for data in sorted(last_op['deleted_created_data'], key=lambda x: x['index']):
            idx = data['index']
            x = data['x']
            y = data['y']
            _created_points_x.insert(idx, x)
            _created_points_y.insert(idx, y)
            # 인덱스 재조정
            adjusted_selected = set()
            for i in _selected_created_indices:
                if i >= idx:
                    adjusted_selected.add(i + 1)
                else:
                    adjusted_selected.add(i)
            _selected_created_indices = adjusted_selected
        
    elif last_op['type'] == 'select_orig':
        # 원본 점 선택 취소
        idx = last_op['index']
        _selected_indices.discard(idx)
        try:
            _selection_order.remove(('orig', idx))
        except ValueError:
            pass
            
    elif last_op['type'] == 'deselect_orig':
        # 원본 점 선택 해제 취소
        idx = last_op['index']
        _selected_indices.add(idx)
        _selection_order.append(('orig', idx))
        
    elif last_op['type'] == 'select_new':
        # 생성 점 선택 취소
        idx = last_op['index']
        _selected_created_indices.discard(idx)
        try:
            _selection_order.remove(('new', idx))
        except ValueError:
            pass
            
    elif last_op['type'] == 'deselect_new':
        # 생성 점 선택 해제 취소
        idx = last_op['index']
        _selected_created_indices.add(idx)
        _selection_order.append(('new', idx))
        
    elif last_op['type'] == 'range_select':
        # 범위 선택 취소: 선택 집합에서 제거하고, 순서에서도 제거
        indices = last_op.get('indices', [])
        added_indices = last_op.get('added_indices', indices)
        for idx in indices:
            _selected_indices.discard(idx)
        # 이 범위 추가로 selection_order에 넣은 항목 제거
        new_order = []
        to_remove = set(('orig', i) for i in added_indices)
        for item in _selection_order:
            if item in to_remove:
                continue
            new_order.append(item)
        _selection_order = new_order
    
    return True

def _redo_last_operation():
    """취소된 동작 되돌리기"""
    global _operation_history, _redo_history, _created_points_x, _created_points_y
    global _selected_indices, _selected_created_indices, _selection_order, _deleted_original_indices
    
    if not _redo_history:
        return False
    
    op = _redo_history.pop()
    _operation_history.append(op)
    
    if op['type'] == 'add_point':
        # 점 추가 되돌리기
        idx = op['index']
        x, y = op['x'], op['y']
        _created_points_x.insert(idx, x)
        _created_points_y.insert(idx, y)
        # 인덱스 재조정
        adjusted_selected = set()
        for i in _selected_created_indices:
            if i >= idx:
                adjusted_selected.add(i + 1)
            else:
                adjusted_selected.add(i)
        _selected_created_indices = adjusted_selected
        
    elif op['type'] == 'remove_point':
        # 점 제거 되돌리기
        idx = op['index']
        if idx < len(_created_points_x):
            del _created_points_x[idx]
            del _created_points_y[idx]
            # 선택에서도 제거
            _selected_created_indices.discard(idx)
            # 인덱스 시프트 반영
            new_order = []
            for t, v in _selection_order:
                if t == 'new':
                    if v == idx:
                        continue
                    elif v > idx:
                        new_order.append(('new', v - 1))
                    else:
                        new_order.append((t, v))
                else:
                    new_order.append((t, v))
            _selection_order = new_order
            # 인덱스 재조정
            adjusted_selected = set()
            for i in _selected_created_indices:
                if i > idx:
                    adjusted_selected.add(i - 1)
                elif i < idx:
                    adjusted_selected.add(i)
            _selected_created_indices = adjusted_selected
            
    elif op['type'] == 'delete_original':
        # 원본 점 삭제 되돌리기
        idx = op['index']
        _deleted_original_indices.add(idx)
        
    elif op['type'] == 'delete_created':
        # 생성 점 삭제 되돌리기
        idx = op['index']
        if idx < len(_created_points_x):
            del _created_points_x[idx]
            del _created_points_y[idx]
            # 선택에서도 제거
            _selected_created_indices.discard(idx)
            # 인덱스 시프트 반영
            new_order = []
            for t, v in _selection_order:
                if t == 'new':
                    if v == idx:
                        continue
                    elif v > idx:
                        new_order.append(('new', v - 1))
                    else:
                        new_order.append((t, v))
                else:
                    new_order.append((t, v))
            _selection_order = new_order
            # 인덱스 재조정
            adjusted_selected = set()
            for i in _selected_created_indices:
                if i > idx:
                    adjusted_selected.add(i - 1)
                elif i < idx:
                    adjusted_selected.add(i)
            _selected_created_indices = adjusted_selected
            
    elif op['type'] == 'group_delete':
        # 그룹 삭제 되돌리기: 모든 삭제된 점들을 다시 삭제
        # 원본 점들 다시 삭제
        for key in op['deleted_original_keys']:
            _deleted_original_indices.add(key)
        
        # 생성된 점들 다시 삭제 (역순으로 삭제하여 인덱스 문제 방지)
        for data in sorted(op['deleted_created_data'], key=lambda x: x['index'], reverse=True):
            idx = data['index']
            if idx < len(_created_points_x):
                del _created_points_x[idx]
                del _created_points_y[idx]
                # 선택에서도 제거
                _selected_created_indices.discard(idx)
                # 인덱스 시프트 반영
                new_order = []
                for t, v in _selection_order:
                    if t == 'new':
                        if v == idx:
                            continue
                        elif v > idx:
                            new_order.append(('new', v - 1))
                        else:
                            new_order.append((t, v))
                    else:
                        new_order.append((t, v))
                _selection_order = new_order
                # 인덱스 재조정
                adjusted_selected = set()
                for i in _selected_created_indices:
                    if i > idx:
                        adjusted_selected.add(i - 1)
                    elif i < idx:
                        adjusted_selected.add(i)
                _selected_created_indices = adjusted_selected
            
    elif op['type'] == 'select_orig':
        # 원본 점 선택 되돌리기
        idx = op['index']
        _selected_indices.add(idx)
        _selection_order.append(('orig', idx))
        
    elif op['type'] == 'deselect_orig':
        # 원본 점 선택 해제 되돌리기
        idx = op['index']
        _selected_indices.discard(idx)
        try:
            _selection_order.remove(('orig', idx))
        except ValueError:
            pass
            
    elif op['type'] == 'select_new':
        # 생성 점 선택 되돌리기
        idx = op['index']
        _selected_created_indices.add(idx)
        _selection_order.append(('new', idx))
        
    elif op['type'] == 'deselect_new':
        # 생성 점 선택 해제 되돌리기
        idx = op['index']
        _selected_created_indices.discard(idx)
        try:
            _selection_order.remove(('new', idx))
        except ValueError:
            pass
            
    elif op['type'] == 'range_select':
        # 범위 선택 되돌리기: 선택 집합에 추가하고, 순서에도 복원
        indices = op.get('indices', [])
        added_indices = op.get('added_indices', indices)
        pre_selected = set(_selected_indices)
        for idx in indices:
            _selected_indices.add(idx)
        for idx in added_indices:
            if idx not in pre_selected:
                _selection_order.append(('orig', idx))
    
    return True

def _on_save_button_clicked(event):
    """선택된 waypoint를 파일로 저장"""
    # Select 순서 또는 Range 그룹을 사용해 저장 허용
    use_selection_order = len(_selection_order) > 0
    use_range_groups = (not use_selection_order) and len(_range_groups) > 0
    if not use_selection_order and not use_range_groups and len(_selected_indices) == 0:
        print('[INFO] 저장할 선택이 없습니다. Select 또는 Select Range로 선택하세요.')
        return
    # 파일명 가져오기
    filename = _txt_filename.text.strip() if _txt_filename is not None else ''
    if filename == '':
        filename = f"selected_{os.path.basename(input_filename)}"
    # 디렉토리 보장 X: 상대 경로로 저장
    sep = delimiter if delimiter is not None else ' '
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            count = 0
            if use_selection_order:
                for t, key in _selection_order:
                    if t == 'orig':
                        if _multifile_enabled:
                            fname, i = key
                            info = next((fd for fd in _file_data if fd['filename'] == fname), None)
                            if info is None or i >= len(info['x_coords']):
                                continue
                            x = float(info['x_coords'][i])
                            y = float(info['y_coords'][i])
                        else:
                            i = key
                            if i in _deleted_original_indices:
                                continue
                            x = float(x_coords[i])
                            y = float(y_coords[i])
                    else: # 'new'
                        i = key
                        x = float(_created_points_x[i])
                        y = float(_created_points_y[i])
                    
                    if sep == ',':
                        f.write(f"{x:.6f},{y:.6f}\n")
                    else:
                        f.write(f"{x:.6f} {y:.6f}\n")
                    count += 1
                print(f"[INFO] {count}개 waypoint를 '{filename}'로 저장했습니다. (선택 순서 기준)")
            elif use_range_groups:
                for grp in _range_groups:
                    if _multifile_enabled:
                        fname, i0, i1 = grp
                        info = next((fd for fd in _file_data if fd['filename'] == fname), None)
                        if info is None:
                            continue
                        for i in range(i0, i1 + 1):
                            if i < len(info['x_coords']):
                                x = float(info['x_coords'][i])
                                y = float(info['y_coords'][i])
                                if sep == ',':
                                    f.write(f"{x:.6f},{y:.6f}\n")
                                else:
                                    f.write(f"{x:.6f} {y:.6f}\n")
                                count += 1
                    else:
                        i0, i1 = grp
                        for i in range(i0, i1 + 1):
                            if i in _deleted_original_indices:
                                continue
                            x = float(x_coords[i])
                            y = float(y_coords[i])
                            if sep == ',':
                                f.write(f"{x:.6f},{y:.6f}\n")
                            else:
                                f.write(f"{x:.6f} {y:.6f}\n")
                            count += 1
                print(f"[INFO] {count}개 waypoint를 '{filename}'로 저장했습니다. (범위 선택 기준)")
            else:
                # 백업: 현재 선택 집합을 인덱스 오름차순으로 저장
                for i in sorted(_selected_indices):
                    if i in _deleted_original_indices:
                        continue
                    try:
                        x = float(x_coords[i])
                        y = float(y_coords[i])
                    except Exception:
                        continue
                    if sep == ',':
                        f.write(f"{x:.6f},{y:.6f}\n")
                    else:
                        f.write(f"{x:.6f} {y:.6f}\n")
                    count += 1
                print(f"[INFO] {count}개 waypoint를 '{filename}'로 저장했습니다. (선택 집합 기준)")
    except Exception as e:
        print(f"[ERROR] 파일 저장 실패: {e}")

def _init_selection_ui():
    """하단에 선택/저장 UI 구성"""
    global _btn_select, _btn_save, _txt_filename, _btn_range, _btn_add, _btn_delete, _btn_undo, _btn_redo, _btn_save_all
    try:
        add_ax = fig.add_axes([0.01, 0.01, 0.10, 0.045])
        _btn_add = Button(add_ax, 'Add Points')
        _btn_add.on_clicked(_on_add_button_clicked)

        btn_ax = fig.add_axes([0.12, 0.01, 0.10, 0.045])
        _btn_select = Button(btn_ax, 'Select')
        _btn_select.on_clicked(_on_select_button_clicked)
        
        range_ax = fig.add_axes([0.23, 0.01, 0.12, 0.045])
        _btn_range = Button(range_ax, 'Select Range')
        _btn_range.on_clicked(_on_range_button_clicked)
        
        delete_ax = fig.add_axes([0.36, 0.01, 0.08, 0.045])
        _btn_delete = Button(delete_ax, 'Delete')
        _btn_delete.on_clicked(_on_delete_button_clicked)
        
        undo_ax = fig.add_axes([0.45, 0.01, 0.06, 0.045])
        _btn_undo = Button(undo_ax, '←')
        _btn_undo.on_clicked(_on_undo_button_clicked)
        
        redo_ax = fig.add_axes([0.52, 0.01, 0.06, 0.045])
        _btn_redo = Button(redo_ax, '→')
        _btn_redo.on_clicked(_on_redo_button_clicked)
        
        txt_ax = fig.add_axes([0.59, 0.01, 0.20, 0.045])
        default_name = f"selected_{os.path.basename(input_filename)}"
        _txt_filename = TextBox(txt_ax, 'File: ', initial=default_name)
        
        save_ax = fig.add_axes([0.80, 0.01, 0.08, 0.045])
        _btn_save = Button(save_ax, 'Save')
        _btn_save.on_clicked(_on_save_button_clicked)
        
        save_all_ax = fig.add_axes([0.89, 0.01, 0.08, 0.045])
        _btn_save_all = Button(save_all_ax, 'Save All')
        _btn_save_all.on_clicked(_on_save_all_button_clicked)
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
    _save_map_offset() # 종료 시 현재 오프셋 저장
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
    # 's' 키를 저장 단축키에서 제거
    if 's' in plt.rcParams['keymap.save']:
        plt.rcParams['keymap.save'].remove('s')
    plt.show()
except KeyboardInterrupt:
    cleanup_and_exit()
finally:
    cleanup_and_exit()