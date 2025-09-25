import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# =============================================================================
# 설정 섹션: 여기서 입력/출력 경로를 수정하세요
# =============================================================================

# 입력 파일 경로
CALIBRATION_FILE = 'imu_calibration_angle.txt'  # 보정각 및 목표 좌표 파일
INPUT_WAYPOINTS_FILE = 'waypoints/full_wp_ver1.txt'         # 원본 waypoint 파일

# 출력 파일 경로
OUTPUT_FILE = 'remapped_utmcoordinates_ver1.txt'    # 리매핑된 좌표 저장 파일

# =============================================================================

# 시작 인덱스(1-based). 예: 21이면 21번째 점을 기준(anchor)으로 정렬/출력
# 내부에서는 0-based로 변환하여 사용합니다.
ANCHOR_INDEX = 353

def read_target_pos_and_calibration(calibration_filename: str):
    try:
        with open(calibration_filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        # 목표 좌표 (기존 규약: 8번째 줄에 존재)
        target_line = lines[7]
        coords_part = target_line.split(':')[1].strip()
        x_str, y_str = coords_part.split(',')
        target_pos = np.array([float(x_str), float(y_str)])

        # 보정각: 첫 번째 줄에서 수치 추출 (rad 또는 deg 모두 허용)
        first_line = lines[0].strip()
        # 첫 줄에서 숫자만 추출
        import re
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", first_line)
        calib_val = float(nums[0]) if nums else 0.0
        # 단위 추정: 2π 초과면 deg로 간주, 아니면 rad로 간주
        if abs(calib_val) <= (2 * math.pi + 1e-6):
            calib_deg = math.degrees(calib_val)
        else:
            calib_deg = calib_val

        print(f"'{calibration_filename}'에서 목표 좌표와 보정각을 읽었습니다: target_pos={target_pos}, calib={calib_deg:.3f} deg")
        return target_pos, calib_deg
    except FileNotFoundError:
        print(f"오류: '{calibration_filename}' 파일을 찾을 수 없습니다. 스크립트와 같은 경로에 파일이 있는지 확인하세요.")
        exit()
    except Exception as e:
        print(f"파일을 읽는 중 오류가 발생했습니다: {e}")
        exit()

target_pos_0, calibration_deg = read_target_pos_and_calibration(CALIBRATION_FILE)

# 2. 데이터 파싱 (파일에서 x, y 좌표와 yaw 값을 직접 불러옴)
data_path = INPUT_WAYPOINTS_FILE

def detect_delimiter_and_cols(path):
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

delimiter, col_count = detect_delimiter_and_cols(data_path)
original_coords = np.loadtxt(data_path, usecols=(0, 1), delimiter=delimiter)

# yaw 컬럼이 없으면 좌표로부터 근사 yaw 계산
if col_count >= 3:
    original_yaw = np.loadtxt(data_path, usecols=(2), delimiter=delimiter)
else:
    # 각 점의 yaw를 다음 점과의 방위로 근사 (마지막 점은 이전과 동일하게)
    dx = np.diff(original_coords[:, 0])
    dy = np.diff(original_coords[:, 1])
    headings = np.degrees(np.arctan2(dy, dx))
    approx_yaw = np.empty(original_coords.shape[0])
    approx_yaw[:-1] = headings
    approx_yaw[-1] = headings[-1] if headings.size > 0 else 0.0
    original_yaw = approx_yaw

def euler_from_quaternion(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch_y = math.copysign(math.pi / 2, sinp)
    else:
        pitch_y = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)
    return roll_x, pitch_y, yaw_z


class IMUOnce(Node):
    def __init__(self) -> None:
        super().__init__('imu_once_reader')
        self.yaw_deg = None
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.get_logger().info('IMU 데이터를 대기 중입니다... (/imu/data)')

    def imu_cb(self, msg: Imu) -> None:
        _, _, yaw_z = euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.yaw_deg = math.degrees(yaw_z)
        self.get_logger().info(f'IMU 수신 완료: yaw={self.yaw_deg:.2f} deg')


def compute_rotation_with_imu(original_coords: np.ndarray, calibration_deg: float, anchor_idx: int) -> np.ndarray:
    # IMU yaw 사용 비활성화: yaw_deg를 0.0으로 고정
    # 아래 ROS2 초기화 및 IMU 구독 코드는 비활성화합니다.
    # rclpy.init()
    # node = IMUOnce()
    # while rclpy.ok() and node.yaw_deg is None:
    #     rclpy.spin_once(node, timeout_sec=0.5)
    # yaw_deg = node.yaw_deg if node.yaw_deg is not None else 0.0
    # node.get_logger().info(f'계산에 사용할 yaw+calib = {yaw_deg:.2f} + {calibration_deg:.2f} deg')
    # node.destroy_node()
    # rclpy.shutdown()
    yaw_deg = 0.0
    print(f'IMU yaw 비활성화: 보정각만 사용(calib={calibration_deg:.2f} deg)')

    # 회전 기준 벡터: n->n+1 (마지막 점이면 n-1->n)
    if original_coords.shape[0] < 2:
        raise ValueError('좌표가 2개 미만입니다. 회전을 수행할 수 없습니다.')
    if anchor_idx < 0 or anchor_idx >= original_coords.shape[0]:
        raise IndexError(f'anchor_idx({anchor_idx})가 좌표 범위를 벗어났습니다.')

    if anchor_idx < original_coords.shape[0] - 1:
        v_anchor = original_coords[anchor_idx + 1] - original_coords[anchor_idx]
        seg_desc = f"{anchor_idx}->${anchor_idx+1}"
    else:
        v_anchor = original_coords[anchor_idx] - original_coords[anchor_idx - 1]
        seg_desc = f"{anchor_idx-1}->{anchor_idx}"

    heading_orig_rad = np.arctan2(v_anchor[1], v_anchor[0])
    heading_orig_deg = np.degrees(heading_orig_rad)
    # 목표 방위(deg): 동쪽 0도를 기준으로 calibration 각도로 정렬
    target_heading_deg = calibration_deg
    # 회전 각도 (deg): 현재 방위 → 목표 방위
    delta_deg = target_heading_deg - heading_orig_deg
    # -180~180로 정규화
    while delta_deg > 180.0:
        delta_deg -= 360.0
    while delta_deg <= -180.0:
        delta_deg += 360.0
    angle_radians = np.radians(delta_deg)

    # 회전 중심: 기준점(anchor) 좌표
    center = original_coords[anchor_idx]
    centered_coords = original_coords - center
    cos_a = np.cos(angle_radians)
    sin_a = np.sin(angle_radians)
    rotation_matrix = np.array([[cos_a, -sin_a],
                                [sin_a,  cos_a]])
    rotated_centered_coords = np.dot(rotation_matrix, centered_coords.T).T
    rotated_coords = rotated_centered_coords + center

    print(f"기준 구간 방위({seg_desc}): {heading_orig_deg:.2f} deg, 목표 방위(calib only): {target_heading_deg:.2f} deg, 회전: {delta_deg:.2f} deg")
    return rotated_coords


"""
ANCHOR_INDEX 사용을 위해 1-based → 0-based로 변환합니다.
경계 체크 후 회전 및 평행이동을 수행합니다.
"""
num_points = original_coords.shape[0]
if ANCHOR_INDEX <= 0:
    raise ValueError('ANCHOR_INDEX는 1 이상의 정수여야 합니다.')
anchor_idx = min(ANCHOR_INDEX - 1, num_points - 1)

rotated_coords = compute_rotation_with_imu(original_coords, calibration_deg, anchor_idx)

# 4. 리매핑(평행 이동) 수행
# 현재 회전된 anchor 점의 좌표
current_pos_anchor = rotated_coords[anchor_idx]

# 필요한 이동량을 계산 (목표 좌표는 파일에서 읽어온 target_pos_0 사용)
translation_vector = target_pos_0 - current_pos_anchor

# 모든 회전된 점들에 동일한 이동량을 더해줌
remapped_coords = rotated_coords + translation_vector

# 5. 모든 waypoint를 유지하여 최종 데이터를 파일로 저장
# ANCHOR_INDEX보다 작은 인덱스의 점들도 모두 보존
final_data = np.hstack((remapped_coords, original_yaw.reshape(-1, 1)))
output_filename = OUTPUT_FILE
np.savetxt(output_filename, final_data, fmt='%f', delimiter='  ')

print(f"\n성공! 리매핑된 좌표가 '{output_filename}' 파일로 저장되었습니다.")
print(f"원본 waypoint 개수: {len(remapped_coords)}개, 기준 인덱스(1-based): {ANCHOR_INDEX}")
print(f"저장된 waypoint 개수: {len(remapped_coords)}개 (모든 점 보존)")
print(f"기준점(1-based {ANCHOR_INDEX})의 최종 좌표: {remapped_coords[anchor_idx]}")