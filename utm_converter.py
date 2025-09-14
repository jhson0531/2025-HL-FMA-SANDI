import utm

# 변환할 데이터 (위도, 경도, Heading Angle)
# 대한민국 지역의 위도/경도 데이터를 리스트 형태로 정리했습니다.

data = [
    [37.239961, 127.083429, 0],
    [37.2399297, 127.0832478, 0],  # 서울시청 근처 - 위도, 경도, 방위각
    [37.2399304, 127.0831765, 0],
    [37.2399056, 127.0830311, 0],
    [37.2399568, 127.0829277, 0]
]

# 결과를 저장할 리스트
converted_data = []

# 각 데이터 포인트를 순회하며 변환
for point in data:
    latitude = point[0]   # 위도
    longitude = point[1]  # 경도
    heading = point[2]    # 방위각 (이 값은 변환에 사용되지 않음)
    
    # utm.from_latlon 함수를 사용하여 UTM 좌표로 변환
    # 대한민국이 속한 UTM Zone 52와 북반구('N')를 지정
    easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
    
    # 변환된 값과 기존 방위각 값을 함께 저장
    converted_data.append((easting, northing, heading, zone_number, zone_letter))

# 변환 결과 출력
print("--- 위도/경도 -> UTM 변환 결과 (52 N 구역, 대한민국) ---")
for easting, northing, head, zone_num, zone_let in converted_data:
    print(f"UTM X: {easting:.6f}, UTM Y: {northing:.6f}, 방위각: {head:.6f}, Zone: {zone_num}{zone_let}")

# 함수로도 사용할 수 있도록 정의
def latlon_to_utm(latitude, longitude):
    """
    위도, 경도를 UTM 좌표로 변환하는 함수
    
    Args:
        latitude (float): 위도
        longitude (float): 경도
    
    Returns:
        tuple: (easting, northing, zone_number, zone_letter)
    """
    return utm.from_latlon(latitude, longitude)
