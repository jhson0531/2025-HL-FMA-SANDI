#!/usr/bin/env python3
"""
카메라 비디오 녹화 스크립트
OpenCV를 사용하여 카메라에서 비디오를 실시간으로 녹화합니다.
"""

import cv2
import datetime
import os
import sys
import signal

class CameraVideoRecorder:
    def __init__(self, camera_index=1, output_dir="recorded_videos"):
        """
        카메라 비디오 녹화기 초기화
        
        Args:
            camera_index (int): 카메라 인덱스 (기본값: 0)
            output_dir (str): 비디오 저장 디렉토리
        """
        self.camera_index = camera_index
        self.output_dir = output_dir
        self.cap = None
        self.out = None
        self.recording = False
        
        # 출력 디렉토리 생성
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"출력 디렉토리 생성: {output_dir}")
    
    def setup_camera(self):
        """카메라 설정"""
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"카메라 {self.camera_index}를 열 수 없습니다.")
            return False
        
        # 카메라 해상도 설정 (1920x1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 실제 설정된 값 확인
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        print(f"카메라 설정 완료: {width}x{height} @ {fps}fps")
        return True
    
    def start_recording(self):
        """녹화 시작"""
        if not self.setup_camera():
            return False
        
        # 현재 시간으로 파일명 생성
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"video_{timestamp}.mp4"
        filepath = os.path.join(self.output_dir, filename)
        
        # 비디오 코덱 설정 (MP4V)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        # VideoWriter 객체 생성
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.out = cv2.VideoWriter(filepath, fourcc, fps, (width, height))
        
        if not self.out.isOpened():
            print("비디오 파일을 생성할 수 없습니다.")
            return False
        
        self.recording = True
        print(f"녹화 시작: {filepath}")
        print("녹화를 중단하려면 'q' 키를 누르거나 Ctrl+C를 사용하세요.")
        
        return True
    
    def record_video(self):
        """비디오 녹화 실행"""
        frame_count = 0
        
        while self.recording:
            ret, frame = self.cap.read()
            
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break
            
            # 프레임을 비디오 파일에 쓰기
            self.out.write(frame)
            
            # 화면에 프레임 표시 (선택사항)
            cv2.imshow('Camera Recording', frame)
            
            frame_count += 1
            
            # 100프레임마다 진행상황 출력
            if frame_count % 100 == 0:
                print(f"녹화 중... {frame_count} 프레임")
            
            # 키 입력 확인
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("사용자가 녹화를 중단했습니다.")
                break
    
    def stop_recording(self):
        """녹화 중단"""
        self.recording = False
        
        if self.out:
            self.out.release()
            print("비디오 파일이 저장되었습니다.")
        
        if self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
    
    def cleanup(self):
        """리소스 정리"""
        self.stop_recording()

def signal_handler(sig, frame):
    """시그널 핸들러 (Ctrl+C 처리)"""
    print("\n녹화를 중단합니다...")
    global recorder
    if recorder:
        recorder.cleanup()
    sys.exit(0)

def main():
    """메인 함수"""
    global recorder
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=== 카메라 비디오 녹화기 ===")
    print("사용 가능한 카메라를 확인 중...")
    
    # 사용 가능한 카메라 확인
    available_cameras = []
    for i in range(5):  # 0-4번 카메라 확인
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    
    if not available_cameras:
        print("사용 가능한 카메라가 없습니다.")
        return
    
    print(f"사용 가능한 카메라: {available_cameras}")
    
    # 카메라 인덱스 선택
    camera_index = 0
    if len(available_cameras) > 1:
        try:
            camera_index = int(input(f"카메라 인덱스를 선택하세요 {available_cameras}: ") or "0")
        except ValueError:
            camera_index = 0
    
    # 녹화기 생성 및 실행
    recorder = CameraVideoRecorder(camera_index=camera_index)
    
    try:
        if recorder.start_recording():
            recorder.record_video()
    except KeyboardInterrupt:
        print("\n녹화가 중단되었습니다.")
    finally:
        recorder.cleanup()

if __name__ == "__main__":
    recorder = None
    main()