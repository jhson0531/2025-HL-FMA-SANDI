#!/usr/bin/env python3

"""
UTM Pure Pursuit 테스트 스크립트
이 스크립트는 utm_pure_pursuit.py의 테스트 모드를 실행합니다.
"""

import subprocess
import sys
import time

def main():
    print("UTM Pure Pursuit 테스트를 시작합니다...")
    print("=" * 50)
    
    try:
        # ROS2 환경 설정 (필요한 경우)
        print("ROS2 환경 확인 중...")
        
        # utm_pure_pursuit 노드 실행
        print("UTM Pure Pursuit 노드 실행 중...")
        print("테스트 모드로 실행됩니다.")
        print("Ctrl+C로 종료할 수 있습니다.")
        print("=" * 50)
        
        # 노드 실행
        process = subprocess.Popen([
            'ros2', 'run', 'nav_controller', 'utm_pure_pursuit'
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        
        # 출력 실시간 표시
        for line in process.stdout:
            print(line.strip())
            
    except KeyboardInterrupt:
        print("\n테스트가 중단되었습니다.")
        if 'process' in locals():
            process.terminate()
    except Exception as e:
        print(f"오류 발생: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
