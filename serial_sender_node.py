import time
import serial
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM1'
#----------------------------------------------

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')
    
    # 시리얼 포트 초기화
    self.ser = serial.Serial(PORT, 115200, timeout=1)  # 아두이노와 동일한 baud rate로 수정
    time.sleep(1)
    
    self.declare_parameter('sub_topic', sub_topic)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)
    
    # 시리얼 읽기 스레드 시작
    self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
    self.serial_thread.start()

    # 주기적으로 기본 명령(steering=0, speed=10) 전송
    self.timer = self.create_timer(0.1, self.send_default_command)

  def read_serial_data(self):
    """아두이노에서 보내는 시리얼 데이터를 읽어서 출력"""
    while True:
      try:
        if self.ser.in_waiting > 0:
          line = self.ser.readline().decode('utf-8', errors='ignore').strip()
          if line:
            print(f"[아두이노] {line}")
        time.sleep(0.1)  # CPU 사용량 줄이기 - sleep 시간 증가
      except Exception as e:
        print(f"시리얼 읽기 오류: {e}")
        # 오류 발생 시에도 계속 실행
        time.sleep(0.5)

  def send_default_command(self):
    """주기적으로 steering=0, speed=10 을 전송"""
    try:
      serial_msg = PCFL.convert_serial_message(0.0, 10.0)
      self.ser.write(serial_msg.encode())
    except Exception as e:
      print(f"시리얼 전송 오류: {e}")

  def input_control_mode(self):
    """비활성화: 기본 명령을 타이머로 전송하므로 입력 모드 사용 안 함"""
    print("입력 모드는 비활성화되었습니다. 타이머로 기본 명령(0,10)을 전송합니다.")

  def data_callback(self, msg):
    steering = msg.steering
    speed = msg.speed
    #left_speed = msg.left_speed
    #right_speed = msg.right_speed

    serial_msg =  PCFL.convert_serial_message(steering, speed)
    #serial_msg =  PCFL.convert_serial_message(steering, left_speed, right_speed)
    self.ser.write(serial_msg.encode())

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      steering = 0
      speed = 0
      #left_speed = 0
      #right_speed = 0
      message = PCFL.convert_serial_message(steering, speed)
      #message = PCFL.convert_serial_message(steering, left_speed, right_speed)
      node.ser.write(message.encode())
      pass
    
  finally:
    node.ser.close()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
