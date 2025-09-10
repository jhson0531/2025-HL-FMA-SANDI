import time
import serial
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
    self.ser = serial.Serial(PORT, 1152000, timeout=1)
    time.sleep(1)
    
    self.declare_parameter('sub_topic', sub_topic)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)
    
    # input을 통한 수동 제어 모드
    self.input_control_mode()

  def input_control_mode(self):
    """input을 통한 수동 제어 모드"""
    print("=== 수동 제어 모드 ===")
    print("steering과 speed 값을 입력하세요 (종료하려면 'q' 입력)")
    
    while True:
      try:
        steering_input = input("steering (-100 ~ 100): ")
        if steering_input.lower() == 'q':
          break
          
        speed_input = input("speed (-100 ~ 100): ")
        if speed_input.lower() == 'q':
          break
          
        steering = float(steering_input)
        speed = float(speed_input)
        
        # 값 범위 체크
        steering = max(-100, min(100, steering))
        speed = max(-100, min(100, speed))
        
        # 시리얼 메시지 전송
        serial_msg = PCFL.convert_serial_message(steering, speed)
        self.ser.write(serial_msg.encode())
        
        print(f"전송됨 - steering: {steering}, speed: {speed}")
        
      except ValueError:
        print("올바른 숫자를 입력하세요.")
      except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
        break

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
      # input_control_mode가 실행되므로 rclpy.spin은 실행되지 않음
      # rclpy.spin(node)
      pass
      
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
