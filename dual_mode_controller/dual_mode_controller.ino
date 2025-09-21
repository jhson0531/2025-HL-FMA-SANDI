/*
Dual Mode Controller
Author: Integrated from RC_Rx_header_example and sketch_sep04a
Date: 2024
Description: 스위치로 자율주행 모드(0)와 조종기 모드(1)를 전환하는 통합 컨트롤러
*/

#include <Arduino.h>
#include <CytronMotorDriver.h>
#include <math.h>
#include "rc_rx.h"
#include <Encoder.h>
#define PI = 3.1415926535897932

// ===== 핀 매핑 =====
const int FORWARD_1  = 4;
const int FORWARD_2  = 5;
const int BACKWARD_1 = 6;
const int BACKWARD_2 = 7;
const int STEERING_1 = 8;
const int STEERING_2 = 9;
const uint8_t ENCODER_A = 20;
const uint8_t ENCODER_B = 21;
const int POT_PIN = A0;
const int MODE_SWITCH_PIN = 30;  // 모드 전환 스위치 핀

// ── 모터 드라이버 객체
CytronMD STEERING(PWM_DIR, STEERING_1, STEERING_2);
CytronMD FORWARD (PWM_DIR, FORWARD_1 , FORWARD_2 );
CytronMD BACKWARD(PWM_DIR, BACKWARD_1, BACKWARD_2);

// 조향 속도 상수
const int STEERING_SPEED = 100;

// 제어 상태 변수
int speed_cmd = 0;
float target_angle_deg = 0;

// 조종기 모드 변수
int speed_PWM = 0;
int target_steering = 512;
int direction = 1;

// ── 엔코더 객체
Encoder encoder(ENCODER_A, ENCODER_B);

// RPM 제어 변수
float target_rpm = 0.0;  // 목표 RPM
float current_rpm = 0.0; // 현재 RPM
long last_encoder_ticks = 0;
unsigned long last_rpm_time = 0;
const int ENCODER_PPR = 300; // 엔코더 Pulses Per Revolution (회전당 펄스 수)

// PID 제어 변수
float pid_kp = 1.0;  // 비례 게인
float pid_ki = 0.0;  // 적분 게인
float pid_kd = 0.0; // 미분 게인
float pid_error = 0.0;
float pid_last_error = 0.0;
float pid_integral = 0.0;
float pid_derivative = 0.0;
float pid_output = 0.0;

// 모드 변수
bool autonomous_mode = true;  // true: 자율주행 모드, false: 조종기 모드
bool last_switch_state = false;

// 명령 주기 제한 변수
unsigned long lastCommandTime = 0;
const unsigned int COMMAND_INTERVAL = 20; // 명령 처리 간 최소 대기 시간(ms)


// 1. 다점 캘리브레이션 데이터 및 함수
// ==========================================================
#define No_Calibration_Point 17
struct 
{
  double X[No_Calibration_Point]; // AD 값 배열
  double Y[No_Calibration_Point]; // 각도(degree) 배열
} cal_data;

// 캘리브레이션 데이터 
void setupCalibrationData() {
  cal_data = {
    {36.0, 73.0, 123.0, 177.0, 205.0, 261.0, 327.0, 414.0, 516.0, 593.0, 668.0,
     758.0, 842.0, 897.0, 941.0, 975.0, 1012.0},   // Potentiometer
    {-25.0, -22.0, -19.0, -17.0, -14.0, -10.0, -7.0, -3.0, 3.0, 5.0, 8.0,
     12.0, 15.0, 19.0, 21.0, 23.0, 25.0}           // 각도 값
  };
}

// Potentiometer 변환값을 넣으면 steering 각도(degree)가 나오는 선형 보간 함수
double linear_mapping(double x) {
  int i = 0;
  for (int j = 0; j < No_Calibration_Point - 1; j++) {
    if (x < cal_data.X[0]) {
       i = 0;
       break;
    }
    if ((x >= cal_data.X[j]) && (x < cal_data.X[j + 1])) {
      i = j;
      break;
    }
    i = No_Calibration_Point - 2; // 범위 초과 시 마지막 구간 사용
  }
  
  double x1 = cal_data.X[i];
  double x2 = cal_data.X[i + 1];
  double y1 = cal_data.Y[i];
  double y2 = cal_data.Y[i + 1];
  
  double y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
  return y;
}
// ==========================================================

// ===== 텔레메트리 주기 =====
const uint16_t LOOP_HZ = 100;
static unsigned long last_pub = 0;

// ===== 함수 선언 =====
void setMotorSpeed(int speed);
void processIncomingByte(byte b);
void processData(const char *data);
void checkModeSwitch();
void autonomousControl();
void rcControl();
float calculateRPM();
int calculatePIDOutput(float target, float current);

// ===== 구동 모터 설정 =====
void setMotorSpeed(int spd) {
    FORWARD.setSpeed(spd);
    BACKWARD.setSpeed(spd);
}

// ===== RPM 계산 함수 =====
float calculateRPM() {
    unsigned long current_time = millis();
    long current_ticks = encoder.read();
    
    if (last_rpm_time == 0) {
        last_rpm_time = current_time;
        last_encoder_ticks = current_ticks;
        return 0.0;
    }
    
    unsigned long time_diff = current_time - last_rpm_time;
    if (time_diff < 50) { // 최소 50ms 간격으로만 계산
        return current_rpm;
    }
    
    long tick_diff = current_ticks - last_encoder_ticks;
    
    // RPM 계산: (tick_diff / ENCODER_PPR) * (60000 / time_diff)
    float rpm = (float)tick_diff / ENCODER_PPR * 60000.0 / time_diff;
    
    last_rpm_time = current_time;
    last_encoder_ticks = current_ticks;
    
    return rpm;
}

// ===== PID 제어 함수 =====
int calculatePIDOutput(float target, float current) {
    pid_error = target - current;
    
    // 적분 항 계산
    pid_integral += pid_error;
    
    // 적분 항 제한 (윈드업 방지)
    if (pid_integral > 100) pid_integral = 100;
    if (pid_integral < -100) pid_integral = -100;
    
    // 미분 항 계산
    pid_derivative = pid_error - pid_last_error;
    
    // PID 출력 계산
    pid_output = pid_kp * pid_error + pid_ki * pid_integral + pid_kd * pid_derivative;
    
    // 출력 제한 (-255 ~ 255)
    if (pid_output > 255) pid_output = 255;
    if (pid_output < -255) pid_output = -255;
    
    pid_last_error = pid_error;
    
    return (int)pid_output;
}

// ===== 모드 스위치 확인 =====
void checkModeSwitch() {
    bool current_switch_state = digitalRead(MODE_SWITCH_PIN);
    
    // 스위치 상태가 변경되었을 때만 모드 전환
    if (current_switch_state != last_switch_state) {
        delay(50); // 디바운싱
        if (digitalRead(MODE_SWITCH_PIN) == current_switch_state) {
            autonomous_mode = !current_switch_state; // 0이면 자율주행, 1이면 조종기
            
            // 모드 전환 시 모터 정지
            STEERING.setSpeed(0);
            setMotorSpeed(0);
            
            Serial.print("Mode switched to: ");
            Serial.println(autonomous_mode ? "AUTONOMOUS" : "RC CONTROL");
            
            last_switch_state = current_switch_state;
        }
    }
}

// ===== 자율주행 모드 제어 =====
void autonomousControl() {
    // 시리얼 통신으로 받은 명령 처리
    while (Serial.available()) {
        processIncomingByte(Serial.read());
    }
    
    // 일정 시간 간격으로만 제어 명령 실행
    unsigned long currentTime = millis();
    if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
        int raw = analogRead(POT_PIN);
        
        // linear_mapping 함수로 현재 각도 계산
        float current_angle_deg = linear_mapping(raw);
        
        // 조향 상태에 따라 동작 제어 
        if (abs(current_angle_deg - target_angle_deg) <= 1) {
            STEERING.setSpeed(0);
        } 
        else if (current_angle_deg > target_angle_deg) {
            STEERING.setSpeed(STEERING_SPEED);
        } else {
            STEERING.setSpeed(-STEERING_SPEED);
        }
        
        // 현재 RPM 계산
        current_rpm = calculateRPM();
        
        // PID 제어로 모터 속도 계산
        int motor_speed = calculatePIDOutput(target_rpm, current_rpm);
        
        // 모터 속도 설정
        setMotorSpeed(motor_speed);
        
        // 마지막 명령 시간 갱신
        lastCommandTime = currentTime;
    }
}

// ===== 조종기 모드 제어 =====
void rcControl() {
    read_rc_rx();        // CH1, CH2, CH3 and CH4 float variables will be calculated
    speed_PWM = round(map(CH3, 0, 100, 0, 150));
    target_steering = round(map(CH1, 0, 100, 0, 1023));
    
    if (CH2 > 90) {
        direction = 1;
    }
    else if (CH2 < 10) {
        direction = -1;
    }
    
    // 일정 시간 간격으로만 제어 명령 실행
    unsigned long currentTime = millis();
    if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
        int current_steering = analogRead(POT_PIN);
        
        // 조향 상태에 따라 동작 제어 
        if (abs(current_steering - target_steering) < 10) {
            STEERING.setSpeed(0);
        } 
        else if (current_steering > target_steering) {
            STEERING.setSpeed(STEERING_SPEED);
        } else {
            STEERING.setSpeed(-STEERING_SPEED);
        }
        
        // 모터 속도 설정
        setMotorSpeed(speed_PWM * direction);
        
        // 마지막 명령 시간 갱신
        lastCommandTime = currentTime;
    }
}

// ===== 수신 바이트 처리 (자율주행 모드용) =====
void processIncomingByte(byte inByte) {
    static char input_line[20];
    static unsigned int input_pos = 0;

    switch (inByte) {
        case '\n':
            input_line[input_pos] = 0;
            processData(input_line);
            input_pos = 0;
            break;
        case '\r':
            break;
        default:
            if (input_pos < 19) {
                input_line[input_pos++] = inByte;
            }
            break;
    }
}

// ===== 명령 파싱/처리 (자율주행 모드용) =====
void processData(const char *data) {
    int sIndex = -1, pIndex = -1;
    for (int i = 0; data[i] != '\0'; i++) {
        if (data[i] == 's') sIndex = i;
        else if (data[i] == 'p') pIndex = i;
    }
    if (sIndex != -1 && pIndex != -1 && pIndex > sIndex) {
        float newTargetAngle = atof(data + sIndex + 1);
        float newTargetRPM   = round(atof(data + pIndex + 1) * 60 /(PI * 0.265)) ;  // p 값은 이제 RPM으로 해석
        
        // 캘리브레이션 데이터의 최대/최소 각도로 제한
        if (newTargetAngle > 25.0) newTargetAngle = 25.0;
        if (newTargetAngle < -25.0) newTargetAngle = -25.0;

        // RPM 제한 (예: -1000 ~ 1000 RPM)
        if (newTargetRPM > 1000.0) newTargetRPM = 1000.0;
        if (newTargetRPM < -1000.0) newTargetRPM = -1000.0;

        target_angle_deg = newTargetAngle;
        target_rpm       = newTargetRPM;
        
        // PID 적분 항 리셋 (새로운 목표값 설정 시)
        pid_integral = 0.0;
        pid_last_error = 0.0;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(POT_PIN, INPUT);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP); // 내부 풀업 저항 사용
    
    // 초기 스위치 상태 읽기
    last_switch_state = digitalRead(MODE_SWITCH_PIN);
    autonomous_mode = !last_switch_state; // 0이면 자율주행, 1이면 조종기
    
    setupCalibrationData(); // 캘리브레이션 데이터 초기화
    init_rc_rx(); // 조종기 수신기 초기화
    
    Serial.print("Initial Mode: ");
    Serial.println(autonomous_mode ? "AUTONOMOUS" : "RC CONTROL");
}

void loop() {
    unsigned long currentTime = millis();
    
    // 모드 스위치 확인
    checkModeSwitch();
    
    // 현재 모드에 따른 제어 실행
    if (autonomous_mode) {
        autonomousControl();
    } else {
        rcControl();
    }
    
    // ── 텔레메트리 (두 모드 공통)
    const unsigned long period = 1000UL / LOOP_HZ;
    if (currentTime - last_pub >= period) {
        last_pub = currentTime;
        long ticks = encoder.read();
        int potRaw = analogRead(POT_PIN);
        
        if (autonomous_mode) {
            // 자율주행 모드: 조향각, 목표 RPM, 현재 RPM, PID 출력 포함
            Serial.print("T,");
            Serial.print(currentTime);
            Serial.print(",");
            Serial.print(ticks);
            Serial.print(",");
            Serial.print(potRaw);
            Serial.print(",");
            Serial.print(target_angle_deg);
            Serial.print(",");
            Serial.print(target_rpm);
            Serial.print(",");
            Serial.print(current_rpm);
            Serial.print(",");
            Serial.println(pid_output);
        } else {
            // 조종기 모드: 기존 형식 유지
            Serial.print("T,");
            Serial.print(currentTime);
            Serial.print(",");
            Serial.print(ticks);
            Serial.print(",");
            Serial.println(potRaw);
        }
    }
}
