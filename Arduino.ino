#include <Arduino.h>
#include <CytronMotorDriver.h>
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <math.h>

// ===== 핀 매핑 =====
const int STEERING_1 = 2;
const int STEERING_2 = 3;
const int FORWARD_1  = 4;
const int FORWARD_2  = 5;
const int BACKWARD_1 = 6;
const int BACKWARD_2 = 7;
const uint8_t ENCODER_A = 18;
const uint8_t ENCODER_B = 19;
const int POT_PIN = A0;

// ── 모터 드라이버 객체
CytronMD STEERING(PWM_DIR, STEERING_1, STEERING_2);
CytronMD FORWARD (PWM_DIR, FORWARD_1 , FORWARD_2 );
CytronMD BACKWARD(PWM_DIR, BACKWARD_1, BACKWARD_2);

// ── 엔코더 객체
Encoder encoder(ENCODER_A, ENCODER_B);

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


// AD 변환값을 넣으면 steering 각도(degree)가 나오는 선형 보간 함수
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


// ===== 제어 파라미터 =====
const int   STEERING_SPEED_MAX = 200;     // 조향 모터 최대 PWM

// [수정] 2. PID 제어 게인 및 관련 변수 추가
// ==========================================================
const float Kp = 10.0f; // P 게인 (재조정이 필요할 수 있습니다)
const float Ki = 2.5f;  // I 게인 (추가)
const float Kd = 4.0f;  // D 게인 (추가)

double error, error_old = 0.0;
double error_s = 0.0, error_d = 0.0;
// ==========================================================

const float ANGLE_DEADBAND_DEG = 0.5f;    // 데드밴드(±) (값을 조금 줄임)

// ===== 제어 변수 =====
int speed_cmd = 0;
float target_angle_deg = 0.0f;

// ===== 제어 루프 주기 =====
const unsigned int CONTROL_INTERVAL_MS = 20;   // 제어 주기를 20ms (50Hz)로 빠르게 설정
unsigned long lastControlTime = 0;

// ===== 텔레메트리 주기 =====
const uint16_t LOOP_HZ = 100;
static unsigned long last_pub = 0;

// ===== 함수 선언 =====
void   setMotorSpeed(int speed);
void   processIncomingByte(byte b);
void   processData(const char *data);

// [삭제] 기존의 rawToSteerDeg 함수는 더 이상 필요 없으므로 삭제합니다.

// ===== 구동 모터 설정 =====
void setMotorSpeed(int spd) {
    FORWARD.setSpeed(spd);
    BACKWARD.setSpeed(spd);
}

// ===== 수신 바이트 처리 (기존과 동일) =====
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

// ===== 명령 파싱/처리 (기존과 동일, 최대/최소 각도만 수정) =====
void processData(const char *data) {
    int sIndex = -1, pIndex = -1;
    for (int i = 0; data[i] != '\0'; i++) {
        if (data[i] == 's') sIndex = i;
        else if (data[i] == 'p') pIndex = i;
    }
    if (sIndex != -1 && pIndex != -1 && pIndex > sIndex) {
        float newTargetAngle = atof(data + sIndex + 1);
        int   newSpeed       = atoi(data + pIndex + 1);
        
        // [수정] 캘리브레이션 데이터의 최대/최소 각도로 제한
        if (newTargetAngle > 25.0 newTargetAngle = 25.0
        if (newTargetAngle < -25.0 newTargetAngle = -25.0;

        target_angle_deg = newTargetAngle;
        speed_cmd        = newSpeed;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(POT_PIN, INPUT);
    setupCalibrationData(); // [추가] 캘리브레이션 데이터 초기화
}

void loop() {
    const unsigned long now = millis();

    while (Serial.available()) {
        processIncomingByte(Serial.read());
    }

    // [수정] 2. 제어 루프: 조향 PID 제어 로직으로 변경
    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
        lastControlTime = now;

        int   raw = analogRead(POT_PIN);
        // [수정] 1. linear_mapping 함수로 현재 각도 계산
        float current_angle_deg = linear_mapping(raw);

        // PID 제어 계산
        error = target_angle_deg - current_angle_deg;
        
        if (fabs(error) < ANGLE_DEADBAND_DEG) {
            error = 0.0f;
            error_s = 0; // 데드밴드 안에서는 누적 오차 초기화
        } else {
            error_s += error * (CONTROL_INTERVAL_MS / 1000.0); // 오차 적분
        }

        error_d = (error - error_old) / (CONTROL_INTERVAL_MS / 1000.0); // 오차 미분
        error_old = error;

        // 적분값이 과도하게 커지는 것을 방지 (Anti-windup)
        error_s = constrain(error_s, -100, 100);

        // PID 계산식으로 최종 PWM 결정
        int steering_pwm = -(Kp * error + Ki * error_s + Kd * error_d);
        steering_pwm = constrain(steering_pwm, -STEERING_SPEED_MAX, STEERING_SPEED_MAX);

        // 모터 제어 출력
        STEERING.setSpeed(steering_pwm);
        setMotorSpeed(speed_cmd);
    }

    // ── 텔레메트리 (기존과 동일)
    const unsigned long period = 1000UL / LOOP_HZ;
    if (now - last_pub >= period) {
        last_pub = now;
        long ticks  = encoder.read();
        int  potRaw = analogRead(POT_PIN);
        Serial.print("T,");
        Serial.print(now);
        Serial.print(",");
        Serial.print(ticks);
        Serial.print(",");
        Serial.println(potRaw);
    }
}
