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

// 조향 속도 상수
const int STEERING_SPEED = 100;

// 제어 상태 변수
int speed_cmd = 0;
float target_angle_deg = 0;

// 명령 주기 제한 변수
unsigned long lastCommandTime = 0; // 마지막 명령 처리 시간
const unsigned int COMMAND_INTERVAL = 20; // 명령 처리 간 최소 대기 시간(ms)


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
void   setMotorSpeed(int speed);
void   processIncomingByte(byte b);
void   processData(const char *data);



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
        if (newTargetAngle > 25.0) newTargetAngle = 25.0;
        if (newTargetAngle < -25.0) newTargetAngle = -25.0;

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
    unsigned long currentTime = millis();

    while (Serial.available()) {
        processIncomingByte(Serial.read());
    }

    // 일정 시간 간격으로만 제어 명령 실행
    if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {

        int   raw = analogRead(POT_PIN);

        // linear_mapping 함수로 현재 각도 계산
        float current_angle_deg = linear_mapping(raw);
        Serial.println(current_angle_deg);

        // 조향 상태에 따라 동작 제어 
        if (abs(current_angle_deg - target_angle_deg) <= 1) {
            STEERING.setSpeed(0);
        } 
        else if (current_angle_deg > target_angle_deg) {
            STEERING.setSpeed(STEERING_SPEED);
        } else {
            STEERING.setSpeed(-STEERING_SPEED);
        }

        // 모터 속도 설정
        setMotorSpeed(speed_cmd);

        // 마지막 명령 시간 갱신
        lastCommandTime = currentTime;
    }

    // ── 텔레메트리 (기존과 동일)
    const unsigned long period = 1000UL / LOOP_HZ;
    if (currentTime - last_pub >= period) {
        last_pub = currentTime;
        long ticks  = encoder.read();
        int  potRaw = analogRead(POT_PIN);
        Serial.print("T,");
        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(ticks);
        Serial.print(",");
        Serial.println(potRaw);
    }
}
