/*
 * Arduino Serial Monitor Test Code
 * 시리얼 통신으로 들어오는 데이터를 모니터링하는 테스트 코드
 * 
 * 기능:
 * - 시리얼 데이터 수신 및 표시
 * - 수신된 데이터 개수 카운트
 * - 타임스탬프 표시
 * - 데이터 분석 (속도, 조향각 등)
 */

// 전역 변수
unsigned long lastReceiveTime = 0;
unsigned long dataCount = 0;
String receivedData = "";
bool newDataReceived = false;

// 시리얼 통신 설정
const int BAUD_RATE = 115200;

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(BAUD_RATE);
  
  // 시작 메시지
  Serial.println("==========================================");
  Serial.println("Arduino Serial Monitor Test Started");
  Serial.println("==========================================");
  Serial.println("Waiting for serial data...");
  Serial.println("Format: s<steering>p<speed>");
  Serial.println("Example: s0p50, s-30p20, s45p0");
  Serial.println("==========================================");
  Serial.println();
  
  // LED 핀 설정 (선택사항)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // 시리얼 데이터 수신 처리
  receiveSerialData();
  
  // 새 데이터가 있으면 처리
  if (newDataReceived) {
    processReceivedData();
    newDataReceived = false;
  }
  
  // 5초마다 상태 정보 출력
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 5000) {
    printStatus();
    lastStatusTime = millis();
  }
  
  // LED 깜빡임 (데이터 수신 표시)
  if (millis() - lastReceiveTime < 100) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void receiveSerialData() {
  // 시리얼 데이터가 있으면 읽기
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      // 줄바꿈 문자를 만나면 데이터 완성
      if (receivedData.length() > 0) {
        newDataReceived = true;
        lastReceiveTime = millis();
        dataCount++;
      }
    } else {
      // 일반 문자는 버퍼에 추가
      receivedData += inChar;
    }
  }
}

void processReceivedData() {
  // 타임스탬프 계산
  unsigned long currentTime = millis();
  unsigned long timeSinceLastData = currentTime - lastReceiveTime;
  
  // 데이터 출력
  Serial.print("[" + String(dataCount) + "] ");
  Serial.print("Time: " + String(currentTime) + "ms | ");
  Serial.print("Data: '" + receivedData + "' | ");
  Serial.print("Length: " + String(receivedData.length()) + " chars");
  
  // 데이터 분석
  analyzeData(receivedData);
  
  Serial.println();
  
  // 버퍼 초기화
  receivedData = "";
}

void analyzeData(String data) {
  // s<steering>p<speed> 형식 분석
  int sIndex = data.indexOf('s');
  int pIndex = data.indexOf('p');
  
  if (sIndex != -1 && pIndex != -1 && pIndex > sIndex) {
    // 조향각 추출
    String steeringStr = data.substring(sIndex + 1, pIndex);
    int steering = steeringStr.toInt();
    
    // 속도 추출
    String speedStr = data.substring(pIndex + 1);
    int speed = speedStr.toInt();
    
    Serial.print(" | Steering: " + String(steering) + "° | Speed: " + String(speed));
    
    // 데이터 유효성 검사
    if (steering >= -90 && steering <= 90 && speed >= 0 && speed <= 100) {
      Serial.print(" [VALID]");
    } else {
      Serial.print(" [INVALID]");
    }
  } else {
    Serial.print(" | [UNKNOWN FORMAT]");
  }
}

void printStatus() {
  Serial.println();
  Serial.println("--- Status ---");
  Serial.println("Total data received: " + String(dataCount));
  Serial.println("Uptime: " + String(millis() / 1000) + " seconds");
  Serial.println("Baud rate: " + String(BAUD_RATE));
  Serial.println("Last data: " + String(millis() - lastReceiveTime) + "ms ago");
  Serial.println("---------------");
  Serial.println();
}

// 추가 유틸리티 함수들
void printHelp() {
  Serial.println();
  Serial.println("=== Help ===");
  Serial.println("Commands:");
  Serial.println("  s<steering>p<speed> - Send command");
  Serial.println("  help - Show this help");
  Serial.println("  status - Show status");
  Serial.println("  clear - Clear screen");
  Serial.println("=============");
  Serial.println();
}

void clearScreen() {
  for (int i = 0; i < 50; i++) {
    Serial.println();
  }
}
