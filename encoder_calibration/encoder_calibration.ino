/*
Encoder PPR Calibration Tool
Author: Created for dual_mode_controller
Date: 2024
Description: 엔코더의 PPR(Pulses Per Revolution) 값을 측정하는 캘리브레이션 도구
*/

#include <Arduino.h>
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// ===== 핀 매핑 =====
const uint8_t ENCODER_A = 20;
const uint8_t ENCODER_B = 21;

// ── 엔코더 객체
Encoder encoder(ENCODER_A, ENCODER_B);

// 캘리브레이션 변수
long start_ticks = 0;
long end_ticks = 0;
int measured_ppr = 0;
bool calibration_started = false;
bool calibration_completed = false;

void setup() {
    Serial.begin(115200);
    
    Serial.println("==========================================");
    Serial.println("    Encoder PPR Calibration Tool");
    Serial.println("==========================================");
    Serial.println();
    Serial.println("This tool is designed to accurately measure");
    Serial.println("the PPR (Pulses Per Revolution) value of an encoder.");
    Serial.println();
    Serial.println("Usage:");
    Serial.println("1. Prepare to manually rotate the motor with encoder");
    Serial.println("2. Press 's' to start calibration");
    Serial.println("3. Rotate the motor exactly one full revolution");
    Serial.println("4. Press 'e' to complete calibration");
    Serial.println("5. Check the measured PPR value");
    Serial.println();
    Serial.println("Current encoder ticks: 0");
    Serial.println("==========================================");
    Serial.println("Commands:");
    Serial.println("  's' - Start calibration");
    Serial.println("  'e' - End calibration");
    Serial.println("  'r' - Reset");
    Serial.println("  'h' - Help");
    Serial.println("==========================================");
}

void loop() {
    // 현재 엔코더 틱 값 실시간 표시
    long current_ticks = encoder.read();
    
    // 1초마다 현재 상태 출력
    static unsigned long last_display = 0;
    if (millis() - last_display >= 1000) {
        last_display = millis();
        
        Serial.print("Current encoder ticks: ");
        Serial.print(current_ticks);
        
        if (calibration_started && !calibration_completed) {
            long tick_diff = current_ticks - start_ticks;
            Serial.print(" | From start: ");
            Serial.print(tick_diff);
            Serial.print(" ticks");
        }
        
        if (calibration_completed) {
            Serial.print(" | Measured PPR: ");
            Serial.print(measured_ppr);
        }
        
        Serial.println();
    }
    
    // 시리얼 명령 처리
    if (Serial.available()) {
        char command = Serial.read();
        
        switch (command) {
            case 's':
            case 'S':
                startCalibration();
                break;
                
            case 'e':
            case 'E':
                endCalibration();
                break;
                
            case 'r':
            case 'R':
                resetCalibration();
                break;
                
            case 'h':
            case 'H':
                showHelp();
                break;
                
            case '\n':
            case '\r':
                // 엔터키 무시
                break;
                
            default:
                Serial.println("Unknown command. Press 'h' for help.");
                break;
        }
    }
}

void startCalibration() {
    if (calibration_completed) {
        Serial.println("Calibration already completed. Press 'r' to reset.");
        return;
    }
    
    start_ticks = encoder.read();
    calibration_started = true;
    calibration_completed = false;
    
    Serial.println();
    Serial.println("=== Calibration Started ===");
    Serial.print("Start ticks: ");
    Serial.println(start_ticks);
    Serial.println("Please rotate the motor exactly one full revolution...");
    Serial.println("Press 'e' when completed.");
    Serial.println("===========================");
}

void endCalibration() {
    if (!calibration_started) {
        Serial.println("Calibration not started. Press 's' to start.");
        return;
    }
    
    end_ticks = encoder.read();
    measured_ppr = abs(end_ticks - start_ticks);
    calibration_completed = true;
    
    Serial.println();
    Serial.println("=== Calibration Completed ===");
    Serial.print("Start ticks: ");
    Serial.println(start_ticks);
    Serial.print("End ticks: ");
    Serial.println(end_ticks);
    Serial.print("Tick difference: ");
    Serial.println(abs(end_ticks - start_ticks));
    Serial.print("Measured PPR: ");
    Serial.println(measured_ppr);
    Serial.println();
    Serial.println("Set this value in dual_mode_controller.ino ENCODER_PPR:");
    Serial.print("const int ENCODER_PPR = ");
    Serial.print(measured_ppr);
    Serial.println(";");
    Serial.println("=============================");
}

void resetCalibration() {
    start_ticks = 0;
    end_ticks = 0;
    measured_ppr = 0;
    calibration_started = false;
    calibration_completed = false;
    
    Serial.println();
    Serial.println("=== Calibration Reset ===");
    Serial.println("All calibration data has been reset.");
    Serial.println("Press 's' to start a new calibration.");
    Serial.println("=========================");
}

void showHelp() {
    Serial.println();
    Serial.println("==========================================");
    Serial.println("              Help");
    Serial.println("==========================================");
    Serial.println("This tool measures the PPR value of an encoder.");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  's' - Start calibration");
    Serial.println("  'e' - End calibration");
    Serial.println("  'r' - Reset");
    Serial.println("  'h' - Show this help");
    Serial.println();
    Serial.println("Usage sequence:");
    Serial.println("1. Prepare to manually rotate the motor");
    Serial.println("2. Press 's' to start");
    Serial.println("3. Rotate the motor exactly one full revolution");
    Serial.println("4. Press 'e' to complete");
    Serial.println("5. Apply the measured PPR value to dual_mode_controller");
    Serial.println();
    Serial.println("Important notes:");
    Serial.println("- Rotate the motor exactly one full revolution only");
    Serial.println("- Measure multiple times to ensure consistent values");
    Serial.println("- Common PPR values: 100, 200, 360, 400, 500, 1000, 2000");
    Serial.println("==========================================");
}
