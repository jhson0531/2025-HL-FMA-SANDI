/*
Author:    Omer Ikram ul Haq
Date:      2014-05-18
Location:  Pakistan
Version:   ver 0.1 beta
Test with: Arduino Mega and Devention rx701
*/
#include <Arduino.h>
#include <CytronMotorDriver.h>
#include <math.h>
#include "rc_rx.h"

#define dt 20         // [ms] Task time 

const int FORWARD_1  = 4;
const int FORWARD_2  = 5;
const int BACKWARD_1 = 6;
const int BACKWARD_2 = 7;
const int STEERING_1 = 8;
const int STEERING_2 = 9;

//CH1 : Rudder
//CH2 : Elevator
//CH3 : Throttle

const int POT_PIN = A0;

CytronMD STEERING(PWM_DIR, STEERING_1, STEERING_2);
CytronMD FORWARD (PWM_DIR, FORWARD_1 , FORWARD_2 );
CytronMD BACKWARD(PWM_DIR, BACKWARD_1, BACKWARD_2);

// 조향 속도 상수
const int STEERING_SPEED = 100;

// 제어 상태 변수
int speed_PWM = 0;
int target_steering = 512;
int direction = 1;

unsigned long t=0;

// 명령 주기 제한 변수
unsigned long lastCommandTime = 0; // 마지막 명령 처리 시간
const unsigned int COMMAND_INTERVAL = 20; // 명령 처리 간 최소 대기 시간(ms)

void setMotorSpeed(int spd) {
    FORWARD.setSpeed(spd);
    BACKWARD.setSpeed(spd);
}

void setup()
{
  Serial.begin(115200); // Initlizing serial port before calling init_rc_rx() function 
  init_rc_rx();
  pinMode(POT_PIN, INPUT);
}

void loop()
{ 
  t = micros()/1000;
  unsigned long currentTime = millis();
  
  read_rc_rx();        // CH1, CH2, CH3 and CH4 float variables will be calculated with this function is called
  speed_PWM = round(map(CH3, 0, 100, 0, 80));
  target_steering = round(map(CH1, 0, 100, 0, 1023));

  if (CH2 > 90) {
    direction = 1;
  }
  else if (CH2 < 10) {
    direction = -1;
  }
  else {

  }

  // 일정 시간 간격으로만 제어 명령 실행
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
  
  while (dt > (micros()/1000)- t){
  // do nothing
  }
  
}
