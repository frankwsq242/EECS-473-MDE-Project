#include <Wire.h>
#include "QuadEncoder.h"

// If in Debug mode
#define DEBUG false

#if DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Pin
#define TFLUNA_I2C_ADDR 0x10  // TF-Luna默认I2C地址
#define AUTOMODE 'A'
#define MANUALMODE 'M'
/// PWM pin, control left motor
#define EN1 15
/// PWM pin, control right motor
#define EN2 14
#define A_1 28
#define A_2 29
#define A_3 30
#define A_4 31
#define encoderPinRA 2
#define encoderPinRB 3
#define encoderPinLA 4
#define encoderPinLB 5
#define CV_RESET 41

// Constants
// const int NUM_LOCATIONS = 100;
const int SAFE_DISTANCE = 60;  // Adjust as needed
// const int MOVE_THRESHOLD = 75;
const int TURN_THRESHOLD = 30;
// const int ERROR_DISTANCE = 0xFFFF;
// const int MAX_DISTANCE = 300;  // Error value from Lidar
const int NUM_COORDINATE_READINGS = 5;
const int outlierThreshold = 5;
const int tolerance = 2;
const float R = 20;  //robot width/2: cm
const float r = 5;   //wheel radius: cm
const int encoderCPR = 17280;
const int w_cmd = 100;

// Variables
int cnt = 0;
int avgIndex = 0;
int v_cmd = 200;
int targetEncoder = 480;
int TURN_THRESHOLD_turn = 10;
int consecutiveOutliers = 0;
QuadEncoder knobLeft(1, encoderPinLA, encoderPinLB);
QuadEncoder knobRight(2, encoderPinRA, encoderPinRB);

long previousTime = 0;
float dPrevious = 0.0;
float dIntegral = 0.0;
float ePrevious = 0;
float eIntegral = 0;
float right_speed = 0;
float integralLeft = 0;
float integralRight = 0;
int previousLeftError = 0;
int previousRightError = 0;
uint16_t prevDistance = 0;
uint16_t distance;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(A_1, OUTPUT);
  pinMode(A_2, OUTPUT);
  pinMode(A_3, OUTPUT);
  pinMode(A_4, OUTPUT);

  knobLeft.setInitConfig();
  knobLeft.init();
  knobRight.setInitConfig();
  knobRight.init();

  Wire.begin();           // 初始化I2C总线
  Wire.setClock(100000);  // 设置I2C时钟速率为100kHz，确保稳定通信
  delay(100);             // 等待传感器上电并启动

  moveStop();
  delay(10);
  DEBUG_PRINTLN("Ready, Steady, Go");
}

char current_mode = AUTOMODE;

void Resetpin() {
    pinMode(CV_RESET, OUTPUT); 
    digitalWrite(CV_RESET, HIGH);  // Set the pin high
    delay(100);  // Wait for the duration (in milliseconds)
    digitalWrite(CV_RESET, LOW);  // Set the pin low
}

void loop() {
  //FIXME
  //while (!Serial2.available());
  char c = 'T';
  c = Serial2.read();

  if(c == 'S'){
    if(current_mode == MANUALMODE){
      current_mode = AUTOMODE;
      Resetpin();
    }else{
      current_mode = MANUALMODE;
    }
  }

  switch (current_mode) {
    case AUTOMODE:
      autoRun();
      DEBUG_PRINTLN("Auto Run");
      break;
    case MANUALMODE:
      manualRun(c);
      DEBUG_PRINTLN("Manual Run");
      break;
    default: return;
  }
}

void printDebugInfo(float debug_Xcoord, String debug_status, float debug_distance) {
    Serial.print(millis());
    Serial.print(",\t");
    Serial.print(debug_Xcoord);
    Serial.print(",\t");
    Serial.print(debug_status);
    Serial.print(",\t");
    Serial.println(debug_distance);
}

void autoRun() {
  float xCoord = getNewX();

  float debug_Xcoord = -1.0;
  String debug_status = "NODATA";
  float debug_distance = -1.0;
  debug_Xcoord = xCoord;

  DEBUG_PRINTLN("X Coordinate is: " + String(xCoord));
  if (xCoord == -999) {  // not received data, entering the next loop
    moveStop();
    delay(100);
    printDebugInfo(debug_Xcoord, debug_status, debug_distance);
    return;
  } else if (xCoord == 999) {  // target lost, updating outlier cnt
    consecutiveOutliers++;
    if (consecutiveOutliers >= outlierThreshold) {  // error stop
      DEBUG_PRINTLN("Consecutive outliers exceed 5. Entering ERROR_STOP.");
      moveStop();
      // TODO
      delay(5000);
    }
    moveStop();
    delay(100);
    debug_status = "LOSTTARGET";
    printDebugInfo(debug_Xcoord, debug_status, debug_distance);
    return;
  } else if (fabs(xCoord) <= 320) {  // normal x coord
    consecutiveOutliers = 0;
    if (fabs(xCoord) <= TURN_THRESHOLD) {  // forward: get 10 distances and calculate forward speed
      float sum = 0;
      for (int i = 0; i < NUM_COORDINATE_READINGS; i++) {
        sum += readDistance();
      }

      float avgDistance = sum / NUM_COORDINATE_READINGS;
      DEBUG_PRINTLN("Average distance is: " + String(avgDistance));
      debug_distance = avgDistance;
      if (avgDistance < SAFE_DISTANCE) {  // normal stop
        DEBUG_PRINTLN("Distance too small. Entering NORMAL_STOP.");
        moveStop();
        delay(1000);
        debug_status = "DIS_SMALL";
        printDebugInfo(debug_Xcoord, debug_status, debug_distance);
      } else {  // forward
        v_cmd = calculateVcmd(avgDistance);
        DEBUG_PRINTLN("V_cmd is: " + String(v_cmd));
        moveForward();
        delay(100);
        moveStop();
        debug_status = "FORWARD";
        printDebugInfo(debug_Xcoord, debug_status, debug_distance);
      }

    } else {  // turning: turning until x coord within threshold
      // Fixme: may need while (true)
      // DEBUG_PRINTLN("Enter turning. Go forward first.");
      // v_cmd = calculateVcmd(SAFE_DISTANCE);  // adjust as needed
      // moveForward();
      // delay(100);
      DEBUG_PRINTLN("Start turning.");
      moveTurning(xCoord < 0, xCoord);
      delay(100);
      moveStop();
      debug_status = "Turning";
      printDebugInfo(debug_Xcoord, debug_status, debug_distance);
      // moveStop();
    }
  }
}

void manualRun(char command) {
  static char lastCommand = '\0'; 
  int speed = 150; 
  
  DEBUG_PRINTLN(command);

  if (command != lastCommand) {
    switch (command) {
      case 'L':  
        setMotorSpeed(EN1, A_1, A_2, 0);
        setMotorSpeed(EN2, A_3, A_4, speed);
        lastCommand = command;
        break;
      case 'R':
        setMotorSpeed(EN1, A_1, A_2, speed);
        setMotorSpeed(EN2, A_3, A_4, 0);
        lastCommand = command;
        break;
      case 'F':
        setMotorSpeed(EN1, A_1, A_2, speed);
        setMotorSpeed(EN2, A_3, A_4, speed);
        lastCommand = command;
        break;
      case 'B':
        setMotorSpeed(EN1, A_1, A_2, -speed);
        setMotorSpeed(EN2, A_3, A_4, -speed);
        lastCommand = command;
        break;
      case 'T':
        moveStop();
        lastCommand = '\0';
        break;
      default:
        moveStop();
        lastCommand = '\0';
        break;
    }
  }
}


int find(String data, char a) {
  int ret = -1;
  for (unsigned int i = 0; i < data.length(); i++) {
    if (data[i] == a) {
      ret = i;
    }
  }
  return ret;
}

float getNewX() {
  // Read a string from the serial input
  Serial1.begin(9600);
  unsigned long startTime = millis();
  unsigned long endTime;
  // Serial1.flush();
  while (!Serial1.available()) {
    endTime = millis();
    if (endTime - startTime > 3000) return -999;
  }
  String data = Serial1.readStringUntil(')');

  data += ')';
  DEBUG_PRINTLN(data);

  // Find the index of the last comma
  float ret;
  // int index = find(data, ',');
  int start = find(data, '(');
  int end = find(data, ')');

  // Check if a comma was found
  if (start != -1 && end != -1) {  // received angle
    String part1 = data.substring(start + 1, end);

    // Convert the extracted strings to integers
    ret = part1.toFloat();
    if (ret == 999) {
      DEBUG_PRINTLN("Error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      // delay(5000);
    }
  } else {  // index = -1: no coordinate
    ret = -999;
  }
  DEBUG_PRINTLN("X coordinate: ");
  DEBUG_PRINTLN(ret);
  Serial1.end();
  return ret;
}

void moveStop() {
  DEBUG_PRINTLN("STOP");
  digitalWrite(A_1, HIGH);
  digitalWrite(A_2, HIGH);
  digitalWrite(A_3, HIGH);
  digitalWrite(A_4, HIGH);
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  delay(5);
  digitalWrite(A_1, LOW);
  digitalWrite(A_2, LOW);
  digitalWrite(A_3, LOW);
  digitalWrite(A_4, LOW);
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}

void moveForward() {
  DEBUG_PRINTLN("FORWARD");

  // Reset encoder count to 0
  knobLeft.write(0);   // Resetting the first encoder count
  knobRight.write(0);  // If you also want to reset the second encoder
  float kp = 2.0;
  float kd = 0.0;
  float ki = 0.1;
  float threshold_pid = 2;

  // unsigned long startTime = millis(); // Use unsigned long for duration
  digitalWrite(A_1, HIGH);
  analogWrite(EN1, v_cmd);
  digitalWrite(A_3, HIGH);
  analogWrite(EN2, (int)(v_cmd * 1.03));
  delay(100);
  while (true) {
    int target = knobLeft.read();  // Use the current encoder count as the target
    float u = pidController(target, kp, kd, ki);
    moveRightMotor(u);
    if (knobLeft.read() > 7000 || knobRight.read() > 7000 || fabs(knobLeft.read() - knobRight.read()) < threshold_pid) {
      break;
    }
  }
}

void moveTurning(bool ifLeft, int x) {
  DEBUG_PRINTLN("TURNING");
  float speed = constrain(130 + 0.5 * fabs(x), 0, 255);
  DEBUG_PRINTLN(speed);
  if (ifLeft) {
    setMotorSpeed(EN1, A_1, A_2, -speed);
    setMotorSpeed(EN2, A_3, A_4, speed);
  } else {
    setMotorSpeed(EN1, A_1, A_2, speed);
    setMotorSpeed(EN2, A_3, A_4, -speed);
  }
}

void setMotorSpeed(int enablePin, int forwardPin, int backwardPin, int speed) {
  if (speed > 0) {
    // Move forward
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
    analogWrite(enablePin, speed);
  } else {
    // Move backward
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
    analogWrite(enablePin, -speed);
  }
}

int calculateVcmd(float currentDistance) {
  float Kp = 1.5;  // Proportional gain - adjust as needed
  float Ki = 0.0;  // Integral gain - adjust as needed
  float Kd = 0.1;  // Derivative gain - adjust as needed
  float error = fabs(currentDistance - SAFE_DISTANCE);

  // Calculate PID components
  dIntegral += error;                    // Integrate error over time
  float derivative = error - dPrevious;  // Change in error

  // PID output for speed
  float output = Kp * error + Ki * dIntegral + Kd * derivative;

  float A = 1.0;
  float B = 120;
  dPrevious = error;

  // Calculate final speed command, ensuring it's within safe limits
  int ret = constrain(A * output + B, 100, 220);  // Assuming 220 is the max speed

  return ret;
}


void moveRightMotor(float u) {
  right_speed = fabs(u);
  if (right_speed > 255) {
    right_speed = 255;
  }

  if (knobRight.read() > knobLeft.read()) {
    right_speed = v_cmd - 7;
  }

  analogWrite(EN2, right_speed);
  digitalWrite(A_3, HIGH);
  digitalWrite(A_4, LOW);
}

float pidController(float target, float kp, float kd, float ki) {
  long currentTime = micros();
  static long previousTime = 0;
  static float ePrevious = 0;
  static float eIntegral = 0;

  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  int encoderCount = knobRight.read();  // Use the count directly
  int e = encoderCount - target;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral += e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  previousTime = currentTime;
  ePrevious = e;

  return u;
}

uint16_t readDistance() {
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(0x00);  // 距离寄存器地址
  if (Wire.endTransmission() != 0) {
    return 0xFFFF;  // 传输错误，返回错误代码
  }

  Wire.requestFrom(TFLUNA_I2C_ADDR, (uint8_t)2);  // 请求2字节的数据
  if (Wire.available() < 2) {
    return 0xFFFF;  // 数据不足，返回错误代码
  }

  uint8_t lowByte = Wire.read();
  uint8_t highByte = Wire.read();
  return (highByte << 8) | lowByte;  // 合成16位的距离值
}
