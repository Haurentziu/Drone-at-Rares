#include <I2Cdev.h>

/*-----------------------------------------------------------------------------------
 *                  (C) Inve_NT Colegiul National Petru Rares
 * ----------------------------------------------------------------------------------
 */

#include <Servo.h>
#include <Wire.h>

#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

#define MIN_VAL 50
#define MAX_VAL 180

#define PITCH_DEFAULT 0
#define ROLL_DEFAULT 0

#define INTERRUPT_PIN 2 //TODO change

//tune these values
#define kP 4
#define kD 3
#define kI 1

#define MOTOR_FRONT_LEFT 11
#define MOTOR_FRONT_RIGHT 10
#define MOTOR_REAR_LEFT 12
#define MOTOR_REAR_RIGHT 13

Servo frontLeft, frontRight, rearRight, rearLeft;  // motors
int ch1, ch2, ch3, ch4, ch5, ch6; //remote channels

//MPU6050 variables
MPU6050 mpu;
uint8_t intStatus;
uint8_t devStatus;
bool dmpReady = false;
uint16_t fifoCount;
uint16_t packetSize; 
uint8_t fifoBuffer[64];
VectorFloat gravity;
Quaternion quat;
volatile bool mpuInterrupt = false;
float yawPitchRoll[3];

//PID variables
float pitchLastError = 0;
float pitchIntegral = 0;
float rollLastError = 0;
float rollIntegral = 0;

int speedMotorFrontLeft = MIN_VAL;
int speedMotorFrontRight = MIN_VAL;
int speedMotorRearLeft = MIN_VAL;
int speedMotorRearRight = MIN_VAL;

void dataReady(){
  mpuInterrupt = true;
}

void setup(){
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  if (devStatus == 0) {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
  
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dataReady, RISING);
      intStatus = mpu.getIntStatus();
  
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
  
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  frontLeft.attach(MOTOR_FRONT_LEFT);
  frontRight.attach(MOTOR_FRONT_RIGHT);
  rearLeft.attach(MOTOR_REAR_LEFT);
  rearRight.attach(MOTOR_REAR_RIGHT);
  
  for(int i = 1; i <= MIN_VAL; ++i){ 
    writeMotors(i, i, i, i); 
    delay(10);        
  }

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  delay(150);    
}


void loop(){
  //reads data from the remote
  /*ch1 = pulseIn(22, HIGH);
  ch2 = pulseIn(24, HIGH);*/
  ch3 = pulseIn(26, HIGH);
  /*ch4 = pulseIn(28, HIGH);
  ch5 = pulseIn(30, HIGH);
  ch6 = pulseIn(32, HIGH);*/

  getAngles(); //reads MPU data
  printYPR();
  int spd = map(ch3, 980, 2000, MIN_VAL, MAX_VAL);
  writeMotors(spd, spd, spd, spd);
  
  stabilizeDrone();

}

void writeMotors(int frontLeftSpeed, int frontRightSpeed, int rearLeftSpeed, int rearRightSpeed) {
  frontLeft.write(frontLeftSpeed);   
  frontRight.write(frontRightSpeed);  
  rearLeft.write(rearLeftSpeed);  
  rearRight.write(rearRightSpeed);  
}

float stabilizeDrone(){
  float pitchError = yawPitchRoll[1] - PITCH_DEFAULT;
  float rollError = yawPitchRoll[2] - ROLL_DEFAULT;

  float pitchPID = computePID(pitchError, pitchLastError, &pitchIntegral);
  float rollPID = computePID(rollError, rollLastError, &rollIntegral);

    
 /* Serial.print(rollPID);
  Serial.print("    \t");
  Serial.print(pitchPID);
  Serial.print("    \t");
  Serial.print(rad2deg(yawPitchRoll[1]));
  Serial.print("    \t");
  Serial.println(rad2deg(yawPitchRoll[2]));*/

  pitchLastError = pitchError;
  rollLastError = rollError;

}

float computePID(float e, float last_e, float *integral){
  float pid;
  float dedt = e - last_e;
  integral[0] += e;

  pid = kP * e + kD * dedt + kI * integral[0];
  
  return pid;
 }



void printYPR(){
  Serial.print("ypr\t");
  Serial.print(yawPitchRoll[0]*180/3.1415);
  Serial.print("\t");
  Serial.print(yawPitchRoll[1]*180/3.1415);
  Serial.print("\t");
  Serial.println(yawPitchRoll[2]*180/3.1415);
}

void getAngles(){
  while (!mpuInterrupt && fifoCount < packetSize) {
    
  }
  
  mpuInterrupt = false;
  intStatus = mpu.getIntStatus();
  
  fifoCount = mpu.getFIFOCount();
  
  if ((intStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  
  } 
  
  else if (intStatus & 0x02) {
      while (fifoCount < packetSize){
        fifoCount = mpu.getFIFOCount();
      }
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&quat, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &quat);
      mpu.dmpGetYawPitchRoll(yawPitchRoll, &quat, &gravity);
  }
}

float rad2deg(float angle){
    return angle * 180 / PI;
}

float roll(float amount){
    if(amount < 0){
        
    }
}












