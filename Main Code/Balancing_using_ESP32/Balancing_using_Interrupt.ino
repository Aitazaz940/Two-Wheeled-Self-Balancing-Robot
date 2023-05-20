#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP32Encoder.h>
#include <PID_v1.h>

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/********* Defining Motor Pins *********/

ESP32Encoder encoder1, encoder2;

const int ENA = 13;
const int IN1 = 16;
const int IN2 = 17;

const int ENB = 14;
const int IN3 = 18;
const int IN4 = 19;

/********* Defining PID Variables *********/

double measured_rpm1, PID_Output1;
double measured_rpm2, PID_Output2;
double input, output;

double setpoint = 185;

/********* Tuning PID Parameters *********/

double Kp = 3;
double Kd = 0.005;
double Ki = 1;

double Kp1 = 1.8;
double Kd1 = 0.008;
double Ki1 = 10;

double Kp2 = 1.8;
double Kd2 = 0.008;
double Ki2 = 10;

/********* Creating PID Objects *********/

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID pid1(&measured_rpm1, &PID_Output1, &output, Kp1, Ki1, Kd1, DIRECT);
PID pid2(&measured_rpm2, &PID_Output2, &output, Kp2, Ki2, Kd2, DIRECT);

/********* Other Definitions *********/

int motorDutyCycle1 = 0;
int motorDutyCycle2 = 0;

int prev_speed1 = 0;
int prev_speed2 = 0;

/// ---------------------------------------------------------- ///
/// Setting PWM Properties for ANALOG WRITE in ESP32 USING LEDC

const int freq1 = 2000;    /// Frequency default is 10khz
const int pwmChannel1 = 0;  /// Channel for PWM
const int resolution1 = 8;  /// 2 ^ 8 = 255 max limit

const int freq2 = 2000;
const int pwmChannel2 = 1;
const int resolution2 = 8;

/// ---------------------------------------------------------- ///

const int sampleTime = 30;
const float delta_t_mins = (float)(sampleTime/1000.0) / 60.0;

const float coutsPerDivEnc = 1505.0;

unsigned int crnt_count1, prev_count1, count_diff1;
unsigned int crnt_count2, prev_count2, count_diff2;

unsigned long crnt_t, prev_t;

volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high --> 23 PIN for ESP32

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() 
{
  Wire.begin();  
  Serial.begin(115200);
  
  pinMode(ENA, OUTPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  ledcSetup(pwmChannel1, freq1, resolution1);
  ledcSetup(pwmChannel2, freq2, resolution2);
  
  ledcAttachPin(ENA, pwmChannel1);
  ledcAttachPin(ENB, pwmChannel2);

  encoder1.attachSingleEdge(26, 27);
  encoder2.attachSingleEdge(32, 33);

  prev_t = 0;  
  prev_count1 = 0;
  prev_count2 = 0;

  pid1.SetMode(AUTOMATIC);
  pid1.SetOutputLimits(0, 255);

  pid2.SetMode(AUTOMATIC);
  pid2.SetOutputLimits(0, 255);
  
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(75);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(8);
  mpu.setZAccelOffset(1051);
  
  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpu.setRate(3);
    Serial.println(F("Enabling interrupt detection (ESP32 External Interrupt at PIN 23..."));
    attachInterrupt(23, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);  
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void pidCompute()
{
  crnt_t = millis();
  
  crnt_count1 = 0;
  count_diff1 = 0;
  measured_rpm1 = 0.0;

  crnt_count2 = 0;
  count_diff2 = 0;
  measured_rpm2 = 0.0;

  if(crnt_t - prev_t > sampleTime)       // do this after every 100 ms
  {
    prev_t = crnt_t;
    
    crnt_count1 = encoder1.getCount();
    crnt_count2 = encoder2.getCount();
    
    count_diff1 = crnt_count1 - prev_count1;
    count_diff2 = crnt_count2 - prev_count2;
    
    prev_count1 = crnt_count1;
    prev_count2 = crnt_count2;
    
    measured_rpm1 = (float)(count_diff1/coutsPerDivEnc)/delta_t_mins;
    measured_rpm2 = (float)(count_diff2/coutsPerDivEnc)/delta_t_mins;
    
    pid1.Compute();
    pid2.Compute();
    
    PID_Output1 = abs(PID_Output1);
    PID_Output2 = abs(PID_Output2);
    
    motorDutyCycle1 = (int) abs(PID_Output1); 
    motorDutyCycle2 = (int) abs(PID_Output2); 
    
    prev_speed1 = motorDutyCycle1;
    prev_speed2 = motorDutyCycle2;
    
    ledcWrite(pwmChannel1, motorDutyCycle1);
    ledcWrite(pwmChannel2, motorDutyCycle2);

    int a = 0;
    int b = 170;
        
//    Serial.print("    setpoint: ");
//    Serial.print(setpoint);
// 
//    Serial.print("    MPU_Input: ");
//    Serial.print(input);
//    Serial.print("    MPU_Output: ");
//    Serial.println(output);
//    Serial.print("    Measured RPM 1: ");
//    Serial.print(measured_rpm1);
//    Serial.print("    Measured RPM 2: ");
//    Serial.println(measured_rpm2);
  }
}

void Forward()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void Reverse()
{
  
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void Stop()
{
  
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void loop() 
{
  //pidCompute();
  
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    pidCompute();
    pid.Compute();
    output = abs(output);
    
    if(input > setpoint)
    {
      Forward();
    }
    else if(input == setpoint)
    {
      Stop();
    }
    else
    {
      Reverse();
    }
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  if((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if(mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);                 //get value for q
    mpu.dmpGetGravity(&gravity, &q);                      //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);            //get value for ypr
    input = ypr[1] * 180/M_PI + 180;
  }
}
