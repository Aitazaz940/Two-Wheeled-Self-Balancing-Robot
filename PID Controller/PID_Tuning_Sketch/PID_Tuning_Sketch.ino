#include <ESP32Encoder.h>
#include <PID_v1.h>>

ESP32Encoder encoder;

const int ENA = 13;
const int IN1 = 16;
const int IN2 = 17;

double Kp = 2;
double Kd = 0.008;
double Ki = 10;

//double Kp = 0.45;
//double Kd = 0.03;
//double Ki = 2;

double measured_rpm, PID_Output, desired_rpm;

PID pid(&measured_rpm, &PID_Output, &desired_rpm, Kp, Ki, Kd, DIRECT);

int motorDutyCycle = 0;

int prev_speed = 0;

/// ---------------------------------------------------------- ///
/// Setting PWM Properties for ANALOG WRITE in ESP32 USING LEDC

const int freq = 2000;    /// Frequency default is 10khz
const int pwmChannel = 0;  /// Channel for PWM
const int resolution = 8;  /// 2 ^ 8 = 255 max limit

/// ---------------------------------------------------------- ///

const int sampleTime = 100;
const float delta_t_mins = (float)(sampleTime/1000.0) / 60.0;

const float coutsPerDivEnc = 530.0;

unsigned int crnt_count, prev_count, count_diff;

unsigned long crnt_t, prev_t;
unsigned long delay_start_t = 0;

void setup()
{
  Serial.begin(115200);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENA, pwmChannel);
  
  encoder.attachSingleEdge(26, 27);
  
  prev_t = 0;  
  prev_count = 0;

  desired_rpm = 80;
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
}

void pidCompute()
{
  crnt_t = millis();
  
  crnt_count = 0;
  count_diff = 0;
  measured_rpm = 0.0;

  if(crnt_t - prev_t > sampleTime)
  {
    prev_t = crnt_t;
    
    crnt_count = encoder.getCount();
    
    count_diff = crnt_count - prev_count;
    
    prev_count = crnt_count;
    
    measured_rpm = (float)(count_diff/coutsPerDivEnc)/delta_t_mins;
    
    pid.Compute();
    
    motorDutyCycle = (int) abs(PID_Output); 
    
    prev_speed = motorDutyCycle;
    
    ledcWrite(pwmChannel, motorDutyCycle);

    Serial.print("    Desired_RPM: ");
    Serial.print(desired_rpm);
    Serial.print("    Measured_RPM_1: ");
    Serial.println(measured_rpm);
  }
}

void loop()
{
  pidCompute();

  if (delay_start_t == 0)
  {
    delay_start_t = millis();
  }
  else if (millis() - delay_start_t >= 10000)
  {
    if (desired_rpm == 200)
    {
      desired_rpm = 80;
    } 
    else if (desired_rpm == 80)
    {
      desired_rpm = 240;
    }
    else 
    {
      desired_rpm = 200;
    }
    delay_start_t = millis();
  }
}
