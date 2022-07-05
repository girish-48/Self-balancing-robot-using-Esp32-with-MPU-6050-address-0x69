#include "Wire.h"
#include <MPU6050_light.h>

int rotation_duration = 100;
int max1 = 0;
int max2 = 0;
  
// Motor A
int motor1Pin1 = 14;
int motor1Pin2 = 12;
int enable1Pin = 13;

// Motor B
int motor2Pin1 = 27;
int motor2Pin2 = 26;
int enable2Pin = 25;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
// resolution is 8, 2^8 = 256. So duty cycle should take values from 0 to 255. Map it accordingly to percentage.

// PID constants
double kp = 25;
double ki = 0;//100;
double kd = 0;//0.2;
float gyroAngle = 0;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void moveForward(int speedDuty)
{ // speedDuty is an int from 0 to 255

  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  ledcWrite(pwmChannel, speedDuty);
  Serial.print("Forward with duty cycle: ");
  Serial.println(speedDuty);
  // should maintain this signal until it starts rotating in the opposite direction
  volatile long int prev_t = millis();
  while(millis() - prev_t < rotation_duration){mpu.update();};
 
}

void moveBackward(int speedDuty)
{

  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  ledcWrite(pwmChannel, speedDuty);
  Serial.print("Backward with duty cycle: ");
  Serial.println(speedDuty);
  // should maintain this signal until it starts rotating in the opposite direction

  volatile long int prev_t = millis();
  while(millis() - prev_t < rotation_duration){mpu.update();};
}



void setup(void)
{
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LED Control PWM functionalities -- setting up a PWM channel to create the signal (there are 16 PWM Channels from 0 to 15)
  // LEDC is used to generate PWM signals, but not necessarily only for LEDs
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled -- channeling the same PWM signal through enable1Pin and enable2Pin
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);
  
}

void loop()
{
   mpu.update();
  
  //if((millis()-timer)>10){ // print data every 10ms
    gyroAngle = mpu.getAngleY();
  Serial.print(" Y : ");
  Serial.println(gyroAngle);
  //timer = millis();  

  // NOW NEED MAKE IT SELF-BALANCE

  int speeda; // duty cycle 0 to 255 for motor

  // PID Control Algorithm

  // Calculate angle of inclination
  currentTime = millis();                             // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation

  error =  gyroAngle;                       // determine error
  cumError += error * elapsedTime/1000;               // compute integral
  rateError = (error - lastError)*1000 / elapsedTime; // compute derivative
  //[-174, -173] --> [0,7]
  
  if(cumError > -2500 && cumError < 2500) {}
  else cumError = 0;
  
  float out = kp * error ;//+ ki * cumError + kd * rateError; // PID output
  Serial.print("error :");
  Serial.print(error);
  Serial.print(", cumError :");
  Serial.print(cumError);
  Serial.print(", rateError :");
  Serial.println(rateError);
  Serial.print("gyroangle: ");
  Serial.println(error);
  Serial.print("Out: ");
  Serial.println(out);
  lastError = error; // remember current error
  int base_speed = 100;
  int c = 155;
  if (out < 0)
  {
    // -ve error means + y axis will tilt downwards
    //speeda = base_speed + (int)(-out);
    if( max1 < int(-out)) max1 = -out;
    speeda = base_speed + map(-out,0,max1,0,c);
    moveForward(speeda); // along + y
  }
  else
  {
    //speeda = base_speed + out;
    if( max2 < out) max2 = out;
    speeda = base_speed + map(out,0,max2,0,c);
    moveBackward(speeda); // along - y
  }

  previousTime = currentTime;

}
