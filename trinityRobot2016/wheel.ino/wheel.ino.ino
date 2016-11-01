#include "Arduino.h"
#include math.h
#include "wheel.h"

wheel::Wheel(int eP)
{
  encoderPin = eP;
  pinMode(encoderPin, INPUT);
  
  lastEncoder = digitalRead(encoderPin);
  currentEncoder = lastEncoder;
  encoderValue = 0;
}

//drives motors; I don't know how this will work yet
//power must be 0<=power<=255
void wheel::driveInches(float inches, int power, int motor){
  
  pinMode(BRAKE[motor], OUTPUT);  // Brake pin on
  digitalWrite(BRAKE[motor], LOW);  // setting brake LOW disable motor brake
  
  analogWrite(PWM[motor], power);     // Set the speed of the motor 
  encoderValue = 0;
  while(encoderValue < inchesToEncoder(inches)){
    updateEncoder();
  }
  digitalWrite(BRAKE[motor], HIGH);  // raise the brake
  encoder_val[motor] = encoderValue; //final encoder value of each motor
}

//updates the encoder value (instantaneous; not the count)
void wheel::updateEncoder(){
  currentEncoder = digitalRead(encoderPin);
  if(currentEncoder != lastEncoder){
    encoderValue++;
  }
  lastEncoder = currentEncoder;
}

//calculates the PID error
float wheel::PID(float value1, float value2)
{
}

//the distance between 2 points
float wheel::distance(float xi, float yi, float xf, float yf)
{
  return sqrt (((xi-xf)^2) +((yi-yf)^2));
}

//the angle between 2 points in rad
float wheel::angle(float xi, float yi, float xf, float yf)
{
  return atan((yi-yf)/(xi-xf));
}

//the distance traveled by the robot in x based on wheel movements
float wheel::distancex(float rad){
  float d1 = encoderToInches(encoder_val[1]);
  float d3 = encoderToInches(encoder_val[3]);
  return (d1+d3)/(2*cos(rad));
}

//the distance traveled by the robot in y based on wheel movements
float wheel::distancey(){
  float d2 = encoderToInches(encoder_val[2]);
  float d4 = encoderToInches(encoder_val[4]);
  return (d2+d4)/(2*cos(rad));
}

//drives the robot
void wheel::drive(float rad, float inches, float power)
{
  time_init = millis();
  for (int n = 0; n < 100;n++)
  {/*
    compute error (tells you if you’ve slipped)
    Error between encoder speed (linear) and accelerometer speed
    Error between encoder speed (angular) and gyro speed
    If these errors are large
      The wheels have slipped or jumped
      Use sonar to recalculate position
      recalculate angle and distance
    end
    */
    float error_dist_x = PID(distancex,
    
    }

    
  
}
}


