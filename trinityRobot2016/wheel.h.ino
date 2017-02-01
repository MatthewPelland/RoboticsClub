#ifndef wheel_h
#define wheel_h
#include "Arduino.h"

class Wheel {
  public: 
    Wheel(int eP, float ROBOT_RADIUS)
    driveInches(int inches, int power, int motor);
    drive(float rad, float inches);

  private:
    int encoderValue;
    int encoderPin;
    int lastEncoder;
    int currentEncoder;
    updateEncoder();
    inchesToEncoder(int inches);
    encoderToInches(int encoder);
	float PID(float current, float setpoint, float KI, float KP, float KD, float MAX_OUTPUT);
	float distance(float xi, float yi, float xf, float yf);
	float angle(float xi, float yi, float xf, float yf);
	float distancex(float rad);
	float distancey(float rad);
	float getLinearSpeedEncoder();
	float getAngularSpeedEncoder();
	float getSpeedAccel();
	float getSpeedGyro();
};

#endif
