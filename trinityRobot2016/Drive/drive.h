#ifndef wheel_h
#define wheel_h
#include <Wire.h>
#include "Arduino.h"
#include "PID_v1.h"
#include "math.h"
#include "Adafruit_TCS34725.h"

#define ENCODER_PIN1A 10 //X
#define ENCODER_PIN1B 10;
#define ENCODER_PIN2A 10 //X
#define ENCODER_PIN2B 10
#define ENCODER_PIN3A 10 //Y
#define ENCODER_PIN3B 10
#define ENCODER_PIN4A 10 //Y
#define ENCODER_PIN4B 10
#define MOTOR_PIN1 10 //X
#define MOTOR_PIN2 10 //X
#define MOTOR_PIN3 10 //Y
#define MOTOR_PIN4 10 //Y
#define COLOR_PIN 10
#define WHEEL_RADIUS 10 
#define ROBOT_RADIUS 10

class Drive {
  public: 
    Drive();
    void drive(int x, int y);
	void turn(int degrees);
	void setInitialPos(int x, int y, int deg);
  private:
	Adafruit_TCS34725 color;
	int time, lastTime, totalDeg;
    int encoderValue[4];
    int lastEncoder[8];
    int currentEncoder[8];
	int currentXPos;
	int currentYPos;
	bool inRoom;
    void updateEncoder(int encoderNum);
	void updateTime();
	void updateInRoom();
    int cmToEncoder(int cm);
    int encoderToCm(int encoder);
	double getLinearSpeedEncoder(char axis);
	double getAngularSpeedEncoder(char axis);
	double getSpeedAccel(char axis, double vel);
	double getSpeedGyro();
	double getGyroData();
	double getAccelerometerData(char axis);
};

#endif
