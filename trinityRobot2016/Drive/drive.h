#ifndef wheel_h
#define wheel_h

#include <Wire.h>
#include "AFMotor.h"
#include "Arduino.h"
#include "PID_v1.h"
#include "math.h"
#include "Adafruit_TCS34725.h"

#define ENCODER_PIN1A 18 //X
#define ENCODER_PIN1B 40
#define ENCODER_PIN2A 19 //X
#define ENCODER_PIN2B 42
#define ENCODER_PIN3A 20 //Y
#define ENCODER_PIN3B 44
#define ENCODER_PIN4A 21 //Y
#define ENCODER_PIN4B 46
#define COLOR_PIN 10
#define WHEEL_RADIUS 4.826/2 //cm
#define ROBOT_RADIUS 35.5 //cm


class Drive {
  public: 
    Drive();
    void drive(int x, int y, int max_speed);
	void turn(int degrees, int max_speed);
	void setInitialPos(int x, int y, int deg);
	void go(double, double);
	AF_DCMotor motor1 = AF_DCMotor(1);
	AF_DCMotor motor2 = AF_DCMotor(2);
	AF_DCMotor motor3 = AF_DCMotor(3);
	AF_DCMotor motor4 = AF_DCMotor(4);
	
  private:
	Adafruit_TCS34725 color;

	int time, lastTime;
	int totalDeg;
    int lastEncoder[4];
    int currentEncoder[4];
	double currentXPos;
	double currentYPos;
	bool inRoom;
    void updateEncoder(int encoderNum);
	void updateTime();
	void updateInRoom();
    double cmToEncoder(int cm);
    double encoderToCm(int encoder);
	double getLinearSpeedEncoder(char axis);
	double getAngularSpeedEncoder(char axis);
	double getSpeedAccel(char axis, double vel);
	double getSpeedGyro();
	double getGyroData();
	double getAccelerometerData(char axis);
};

#endif