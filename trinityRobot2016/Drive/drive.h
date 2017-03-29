#ifndef wheel_h
#define wheel_h
#include <Wire.h>
#include "AFMotor.h"
#include "Arduino.h"
#include "PID_v1.h"
#include "math.h"
#include "Adafruit_TCS34725.h"

#define ENCODER_PIN1A 40 //X
#define ENCODER_PIN1B 41;
#define ENCODER_PIN2A 42 //X
#define ENCODER_PIN2B 43
#define ENCODER_PIN3A 44 //Y
#define ENCODER_PIN3B 45
#define ENCODER_PIN4A 46 //Y
#define ENCODER_PIN4B 47
//#define MOTOR_PIN1 10 //X
//#define MOTOR_PIN2 10 //X
//#define MOTOR_PIN3 10 //Y
//#define MOTOR_PIN4 10 //Y
#define COLOR_PIN 10
#define WHEEL_RADIUS 4.826 //cm
#define ROBOT_RADIUS 35.5 //cm

class Drive {
  public: 
    Drive();
    void drive(int x, int y, int max_speed);
	void turn(int degrees, int max_speed);
	void setInitialPos(int x, int y, int deg);
	void go(int speed1, int speed2, int speed3, int speed4);
  private:
	Adafruit_TCS34725 color;
	AF_DCMotor motor1 = AF_DCMotor(1);
	AF_DCMotor motor2 = AF_DCMotor(2);
	AF_DCMotor motor3 = AF_DCMotor(3);
	AF_DCMotor motor4 = AF_DCMotor(4);
	

	double angle;
	int time, lastTime, totalDeg;
    int encoderValue[4];
    int lastEncoder[8];
    int currentEncoder[8];
	double currentXPos;
	double currentYPos;
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
