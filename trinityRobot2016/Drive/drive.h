#ifndef wheel_h
#define wheel_h
#include <wire.h>
#include "Arduino.h"
#include "PID_v1.h"
#include "math.h"
#include "Adafruit_TCS34725.h"

#define ENCODER_PIN1A 10 //X
#define ENCODER_PIN1B 10;
#define ENCODER_PIN2A 10 //X
#define ENCODER_PIN2B
#define ENCODER_PIN3A 10 //Y
#define ENCODER_PIN3B
#define ENCODER_PIN4A 10 //Y
#define ENCODER_PIN4B
#define MOTOR_PIN1 10 //X
#define MOTOR_PIN2 10 //X
#define MOTOR_PIN3 10 //Y
#define MOTOR_PIN4 10 //Y
#define COLOR_PIN 10
#define WHEEL_RADIUS 10 
#define ROBOT_RADIUS 10

class Drive {
  public: 
    Drive()
    drive(int x, int y);
	turn(int degrees);

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
	float getLinearSpeedEncoder(char axis);
	float getAngularSpeedEncoder(char axis, float vel);
	float getSpeedAccel();
	float getSpeedGyro();
	float getAccelerometerData(char axis);
};

#endif
