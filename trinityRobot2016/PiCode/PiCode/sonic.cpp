#include <ctime>
#include <iostream>
#include <wiringPi.h>

#include "sonic.h"

SonicSensor::SonicSensor(int trig, int echo) {
    if(wiringPiSetupGpio() == -1){
       std::cout << "ATTEMPTING TO USE SONAR SENSOR BEFORE WIRINGPIGPIO INITIALIZATION!" << std::endl;
    }
	this->trig = trig;
	this->echo = echo;

	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
}

double SonicSensor::getDistance() {
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	time_t initialTime = clock();
	time_t startTime = clock();
	time_t stopTime = clock();

	while (digitalRead(echo) == 0) {
		startTime = clock();
		if((startTime - initialTime) > 1000000){
			std::cout << "sonic timeout" << std::endl;
			return -1;
		}
	}

	while (digitalRead(echo) == 1) {
		stopTime = clock();
		if((stopTime - initialTime) > 2000000){
			std::cout << "sonic timeout" << std::endl;
			return -1;
		}
	}

	time_t timeElapsed = stopTime - startTime;

	return (timeElapsed * 343.0) / 20000.0;
}

