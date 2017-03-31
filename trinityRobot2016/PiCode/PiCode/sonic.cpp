#include <ctime>
#include <wiringPi.h>

#include "sonic.h"

SonicSensor::SonicSensor(int trig, int echo) {
	this->trig = trig;
	this->echo = echo;

	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
}

double SonicSensor::getDistance() {
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	time_t startTime = clock();
	time_t stopTime = clock();

	while (digitalRead(echo) == 0) {
		startTime = clock();
	}

	while (digitalRead(echo) == 1) {
		stopTime = clock();
	}

	time_t timeElapsed = stopTime - startTime;

	return (timeElapsed * 343.0) / 20000.0;
}

