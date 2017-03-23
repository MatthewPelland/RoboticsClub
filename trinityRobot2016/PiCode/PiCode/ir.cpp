#include <wiringPi.h>
#include <mcp3004.h>
#include "pins.h"

#include "ir.h"

IRSensor::IRSensor(int analogIn) {
	mcp3004Setup(BASE, SPI_CHAN);
	analogPin = analogIn;
	pinMode(analogPin, INPUT);
}

int IRSensor::getFireIntensity() {
	return analogRead(A0);
}