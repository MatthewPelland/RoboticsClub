
#include <wiringPi.h>
#include "pins.h"

#include "ir.h"

IRSensor::IRSensor(int pin){
    this->pin = pin;
    pinMode(pin, INPUT);
}

bool IRSensor::fireDetected(){
    return !digitalRead(pin);
}

