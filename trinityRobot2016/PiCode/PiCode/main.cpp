#include "Robot.h"



void main() {
	//wiringPiSetup();
	//pinModes
	Robot robit = Robot();
	while (1)
		robit.update();
}