
#include "Robot.h"
#include <wiringpi.h>

void main() {
	wiringPiSetupGPIO();
	Robot robit = Robot(3);
	digitalWrite(SOUNDLED, HIGH);
	while (robit.update());
}
