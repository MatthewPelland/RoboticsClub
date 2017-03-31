
#include "Robot.h"

int main() {
	wiringPiSetupGpio();
	Robot robit = {1};
	digitalWrite(SOUNDLED, HIGH);
	while (robit.update());
	return 0;
}
