
#include<stdio.h>
#include<stdlib.h>


void main(void) {
	char* string = (char*)malloc(100);

	scanf("%s", &string);
	int x; 
	sscanf(string, "%d", &x);
	printf("%d", x);
}

//#include "Robot.h"
//#include <wiringpi.h>
//
//void main() {
//	wiringPiSetupGPIO();
//	//pinModes
//	int count = 0;
//	while (1) {
//		if (detectStartingTone())
//			count++;
//		else
//			count = 0;
//		if (count >= 20)
//			doThatRobotThang();
//	}
//}
//
//bool detectStartingTone() {
//	//do stuff
//}
//
//void doThatRobotThang() {
//	digitalWrite(SOUNDLED, HIGH);
//	Robot robit = Robot(3);
//	while (1)
//		robit.update();
//}