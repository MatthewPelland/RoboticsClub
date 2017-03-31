#include <iostream>
#include "Robot.h"
int main() {
//	Robot robit;
//        wiringPiSetupGpio();
        Robot robit;
	robit = Robot();
	robit.testGyro();
//        digitalWrite(SOUNDLEDPIN, HIGH);

//        robit.outputGrid();
/*
        robit.moves.clear();
        robit.moves.push_back(Point(0, 20));
        robit.moveTo(robit.moves);
*/
/*
        robit.scanSurroundings();
        robit.outputGrid();
*/
/*
        robit.scanSurroundings();
        robit.createTargetPath(robit.findNextTarget());
        robit.moveTo(robit.moves);
*/

/*

*/
//      while (robit.update());
        return 0;
}
