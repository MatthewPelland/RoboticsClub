

#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "Robot.h"

int main() {
    if(wiringPiSetupGpio() < 0){
		std::cout << "NOPE" << std::endl;
		exit(1);
	}
    Robot robit;

	//robit = Robot();
//	std::cout<< "hello" << std::flush;
	serialFlush(robit.arduinoSerial);
//	char* buffer = "c";
//	while(robit.update());
//	robit.scanSurroundings();
//	robit.outputGrid();

//int fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
//if(fd < 0)
//    ;  // handle error

// Try to write some data
//ssize_t written = write(fd, "data", 4);
//if(written >= 0)
 //   std::cout << "we supposedly wrote" << std::flush;  // handle successful write (which might be a partial write!)
//else if(errno == EWOULDBLOCK)
//    std::cout << "BLOCKING" << std::flush;  // handle case where the write would block
//else
//    ;  // handle real error


//	while(serialDataAvail(robit.arduinoSerial) == 0)
//		std::cout << "waiting" << std::flush;
//	char get = serialGetchar(robit.arduinoSerial);
//	std::cout << get << std::flush;
//	robit.testGyro();
//        digitalWrite(SOUNDLEDPIN, HIGH);

//        robit.outputGrid();

//        robit.moves.clear();
//        robit.moves.push_back(Point(0, 20));
//        robit.moveTo(robit.moves);

while(1){
	std::cout << "scanSurroundings" << std::endl;

        robit.scanSurroundings();

	std::cout << "outputting" << std::endl;

        robit.outputGrid();

	robit.createTargetPath(robit.findNextTarget());

	std::cout << "nummoves" << robit.moves.size() << std::endl;

	robit.moveTo(robit.moves);

	std::cout << "program completed brah" << std::endl;
}


/*
        robit.scanSurroundings();
        robit.createTargetPath(robit.findNextTarget());
        robit.moveTo(robit.moves);
*/

/*

*/
//      while (robit.update());
 //       return 0;
}
