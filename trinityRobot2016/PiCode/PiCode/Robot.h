#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>

#include "pins.h"
#include "sonic.h"
#include "ir.h"
#include "motion.h"
#include "constants.h"
#include "Point.h"
#include "doublePoint.h"
#include "gridVal.h"

class Robot {
public:
	Robot();
//    Robot(int aLevel);
//    ~Robot();
    bool update(); //this is where the magic happens  
	FILE* fp;
	int level; //level of the competition
	Point currentPosCells; // robot's current pos
	gridVal** grid;
	int** distanceField;
	double angle;
	double angVel;
	std::vector<Point> moves;
	SonicSensor sonic0 = SonicSensor(SONIC1_TRIG, SONIC1_ECHO);//front
	SonicSensor sonic1 = SonicSensor(SONIC2_TRIG, SONIC2_ECHO);//right
	SonicSensor sonic2 = SonicSensor(SONIC3_TRIG, SONIC3_ECHO);//rear
	SonicSensor sonic3 = SonicSensor(SONIC4_TRIG, SONIC4_ECHO);//left
	IRSensor flameSensor = IRSensor(FLAMESENSORPIN);
	MotionSensor sensor = MotionSensor(wiringPiI2CSetup(0x68));

	int sensorDist_cm;
	int arduinoSerial;
	int unextinguishedCandleCount;
	int inRoom;
	//baby status variables
	bool babySaved;
	bool cradleFound;
	bool babyObtained;
	bool safeZoneFound;
	Point safeZoneLocation;
	std::chrono::high_resolution_clock::time_point currTime;
	std::chrono::high_resolution_clock::time_point prevTime;
	std::chrono::high_resolution_clock::time_point startTime;
//	std::ofstream arduinoCommands;
	void testGyro();
	void takePicture();
	bool is_diagonal_candidate(int x, int y);
	double customAtan(double x, double y);
	bool checkImageForCradle();
	bool checkImageForSafeZone();
	void waitForDoneConfirmation();
	void outputGrid();
	void updateTime();
	std::vector<Point> findAberrantMinimums(double sonarData[360], const double slope_threshold, const double aberration_size_threshold);
	std::vector<Point> locateCandles(double sonarData[4][360]);
	void initialScan();
	void scanSurroundings(bool ignoreCandles = false);
	void updateGridVal(int cellX, int cellY, int type);
	Point findNextTarget(bool ignoreCandles = false);
	void computeDistanceField(Point target);
	int sizeOfUnknown(Point unk);
	void clearUnknownRegion(Point unk);
	Point closestOpenCell(Point target);
	int distanceToWall(int x, int y);
	void moveTo(std::vector<Point>, bool takePictures = false); 
	int createTargetPath(Point target, int thresholdDistance = 100);
	void extinguishCandle(Point target);
	std::vector<Point> findOpenNeighbors(Point currentPos);
        double updateAngle(double timeDelta);
	void routeToStart();
	void goSaveBaby();
	double distance(Point a, Point b);
	std::vector<Point> checkForCradle(double sonarData[4][360]);
	std::vector<Point> checkForWindow(double sonarData[4][360]);

};

#endif // ROBOT_H_
