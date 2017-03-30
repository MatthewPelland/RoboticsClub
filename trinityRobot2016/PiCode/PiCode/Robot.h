#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#include <cmath>
#include <chrono>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "pins.h"
#include "sonar.h"
#include "ir.h"
#include "gyro.h"
#include "constants.h"
#include <fstream>



class Point {
public:
	int x, y;

    Point() {
	x = 0;
	y = 0;
    }

    Point(int x_, int y_) {
	x = x_;
	y = y_;
    }
  
    static double distance(Point a, Point b) {
		return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
    }

};

class DoublePoint {
public:
	double x, y;

	DoublePoint() {
		x = 0;
		y = 0;
	}

	DoublePoint(double x_, double y_) {
		x = x_;
		y = y_;
	}
};

struct gridVal {
	int timesScanned;
	double cellType;
	gridVal() {
		timesScanned = 0;
		cellType = -1;
	}
};

class Robot {
public:
    Robot(int level);
    ~Robot();
    bool update(); //this is where the magic happens  
private:
	int level; //level of the competition
	Point currentPosCells; // robot's current pos
	gridVal grid[GRIDSIZE_CELLS][GRIDSIZE_CELLS];
	int distanceField[GRIDSIZE_CELLS][GRIDSIZE_CELLS];
	double angle;
	double angVel;
	Point target;
	std::vector<Point> moves;
	MotionSensor sensor = MotionSensor(wiringPiI2CSetup(0x68));
	SonicSensor sonic0 = SonicSensor(SONIC1_TRIG, SONIC1_ECHO);//front
	SonicSensor sonic1 = SonicSensor(SONIC2_TRIG, SONIC2_ECHO);//right
	SonicSensor sonic2 = SonicSensor(SONIC3_TRIG, SONIC3_ECHO);//rear
	SonicSensor sonic3 = SonicSensor(SONIC4_TRIG, SONIC4_ECHO);//left
	IRSensor flameSensor = IRSensor(FLAMESENSORPIN);
	int sensorDist_cm;
	int arduinoSerial;
	int unextinguishedCandleCount;
	bool inRoom;
	//baby status variables
	bool babySaved;
	bool cradleFound;
	bool babyObtained;
	bool safeZoneFound;
	Point safeZoneLocation;
	std::chrono::high_resolution_clock::time_point currTime;
	std::chrono::high_resolution_clock::time_point prevTime;
	std::ofstream arduinoCommands;

	void waitForDoneConfirmation();
	void outputGrid();
	void updateTime();
	std::vector<Point> findAberrantMinimums(double sonarData[360], const double slope_threshold, const double aberration_size_threshold);
	std::vector<Point> locateCandles(double sonarData[4][360]);
	void initialScan();
	void scanSurroundings(bool ignoreCandles);
	void updateGridVal(int cellX, int cellY, int type);
	Point findNextTarget(bool);
	void computeDistanceField(Point target);
	Point closestOpenCell(Point target);
	int distanceToWall(int x, int y);
	void moveTo(std::vector<Point>, bool takePictures); 
	int createTargetPath(Point target, int thresholdDistance);
	void extinguishCandle(Point target);
	std::vector<Point> Robot::findOpenNeighbors(Point currentPos);
    double updateAngle(double timeDelta);
	void routeToStart();
	void goSaveBaby();
	double distance(Point a, Point b);
	std::vector<Point> checkForCradle(double sonarData[4][360]);
	std::vector<Point> checkForWindow(double sonarData[4][360]);
};

#endif // ROBOT_H_