#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>

constexpr double PI = 3.1415926535;

constexpr int FLAMESENSORTHRESHOLD = 0;
//if gridValue is below or equal to .25, treat it as clear
constexpr double CLEARTHRESHOLD = .25;
//we would have had to have had this have to have had code
//pins
constexpr int ACCELEROMETERPIN = 0;
constexpr int SONARPIN1 = 1;
constexpr int SONARPIN2 = 2;
constexpr int SONARPIN3 = 3;
constexpr int SONARPIN4 = 4;
constexpr int FLAMESENSORPIN = 5;

//gridValue constants
constexpr int UNKNOWN = -1;
constexpr int FLAME = 2;
constexpr int EXTINGUISHED = 3;
constexpr int SAFEZONE = 4;

//size constants
constexpr int ARENALENGTH_CM = 244;
constexpr int CELLSIZE_CM = 1;
constexpr int ROBOT_DIAMETER_CM = 31;
//big enough to hold entire maze no matter where we start
constexpr int GRIDSIZE_CELLS = 5 * ARENALENGTH_CM/CELLSIZE_CM;

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

struct gridVal {
	int timesScanned;
	double cellType;
	gridVal() {
		timesScanned = 0;
		cellType = 0;
	}
};

class Robot {
public:
    Robot();
    ~Robot();
    void update(); //this is where the magic happens  
private:
	Point currentPosCells; // robot's current pos
	gridVal grid[GRIDSIZE_CELLS][GRIDSIZE_CELLS];
	int distanceField[GRIDSIZE_CELLS][GRIDSIZE_CELLS];
	int createTargetPath(Point target);
	double angle;
	double angVel;
	Point target;
	std::vector<Point> moves;
	int sensorDist_cm;
	int arduinoSerial;
	
	void initialScan();
	void scanSurroundings();
	Point findClosestUnknown();
	void computeDistanceField(Point target);
	void moveTo(std::vector<Point>); 
	std::vector<Point> Robot::findOpenNeighbors(Point currentPos);
    double updateAngle(double timeDelta);
    double getSonarData(int pin);
	void returnToStart();
	//	void SonarAdjust(int * sonarData);
};

#endif // ROBOT_H_
