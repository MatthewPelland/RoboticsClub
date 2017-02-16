#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>

constexpr double PI = 3.1415926535;
constexpr int ACCELEROMETERPIN = 0;
constexpr int SONARPIN1 = 1;
constexpr int SONARPIN2 = 2;
constexpr int SONARPIN3 = 3;
constexpr int SONARPIN4 = 4;

constexpr int ARENALENGTH_CM = 244;
constexpr int CELLSIZE_CM = 1;
constexpr int ROBOT_DIAMETER_CM = 31;
//big enough to hold entire maze no matter where we start
constexpr int GRIDSIZE_CELLS = 5 * ARENALENGTH_CM/CELLSIZE_CM;

enum Cell { UNKNOWN, WALL, CLEAR };

class Point {
    int x, y;

    vector2i() {
	x = 0;
	y = 0;
    }

    vector2i(int x_, int y_) {
	x = x_;
	y = y_;
    }
  
    static double distance(Point a, Point b) {
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
    }

};

class Robot {
public:
    Robot();
    ~Robot();
    //Arduino Commands
    void moveRobotTo(std::vector<Point>); //Updates position values
    void SonarAdjust(int * sonarData);

    //PI Commands
    void update(); //this is where the magic happens

    void scanSurroundings();
    void initialScan();

    Point currentPos; // robot's current pos
    double angle;
    double angVel;
    int grid[GRIDSIZE_CELLS][GRIDSIZE_CELLS];
    int distanceField[gridSize][gridSize];
  
private:
    int create_target_path();
    bool advance();
    void openNeighbors(Point &coords, std::vector<Point> *openNeighbors);
    Point findClosestUnknown();
    double updateAngle(double timeDelta);
    std::vector<Point> moves;

    double getAcceleration(int pin);
    void cleanSonarData(double * sonarArrays, int arrayLength);
    double getSonarData(int pin);
    Point target;
    int sensorDist_cm;

    int arduinoSerial;
};

#endif // ROBOT_H_
