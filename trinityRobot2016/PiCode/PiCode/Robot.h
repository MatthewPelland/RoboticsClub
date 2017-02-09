
#include <vector>
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>
#define M_PI 3.1415926535

enum Cell { UNKNOWN, WALL, CLEAR };

#define ACCELEROMETERPIN 0 
#define SONARPIN1 1
#define SONARPIN2 2
#define SONARPIN3 3
#define SONARPIN4 4

#define ARENALENGTH 244 //centimeters
#define CELLSIZE 1 //measured in centimeters
#define ROBOTSIZE 31 //robot diameter in cm
#define GRIDSIZE int(4.5 * arenaLength/cellSize)//big enough to hold entire maze no matter where we start

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

  void scan_surroundings();
  void initialScan();

  Point gridPos; // robot's current pos
  double angle;
  double angVel;
  int grid[GRIDSIZE][GRIDSIZE];
  int distanceField[gridSize][gridSize]; // 
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
  int sensorDist;//measured in centimeters

  int arduinoSerial;
};

