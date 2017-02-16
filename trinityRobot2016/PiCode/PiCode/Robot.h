
#include <vector>
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>
#define M_PI 3.1415926535

#define UNKNOWN -1
#define FLAME 2
#define EXTINGUISHED 3
#define SAFEZONE 4

#define ACCELEROMETERPIN 0 
#define SONARPIN1 1
#define SONARPIN2 2
#define SONARPIN3 3
#define SONARPIN4 4

#define ARENALENGTH 244 //centimeters
#define CELLSIZE 1 //measured in centimeters
#define ROBOTSIZE 31 //robot diameter in cm
#define GRIDSIZE int(4.5 * ARENALENGTH/CELLSIZE)//big enough to hold entire maze no matter where we start
#define CLEAR_THRESHOLD 0.25

typedef struct vector2d {
	double x, y;

	vector2d() {
		x = 0;
		y = 0;
	}

	vector2d(double x, double y) {
		this->x = x;
		this->y = y;
	}
} vec2d;

typedef struct vector2i {
	int x, y;

	vector2i() {
		x = 0;
		y = 0;
	}

	vector2i(int x, int y) {
		this->x = x;
		this->y = y;
	}
} vec2i;

typedef struct gridValueVector {
	int timesScanned;
	double cellType;

	gridValueVector() {
		timesScanned = 0;
		cellType = UNKNOWN;
	}

	gridValueVector(int timesScanned, double cellType) {
		this->timesScanned = timesScanned;
		this->cellType = cellType;
		//number from 0-1 signifies wall/clear
		//UNKNOWN -1
		//FLAME 2
		//EXTINGUISHED 3
		//SAFEZONE 4
	}

}gridVal;

class Robot {
public:
	Robot();
	~Robot();
	//Arduino Commands
	void moveRobotTo(std::vector<vec2i>); //Updates position values
	void SonarAdjust(int * sonarData);

	//PI Commands
	void update(); //this is where the magic happens

	void scan_surroundings();
	void initialScan();
	void computeDistanceField(vec2i target);

	vec2i gridPos;
	double angle;
	double angVel;
	gridVal grid[GRIDSIZE][GRIDSIZE];
	int distanceField[GRIDSIZE][GRIDSIZE];
private:
	int createTargetPath(vec2i target);
	void openNeighbors(vec2i &coords, std::vector<vec2i> *openNeighbors);
	vec2i findClosestUnknown();
	inline double distance(vec2i a, vec2i b);
	double updateAngle(double timeDelta);
	std::vector<vec2i> moves;

	double getAngularVelocity(int pin);
	void cleanSonarData(double * sonarArrays, int arrayLength);
	double getSonarData(int pin);
	vec2i target;
	int sensorDist;//measured in centimeters

	int arduinoSerial;
	void returnToStart();
};

