
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

	vec2i gridPos;
	double angle;
	double angVel;
	int grid[GRIDSIZE][GRIDSIZE];
	int distanceField[gridSize][gridSize];
private:
	int create_target_path();
	bool advance();
	void openNeighbors(vec2i &coords, std::vector<vec2i> *openNeighbors);
	vec2i findClosestUnknown();
	inline double distance(vec2i a, vec2i b);
	double updateAngle(double timeDelta);
	std::vector<vec2i> moves;

	double getAcceleration(int pin);
	void cleanSonarData(double * sonarArrays, int arrayLength);
	double getSonarData(int pin);
	vec2i target;
	int sensorDist;//measured in centimeters

	int arduinoSerial;
};

