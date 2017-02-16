#include "Robot.h"
// For std::sleep
#include <thread>
#include <chrono>

Robot::Robot() {
    this -> angle = 0;
    this -> sensorDist_cm = 100; //maximum readable distance

    for (int i = 0; i < gridSize; i++) {
	memset(grid[i], UNKNOWN, sizeof(int) * gridSize);
    }

    for (int i = 0; i < gridSize; i++) {
	memset(distanceField[i], -1, sizeof(int) * gridSize);
    }

    currentPos.x = gridSize / 2;
    currentPos.y = gridSize / 2;

    initialScan();
    create_target_path();

    //initialize Serial
    arduinoSerial = serialOpen("/dev/ttyACM0", 9600);

}

Robot::~Robot() {
    //delete grid;
    //delete distanceField;
}

void Robot::update() {//only function called from outside the loop
    moveTo(moves);
    scanSurroundings();
    if (create_target_path()) { //returns 1 if map completed
	finalRoute();
    }
}

void Robot::initialScan() {
    //can be made much faster

    double angleDelta = 0;
    std::vector<double> sonarData(360);
    serialPrintf("r 360");
    while (angleDelta < PI / 2.0) {
	angleDelta += updateAngle(accelerometerPin);
	for (int i = 0; i < 4; i++) {
	    sonarData[(int)(angleDelta / (PI / 2.0) * 90) + i * 90] = \
		getSonarData(SONARPIN1 + i);
	}
    }

    cleanSonarData(&sonarData);

    int mostPerpindicular = 0;
    int bestPerpindicularity = 10000000;
    int perpindicularity;
    for (int i = 0; i < 360; i++) {
	perpindicularity = 0;
	for (int j = -5; j <= 5; j++) {
	    perpindicularity += abs(sonarData[(i + j + 360) % 360] - \
				    sonarData[i]);
	}
	if (perpindicularity < bestPerpindicularity) {
	    bestPerpindicularity = perpindicularity;
	    mostPerpindicular = i;
	}
    }
    serialPrintf("r %d", mostPerpindicular < 180 \
		 ? mostPerpindicular \
		 : mostPerpindicular - 360);
    //wait for confirmation? wait certain amount of time? some waiting
    //must occur.
    //robot is now oriented parallel to walls.
}

void Robot::scanSurroundings() {//this function is fully converted
    double angleDelta = 0;
    std::vector<double> sonarData(360);
    //send command to rotate nonstop, or just 360+something degrees at
    //speed to be determined
    serialPrintf("r 360\n");
    while (angleDelta < PI/2.0) {
	angleDelta += updateAngle(ACCELEROMETERPIN);
	for (int i = 0; i < 4; i++) {
	    sonarData[(int)(angleDelta / (PI / 2.0) * 90) + i * 90] = \
		getSonarData(SONARPIN1 + i);
	}
    }

    cleanSonarData(&sonarData);

    //Robot rotates 360 degrees, recording distance data, and uses that
    //data to construct where the walls are in the gridmap
    for (int i = 0; i < 360; i ++) {
	double distanceReading = sonarData[i];
	if (distanceReading < sensorDist_cm) {
	    int targetCellX = int((distanceReading * cos(angle)) / cellSize + \
				  currentPos.x);
	    int targetCellY = int((distanceReading * sin(angle)) / cellSize + \
				  currentPos.y);
	    grid[targetCellX][targetCellY] = WALL;
	    //
	    for (int i = -; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
		    if (grid[targetCellX + i][targetCellY + j] != WALL) {
			grid[targetCellX + i][targetCellY + j] = WALL;
		    }
		}
	    }
	}
	for (int i = 0; i < distanceReading; i++) {
	    int cellX = int(i * cos(angle) / cellSize + currentPos.x);
	    int cellY = int(i * sin(angle) / cellSize + currentPos.y);
	    if (grid[cellX][cellY] == UNKNOWN) {
		grid[cellX][cellY] = CLEAR;
	    }
	}
    }
}

int Robot::create_target_path() {
	
    //fills moves with list of all moves necessary to reach target tile
    //moves contains waypoints at each poinit the robot switches direction
	
    moves.clear();

    target = findClosestUnknown();
    //if there are no more unknowns on the map
    if (target.x == -1)
	return 1; //map completed
    //otherwise...
    int currentDist = distanceField[target.x][target.y];
    Point currentSquare = target;
    Point prevDirection(0, 0);
    Point currDirection(0, 0);
    for (int reverseCount = 0;
	 reverseCount < distanceField[target.x][target.y];
	 reverseCount++) {
	std::vector<Point> openNeighbors = findOpenNeighbors(currentSquare);
	for (int i = 0; i < openNeighbors.size(); i++) {
	    if (distanceField[openNeighbors[i].x][openNeighbors[i].y]  == \
		currentDist - 1) {
		//neighbor is one square closer to robot than the current square
		currentDist = \
		    distanceField[openNeighbors[i].x][openNeighbors[i].y];
		prevDirection.x = currDirection.x;
		prevDirection.y = currDirection.y;
		currDirection.x = openNeighbors[i].x - currentSquare.x;
		currDirection.y = openNeighbors[i].y - currentSquare.y;

		if (prevDirection.x != currDirection.x || \
		    prevDirection.y != currDirection.y) {
		    //if there is a change in direction
		    moves.insert(moves.begin(), currentSquare);
		}
		currentSquare = openNeighbors[i];
	    }
	}
    }
    return 0;//map not completed
}

Point Robot::findClosestUnknown() {//This one's pretty fast

    //build in code to ignore small regions of unknown

    memset(distanceField, -1, sizeof(int)*gridSize*gridSize);
	
    //finds the closest unknown tile
    std::vector<Point> boundary;
    boundary.push_back(Point(currentPos.x, currentPos.y));
    distanceField[currentPos.x][currentPos.y] = 0;
    while (boundary.size() > 0) {
	Point checking = boundary[0];
	boundary.erase(boundary.begin());
	std::vector<Point> openNeighbor = findOpenNeighbors(checking);
	for (int i = 0; i < openNeighbors.size(); i++) {
	    if (grid[openNeighbors[i].x][openNeighbors[i].y] == UNKNOWN) {
		distanceField[openNeighbors[i].x][openNeighbors[i].y] = \
		    distanceField[checking.x][checking.y] + 1;
		return openNeighbors[i]; //closest unknown tile
	    }
	    else {
		if (distanceField[openNeighbors[i].x][openNeighbors[i].y] == \
		    -1) {
		    boundary.push_back(openNeighbors[i]);
		    distanceField[openNeighbors[i].x][openNeighbors[i].y] = \
			distanceField[checking.x][checking.y] + 1;
		}
	    }
	}
    }
    return Point(-1, -1);  //no unknown tiles left, maze completely mapped
}

int unknownRegionSize(Point initial) {
	
	
}


std::vector<Point> Robot::findOpenNeighbors(Point &currentPos) {
    std::vector< Point > openNeighbors;
    for (int x_offset = -1; x_offset < 2; x_offset++) {
	for (int y_offset = -1; y_offset < 2; y_offset++) {
	    if (!is_diagonal_candidate(x_offset, y_offset) && \
		grid[currentPos.x + x_offset][currentPos.y + y_offset] != WALL) {
		openNeighbors.push_back(Point(currentPos.x + x_offset,
					      currentPos.y + y_offset));
	    }
	}
    }
    return openNeighbors;
}

/*
  x\y 1  0 -1
  1   n     n
  0      X
  -1  n     n
*/
bool is_diagonal_candidate(int x_offset, int y_offset) {
    return ((x_offset + y_offset + 2) % 2 == 0)
	}

void Robot::moveTo(std::vector<Point>) {
    // send commands to Arduino to go somewhere by grid coordinates
    serialPrintf(arduinoSerial, "m ");//m for moves
    for (int i = 0; i < moves.size(); i++) {
	serialPrintf(arduinoSerial, "%d %d ", moves[i].x, moves[i].y);
    }
    serialPrintf("\n");

    bool atDestination = false;
    while (!atDestination) {
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

double Robot::updateAngle(double timeDelta) {
    double angVel = getAcceleration(accelerometerPin);
    double angleDelta = angVel * timeDelta;
    angle += angleDelta;
    return angleDelta;
}

double Robot::getSonarData(int sonarPin) {
    return digitalRead(sonarPin);
}

void Robot::cleanSonarData(std::vector<double> *sonarData) {
    //   THIS IS A REALLY GOOD IDEA MUST DO |
    //                                      V
    //why?  to prevent false walls because of flare?  I guess so.

    /////////////////////////////////////////////////////////////////////////
    // when there is a change from wall to clear, make all around it clear //
    /////////////////////////////////////////////////////////////////////////

    normalize(sonarData);

    //eliminate false walls
    for (int point = 0; point < arraySize; point++) {
	if (abs(sonarData[point] - sonarData[point+1]) > 10)
	    sonarData[point] = sonarData[point+1];
    }
}

/*
  Normalize / flatten the values in VECTOR by averaging DIAMETERODD/2
  elements on each side of each element
  
  Note: diameterOdd should be odd
*/
void normalize(std::vector<double> *vector, int diameterOdd = 5) {
    const std::vector<double> constCopy = *vector;
    int size = vector -> size();

    int sum;
    for (int point = 0; point < size; point++) {
	sum = 0;
	for (int i = -diameterOdd/2; i <= diameterOdd/2; i++) {
	    average += constCopy[(point + i + size) % size];
	}
	(*vector)[point] = sum/diameterOdd;
    }
}

double Robot::getAcceleration(int pin) {
    return 0;
}

//route robot back to start, extinguishing any remaining candles on the way
void finalRoute() { 
    //the initial position is simly stored at (0, 0) in the gridMap.

}
