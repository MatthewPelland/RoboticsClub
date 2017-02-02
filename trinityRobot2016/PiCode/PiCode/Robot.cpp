#include "Robot.h"

Robot::Robot() {
	this -> angle = 0;
	this -> sensorDist = 100; //maximum readable distance in cm.

	for (int i = 0; i < gridSize; i++) {
		memset(grid[i], UNKNOWN, sizeof(int) * gridSize);
	}

	for (int i = 0; i < gridSize; i++) {
		memset(distanceField[i], -1, sizeof(int) * gridSize);
	}

	gridPos.x = gridSize / 2;
	gridPos.y = gridSize / 2;

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
	moveRobotTo(moves);
	scan_surroundings();
	if (create_target_path()) { //returns 1 if map completed
		finalRoute();
	}
}

void Robot::initialScan() {
	//can be made much faster

	double angleDelta = 0;
	double sonarData[360];
	serialPrintf("r 360");
	while (angleDelta < M_PI / 2.0) {
		angleDelta += updateAngle(accelerometerPin);
		for (int i = 0; i < 4; i++) {
			sonarData[(int)(angleDelta / (M_PI / 2.0) * 90) + i * 90] = getSonarData(SONARPIN1 + i);
		}
	}

	cleanSonarData(sonarData, 360);

	int mostPerpindicular = 0;
	int bestPerpindicularity = 10000000;
	int perpindicularity;
	for (int i = 0; i < 360; i++) {
		perpindicularity = 0;
		for (int j = -5; j <= 5; j++) {
			perpindicularity += abs(sonarData[(i + j + 360) % 360] - sonarData[i]);
		}
		if (perpindicularity < bestPerpindicularity) {
			bestPerpindicularity = perpindicularity;
			mostPerpindicular = i;
		}
	}
	serialPrintf("r %d", mostPerpindicular < 180 ? mostPerpindicular : mostPerpindicular - 360);
	//wait for confirmation?  wait certain amount of time?  some waiting must occur.
	//robot is now oriented parallel to walls.
}

void Robot::scan_surroundings() {//this function is fully converted
	double angleDelta = 0;
	double sonarData[360];
	//send command to rotate nonstop, or just 360+something degrees at speed to be determined
	serialPrintf("r 360\n");
	while (angleDelta < M_PI/2.0) {
		angleDelta += updateAngle(ACCELEROMETERPIN);
		for (int i = 0; i < 4; i++) {
			sonarData[(int)(angleDelta / (M_PI / 2.0) * 90) + i * 90] = getSonarData(SONARPIN1 + i);
		}
	}

	cleanSonarData(sonarData, 360);

	//Robot rotates 360 degrees, recording distance data, and uses that data to construct where the walls are in the gridmap
	for (int i = 0; i < 360; i ++) {
		double distanceReading = sonarData[i];
		if (distanceReading < sensorDist) {
			int targetCellX = int((distanceReading * cos(angle)) / cellSize + gridPos.x);
			int targetCellY = int((distanceReading * sin(angle)) / cellSize + gridPos.y);
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
			int cellX = int(i * cos(angle) / cellSize + gridPos.x);
			int cellY = int(i * sin(angle) / cellSize + gridPos.y);
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
	vec2i currentSquare = target;
	vec2i prevDirection(0, 0);
	vec2i currDirection(0, 0);
	for (int reverseCount = 0; reverseCount < distanceField[target.x][target.y]; reverseCount++) {
		std::vector<vec2i> openNeighbor;
		openNeighbors(currentSquare, &openNeighbor);
		for (int i = 0; i < openNeighbor.size(); i++) {
			//if the neighbor is one square closer to robot than the current square
			if (distanceField[openNeighbor[i].x][openNeighbor[i].y] == currentDist - 1) {
				currentDist = distanceField[openNeighbor[i].x][openNeighbor[i].y];
				prevDirection.x = currDirection.x;
				prevDirection.y = currDirection.y;
				currDirection.x = openNeighbor[i].x - currentSquare.x;
				currDirection.y = openNeighbor[i].y - currentSquare.y;

				if (prevDirection.x != currDirection.x || prevDirection.y != currDirection.y) {
					//if there is a change in direction
					moves.insert(moves.begin(), currentSquare);
				}
				currentSquare = openNeighbor[i];
			}
		}
	}
	return 0;//map not completed
}

vec2i Robot::findClosestUnknown() {//This one's pretty fast

	//build in code to ignore small regions of unknown

	memset(distanceField, -1, sizeof(int)*gridSize*gridSize);
	//distanceField = (int **)malloc(gridSize * sizeof(int *));
	for (int i = 0; i < gridSize; i++) {
		memset(distanceField[i], -1, sizeof(int) * gridSize);
	}

	//finds the closest unknown tile
	std::vector<vec2i> boundary;
	boundary.push_back(vec2i(gridPos.x, gridPos.y));
	distanceField[gridPos.x][gridPos.y] = 0;
	while (boundary.size() > 0) {
		vec2i checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<vec2i> openNeighbor;
		openNeighbors(checking, &openNeighbor);
		for (int i = 0; i < openNeighbor.size(); i++) {
			if (grid[openNeighbor[i].x][openNeighbor[i].y] == UNKNOWN) {
				distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				return openNeighbor[i]; //closest unknown tile
			}
			else {
				if (distanceField[openNeighbor[i].x][openNeighbor[i].y] == -1) {
					boundary.push_back(openNeighbor[i]);
					distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				}
			}
		}
	}
	return vec2i(-1, -1);  //no unknown tiles left, maze completely mapped
}

int unknownRegionSize(vec2i initial) {
	
	
}

void Robot::openNeighbors(vec2i &coords, std::vector<vec2i> *openNeighbors) {
	//openNeighbors constant sized array 
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			if (((i + j + 2) % 2 == 1) && grid[coords.x + i][coords.y + j] != WALL) {
				openNeighbors->push_back(vec2i(coords.x + i, coords.y + j));
			}
		}
	}
	//returns list of coordinates neighboring input coordinates where robot could move
}

double Robot::distance(vec2i a, vec2i b) {
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

void Robot::moveRobotTo(std::vector<vec2i>) {
	//send command to Arduino to go somewhere by grid coordinates
	serialPrintf(arduinoSerial, "m ");//m for moves
	for (int i = 0; i < moves.size(); i++) {
		serialPrintf(arduinoSerial, "%d %d ", moves[i].x, moves[i].y);
		//wait for response saying we're there, before sending next data
	}
	serialPrintf("\n");
}

double Robot::updateAngle(double timeDelta) {
	double angVel = getAcceleration(accelerometerPin);
	angle += angVel * timeDelta;
	return angVel * timeDelta;
}

double Robot::getSonarData(int sonarPin) {
	return digitalRead(sonarPin);
}

void Robot::cleanSonarData(double * sonarData, int arraySize) {//clean sonar data
	//   THIS IS A REALLY GOOD IDEA MUST DO |
	//                                      V
	//why?  to prevent false walls because of flare?  I guess so.

	//////////////////////////////////////////////////////////////////////////////////////////////
	//           when there is a change from wall to clear, make all around it clear            //
	//////////////////////////////////////////////////////////////////////////////////////////////

	double* copy = (double*) malloc(sizeof(int) * arraySize);
	for (int i = 0; i < arraySize; i++)
		copy[i] = sonarData[i];


	int average;

	//clean out bumps and weirdos in data
	for (int point = 0; point < arraySize; point++) {
		average = 0;
		for (int j = -2; j <= 2; j++) {
			average += copy[(point + j + arraySize) % arraySize];
		}
		average /= 5;
		sonarData[point] = average;
	}

	//eliminate false walls

	for (int point = 0; point < arraySize; point++) {
		if (abs(sonarData[point] - sonarData[point+1]) > 10)
			sonarData[point] = sonarData[point+1]
	}
}

double Robot::getAcceleration(int pin) {
	return 0;
}

void finalRoute() { //route robot back to start, extinguishing any remaining candles on the way there
	//the initial position is simly stored at (0, 0) in the gridMap.

}

