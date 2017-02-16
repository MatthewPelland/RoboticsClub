#include "Robot.h"
// For std::sleep
#include <thread>
#include <chrono>

Robot::Robot() {
    this -> angle = 0;
    this -> sensorDist_cm = 200; //maximum readable distance

	for (int i = 0; i < GRIDSIZE_CELLS; i++) {
		for (int j = 0; j < GRIDSIZE_CELLS; j++) {
			grid[i][j] = gridVal();
		}
	}

    for (int i = 0; i < GRIDSIZE_CELLS; i++) {
	memset(distanceField[i], -1, sizeof(int) * GRIDSIZE_CELLS);
    }

    currentPos.x = GRIDSIZE_CELLS / 2;
    currentPos.y = GRIDSIZE_CELLS / 2;

    initialScan();
    create_target_path(findClosestUnknown());

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
	if (createTargetPath(findClosestUnknown())) { //returns 1 if map completed
		returnToStart();
	}
}

void Robot::initialScan() {
    //can be made much faster

    double angleDelta = 0;
    std::vector<double> sonarData(360);
    serialPrintf("r 360");
    while (angleDelta < PI / 2.0) {
		angleDelta += updateAngle(ACCELEROMETERPIN);
		for (int i = 0; i < 4; i++) {
			sonarData[(int)(angleDelta / (PI / 2.0) * 90) + i * 90] = \
			getSonarData(SONARPIN1 + i);
		}
    }

    cleanSonarData(sonarData);

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

void Robot::scanSurroundings() {//double check that this one is right
	double angleDelta = 0;
	std::vector<double> sonarData(360);
	//send command to rotate nonstop, or just 360+something degrees at
	//speed to be determined
	serialPrintf("r 360\n");
	while (angleDelta < PI / 2.0) {
		angleDelta += updateAngle(ACCELEROMETERPIN);
		for (int i = 0; i < 4; i++) {
			sonarData[(int)(angleDelta / (PI / 2.0) * 90) + i * 90] = \
				getSonarData(SONARPIN1 + i);
		}

		//cleanSonarData(sonarData, 360);

		//Robot rotates 360 degrees, recording distance data, and uses that data to construct where the walls are in the gridmap
		for (int i = 0; i < 360; i++) {
			//do sonar stuff
			double distanceReading = sonarData[i];
			if (distanceReading < sensorDist_cm) {
				int targetCellX = int((distanceReading * cos(angle)) / CELLSIZE_CM + currentPos.x);
				int targetCellY = int((distanceReading * sin(angle)) / CELLSIZE_CM + currentPos.y);
				gridVal targetCell = grid[targetCellX][targetCellY];
				//check fire sensor
				int fireSensorData = analogRead(FLAMESENSORPIN);
				if (fireSensorData >= FLAMESENSORTHRESHOLD) {
					targetCell.cellType = FLAME;
				}
				if (targetCell.cellType <= 1) {
					grid[targetCellX][targetCellY].cellType = (targetCell.cellType*targetCell.timesScanned + 1) / (targetCell.timesScanned++ + 1);//note the ++
				}
				//increase apparent size of walls to compensate for the size of the robot 
				//so we don't have to keep track of distance to walls elsewhere.
				//this method may have to change
				for (int i = -ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; i <= ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; i++) {
					for (int j = -ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; j <= ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; j++) {
						gridVal offsetTarget = grid[targetCellX + i][targetCellY + j];
						if (offsetTarget.cellType <= 1) {
							grid[targetCellX + i][targetCellY + j].cellType = (offsetTarget.cellType*offsetTarget.timesScanned + 1) / (offsetTarget.timesScanned++ + 1);
						}
					}
				}
			}
			for (int i = 0; i < distanceReading; i++) {
				int cellX = int(i * cos(angle) / CELLSIZE_CM + currentPos.x);
				int cellY = int(i * sin(angle) / CELLSIZE_CM + currentPos.y);
				if ((int)(grid[cellX][cellY].cellType) == UNKNOWN) {
					grid[cellX][cellY].cellType = (grid[cellX][cellY].cellType*grid[cellX][cellY].timesScanned) / (grid[cellX][cellY].timesScanned++ + 1);
				}
			}
		}
	}
}

int Robot::createTargetPath(Point target) {
	
    //fills moves with list of all moves necessary to reach target tile
    //moves contains waypoints at each poinit the robot switches direction
	moves.clear();

	//target = findClosestUnknown();
	//if there are no more unknowns on the map
	if (target.x == -1)
		return 1; //map completed
				  //otherwise...
	int currentDist = distanceField[target.x][target.y];
	Point currentSquare = target;
	Point prevDirection(0, 0);
	Point currDirection(0, 0);
	for (int reverseCount = 0; reverseCount < distanceField[target.x][target.y]; reverseCount++) {
		std::vector<Point> openNeighbor;
		openNeighbor = openNeighbors(currentSquare);
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
			currentSquare = openNeighbor[i];
		}
	}
    return 0;//map not completed
}

Point Robot::findClosestUnknown() {//This one's pretty fast
	memset(distanceField, -1, sizeof(int)*GRIDSIZE_CELLS*GRIDSIZE_CELLS);
	//distanceField = (int **)malloc(GRIDSIZE_CELLS * sizeof(int *));

	//finds the closest unknown tile
	std::vector<vec2i> boundary;
	boundary.push_back(vec2i(gridPos.x, gridPos.y));
	distanceField[gridPos.x][gridPos.y] = 0;
	while (boundary.size() > 0) {
		vec2i checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<Point> openNeighbors;
		openNeighbors = openNeighbors(checking);
		for (int i = 0; i < openNeighbor.size(); i++) {
			if (grid[openNeighbor[i].x][openNeighbor[i].y].cellType == UNKNOWN) {
				distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				return openNeighbor[i]; //closest unknown tile
			}
			else if (distanceField[openNeighbor[i].x][openNeighbor[i].y] == -1) {
					boundary.push_back(openNeighbor[i]);
					distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				}
		}
    }
    return Point(-1, -1);  //no unknown tiles left, maze completely mapped
}

void Robot::computeDistanceField(Point target) {
	memset(distanceField, -1, sizeof(int)*GRIDSIZE_CELLS*GRIDSIZE_CELLS);
	//distanceField = (int **)malloc(GRIDSIZE_CELLS * sizeof(int *));
	for (int i = 0; i < GRIDSIZE_CELLS; i++) {
		memset(distanceField[i], -1, sizeof(int) * GRIDSIZE_CELLS);
	}

	//finds the closest unknown tile
	std::vector<Point> boundary;
	boundary.push_back(Point(currentPos.x, currentPos.y));
	distanceField[currentPos.x][currentPos.y] = 0;
	while (boundary.size() > 0) {
		Point checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<Point> openNeighbor;
		findOpenNeighbors(checking);
		for (int i = 0; i < openNeighbor.size(); i++) {
			if (openNeighbor[i].x == target.x && openNeighbor[i].y == target.y) {
				distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
			}
			else {
				if (distanceField[openNeighbor[i].x][openNeighbor[i].y] == -1) {
					boundary.push_back(openNeighbor[i]);
					distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				}
			}
		}
	}
}

std::vector<Point> Robot::findOpenNeighbors(Point currentPos) {
    std::vector< Point > openNeighbors;
    for (int x_offset = -1; x_offset < 2; x_offset++) {
		for (int y_offset = -1; y_offset < 2; y_offset++) {
			if (!is_diagonal_candidate(x_offset, y_offset) && \
			grid[currentPos.x + x_offset][currentPos.y + y_offset].cellType < .3) {
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
	return ((x_offset + y_offset + 2) % 2 == 0);
}

//Precondition: moves has been uptdated for the current target
void Robot::moveTo(std::vector<Point> moves) {
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
    double angVel = getAcceleration(ACCELEROMETERPIN);
    double angleDelta = angVel * timeDelta;
    angle += angleDelta;
    return angleDelta;
}

double Robot::getSonarData(int sonarPin) {
    return digitalRead(sonarPin);
}

void Robot::returnToStart() { //route robot back to start, extinguishing any remaining candles on the way there
					//the initial position is simly stored at (0, 0) in the gridMap.
	createTargetPath(Point(GRIDSIZE_CELLS/2, GRIDSIZE_CELLS/2));
}

double Robot::getAcceleration(int pin) {
    return 0;
}
