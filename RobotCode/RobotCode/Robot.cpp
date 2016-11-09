#include <math.h>

#include "Robot.h"

Robot::Robot(float xpos, float ypos, float angle) {
	pos.x = xpos;
	pos.y = ypos;
	this->angle = angle;
	sensorDist = int(800.0 / 244.0 * 100);

	memset(grid, UNKNOWN, gridSize*gridSize);
	memset(distanceField, -1, gridSize*gridSize);

	gridPos.x = gridSize / 2;
	gridPos.y = gridSize / 2;
	targetReached = true;
}

Robot::~Robot() {
	//I should free the allocated memory
	//but if this destructor is ever called
	//something went very wrong
}

void Robot::update() {
	if (advance()) {
		scan_surroundings();
		create_target_path();
	}
}

bool Robot::advance() {
	if (moves.size() == 0) {
		return true;
	} else {
		int deltaX = moves[0].x - gridPos.x;
		int deltaY = moves[0].y - gridPos.y;

		gridPos.x += deltaX;
		gridPos.y += deltaY;
		pos.x += deltaX * precision;
		pos.y += deltaY * precision;
		moves.erase(moves.begin());
		return false;
	}
}

void Robot::scan_surroundings() {
	//   THIS IS A REALLY GOOD IDEA MUST DO |
	//                                      V

	//////////////////////////////////////////////////////////////////////////////////////////////
	//when there is a wall ignore readings from previous and next few, unless reading was a wall//
	//////////////////////////////////////////////////////////////////////////////////////////////

	//Robot rotates 360 degrees, recording distance data, and uses that data to construct where the walls are in the gridmap
	for (float targetAngle = angle + 2 * M_PI; angle < targetAngle; angle += M_PI / 180) {
		int distanceReading = computeDistance();
		if (distanceReading < sensorDist) {
			int targetCellX = int((distanceReading * cos(angle)) / precision + gridPos.x);
			int targetCellY = int((distanceReading * sin(angle)) / precision + gridPos.y);
			grid_set(targetCellX, targetCellY, WALL);
			for (char i = -1; i <= 1; i++) {
				for (char j = -1; j <= 1; j++) {
					if (grid_get(targetCellX + i, targetCellY + j) != WALL) {
						grid_set(targetCellX + i, targetCellY + j, WALL);
					}
				}
			}
		}
		for (int i = 0; i < distanceReading; i++) {
			int cellX = int(i * cos(angle) / precision + gridPos.x);
			int cellY = int(i * sin(angle) / precision + gridPos.y);
			if (grid_get(cellX, cellY) == UNKNOWN) {
				grid_set(cellX, cellY, CLEAR);
			}
		}
	}
}

void Robot::create_target_path() {
	//fills moves with list of all moves necessary to reach target tile
	moves.clear();

	target = findClosestUnknown();
	//if there are no more unknowns on the map
	if (target.x == -1) {
		return; //map completed
	}
	//otherwise...
	int currentDist = distanceField(target.x, target.y);
	vec2i currentSquare = target;
	vec2i initialDirection(7, 7);
	bool changedDirection = false;
	for (int reverseCount = 0; reverseCount < distanceField(target.x, target.y); reverseCount++) {
		vec2i openNeighbor[4];
		int size;
		openNeighbors(currentSquare, openNeighbor, &size);
		for (int i = 0; i < size; i++) {
			//if the neighbor is one square closer to robot than the current square
			if (distanceField(openNeighbor[i].x, openNeighbor[i].y) == currentDist - 1) {
				currentDist = distanceField(openNeighbor[i].x, openNeighbor[i].y);
				if (initialDirection.x == 7) {
					initialDirection.x = openNeighbor[i].x - currentSquare.x;
					initialDirection.y = openNeighbor[i].y - currentSquare.y;
				}

				if (changedDirection) {
					moves.insert(moves.begin(), openNeighbor[i]);
				} else {
					if (openNeighbor[i].x - currentSquare.x != initialDirection.x || openNeighbor[i].y - currentSquare.y != initialDirection.y
						|| openNeighbor[i].x - target.x > sensorDist || openNeighbor[i].y - target.y > sensorDist) {
						changedDirection = true;
						//this loop runs from target to robot, so moves are inserted to the front as we come across them
						moves.insert(moves.begin(), openNeighbor[i]);
					}
				}


				///////////////////////////////////////////////////
				//                                               //
				//         can still be improved...              //
				//                                               //
				///////////////////////////////////////////////////


				///////////////////////////////////////////////////////////
				// every time direction changes, create a waypoint       //
				// robot motion profiles to each waypoint, then corrects //
				///////////////////////////////////////////////////////////



				currentSquare = openNeighbor[i];
			}
		}
	}
	//return 0;//map not completed
}

vec2i Robot::findClosestUnknown() {
	memset(distanceField, -1, gridSize*gridSize);

	//finds the closest unknown tile
	std::vector<vec2i> boundary;
	boundary.push_back(vec2i(gridPos.x, gridPos.y));
	distanceField(gridPos.x, gridPos.y) = 0;
	while (boundary.size() > 0) {
		vec2i checking = boundary[0];
		boundary.erase(boundary.begin());
		vec2i openNeighbor[4];
		int size;
		openNeighbors(checking, openNeighbor, &size);
		for (int i = 0; i < size; i++) {
			if (grid_get(openNeighbor[i].x, openNeighbor[i].y) == UNKNOWN) {//shouldn't keep going infinitely, this is awkward
				distanceField(openNeighbor[i].x, openNeighbor[i].y) = distanceField(checking.x, checking.y) + 1;
				return openNeighbor[i]; //closest unknown tile
			} else {
				if (distanceField(openNeighbor[i].x, openNeighbor[i].y) == -1) {
					boundary.push_back(openNeighbor[i]);
					distanceField(openNeighbor[i].x, openNeighbor[i].y) = distanceField(checking.x, checking.y) + 1;
				}
			}
		}
	}
	return vec2i(-1, -1);  //no unknown tiles left, maze completely mapped
}

void Robot::openNeighbors(vec2i &coords, vec2i *openNeighbors, int *size) {
	*size = 0;
	for (char i = -1; i < 2; i++) {
		for (char j = -1; j < 2; j++) {
			if (((i + j + 2) % 2 == 1) && grid_get(coords.x + i, coords.y + j) != WALL) {
				openNeighbors[(*size)++] = vec2i(coords.x + i, coords.y + j);
			}
		}
	}
}

int Robot::computeDistance() {
	//returns distance to closest wall in direction robot is facing
	vec2 looking(cos(angle), sin(angle));
	for (int i = 10; i < sensorDist; i++) {
		//looking.setMag(i);
		float len = sqrt(pow(looking.x, 2) + pow(looking.y, 2));
		looking.x *= i / len;
		looking.y *= i / len;
		vec2 maybeWall(looking.x + pos.x, looking.y + pos.y);
		if (is_wall(maybeWall)) {
			return i;
		}
	}
	return sensorDist;
}

bool Robot::is_wall(vec2 &loc) {
	return true;
}

vec2 Robot::get_position() {
	return pos;
}

float Robot::distance(vec2i a, vec2i b) {
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

void Robot::grid_set(int x, int y, unsigned char value) {
	switch (x % 4) {
	case 0:
		grid(x, y).a = value;
		break;
	case 1:
		grid(x, y).b = value;
		break;
	case 2:
		grid(x, y).c = value;
		break;
	case 3:
		grid(x, y).d = value;
		break;
	}
}

unsigned char Robot::grid_get(int x, int y) {
	switch (x % 4) {
	case 0:
		return grid(x, y).a;
		break;
	case 1:
		return grid(x, y).b;
		break;
	case 2:
		return grid(x, y).c;
		break;
	case 3:
		return grid(x, y).d;
		break;
	}
}