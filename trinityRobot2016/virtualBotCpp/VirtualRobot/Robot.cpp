#include <iostream>
#include <chrono>

#include "Robot.h"

#include "SDL_image.h"

//#define M_PI 3.14159265358979323846
#include <cmath>

Robot::Robot(double xpos, double ypos, double angle) {
	pos.x = xpos;
	pos.y = ypos;
	this->angle = angle;
	sensorDist = int(800.0 / 244.0 * 100);

	for (int i = 0; i < gridSize; i++) {
		memset(grid[i], UNKNOWN, sizeof(int) * gridSize);
	}

	for (int i = 0; i < gridSize; i++) {
		memset(distanceField[i], -1, sizeof(int) * gridSize);
	}

	gridPos.x = gridSize / 2;
	gridPos.y = gridSize / 2;
	targetReached = true;
	targetDiameter = 0;

	SDL_Surface *img = IMG_Load("data/mazeA.jpg");
	SDL_Rect z = { 0, 0, width / 2, height };
	fmt = img->format;
	map_image = SDL_CreateRGBSurface(0, width, height, fmt->BitsPerPixel, fmt->Rmask, fmt->Gmask, fmt->Bmask, fmt->Amask);
	SDL_BlitScaled(img, 0, map_image, &z);
}

Robot::~Robot() {
	//delete grid;
	//delete distanceField;
}

void Robot::update() {
	if (advance()) {
		scan_surroundings();
		create_target_path();
	}
}

bool Robot::advance() {
	if (moves.size() == 0)
		return true;
	else {
		int deltaX = moves[0].x - gridPos.x;//watch out I may have screwed this up if it's supposed to be a 2d array..
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
	double temp = angle;
	for (double targetAngle = angle + 2 * M_PI; angle < targetAngle; angle += M_PI / 180) {
		double distanceReading = computeDistance();
		if (distanceReading < sensorDist) {
			int targetCellX = int((distanceReading * cos(angle)) / precision + gridPos.x);
			int targetCellY = int((distanceReading * sin(angle)) / precision + gridPos.y);
			grid[targetCellX][targetCellY] = WALL;
			for (int i = -1; i <= 1; i++) {
				for (int j = -1; j <= 1; j++) {
					if (grid[targetCellX + i][targetCellY + j] != WALL) {
						grid[targetCellX + i][targetCellY + j] = WALL;
					}
				}
			}
		}
		for (int i = 0; i < distanceReading; i++) {
			int cellX = int(i * cos(angle) / precision + gridPos.x);
			int cellY = int(i * sin(angle) / precision + gridPos.y);
			if (grid[cellX][cellY] == UNKNOWN) {
				grid[cellX][cellY] = CLEAR;
			}
		}
	}
	angle = temp;
}

int Robot::create_target_path() {
	//fills moves with list of all moves necessary to reach target tile
	moves.clear();
	//int** distanceFieldClone = (int**)malloc(sizeof(int)*gridSize*gridSize);
	//memcpy(distanceFieldClone, distanceField, sizeof(int)*gridSize*gridSize);

	target = findClosestUnknown();
	targetDiameter = 250;
	//if there are no more unknowns on the map
	if (target.x == -1)
		return 1; //map completed
				  //otherwise...
	int currentDist = distanceField[target.x][target.y];
	vec2i currentSquare = target;
	vec2i initialDirection(7, 7);
	bool changedDirection = false;
	for (int reverseCount = 0; reverseCount < distanceField[target.x][target.y]; reverseCount++) {
		std::vector<vec2i> openNeighbor;
		openNeighbors(currentSquare, &openNeighbor);
		for (int i = 0; i < openNeighbor.size(); i++) {
			//if the neighbor is one square closer to robot than the current square
			if (distanceField[openNeighbor[i].x][openNeighbor[i].y] == currentDist - 1) {
				currentDist = distanceField[openNeighbor[i].x][openNeighbor[i].y];
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
	return 0;//map not completed
}

vec2i Robot::findClosestUnknown() {
	memset(distanceField, -1, sizeof(int)*gridSize*gridSize);
	//distanceField = (int **)malloc(gridSize * sizeof(int *));
	for (int i = 0; i < gridSize; i++) {
		//distanceField[i] = (int *)malloc(gridSize * sizeof(int));
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
			if (grid[openNeighbor[i].x][openNeighbor[i].y] == UNKNOWN) {//shouldn't keep going infinitely, this is awkward
				distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				return openNeighbor[i]; //closest unknown tile
			} else {
				if (distanceField[openNeighbor[i].x][openNeighbor[i].y] == -1) {
					boundary.push_back(openNeighbor[i]);
					distanceField[openNeighbor[i].x][openNeighbor[i].y] = distanceField[checking.x][checking.y] + 1;
				}
			}
		}
	}
	return vec2i(-1, -1);  //no unknown tiles left, maze completely mapped
}

void Robot::openNeighbors(vec2i &coords, std::vector<vec2i> *openNeighbors) {
	//typedef std::chrono::high_resolution_clock Clock;
	//typedef std::chrono::nanoseconds nanoseconds;
	//Clock::time_point t0 = Clock::now();
	
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			if (((i + j + 2)%2 == 1) && grid[coords.x + i][coords.y + j] != WALL) {
				openNeighbors->push_back(vec2i(coords.x + i, coords.y + j));
			}
		}
	}

	//Clock::time_point t1 = Clock::now();
	//nanoseconds ms = std::chrono::duration_cast<nanoseconds>(t1 - t0);
	//std::cout << ms.count() << "ns\n";	//returns list of coordinates neighboring input coordinates where robot could move
}

int Robot::computeDistance() {
	//returns distance to closest wall in direction robot is facing
	vec2 looking(cos(angle), sin(angle));
	for (int i = 10; i < sensorDist; i++) {
		//looking.setMag(i);
		double len = sqrt(pow(looking.x, 2) + pow(looking.y, 2));
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
	Uint32 col = ((Uint32*)map_image->pixels)[(int)loc.y*(map_image->pitch / sizeof(Uint32)) + (int)loc.x];
	Uint8 r, g, b;
	SDL_GetRGB(col, fmt, &r, &g, &b);
	return (r >= 128 && g >= 0 && b >= 128);
}

vec2 Robot::get_position() {
	return pos;
}

double Robot::distance(vec2i a, vec2i b) {
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

