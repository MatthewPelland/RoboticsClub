#pragma once

#include <vector>

#include "SDL.h"

enum Cell { UNKNOWN, WALL, CLEAR, BLOCKED };

#define width 1600
#define height 800
#define FPS 60

#define precision 5.0
#define gridSize int(1600/precision)
#define robotsize 10

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
} vec2;

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
	Robot(double xpos, double ypos, double angle);
	~Robot();
	void scan_surroundings();
	vec2 get_position();
	void update();

	vec2i gridPos;
	int grid[gridSize][gridSize];
	int distanceField[gridSize][gridSize];
private:
	int create_target_path();
	bool advance();
	int computeDistance();
	bool is_wall(vec2 &loc);
	void openNeighbors(vec2i &coords, std::vector<vec2i> *openNeighbors);
	vec2i findClosestUnknown();
	inline double distance(vec2i a, vec2i b);

	SDL_Surface *map_image;

	vec2 pos, vel, acc;
	std::vector<vec2i> moves;

	SDL_PixelFormat *fmt;

	vec2i target;

	double angle, vAng;
	int targetDiameter;
	int sensorDist;//in pixels to approximate real world
	bool targetReached;
};

