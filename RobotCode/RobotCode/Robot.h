#pragma once

#include <StandardCplusplus.h>
#include <vector>

enum Cell { UNKNOWN, WALL, CLEAR, BLOCKED };

//#define precision 5.0
#define precision 10.0 //
#define gridSize int((1600/precision)+1)
#define robotsize 10

#define grid(x, y) grid[((x) / 4) * gridSize + (y)]
#define distanceField(x, y) distanceField[(x) * gridSize + (y)]

typedef struct {
	unsigned char a : 2;
	unsigned char b : 2;
	unsigned char c : 2;
	unsigned char d : 2;
} Value;

typedef struct vector2d {
	float x, y;

	vector2d() {
		x = 0;
		y = 0;
	}

	vector2d(float x, float y) {
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
	Robot(float xpos, float ypos, float angle);
	~Robot();
	void scan_surroundings();
	vec2 get_position();
	void update();

	vec2i gridPos;
	Value grid[(gridSize / 4) * gridSize];
	char distanceField[gridSize*gridSize];
private:
	void create_target_path();
	bool advance();
	int computeDistance();
	bool is_wall(vec2 &loc);
	void openNeighbors(vec2i &coords, vec2i *openNeighbors, int *size);
	vec2i findClosestUnknown();
	inline float distance(vec2i a, vec2i b);
	inline void grid_set(int x, int y, unsigned char value);
	inline unsigned char grid_get(int x, int y);

	vec2 pos, vel, acc;
	std::vector<vec2i> moves;

	vec2i target;

	float angle, vAng;
	int sensorDist;//in pixels to approximate real world
	bool targetReached;
};

