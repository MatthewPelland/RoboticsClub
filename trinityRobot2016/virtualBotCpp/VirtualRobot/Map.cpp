#include <iostream>
#include <chrono>
#include <ctime>

#include "SDL.h"
#include "SDL_image.h"

#include "Map.h"

Map::Map() :
	robot(width / 4, height / 2, 0) {
	SDL_Init(SDL_INIT_EVERYTHING);
	IMG_Init(IMG_INIT_JPG);
	window = SDL_CreateWindow("Virtual Robert", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, NULL);
	screen = SDL_GetWindowSurface(window);
	fmt = *(SDL_GetWindowSurface(window)->format);

	map_image = IMG_Load("data/mazeA.jpg");
	if (!map_image) {
		std::cout << "Error: " << IMG_GetError() << std::endl;
	}

	canvas = CreateSurface(width, height);
}

Map::~Map() {
	SDL_FreeSurface(map_image);
	SDL_FreeSurface(canvas);
	SDL_FreeSurface(screen);

	SDL_DestroyWindow(window);

	IMG_Quit();
	SDL_Quit();
}

SDL_Surface* Map::CreateSurface(int w, int h) {
	return SDL_CreateRGBSurface(0, w, h, fmt.BitsPerPixel, fmt.Rmask, fmt.Gmask, fmt.Bmask, fmt.Amask);
}

Uint32 Map::Color(Uint8 r, Uint8 g, Uint8 b) {
	return SDL_MapRGB(&fmt, r, g, b);
}

void Map::run() {
	auto oldTime = std::chrono::high_resolution_clock::now();
	auto curTime = std::chrono::high_resolution_clock::now();
	while (1) {
		oldTime = curTime;
		curTime = std::chrono::high_resolution_clock::now();
		long long delta = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - oldTime).count();
		if (delta > 1.0 / FPS) {
			input();
			robot.update();
			draw();
		}
		//std::cout << "FPS: " << delta << std::endl;
	}
}

void Map::input() {
	SDL_Event ev;
	while (SDL_PollEvent(&ev)) {
		switch (ev.type) {
			//case SDL_KEYDOWN:
			//	//case SDL_KEYUP:
			//	switch (ev.key.keysym.sym) {
			//	case 'a':
			//	case 'b':
			//	case 'c':
			//	case 'd':
			//		//mazeGeneration = false;
			//		//generating = false;
			//		//String map = "maze" + Character.toUpperCase(key) + ".jpg";
			//		//maze = loadImage(map);
			//		//maze.resize(width / 2, height);
			//		break;
			//	case 's':
			//		robot.scan_surroundings();
			//		break;
			//	case 'r':
			//		mazeGeneration = true;
			//		generating = true;
			//		break;
			//	case ' ':
			//		generating = false;
			//		break;
			//	case 'x':
			//		go = true;
			//		break;
			//	}
			//	break;
		case SDL_QUIT:
			exit(0);
		}
	}
}

void Map::draw() {
	SDL_Rect z = { 0, 0, width / 2, height };
	SDL_BlitScaled(map_image, 0, canvas, &z);

	//display grid
	Uint32 color;
	for (int i = 0; i < gridSize; i++) {
		for (int j = 0; j < gridSize; j++) {
			switch (robot.grid[i][j]) {
			case CLEAR:
				if (robot.distanceField[i][j] != -1) {
					color = SDL_MapRGB(&fmt, (robot.distanceField[i][j] * 10) % 255, 255 - (robot.distanceField[i][j] * 10) % 255, 0);
				} else {
					color = SDL_MapRGB(&fmt, 0, 0, 0);
				}
				break;
			case WALL:
				color = SDL_MapRGB(&fmt, 255, 255, 255);
				break;
			default:
				color = SDL_MapRGB(&fmt, 128, 128, 128);
			}
			if ((i - robot.gridPos.x)*precision / 2 > -400) {
				SDL_Rect r = { 1200 + (i - robot.gridPos.x)*precision / 2, 400 + (j - robot.gridPos.y)*precision / 2, precision / 2, precision / 2 };
				SDL_FillRect(canvas, &r, color);
			}
		}
	}

	//draw robit
	z = { (int)robot.get_position().x - robotsize / 2 , (int)robot.get_position().y - robotsize / 2, robotsize, robotsize };
	SDL_FillRect(canvas, &z, Color(75, 244, 75));

	z = { (int)(3.0 / 4.0 * width) - robotsize / 2 , height / 2 - robotsize / 2, robotsize, robotsize };
	SDL_FillRect(canvas, &z, Color(75, 244, 75));

	//
	SDL_BlitSurface(canvas, 0, screen, 0);
	SDL_UpdateWindowSurface(window);
}