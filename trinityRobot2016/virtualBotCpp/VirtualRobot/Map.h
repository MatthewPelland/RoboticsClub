#pragma once


#include "SDL.h"

#include "Robot.h"

class Map {
public:
	Map();
	~Map();
	void run();

private:
	void input();
	void draw();


	inline SDL_Surface* CreateSurface(int w, int h);
	inline Uint32 Color(Uint8 r, Uint8 g, Uint8 b);

	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_PixelFormat fmt;
	SDL_Surface *screen;
	SDL_Surface *canvas;
	SDL_Surface *map_image;

	Robot robot;

	bool mazeGeneration = false, generating = false, go = false;
};

