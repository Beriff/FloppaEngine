#include "SDL.h"
#include <stdio.h>
#include <bitset>
#include <iostream>

#include "scene.h"
#include "parser.h"

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const float WIDTH_INV = 1 / (float)SCREEN_WIDTH;
const float HEIGHT_INV = 1 / (float)SCREEN_HEIGHT;

using namespace floppa::render;

scene scene1 = scene();
meshinfo cube = meshinfo();

/*bool loadContent() {

	bool success = true;
}*/

SDL_Texture* surface2texture(SDL_Renderer* renderer, SDL_Surface* surface) {
	SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
	SDL_FreeSurface(surface);
	return texture;
}

bool init(SDL_Window** window, SDL_Surface** surface, SDL_Renderer** renderer, SDL_Texture** texture) {
	cube.parse("C:\\Users\\Maxim\\Desktop\\cube.obj");
	cube.loadpolygons(&scene1);
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		printf("SDL initialization failed: %s\n", SDL_GetError());
		return false;
	}
	*window = SDL_CreateWindow("Sample title", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_ALLOW_HIGHDPI);
	if (*window == NULL) {
		printf("Window initialization failed: %s\n", SDL_GetError());
		return false;
	}
	*surface = SDL_GetWindowSurface(*window);
	*renderer = SDL_CreateRenderer(*window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	*texture = SDL_CreateTexture(*renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);

	return true;
}

void finish_application(SDL_Window* app_window, SDL_Renderer* renderer) {
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(app_window);
	SDL_Quit();
}

void process_keys() {

}

bool update(SDL_Window* window, SDL_Renderer* renderer) {

	SDL_Event sdlevent;
	while (SDL_PollEvent(&sdlevent)) {
		switch (sdlevent.type)
		{
		case SDL_QUIT:
			return false;
			break;
		case SDL_KEYDOWN:
			switch (sdlevent.key.keysym.sym) {
			case SDLK_w:
				scene1.scene_camera.moveforward(5);
				break;
			case SDLK_s:
				scene1.scene_camera.moveforward(-5);
				break;
			case SDLK_d:
				scene1.scene_camera.rotateright(5);
				break;
			case SDLK_a:
				scene1.scene_camera.rotateleft(5);
			default:
				break;
			}
		default:
			break;
		}
	}
	
	//SDL_UpdateWindowSurface(window);
	return true;

}

int main(int argc, char* argv[]) {
	SDL_Window* window = NULL;
	SDL_Surface* screenSurface = NULL;
	SDL_Renderer* renderer = NULL;
	SDL_Surface* mainsurface = NULL;
	int t = 1;

	SDL_Texture* mainTexture;

	if (init(&window, &screenSurface, &renderer, &mainTexture)) {
		while (true) {
			SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
			SDL_RenderClear(renderer);

			scene1.render(mainTexture, SCREEN_WIDTH, SCREEN_HEIGHT, WIDTH_INV, HEIGHT_INV);
			SDL_RenderCopy(renderer, mainTexture, NULL, NULL);
			SDL_RenderPresent(renderer);
			if (!update(window, renderer)) { break; }
			
			
		}

		finish_application(window, renderer);
		return 0;
	}
	else {
		return -1;
	}
}