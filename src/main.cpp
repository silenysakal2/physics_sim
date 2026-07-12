#include "physics.hpp"
#include <SDL3/SDL.h>

#include <cmath>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unistd.h>


constexpr int WIDTH  = 1024;
constexpr int HEIGHT = 1024;
constexpr int SCALE = 64;


static inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
	return (255 << 24) | (r << 16) | (g << 8) | b;
}

static void putPixel(SDL_Surface* surface, int x, int y, uint32_t color)
{
	if (x < 0 || y < 0 || x >= surface->w || y >= surface->h)
		return;

	uint32_t* pixels = (uint32_t*)surface->pixels;
	pixels[y * surface->w + x] = color;
}

static void clear_surface(SDL_Surface* surface, uint32_t color)
{
	uint32_t* pixels = (uint32_t*)surface->pixels;

	for (int i = 0; i < surface->w * surface->h; ++i)
		pixels[i] = color;
}

void draw_circle(SDL_Surface* target, Vec2 c, float r, uint32_t color)
{
	float r_sq = r * r;
	float r_in_sq = (r-1.5) * (r-1.5);
	for(int x = std::max(0, (int) (c.x - r - 1)); x < std::min(target->w, (int) (c.x + r + 2)); x++)
		for(int y = std::max(0, (int) (c.y - r - 1)); y < std::min(target->h, (int) (c.y + r + 2)); y++) {
			float len_sq = ((y-c.y) * (y-c.y)) + ((x-c.x) * (x-c.x));
			if((len_sq < r_sq) && (len_sq >= r_in_sq))
				((uint32_t*) target->pixels)[target->w * y + x] = color;
		}
}

static void draw_line(SDL_Surface* target, Vec2 a, Vec2 b, uint32_t color)
{
	if((a.x < 0) || (a.y < 0) || (b.x < 0) || (b.y < 0) || (a.x > target->w-1) || (a.y > target->h-1) || (b.x > target->w-1) || (b.y > target->h-1))
		return;
	if(a.x > b.x)
		std::swap(a, b);
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	int x = (int) std::round(a.x);
	int y = (int) std::round(a.y);
	((uint32_t*) target->pixels)[target->w * y + x] = color;
	int x_end = (int) std::round(b.x);
	int y_end = (int) std::round(b.y);
	while(true) {
		if ((x >= x_end) && ((dy > 0) ? (y >= y_end) : (y <= y_end)))
			break;
		if(dy > 0) {
			if(((dx * (y-a.y)) > (dy * (x-a.x))) || (((dx * (y-a.y)) == (dy * (x-a.x))) && (dx > dy)))
				x++;
			else
				y++;
		}
		else {
			if(((dx * (y-a.y)) < (dy * (x-a.x))) || (((dx * (y-a.y)) == (dy * (x-a.x))) && (dx > -dy)))
				x++;
			else
				y--;
		}

		((uint32_t*) target->pixels)[target->w * y + x] = color;
	}
}

void draw_object(SDL_Surface* target, const Object& object, uint32_t color)
{
	for(size_t i = 0; i < object.hitbox.circle_count; i++)
		draw_circle(target, (object.pos + object.hitbox.circles[i].c.rotate(object.sin, object.cos)) * SCALE, object.hitbox.circles[i].r * SCALE, color);
	for(size_t i = 0; i < object.hitbox.polygon_count; i++)
		for(size_t index = 0; index < object.hitbox.polygons[i].n-1; index++)
			draw_line(target, (object.pos + object.hitbox.polygons[i].points[index].rotate(object.sin, object.cos)) * SCALE, (object.pos + object.hitbox.polygons[i].points[index+1].rotate(object.sin, object.cos)) * SCALE, color);
	draw_circle(target, object.pos * SCALE, 8, 0xff'ff'ff'ff);
	draw_line(target, object.pos * SCALE, (object.pos + ((Vec2) {object.cos, object.sin})) * SCALE, 0xff'ff'ff'ff);
}


int main(int argc, char** argv)
{
	if (!SDL_Init(SDL_INIT_VIDEO))
	{
		SDL_Log("SDL_Init failed: %s", SDL_GetError());
		return -1;
	}

	SDL_Window* window = SDL_CreateWindow(
		"Physics sim sample",
		WIDTH,
		HEIGHT,
		0
	);

	SDL_Renderer* renderer =
		SDL_CreateRenderer(window, nullptr);

	SDL_Texture* texture =
		SDL_CreateTexture(
			renderer,
			SDL_PIXELFORMAT_ARGB8888,
			SDL_TEXTUREACCESS_STREAMING,
			WIDTH,
			HEIGHT
		);

	SDL_Surface* framebuffer =
		SDL_CreateSurface(
			WIDTH,
			HEIGHT,
			SDL_PIXELFORMAT_ARGB8888
		);

	Scene my_scene((float) 1. / 60, {0, 9.81});
	Scene my_scene2((float) 1. / 15, {0, 9.81});
	Vec2 cage_points[4];
	cage_points[0] = {1, 1};
	cage_points[1] = {1, 15};
	cage_points[2] = {15, 15};
	cage_points[3] = {15, 1};
	PolygonHitbox cage_poly = {4, cage_points};
	Object cage(OoM_MASS_BIT | NO_GRAVITY_BIT | COLLISION_LAYERS_BIT, {0, 0}, 0, 1, 1, 0.5, 0, 0, NULL, 1, &cage_poly, true);
	my_scene.push_object(cage);
	my_scene2.push_object(cage);
	for(int xi = 2; xi < 15; xi++)
		for(int yi = 2; yi < 4; yi++) {
			CircleHitbox hitbox = {{0, 0}, 0.4};
			Object my_obj(COLLISION_LAYERS_BIT, {xi, yi}, 0, 1, 1, 0.5, 0.5, 1, &hitbox, 0, NULL, true);
			my_scene.push_object(my_obj);
			my_scene2.push_object(my_obj);
		}
	//Object my_object({5, 4}, 1);
	//Object my_object2({5, 4}, 1);

	bool running = true;
	//uint64_t TIME_STEP = 100'000;
	uint64_t TIME_STEP = 16'667;
	//uint64_t TIME_STEP = 33'333;
	uint64_t frame_i = 1;
	uint64_t next_frame_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() + TIME_STEP;

	while (running) {
		SDL_Event e;
		while (SDL_PollEvent(&e)) {
			if (e.type == SDL_EVENT_QUIT)
				running = false;
		}

		clear_surface(framebuffer, 0xff'00'00'00);

		if(!(frame_i % 4))
			my_scene2.tick();
		my_scene.tick();
		frame_i++;

		uint64_t time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if(time < next_frame_timestamp)
			usleep(next_frame_timestamp - time);
		next_frame_timestamp += TIME_STEP;

		for(int x = 0; x < (512 - ((int) (128 * 0.4 / M_SQRT2))); x++) {
			int y = x + 512 + ((int) (128 * 0.4 / M_SQRT2));
			((uint32_t*) framebuffer->pixels)[framebuffer->w * y + x] = 0xffff0000;
		}
		//draw_object(framebuffer, my_object2, 0xff'ff'ff'ff);
		//draw_object(framebuffer, my_object, 0xff'00'00'ff);
		for(int i = 0; i < my_scene.objects.size(); i++)
			draw_object(framebuffer, my_scene.objects[i], 0xff'55'55'55);
		for(int i = 0; i < my_scene2.objects.size(); i++)
			//draw_object(framebuffer, my_scene2.objects[i], 0xaa'00'aa'ff);

		SDL_UpdateTexture(
			texture,
			nullptr,
			framebuffer->pixels,
			framebuffer->pitch
		);
		SDL_RenderClear(renderer);
		SDL_RenderTexture(renderer, texture, nullptr, nullptr);
		SDL_RenderPresent(renderer);
	}

	SDL_DestroySurface(framebuffer);
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);

	SDL_Quit();

	return 0;
}
