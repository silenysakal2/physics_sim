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

void draw_circle(SDL_Surface* target, float cx, float cy, float r, uint32_t color)
{
	float r_sq = r * r;
	float r_in_sq = (r-4) * (r-4);
	for(int x = std::max(0, (int) (cx - r - 1)); x < std::min(target->w, (int) (cx + r + 2)); x++)
		for(int y = std::max(0, (int) (cy - r - 1)); y < std::min(target->h, (int) (cy + r + 2)); y++) {
			float len_sq = ((y-cy) * (y-cy)) + ((x-cx) * (x-cx));
			if((len_sq < r_sq) && (len_sq >= r_in_sq))
				((uint32_t*) target->pixels)[target->w * y + x] = color;
		}
}

void draw_object(SDL_Surface* target, const Object& object, uint32_t color)
{
	for(size_t i = 0; i < object.hitbox.circle_count; i++)
		draw_circle(target, (object.pos.x + (object.cos * object.hitbox.circles[i].c.x) - (object.sin * object.hitbox.circles[i].c.y)) * SCALE, (object.pos.y + (object.sin * object.hitbox.circles[i].c.x) + (object.cos * object.hitbox.circles[i].c.y)) * SCALE, object.hitbox.circles[i].r * SCALE, color);
	draw_circle(target, object.pos.x * SCALE, object.pos.y * SCALE, 8, 0xff'ff'ff'ff);
}

static void drawLine(SDL_Surface* surface,
					 int x0, int y0,
					 int x1, int y1,
					 uint32_t color)
{
	int dx = abs(x1 - x0);
	int dy = -abs(y1 - y0);

	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;

	int err = dx + dy;

	while (true)
	{
		putPixel(surface, x0, y0, color);

		if (x0 == x1 && y0 == y1)
			break;

		int e2 = 2 * err;

		if (e2 >= dy)
		{
			err += dy;
			x0 += sx;
		}

		if (e2 <= dx)
		{
			err += dx;
			y0 += sy;
		}
	}
}

static void fillTriangle(SDL_Surface* surface,
						 Vec2 v0, Vec2 v1, Vec2 v2,
						 uint32_t color)
{
	auto edge = [](Vec2 a, Vec2 b, Vec2 c)
	{
		return (c.x - a.x) * (b.y - a.y)
			 - (c.y - a.y) * (b.x - a.x);
	};

	int minX = (int)std::floor(std::min({v0.x, v1.x, v2.x}));
	int maxX = (int)std::ceil (std::max({v0.x, v1.x, v2.x}));

	int minY = (int)std::floor(std::min({v0.y, v1.y, v2.y}));
	int maxY = (int)std::ceil (std::max({v0.y, v1.y, v2.y}));

	for (int y = minY; y <= maxY; ++y)
	{
		for (int x = minX; x <= maxX; ++x)
		{
			Vec2 p = {(float)x + 0.5f, (float)y + 0.5f};

			float w0 = edge(v1, v2, p);
			float w1 = edge(v2, v0, p);
			float w2 = edge(v0, v1, p);

			if (w0 >= 0 && w1 >= 0 && w2 >= 0)
				putPixel(surface, x, y, color);
		}
	}
}

static void drawPolygon(SDL_Surface* surface)
{
	std::vector<Vec2> verts =
	{
		{120, 120},
		{220, 100},
		{320, 180},
		{280, 300},
		{150, 260}
	};

	// Triangle fan
	for (size_t i = 1; i + 1 < verts.size(); ++i)
	{
		fillTriangle(surface,
					 verts[0],
					 verts[i],
					 verts[i + 1],
					 rgb(255, 140, 60));
	}
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

	Scene my_scene;
	Scene my_scene2;
	for(int xi = 0; xi < 1; xi++)
		for(int yi = 0; yi < 2; yi++) {
			Object my_obj({(float) (1 + 5*xi), (float) (1 + 2.5*yi)}, {(float) (yi * 0.01), (float) (xi * 0.01)}, 0, 1);
			my_scene.push_object(my_obj);
			my_obj.vel *= 4;
			my_scene2.push_object(my_obj);
		}
	//Object my_object({5, 4}, 1);
	//Object my_object2({5, 4}, 1);

	bool running = true;
	uint64_t TIME_STEP = 100'000;
	//uint64_t TIME_STEP = 16'667;
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
			my_scene2.tick({0, 0.001 * 16});
		my_scene.tick({0, 0.001});
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
			draw_object(framebuffer, my_scene2.objects[i], 0xaa'00'aa'ff);

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
