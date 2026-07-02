#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>


struct Vec2
{
    float x, y;

    inline Vec2 operator+(const Vec2 other) const;
    inline void operator+=(const Vec2 other);
    inline Vec2 operator-() const;
    inline Vec2 operator-(const Vec2 other) const;
    inline float operator*(const Vec2 other) const;
    inline Vec2 operator*(const float mul) const;
};
inline Vec2 operator*(const float mul, Vec2 vec);

struct Vec3
{
    float x, y, z;
};

struct CircleHitbox
{
	Vec2 c;
	float r;
};

struct PolygonHitbox
{
	uint32_t n;
	Vec2 *points;
};

struct Object
{
	uint32_t flags;
	/*
	Bits 0--15: Something in the future (like gluing objects to scene)
	Bits 16--31: Collision layers (not yet implemented)
	*/
	Vec2 pos;
	Vec2 vel;
	Vec2 acc; // Meant to save the acceleration applied at the last update(), to be used for bouncing later
	float angle;
	float ang_vel;
	float ang_acc; // The same as `acc`

	CircleHitbox hitbox;
	float mass;
	float moment_of_inertia;

	Object(Vec2 pos, float r);

	void update(Vec2 acc);
	inline void collision(Object *other);
	inline void bounce(
		Vec2 normal, // Should point towards this object; does NOT have to be a unit vector (it's faster than doing square roots)
		Vec2 comfv, // Velocity of the center-of-mass frame
		float depth, // How deep the object was in the other collider. Has to be scaled according to the normal vector's length: when you multiply them, that must produce the actual shift
		Vec2 relative_acc // Relative acceleration
	);
};

struct Scene
{
	std::vector<Object> objects;

	void tick(Vec2 acc);
	void push_object(const Object& object);
};
