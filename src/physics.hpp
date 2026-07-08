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
    inline Vec2 operator/(const float div) const;
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
	Bits 0--7: Reserved for memory ownership stuff in the future
	Bits 8--11: Order-of-magnitude mass
	Bit 12: No *positional* acceleration
	Bit 13: No angular acceleration
	Bits 14 and 15: idk
	Bits 16--31: Collision layers
	*/
	Vec2 pos;
	Vec2 vel;
	Vec2 acc; // Meant to save the acceleration applied at the last update(), to be used for bouncing later
	float angle;
	float ang_vel;
	float ang_acc; // The same as `acc`
	float sin, cos; // Of the angle

	float mass_inv; // 1 / mass
	float moi_inv; // 1 / moment of inertia

	CircleHitbox hitbox;

	Object(Vec2 pos, Vec2 vel, float angle, float r);

	void update(Vec2 acc);
	inline void collision(Object *other);
	inline void bounce(
		Vec2 normal, // Should point towards this object; does NOT have to be a unit vector (it's faster than doing square roots)
		Vec2 comfv, // Velocity of the center-of-mass frame
		float depth, // How deep the object was in the other collider. Has to be scaled according to the normal vector's length: when you multiply them, that must produce the actual shift
		Vec2 relative_acc, // Relative acceleration
		float time_ratio = 0.5 // How much of a tick has passed *before* the bounce has happened
	);
	inline void nudge(
		Vec2 impulse,
		Vec2 rel, // The point at which it was hit, relative to its pos (=COM)
		float time_ratio = 0.5
	);
};

struct Scene
{
	std::vector<Object> objects;

	void tick(Vec2 acc);
	void push_object(const Object& object);
};
