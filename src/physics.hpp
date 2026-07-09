#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>


struct Vec2
{
	float x, y;

	inline Vec2 operator+(const Vec2 other) const {
		return {this->x + other.x, this->y + other.y};}
	inline void operator+=(const Vec2 other) {
		*this = *this + other;}
	inline Vec2 operator-() const {
		return {-this->x, -this->y};}
	inline Vec2 operator-(const Vec2 other) const {
		return {this->x - other.x, this->y - other.y};}
	inline float operator*(const Vec2 other) const {
		return (this->x * other.x) + (this->y * other.y);}
	inline Vec2 operator*(const float mul) const {
		return {mul * this->x, mul * this->y};}
	inline void operator*=(const float mul) {
		*this = *this * mul;}
	inline Vec2 operator/(const float div) const {
		return {this->x / div, this->y / div};}
};
inline Vec2 operator*(const float mul, Vec2 vec) {
	return {mul * vec.x, mul * vec.y};}

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
	uint32_t n; // TODO: Implement actual polygon hitboxes
	Vec2 *points;
};

struct Hitbox
{
	size_t circle_count;
	CircleHitbox *circles;
	size_t polygon_count;
	PolygonHitbox *polygons;
};

const uint32_t NO_GRAVITY_BIT = 0x00'00'10'00;
const uint32_t COLLISION_LAYERS_BIT = 0xff'ff'00'00;
struct Object
{
	uint32_t flags;
	/*
	Bits 0--7: Reserved for memory ownership stuff in the future [TODO]
	Bits 8--11: Order-of-magnitude mass [TODO]
	Bit 12: No gravity acceleration
	Bits 13--15: idk
	Bits 16--31: Collision layers
	*/
	Vec2 pos;
	Vec2 vel;
	Vec2 acc; // Here, you set the desired acceleration; it's then used for sub-tick collisions
	float angle;
	float ang_vel;
	float ang_acc; // The same as `acc`
	float sin, cos; // Of the angle

	float mass_inv; // 1 / mass
	float moi_inv; // 1 / moment of inertia
	float restitution;

	Hitbox hitbox;

	Object(Vec2 pos, Vec2 vel, float angle, float r, uint32_t flags = COLLISION_LAYERS_BIT);

	void update();
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

inline void collision(Object *a, Object *b);
inline bool collision(Object *a, Object *b, CircleHitbox *ha, CircleHitbox *hb, Vec2 *hit_a, Vec2 *hit_b, Vec2 *normal, float *hit_vel_normal, float *time_ratio);

struct Scene
{
	float dt; // The scene automatically adjusts everything for its delta time; set this to 1 if you don't want that
	Vec2 gravity;
	std::vector<Object> objects;

	Scene(float dt, Vec2 gravity);
	void tick();
	void push_object(const Object& object);
};
