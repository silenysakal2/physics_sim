#include "physics.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>


inline Vec2 Vec2::operator+(const Vec2 other) const
{
	return {this->x + other.x, this->y + other.y};
}
inline void Vec2::operator+=(const Vec2 other)
{
	*this = *this + other;
}

inline Vec2 Vec2::operator-() const
{
	return {-this->x, -this->y};
}
inline Vec2 Vec2::operator-(const Vec2 other) const
{
	return {this->x - other.x, this->y - other.y};
}

inline float Vec2::operator*(const Vec2 other) const
{
	return (this->x * other.x) + (this->y * other.y);
}

inline Vec2 Vec2::operator*(const float mul) const
{
	return {mul * this->x, mul * this->y};
}
inline Vec2 operator*(const float mul, Vec2 vec)
{
	return {mul * vec.x, mul * vec.y};
}



Object::Object(Vec2 pos, float r)
{
	this->pos = pos;
	this->vel = {0, 0};
	this->hitbox = {{0, 0}, r};
	this->mass = r * r;
}


void Object::update(Vec2 acc)
{
	this->pos += this->vel + (acc*0.5);
	this->vel += acc;
	this->acc = acc;

	if(this->pos.x < this->hitbox.r)
		bounce({1, 0}, {0, 0}, this->hitbox.r - this->pos.x, this->acc);
	if(this->pos.x > (16 - this->hitbox.r))
		bounce({-1, 0}, {0, 0}, this->pos.x - 16 + this->hitbox.r, this->acc);
	if((this->pos.y - 8) > this->pos.x)
		bounce({1, -1}, {0, 0}, (this->pos.y - 8 - this->pos.x), this->acc);
	if((this->pos.y + this->pos.x) > 24)
		bounce({-1, -1}, {0, 0}, (this->pos.y + this->pos.x - 24), this->acc);
	if(this->pos.y > (16 - this->hitbox.r))
		bounce({0, -1}, {0, 0}, this->pos.y - 16 + this->hitbox.r, this->acc);
}


inline void Object::bounce(
	Vec2 normal, // Should point towards this object; does NOT have to be a unit vector (it's faster than doing square roots)
	Vec2 comfv, // Velocity of the center-of-mass frame
	float depth, // How deep the object was in the other collider. Has to be scaled according to the normal vector's length: it should be as larger than the actual shift as is the length of the normal vector
	Vec2 rel_acc, // Relative acceleration
	float time_ratio // How much of a tick has passed *before* the bounce has happened
)
{
	Vec2 tangent = {normal.y, -normal.x};

	float vel_normal = (this->vel - comfv) * normal; // Made to be *compatible* with `depth` in terms of scaling
	float vel_tangent = (this->vel - comfv) * tangent;
	float acc_normal = rel_acc * normal;
	//float acc_tangent = rel_acc * tangent_inv_scaled; // This actually gets never used

	float normal_len_sq = normal * normal; // Here we deal with non-unit normal vectors without actually needing sqrt
	Vec2 normal_inv_scaled = (1 / normal_len_sq) * normal;
	Vec2 tangent_inv_scaled = {normal_inv_scaled.y, -normal_inv_scaled.x};

	this->pos += (depth - (time_ratio * (vel_normal - (time_ratio * acc_normal)))) * normal_inv_scaled;
	this->vel = (vel_tangent * tangent_inv_scaled) - (0.95 * normal_inv_scaled * (vel_normal - (2*time_ratio * acc_normal))) + comfv;
}


inline void Object::collision(Object *other)
{
	Vec2 rel = other->pos - this->pos;
	float dist_sq = rel * rel;
	float max_dist_sq = (this->hitbox.r + other->hitbox.r) * (this->hitbox.r + other->hitbox.r);

	if(dist_sq < max_dist_sq) {
		float inv_mass_sum = 1 / (this->mass + other->mass);
		Vec2 comfv = ((this->vel * this->mass) + (other->vel * other->mass)) * inv_mass_sum;
		float depth = 0.5 * (max_dist_sq - dist_sq);
		// TODO: Am I approximating depth correctly?

		Vec2 normal = rel;
		float vel_normal = (other->vel - this->vel) * normal;
		float acc_normal = (other->acc - this->acc) * normal;
		float time_ratio = 0.5;
		constexpr int ITERATION_COUNT = 2;
		for(int it_i = 0; it_i < ITERATION_COUNT; it_i++) { // An approximation of when within the tick has the bounce occured, without needing a sqrt
			float avg_vel = vel_normal - (0.5*time_ratio * acc_normal); // Estimated average velocity after bouncing; based on the previous bounce time ratio
			time_ratio = depth / avg_vel;
		}
		if(time_ratio < 0) time_ratio = 0;
		if(time_ratio > 1) time_ratio = 1;

		this ->bounce(-rel, comfv, depth * other->mass * inv_mass_sum, {this ->acc - other->acc}, time_ratio);
		other->bounce( rel, comfv, depth * this ->mass * inv_mass_sum, {other->acc - this ->acc}, time_ratio);
	}
}


void Scene::tick(Vec2 acc)
{
	for(size_t i = 0; i < this->objects.size(); i++)
		this->objects[i].update(acc);
	for(size_t a = 0; a < this->objects.size(); a++)
		for(size_t b = 0; b < a; b++)
			this->objects[a].collision(&this->objects[b]);
}

void Scene::push_object(const Object& object)
{
	this->objects.push_back(object);
}
