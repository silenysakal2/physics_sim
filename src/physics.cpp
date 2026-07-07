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

inline Vec2 Vec2::operator/(const float div) const
{
	return {this->x / div, this->y / div};
}



Object::Object(Vec2 pos, Vec2 vel, float angle, float r)
{
	this->pos = pos;
	this->vel = vel;
	this->angle = angle;
	this->ang_vel = 0;
	this->ang_acc = 0; // TODO: Remove this when adding angular acceleration
	this->hitbox = {{1, 0}, r};
	this->mass = r * r;
	this->moment_of_inertia = 1;
}


void Object::update(Vec2 acc)
{
	this->pos += this->vel + (0.5*acc);
	this->vel += acc;
	this->acc = acc;

	this->angle += this->ang_vel + (0.5*this->ang_acc);
	this->ang_vel += this->ang_acc;
	// TODO: Stuff with (nonzero) angular acceleration applied
	sincosf(this->angle, &this->sin, &this->cos);
	this->sin = std::sin(this->angle);
	this->cos = std::cos(this->angle);

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
	// TODO: Fix restitution
}


inline void Object::nudge(
	Vec2 impulse,
	Vec2 rel, // The point at which it was hit, relative to its pos (=COM)
	float time_ratio
)
{
	this->vel += impulse / this->mass;
	this->pos += impulse / this->mass * (2*time_ratio);
	this->ang_vel += ((Vec2) {-rel.y, rel.x}) * impulse / this->moment_of_inertia;
	this->angle += ((Vec2) {-rel.y, rel.x}) * impulse / this->moment_of_inertia * (2*time_ratio);
}


inline void Object::collision(Object *other)
{
	Vec2 ca = {(this ->cos * this ->hitbox.c.x) - (this ->sin * this ->hitbox.c.y), (this ->sin * this ->hitbox.c.x) + (this ->cos * this ->hitbox.c.y)};
	Vec2 cb = {(other->cos * other->hitbox.c.x) - (other->sin * other->hitbox.c.y), (other->sin * other->hitbox.c.x) + (other->cos * other->hitbox.c.y)};
	Vec2 rel = (other->pos + cb) - (this->pos + ca);
	float dist_sq = rel * rel;
	float max_dist_sq = (this->hitbox.r + other->hitbox.r) * (this->hitbox.r + other->hitbox.r);

	if(dist_sq < max_dist_sq) {
		float inv_mass_sum = 1 / (this->mass + other->mass);
		Vec2 comfv = ((this->vel * this->mass) + (other->vel * other->mass)) * inv_mass_sum;
		float depth = 0.5 * (max_dist_sq - dist_sq);

		// Calculate relative hit positions
		// Hit positions could only be approximated more precisely with more sin and cos calculations, which I want to avoid.
		// TODO: Better approximation (will involve depth and time_ratio recalculation all at once)
		float dist_inv = 1 / (this->hitbox.r + other->hitbox.r - depth);
		Vec2 hit_a = ca + (this ->hitbox.r * dist_inv * rel); // Collision location relative to A's COM
		Vec2 hit_b = cb - (other->hitbox.r * dist_inv * rel);
		Vec2 hit_a_rot = ((Vec2) {-hit_a.y, hit_a.x});
		Vec2 hit_b_rot = ((Vec2) {-hit_b.y, hit_b.x});

		Vec2 normal = rel;
		float normal_sq_inv = 1 / (normal * normal); // Using this we deal with non-unit normal vectors without actually needing sqrt
		float hit_vel_normal = (other->vel + (other->ang_vel * hit_b_rot) - (this->vel + (this ->ang_vel * hit_a_rot))) * normal; // Made to be *compatible* with `depth` in terms of scaling
		float hit_acc_normal = (other->acc + (other->ang_acc * hit_b_rot) - (this->acc + (this ->ang_acc * hit_a_rot))) * normal;
		float time_ratio = 0.5;
		constexpr int ITERATION_COUNT = 2;
		for(int it_i = 0; it_i < ITERATION_COUNT; it_i++) { // An approximation of when within the tick has the bounce occured, without needing a sqrt
			float avg_vel = hit_vel_normal - (0.5*time_ratio * hit_acc_normal); // Estimated average velocity after bouncing; based on the previous bounce time ratio
			time_ratio = depth / avg_vel;
		}
		if(time_ratio < 0) time_ratio = 0;
		if(time_ratio > 1) time_ratio = 1;
		std::cout << "Time ratio: " << time_ratio << '\n';
		time_ratio = 0.5; // Hotfix [TODO: fix properly]

		// Effective mass: if you only care about the surroundings of the hit, you can make the object act as if its COM lied along the normal vector
		// TODO: Add friction
		// TODO: Make the onject store inverse mass and moment of inertia to make this 3x less division-intensive
		float eff_mass_a = 1 / (
			(1 / this->mass) +
			(1 / this->moment_of_inertia * (normal * hit_a_rot) * (normal * hit_a_rot) * normal_sq_inv)
		);
		float eff_mass_b = 1 / (
			(1 / this->mass) +
			(1 / this->moment_of_inertia * (normal * hit_b_rot) * (normal * hit_b_rot) * normal_sq_inv)
		);
		std::cout << "Eff masses: " << eff_mass_a << ' ' << eff_mass_b << '\n';
		// TODO: Make relative velocity more precise at this point in time

		Vec2 impulse = 2 * eff_mass_a * eff_mass_b / (eff_mass_a + eff_mass_b) * normal_sq_inv * hit_vel_normal * normal;
		std::cout << "Impulse: " << impulse.x << ' ' << impulse.y << '\n';
		// TODO: Add restitution

		this ->nudge( impulse, hit_a, time_ratio);
		other->nudge(-impulse, hit_b, time_ratio);
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
