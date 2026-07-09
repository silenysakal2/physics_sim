#include "physics.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>



Object::Object(Vec2 pos, Vec2 vel, float angle, float r, uint32_t flags)
{
	this->flags = flags;
	this->pos = pos;
	this->vel = vel;
	this->acc = {0, 0};
	this->angle = angle;
	this->ang_vel = 0;
	this->ang_acc = 0;
	this->mass_inv = 1 / (r * r);
	this->moi_inv = 1;
	this->restitution = 0.95;
	this->hitbox.circle_count = 2;
	// TODO: Make the hitboxes be all in an array for faster processing
	this->hitbox.circles = (CircleHitbox*) malloc(2 * sizeof(CircleHitbox));
	this->hitbox.circles[0] = {{r, 0}, r};
	this->hitbox.circles[1] = {{-r, 0}, r};
	this->hitbox.polygon_count = 0;
}


void Object::update()
{
	this->pos += this->vel + (0.5*this->acc);
	this->vel += acc;

	this->angle += this->ang_vel + (0.5*this->ang_acc);
	this->ang_vel += this->ang_acc;
	sincosf(this->angle, &this->sin, &this->cos);

	// Hard-coded boundaries [TODO: remove when possible]
	if(this->pos.x < this->hitbox.circles[0].r)
		bounce({1, 0}, {0, 0}, this->hitbox.circles[0].r - this->pos.x, this->acc);
	if(this->pos.x > (16 - this->hitbox.circles[0].r))
		bounce({-1, 0}, {0, 0}, this->pos.x - 16 + this->hitbox.circles[0].r, this->acc);
	if((this->pos.y - 8) > this->pos.x)
		bounce({1, -1}, {0, 0}, (this->pos.y - 8 - this->pos.x), this->acc);
	if((this->pos.y + this->pos.x) > 24)
		bounce({-1, -1}, {0, 0}, (this->pos.y + this->pos.x - 24), this->acc);
	if(this->pos.y > (16 - this->hitbox.circles[0].r))
		bounce({0, -1}, {0, 0}, this->pos.y - 16 + this->hitbox.circles[0].r, this->acc);
}


inline void Object::bounce(
	Vec2 normal, // Should point towards this object; does NOT have to be a unit vector (it's faster than doing square roots)
	Vec2 comfv, // Velocity of the center-of-mass frame
	float depth, // How deep the object was in the other collider. Has to be scaled according to the normal vector's length: it should be as larger than the actual shift as is the length of the normal vector
	Vec2 rel_acc, // Relative acceleration
	float time_ratio // How much of a tick has passed *before* the bounce has happened
)
{
	// TODO: Remove this function when possible
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


inline void Object::nudge(
	Vec2 impulse,
	Vec2 rel, // The point at which it was hit, relative to its pos (=COM)
	float time_ratio
)
{
	this->vel += impulse * this->mass_inv;
	this->pos += impulse * this->mass_inv * (time_ratio);
	this->ang_vel += ((Vec2) {-rel.y, rel.x}) * impulse * this->moi_inv;
	this->angle += ((Vec2) {-rel.y, rel.x}) * impulse * this->moi_inv * (time_ratio);
}


inline bool collision(Object *a, Object *b, CircleHitbox *ha, CircleHitbox *hb, Vec2 *hit_a, Vec2 *hit_b, Vec2 *normal, float *hit_vel_normal, float *time_ratio)
{
	Vec2 ca = ha->c.rotate(a->sin, a->cos);
	Vec2 cb = hb->c.rotate(b->sin, b->cos);
	Vec2 rel = (b->pos + cb) - (a->pos + ca);
	float dist_sq = rel * rel;
	float max_dist_sq = (ha->r + hb->r) * (ha->r + hb->r);

	if(dist_sq < max_dist_sq) {
		Vec2 rel_vel = b->vel - a->vel;
		Vec2 rel_acc = b->acc - a->acc;
		*time_ratio = 0.5;
		constexpr int ITERATION_COUNT = 2;
		for(int it_i = 0; it_i < ITERATION_COUNT; it_i++) { // An approximation of when within the tick the bounce occured
			// Recalculate the relative circle positions according to the time_ratio
			ca = ha->c.rotate(a->sin, a->cos) + (*time_ratio * a->ang_vel * ha->c.rotate(-a->cos, a->sin));
			cb = hb->c.rotate(b->sin, b->cos) + (*time_ratio * b->ang_vel * hb->c.rotate(-b->cos, b->sin));
			rel = (b->pos + cb) - (a->pos + ca) - (*time_ratio * (rel_vel - (*time_ratio*0.5*rel_acc)));
			dist_sq = rel * rel;
			float depth = 0.5 * (max_dist_sq - dist_sq); // How deep into each other the objects were, at the estimated time ratio; it should be about as much longer than the actual depth as the normal vector
			// We multiply by the sum of radii to approximately match it with the length of the normal vector, by whose length the normal relative velocity will be multiplied
			*normal = rel; // Not necessarily a unit vector

			// Calculate relative hit positions
			// Hit positions could only be approximated more precisely with more sin and cos calculations, which I want to avoid.
			float dist_inv = 1 / (ha->r + hb->r - depth);
			*hit_a = ca + (ha->r * dist_inv * rel); // Collision location relative to A's COM
			*hit_b = cb - (hb->r * dist_inv * rel);
			Vec2 hit_a_rot = ((Vec2) {-hit_a->y, hit_a->x}); // Those are to get compiled away
			Vec2 hit_b_rot = ((Vec2) {-hit_b->y, hit_b->x});

			float hit_acc_normal = (b->acc + (b->ang_acc * hit_b_rot) - (a->acc + (a->ang_acc * hit_a_rot))) * (*normal);
			*hit_vel_normal = (b->vel + (b->ang_vel * hit_b_rot) - (a->vel + (a->ang_vel * hit_a_rot))) * (*normal) - (*time_ratio * hit_acc_normal); // Made to be *compatible* with `depth` in terms of scaling
			// hit_vel_normal is negative
			*time_ratio -= depth / (*hit_vel_normal);
			if(*time_ratio < -0.5) *time_ratio = -0.5; // time_ratio should technically be within [0, 1], but I found this to be more stable
			if(*time_ratio > 1.5) *time_ratio = 1.5;
		}

		return true;
	}
	else
		return false;
}


inline void collision(Object *a, Object *b)
{
	if(a->flags & b->flags & COLLISION_LAYERS_BIT) {
		float time_ratio;
		Vec2 hit_a, hit_b;
		Vec2 normal;
		float hit_vel_normal;
		for(size_t ai = 0; ai < a->hitbox.circle_count; ai++)
			for(size_t bi = 0; bi < b->hitbox.circle_count; bi++)
				if(collision(a, b, a->hitbox.circles + ai, b->hitbox.circles + bi, &hit_a, &hit_b, &normal, &hit_vel_normal, &time_ratio)) {
					// TODO: Add friction

					// Effective mass: if you only care about the surroundings of the hit, you can make the object act as if its COM lied along the normal vector
					Vec2 hit_a_rot = {-hit_a.y, hit_a.x};
					Vec2 hit_b_rot = {-hit_b.y, hit_b.x};
					float normal_sq_inv = 1 / (normal * normal); // This should get compiled away with inlining

					float eff_mass_a = 1 / (
						a->mass_inv +
						(a->moi_inv * (normal * hit_a_rot) * (normal * hit_a_rot) * normal_sq_inv)
					);
					float eff_mass_b = 1 / (
						b->mass_inv +
						(b->moi_inv * (normal * hit_b_rot) * (normal * hit_b_rot) * normal_sq_inv)
					);

					Vec2 impulse = 2 * eff_mass_a * eff_mass_b / (eff_mass_a + eff_mass_b) * normal_sq_inv * hit_vel_normal * normal;
					impulse *= a->restitution * b->restitution;
					a->nudge( impulse, hit_a, time_ratio);
					b->nudge(-impulse, hit_b, time_ratio);
				}
	}
}


Scene::Scene(float dt, Vec2 gravity)
{
	this->dt = dt;
	this->gravity = gravity * (dt * dt);
}


void Scene::tick()
{
	for(size_t i = 0; i < this->objects.size(); i++)
		this->objects[i].update();
	for(size_t a = 0; a < this->objects.size(); a++)
		for(size_t b = 0; b < a; b++)
			collision(&this->objects[a], &this->objects[b]);
}

void Scene::push_object(const Object& object)
{
	this->objects.push_back(object);
	Object& obj = this->objects[this->objects.size()-1];
	obj.vel *= this->dt;
	obj.acc *= this->dt * this->dt;
	if(!(obj.flags & NO_GRAVITY_BIT))
		obj.acc += this->gravity;
	obj.ang_vel *= this->dt;
	obj.ang_acc *= this->dt * this->dt;
}
