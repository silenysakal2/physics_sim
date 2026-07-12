#include "physics.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>



Object::Object(uint32_t flags, Vec2 pos, float angle, float mass, float moment_of_inertia, float restitution, float friction, size_t circle_count, CircleHitbox *circles, size_t polygon_count, PolygonHitbox *polygons, bool copy_hitboxes)
{
	this->flags = flags & (OoM_MASS_BIT | NO_GRAVITY_BIT | COLLISION_LAYERS_BIT); // Remove any unwanted stuff
	this->pos = pos;
	this->vel = {0, 0};
	this->acc = {0, 0};
	this->angle = angle;
	this->ang_vel = 0;
	this->ang_acc = 0;
	this->mass_inv = 1 / mass;
	this->moi_inv = 1 / moment_of_inertia;
	this->restitution = restitution;
	this->friction = friction;

	// TODO: Make the hitboxes be all in an array for faster processing
	this->hitbox.circle_count = circle_count;
	if(copy_hitboxes) {
		this->hitbox.circles = (CircleHitbox*) malloc(circle_count * sizeof(CircleHitbox));
		memcpy(this->hitbox.circles, circles, circle_count * sizeof(CircleHitbox));
	}
	else
		this->hitbox.circles = circles;

	this->hitbox.polygon_count = polygon_count;
	if(copy_hitboxes) {
		size_t poly_segments = 0;
		for(size_t i = 0; i < polygon_count; i++)
			poly_segments += polygons[i].n;
		char *pool = (char*) malloc((poly_segments * sizeof(Vec2)) + (polygon_count * sizeof(PolygonHitbox))); // A pool of memory from which the poly hitboxes will be allocated
		this->hitbox.polygons = (PolygonHitbox*) pool;
		pool += polygon_count * sizeof(PolygonHitbox);
		for(size_t i = 0; i < polygon_count; i++) {
			this->hitbox.polygons[i].n = polygons[i].n;
			this->hitbox.polygons[i].points = (Vec2*) pool;
			memcpy(this->hitbox.polygons[i].points, polygons[i].points, polygons[i].n * sizeof(Vec2));
			pool += polygons[i].n * sizeof(Vec2);
		}
	}

	if(copy_hitboxes)
		this->flags |= SELF_OWNED_HITBOXES_BIT;
}


void Object::update()
{
	this->pos += this->vel + (0.5*this->acc);
	this->vel += acc;

	this->angle += this->ang_vel + (0.5*this->ang_acc);
	this->ang_vel += this->ang_acc;
	sincosf(this->angle, &this->sin, &this->cos);
}


inline void Object::nudge(
	Vec2 impulse,
	Vec2 push, // When object need to be pushed out of each other
	Vec2 rel // The point at which it was hit, relative to its pos (=COM)
)
{
	this->vel += impulse * this->mass_inv;
	this->pos += push * this->mass_inv;
	this->ang_vel += ((Vec2) {-rel.y, rel.x}) * impulse * this->moi_inv;
	this->angle += ((Vec2) {-rel.y, rel.x}) * push * this->moi_inv;
}


inline void bounce(
	Object *a, Object *b,
	Vec2 hit_a, Vec2 hit_b, // Where were the objects hit, relative to their pos
	Vec2 normal,
	float hit_vel_normal,
	float depth_end, // Must be compatible with the normal vector in terms of scaling
	float time_ratio
)
{
	Vec2 hit_a_rot = {-hit_a.y, hit_a.x};
	Vec2 hit_b_rot = {-hit_b.y, hit_b.x};
	Vec2 tangent = {-normal.y, normal.x}; // This should get compiled away
	float vel_tangent = (b->vel - a->vel - (time_ratio * (b->acc - a->acc)) + ((b->ang_vel - (time_ratio * b->ang_acc)) * hit_b_rot) - ((a->ang_vel - (time_ratio * a->ang_acc)) * hit_a_rot)) * tangent;
	Vec2 impulse_direction;
	if(vel_tangent > 0)
		impulse_direction = normal - (a->friction * b->friction * tangent);
	else
		impulse_direction = normal + (a->friction * b->friction * tangent);
	float normal_sq_inv = 1 / (normal * normal);
	float imp_sq_inv = 1 / (impulse_direction * impulse_direction);

	// Effective mass: if you only care about the surroundings of the hit, you can make the object act as if its COM lied along the normal vector
	float eff_mass_a = 1 / (
		a->mass_inv +
		(a->moi_inv * (normal * hit_a_rot) * (normal * hit_a_rot) * imp_sq_inv)
	);
	float eff_mass_b = 1 / (
		b->mass_inv +
		(b->moi_inv * (normal * hit_b_rot) * (normal * hit_b_rot) * imp_sq_inv)
	);

	if((a->flags & OoM_MASS_BIT) == (b->flags & OoM_MASS_BIT)) {
		Vec2 impulse = eff_mass_a * eff_mass_b / (eff_mass_a + eff_mass_b) * normal_sq_inv * hit_vel_normal * impulse_direction;
		impulse *= 1 + (a->restitution * b->restitution);
		Vec2 push = eff_mass_a * eff_mass_b / (eff_mass_a + eff_mass_b) * normal_sq_inv * (time_ratio * hit_vel_normal - depth_end) * impulse_direction;
		a->nudge( impulse,  push, hit_a);
		b->nudge(-impulse, -push, hit_b);
	}

	else if((a->flags & OoM_MASS_BIT) < (b->flags & OoM_MASS_BIT)) {
		Vec2 impulse = eff_mass_a * normal_sq_inv * hit_vel_normal * impulse_direction;
		impulse *= 1 + (a->restitution * b->restitution);
		Vec2 push = eff_mass_a * normal_sq_inv * (time_ratio * hit_vel_normal - depth_end) * impulse_direction;
		a->nudge(impulse, push, hit_a);
	}
	else {
		Vec2 impulse = eff_mass_b * normal_sq_inv * hit_vel_normal * impulse_direction;
		impulse *= 1 + (a->restitution * b->restitution);
		Vec2 push = eff_mass_b * normal_sq_inv * (time_ratio * hit_vel_normal - depth_end) * impulse_direction;
		b->nudge(-impulse, -push, hit_b);
	}
}


inline bool collision(Object *a, Object *b, CircleHitbox *ha, CircleHitbox *hb, Vec2 *hit_a, Vec2 *hit_b, Vec2 *normal, float *hit_vel_normal, float *depth_end, float *time_ratio)
{
	Vec2 ca = ha->c.rotate(a->sin, a->cos);
	Vec2 cb = hb->c.rotate(b->sin, b->cos);
	Vec2 rel = (b->pos + cb) - (a->pos + ca);
	float dist_sq = rel * rel;
	float max_dist_sq = (ha->r + hb->r) * (ha->r + hb->r);

	if(dist_sq < max_dist_sq) {
		*depth_end = 0.5 * (max_dist_sq - dist_sq);
		Vec2 rel_vel = b->vel - a->vel;
		Vec2 rel_acc = b->acc - a->acc;
		float dist_inv = 1 / (ha->r + hb->r);
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


inline bool collision(Object *a, Object *b, CircleHitbox *ha, PolygonHitbox *hb, Vec2 *hit_a, Vec2 *hit_b, Vec2 *normal, float *hit_vel_normal, float *depth_end, float *time_ratio)
{
	for(size_t i = 0; i < hb->n; i++) {
		CircleHitbox point;
		point.c = hb->points[i];
		point.r = 0;
		// TODO: Implement a specialized function
		if(collision(a, b, ha, &point, hit_a, hit_b, normal, hit_vel_normal, depth_end, time_ratio))
			return true;

		// Line collision
		if(i < (hb->n-1)) {
			Vec2 p0_rel = hb->points[i].rotate(b->sin, b->cos);
			Vec2 p1_rel = hb->points[i+1].rotate(b->sin, b->cos);
			Vec2 p0 = b->pos + p0_rel;
			Vec2 p1 = b->pos + p1_rel;
			Vec2 line = p1_rel - p0_rel;
			float len = std::sqrt(line * line); // TODO: Cache the sqrt in the hitbox struct (Maybe unneeded in the end?)
			float len_sq_inv = 1 / (line * line);
			Vec2 c = a->pos + ha->c.rotate(a->sin, a->cos);
			Vec2 c_rel = c - p0; // Relative to p0
			float along_line = c_rel * line * len_sq_inv;
			if((along_line >= -(1./1024)) && (along_line <= (1025./1024))) { // Circle within the range of the line
				*normal = {-line.y, line.x};
				float dist = -(*normal) * c_rel;
				// TODO: Make this more precise
				*hit_b = line * along_line + p0_rel;
				*hit_a = b->pos + (*hit_b) - a->pos;
				Vec2 hit_a_rot = ((Vec2) {-hit_a->y, hit_a->x}); // Those are to get compiled away
				Vec2 hit_b_rot = ((Vec2) {-hit_b->y, hit_b->x});
				Vec2 rel_vel = b->vel - a->vel;
				*hit_vel_normal = (rel_vel + (b->ang_vel * hit_b_rot) - (a->ang_vel * hit_a_rot)) * (*normal);
				if((dist < (ha->r * len)) && (dist > (*hit_vel_normal * len * 2)) && (*hit_vel_normal < 0)) { // In each other AND not too far in AND they're moving closer
					*depth_end = len * ha->r - dist;
					*time_ratio = 0.5;
					return true;
				}
			}
		}
	}
	return false;
}


inline bool collision(Object *a, Object *b, PolygonHitbox *ha, PolygonHitbox *hb, Vec2 *hit_a, Vec2 *hit_b, Vec2 *normal, float *hit_vel_normal, float *depth_end, float *time_ratio)
{
	CircleHitbox point;
	point.r = 0;
	for(size_t i = 0; i < ha->n; i++) {
		point.c = ha->points[i];
		// TODO: implement a specialized function: we don't need corner-corner collision
		if(collision(a, b, &point, hb, hit_a, hit_b, normal, hit_vel_normal, depth_end, time_ratio))
			return true;
	}
	for(size_t i = 0; i < hb->n; i++) {
		point.c = hb->points[i];
		if(collision(b, a, &point, ha, hit_b, hit_a, normal, hit_vel_normal, depth_end, time_ratio)) {
			*normal *= -1; // Swap A nad B
			return true;
		}
	}
	return false;
}


inline void collision(Object *a, Object *b)
{
	if(a->flags & b->flags & COLLISION_LAYERS_BIT) {
		Vec2 hit_a, hit_b;
		Vec2 normal;
		float hit_vel_normal;
		float depth_end; // The depth at the end of the tick (so that the objects can push each other out)
		float time_ratio;
		for(size_t ai = 0; ai < a->hitbox.circle_count; ai++) {
			for(size_t bi = 0; bi < b->hitbox.circle_count; bi++)
				if(collision(a, b, a->hitbox.circles + ai, b->hitbox.circles + bi, &hit_a, &hit_b, &normal, &hit_vel_normal, &depth_end, &time_ratio))
					bounce(a, b, hit_a, hit_b, normal, hit_vel_normal, depth_end, time_ratio);
			for(size_t bi = 0; bi < b->hitbox.polygon_count; bi++)
				if(collision(a, b, a->hitbox.circles + ai, b->hitbox.polygons + bi, &hit_a, &hit_b, &normal, &hit_vel_normal, &depth_end, &time_ratio))
					bounce(a, b, hit_a, hit_b, normal, hit_vel_normal, depth_end, time_ratio);
		}
		for(size_t ai = 0; ai < a->hitbox.polygon_count; ai++) {
			for(size_t bi = 0; bi < b->hitbox.circle_count; bi++)
				if(collision(b, a, b->hitbox.circles + bi, a->hitbox.polygons + ai, &hit_b, &hit_a, &normal, &hit_vel_normal, &depth_end, &time_ratio))
					bounce(b, a, hit_b, hit_a, normal, hit_vel_normal, depth_end, time_ratio);
			for(size_t bi = 0; bi < b->hitbox.polygon_count; bi++)
				if(collision(a, b, a->hitbox.polygons + ai, b->hitbox.polygons + bi, &hit_a, &hit_b, &normal, &hit_vel_normal, &depth_end, &time_ratio))
					bounce(a, b, hit_a, hit_b, normal, hit_vel_normal, depth_end, time_ratio);
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
		for(size_t b = a+1; b < this->objects.size(); b++)
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
