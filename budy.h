#ifndef BUDY_H
#define BUDY_H

#include "../candle/ecs/ecm.h"

typedef struct
{
	c_t super; /* extends c_t */

	int bodyId;
	int shapeId;
	float mass;
	int update_id;
} c_budy_t;

DEF_CASTER(ct_budy, c_budy, c_budy_t)

c_budy_t *c_budy_sphere_new(float mass, float radius);
c_budy_t *c_budy_box_new(float mass, vec3_t half_extents);
c_budy_t *c_budy_cylinder_new(float mass, float radius, float height);
c_budy_t *c_budy_mesh_new(float mass, const char *fileName, vec3_t scale);
c_budy_t *c_budy_plane_new(float mass, vec3_t normal, float constant);
c_budy_t *c_budy_capsule_new(float mass, float radius, float height);

#endif /* !BUDY_H */
