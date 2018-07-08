#ifndef PHYSICS_H
#define PHYSICS_H

#include <ecm.h>

typedef float(*collider_cb)(c_t *self, vec3_t pos);
typedef float(*velocity_cb)(c_t *self, vec3_t pos);

typedef struct c_bullet_t
{
	c_t super;
	int running;
	float time_scale;
} c_bullet_t;

DEF_CASTER("bullet", c_bullet, c_bullet_t)

c_bullet_t *c_bullet_new(void);

#endif /* !PHYSICS_H */
