#include <candle.h>
#include <utils/mafs.h>
#include <components/spacial.h>
#include <PhysicsClientC_API.h>
#include <PhysicsDirectC_API.h>
#include <SharedMemoryInProcessPhysicsC_API.h>
#include "budy.h"
#include "bullet.h"


c_budy_t *c_budy_new()
{
	c_budy_t *self = component_new("budy");

	self->shapeId = bullet_createCollisionShape(GEOM_PLANE, 0.0f, d3(0,0,0), -1, NULL, d3(1,1,1),
			d3(0,0,1), 0, d3(0,0,0), d4(0,0,0,1), 0);


	return self;
}

void c_budy_init(c_budy_t *self)
{
	self->mass = 1.0f;
}

static inline void *c_budy_shape_init(c_budy_t *self)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	return b3CreateCollisionShapeCommandInit(sm);
}

static inline int c_budy_shape_end(c_budy_t *self, void *cmd)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	c_spacial_t *sc = c_spacial(self);

	b3SharedMemoryStatusHandle statusHandle =
		b3SubmitClientCommandAndWaitStatus(sm, cmd);
	int statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_CREATE_COLLISION_SHAPE_COMPLETED)
	{
		return -1;
	}
	self->shapeId = b3GetStatusCollisionShapeUniqueId(statusHandle);
	self->bodyId = bullet_createMultiBody(self->mass, self->shapeId,
			-1, d3(_vec3(sc->pos)), d4(0,0,0,1), d3(0, 0, 0), d4(0,0,0,1), 0,
			NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			1, 0, 0);

	return -1;
}

c_budy_t *c_budy_sphere_new(float mass, float radius)
{
	c_budy_t *self = component_new("budy");
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3CreateCollisionShapeAddSphere(cmd, radius);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_box_new(float mass, vec3_t half_extents)
{
	c_budy_t *self = component_new("budy");
	self->mass = mass;
	d3_t extents = d3(_vec3(half_extents));

	void *cmd = c_budy_shape_init(self);
	b3CreateCollisionShapeAddBox(cmd,
			extents._);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_cylinder_new(float mass, float radius, float height)
{
	c_budy_t *self = component_new("budy");
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3CreateCollisionShapeAddCylinder(cmd,
			radius, height);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_mesh_new(float mass, const char *fileName, vec3_t scale)
{
	d3_t mesh_scale = d3(_vec3(scale));
	c_budy_t *self = component_new("budy");
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3CreateCollisionShapeAddMesh(cmd, fileName,
			mesh_scale._);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_plane_new(float mass, vec3_t normal, float constant)
{
	c_budy_t *self = component_new("budy");
	self->mass = mass;
	d3_t nrm = d3(_vec3(normal));

	void *cmd = c_budy_shape_init(self);
	b3CreateCollisionShapeAddPlane(cmd,
			nrm._, constant);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_capsule_new(float mass, float radius, float height)
{
	c_budy_t *self = component_new("budy");
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3CreateCollisionShapeAddCapsule(cmd,
			radius, height);

	c_budy_shape_end(self, cmd);
	return self;
}

int c_budy_update(c_budy_t *self, float *dt)
{
	d3_t p;
	d4_t q;
	c_spacial_t *sc = c_spacial(self);
	if(sc->update_id > self->update_id)
	{
		p = d3(_vec3(sc->pos));
		q = d4(_vec4(sc->rot_quat));
		bullet_resetBasePositionAndOrientation(self->bodyId, p, q, 0);
	}
	if(bullet_getBasePositionAndOrientation(self->bodyId, &p, &q, 0) > 0)
	{
		c_spacial_lock(sc);
		sc->pos = vec3(_d3(p));
		sc->rot_quat = vec4(_d4(q));
		sc->update_id++;
		sc->modified = 1;
		self->update_id = sc->update_id;
		c_spacial_unlock(sc);
	}

	return CONTINUE;
}

void ct_budy(ct_t *self)
{
	ct_init(self, "budy", sizeof(c_budy_t), c_budy_init));
	ct_set_init(self, (init_cb)c_budy_init);
	ct_add_dependency(self, ct_spatial);
	ct_listener(ct, WORLD, sig("world_update"), c_budy_update);
}

