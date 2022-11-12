#include "../candle/candle.h"
#include "../candle/utils/mafs.h"
#include "../candle/components/spatial.h"
#include "../candle/components/node.h"
#include "../candle/components/model.h"
#include "budy.h"
#include "bullet.h"
#include "blw.h"


void c_budy_init(c_budy_t *self)
{
	self->bodyId = -1;
	self->mass = 1.0f;
}

static inline void *c_budy_shape_init(c_budy_t *self)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	return b3wCreateCollisionShapeCommandInit(sm);
}

static inline int c_budy_shape_end(c_budy_t *self, void *cmd)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	c_spatial_t *sc = c_spatial(self);

	b3SharedMemoryStatusHandle statusHandle =
		b3wSubmitClientCommandAndWaitStatus(sm, cmd);
	int statusType = b3wGetStatusType(statusHandle);
	if (statusType != CMD_CREATE_COLLISION_SHAPE_COMPLETED)
	{
		return -1;
	}
	self->shapeId = b3wGetStatusCollisionShapeUniqueId(statusHandle);
	self->bodyId = bullet_createMultiBody(c_bullet(&SYS), self->mass, self->shapeId,
			-1, d3(_vec3(sc->pos)), d4(0,0,0,1), d3(0, 0, 0), d4(0,0,0,1), 0,
			NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			1, 0);

	return -1;
}

c_budy_t *c_budy_sphere_new(float mass, float radius)
{
	c_budy_t *self = component_new(ct_budy);
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3wCreateCollisionShapeAddSphere(cmd, radius);
	c_budy_shape_end(self, cmd);
	if (self->bodyId == -1)
		c_model_set_visible(c_model(self), false);
	return self;
}

c_budy_t *c_budy_box_new(float mass, vec3_t half_extents)
{
	c_budy_t *self = component_new(ct_budy);
	self->mass = mass;
	d3_t extents = d3(_vec3(half_extents));

	void *cmd = c_budy_shape_init(self);
	b3wCreateCollisionShapeAddBox(cmd, &extents.x);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_cylinder_new(float mass, float radius, float height)
{
	c_budy_t *self = component_new(ct_budy);
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3wCreateCollisionShapeAddCylinder(cmd,
			radius, height);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_mesh_new(float mass, const char *fileName, vec3_t scale)
{
	d3_t mesh_scale = d3(_vec3(scale));
	c_budy_t *self = component_new(ct_budy);
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3wCreateCollisionShapeAddMesh(cmd, fileName, &mesh_scale.x);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_plane_new(float mass, vec3_t normal, float constant)
{
	c_budy_t *self = component_new(ct_budy);
	self->mass = mass;
	d3_t nrm = d3(_vec3(normal));

	void *cmd = c_budy_shape_init(self);
	b3wCreateCollisionShapeAddPlane(cmd, &nrm.x, constant);
	c_budy_shape_end(self, cmd);

	return self;
}

c_budy_t *c_budy_capsule_new(float mass, float radius, float height)
{
	c_budy_t *self = component_new(ct_budy);
	self->mass = mass;

	void *cmd = c_budy_shape_init(self);
	b3wCreateCollisionShapeAddCapsule(cmd, radius, height);

	c_budy_shape_end(self, cmd);
	return self;
}

static
void c_budy_update_position(c_budy_t *self)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	c_spatial_t *sc = c_spatial(self);

	if (self->bodyId == -1)
		return;
	if(sc->update_id > self->update_id || self->mass == 0)
	{
		d3_t p;
		d4_t q;
		p = d3(_vec3(sc->pos));
		q = d4(_vec4(sc->rot_quat));
		b3SharedMemoryCommandHandle commandHandle;

		commandHandle = b3wCreatePoseCommandInit(sm, self->bodyId);

		b3wCreatePoseCommandSetBasePosition(commandHandle, p.x, p.y, p.z);
		b3wCreatePoseCommandSetBaseOrientation(commandHandle, q.x, q.y, q.z, q.w);

		b3wSubmitClientCommandAndWaitStatus(sm, commandHandle);

		/* commandHandle = b3wInitChangeDynamicsInfo(sm); */
		/* b3wChangeDynamicsInfoSetActivationState(commandHandle, self->bodyId, 1); */
		/* b3wSubmitClientCommandAndWaitStatus(sm, commandHandle); */
	}
}

static
void c_budy_get_position(c_budy_t *self)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	if (self->bodyId == -1)
		return;

	b3SharedMemoryCommandHandle cmd_handle =
		b3wRequestActualStateCommandInit(sm, self->bodyId);
	b3SharedMemoryStatusHandle status_handle =
		b3wSubmitClientCommandAndWaitStatus(sm, cmd_handle);

	const int status_type = b3wGetStatusType(status_handle);
	const double* actualStateQ;
	// const double* jointReactionForces[];

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		puts("getBasePositionAndOrientation failed.");
		return;
	}

	b3wGetStatusActualState(
		status_handle, 0 /* body_unique_id */,
		0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
		0 /*root_local_inertial_frame*/, &actualStateQ,
		0 /* actual_state_q_dot */, 0 /* joint_reaction_forces */);

	c_spatial_t *sc = c_spatial(self);
	c_spatial_lock(sc);
	sc->pos.x = actualStateQ[0];
	sc->pos.y = actualStateQ[1];
	sc->pos.z = actualStateQ[2];

	sc->rot_quat.x = actualStateQ[3];
	sc->rot_quat.y = actualStateQ[4];
	sc->rot_quat.z = actualStateQ[5];
	sc->rot_quat.w = actualStateQ[6];
	sc->update_id++;
	sc->modified = 1;
	self->update_id = sc->update_id;
	c_spatial_unlock(sc);
}

void budy_applyExternalForce(c_budy_t *self, int linkIndex, d3_t force,
                             d3_t position, int flags)
{
	if (self->bodyId == -1)
		return;
	void *sm = c_bullet_sm(c_bullet(&SYS));
	b3SharedMemoryCommandHandle command;

	if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
	{
		puts("flag has to be either WORLD_FRAME or LINK_FRAME");
		return;
	}
	command = b3wApplyExternalForceCommandInit(sm);
	b3wApplyExternalForce(command, self->bodyId, linkIndex, (double*)&force,
			(double*)&position, flags);
	b3wSubmitClientCommandAndWaitStatus(sm, command);
}

int bullet_applyExternalTorque(c_budy_t *self, int linkIndex, d3_t torque, int flags)
{
	if (self->bodyId == -1)
		return 0;
	void *sm = c_bullet_sm(c_bullet(&SYS));

	if (linkIndex < -1)
	{
		puts("Invalid link index, has to be -1 or larger");
		return 0;
	}
	if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
	{
		puts("flag has to be either WORLD_FRAME or LINK_FRAME");
		return 0;
	}
	b3SharedMemoryCommandHandle command =
		b3wApplyExternalForceCommandInit(sm);
	b3wApplyExternalTorque(command, self->bodyId, linkIndex, (double*)&torque,
			flags);
	b3wSubmitClientCommandAndWaitStatus(sm, command);

	return 1;
}


int c_budy_update(c_budy_t *self, float *dt)
{
	c_budy_update_position(self);
	if (self->mass != 0)
		c_budy_get_position(self);
	return CONTINUE;
}

void c_budy_destroy(c_budy_t *self)
{
	void *sm = c_bullet_sm(c_bullet(&SYS));
	if (self->bodyId >= 0 && b3wCanSubmitCommand(sm))
	{
		b3wSubmitClientCommandAndWaitStatus(sm, b3wInitRemoveBodyCommand(sm, self->bodyId));
	}
}

void ct_budy(ct_t *self)
{
	ct_init(self, "budy", sizeof(c_budy_t));
	ct_set_init(self, (init_cb)c_budy_init);
	ct_set_destroy(self, (destroy_cb)c_budy_destroy);
	ct_add_dependency(self, ct_node);
	ct_add_listener(self, WORLD, 0, ref("world_update"), c_budy_update);
}

