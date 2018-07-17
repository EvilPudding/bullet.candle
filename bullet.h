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


void *c_bullet_sm(c_bullet_t *self);

int bullet_createCollisionShape(int shapeType, double radius,
		d3_t halfExtents, double height, const char *fileName,
		d3_t meshScale, d3_t planeNormal, int flags,
		d3_t collisionFramePosition, d4_t collisionFrameOrientation,
		int physicsClientId);

int bullet_createMultiBody(
	double baseMass,
	int baseCollisionShapeIndex,
	int baseVisualShapeIndex,

	d3_t basePosition,
	d4_t baseOrientation,
	d3_t baseInertialFramePosition,
	d4_t baseInertialFrameOrientation,

	int numLinkMasses,
	double *linkMasses,
	int *linkCollisionShapeIndices,
	int *linkVisualShapeIndices,
	d3_t *linkPositions,
	d4_t *linkOrientations,
	d3_t *linkInertialFramePositions,
	d4_t *linkInertialFrameOrientations,

	int *linkParentIndices,
	int *linkJointTypes,
	d3_t *linkJointAxes,
	int useMaximalCoordinates,
	int flags /* -1 */,
	int physicsClientId);

int bullet_getBasePositionAndOrientation(int bodyUniqueId, d3_t
		*basePosition, d4_t *baseOrientation, int physicsClientId);

void bullet_resetBasePositionAndOrientation(int bodyUniqueId, d3_t pos, d4_t
		orn, int physicsClientId);

#include "budy.h"

#endif /* !PHYSICS_H */
