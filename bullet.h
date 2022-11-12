#ifndef CANDLE_BULLET_H
#define CANDLE_BULLET_H

#include "../candle/ecs/ecm.h"

typedef struct c_bullet_t
{
	c_t super;
	void *sm;
	int running;
	float time_scale;
} c_bullet_t;

DEF_CASTER(ct_bullet, c_bullet, c_bullet_t)

c_bullet_t *c_bullet_new(void);


void *c_bullet_sm(c_bullet_t *self);

int bullet_createMultiBody(c_bullet_t *self,
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
	int flags /* -1 */);

#include "budy.h"

#endif /* !CANDLE_BULLET_H */
