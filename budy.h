#ifndef BUDY_H
#define BUDY_H

#include <ecm.h>

typedef struct
{
	c_t super; /* extends c_t */

	int bodyId;
	int shapeId;
} c_budy_t;

DEF_CASTER("budy", c_budy, c_budy_t)

c_budy_t *c_budy_new();

#endif /* !BUDY_H */
