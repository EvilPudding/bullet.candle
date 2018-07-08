#include "budy.h"

c_budy_t *c_budy_new()
{
	c_budy_t *self = component_new("budy");
	return self;
}


REG()
{
	ct_new("budy", sizeof(c_budy_t), NULL, NULL, 1, ref("spacial"));
}

