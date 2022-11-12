#include "bullet.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../candle/utils/mafs.h"
#include "blw.h"

/* static b3PhysicsClientHandle getPhysicsClient(c_bullet_t *self) */
/* { */
/* 	if (self->sm) */
/* 	{ */
/* 		if (b3wCanSubmitCommand(self->sm)) */
/* 		{ */
/* 			return self->sm; */
/* 		} */
/* 		//broken connection? */
/* 		b3wDisconnectSharedMemory(self->sm); */
/* 	} */
/* 	return 0; */
/* } */

void *c_bullet_sm(c_bullet_t *self)
{
	return self->sm;
}

// Step through one timestep of the simulation
void bullet_stepSimulation(c_bullet_t *self)
{
	b3SharedMemoryStatusHandle statusHandle;
	int type;

	if (b3wCanSubmitCommand(self->sm))
	{
		statusHandle = b3wSubmitClientCommandAndWaitStatus(
			self->sm, b3wInitStepSimulationCommand(self->sm));
		type = b3wGetStatusType(statusHandle);
		if (type != CMD_STEP_FORWARD_SIMULATION_COMPLETED)
			exit(1);
	}
}

int bullet_connectPhysicsServer(c_bullet_t *self)
{
	self->sm = b3wConnectPhysicsDirect();

	if (self->sm)
	{
		if (b3wCanSubmitCommand(self->sm))
		{
			b3SharedMemoryCommandHandle command;
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;

			command = b3wInitSyncBodyInfoCommand(self->sm);
			statusHandle = b3wSubmitClientCommandAndWaitStatus(self->sm, command);
			statusType = b3wGetStatusType(statusHandle);

			if (statusType != CMD_SYNC_BODY_INFO_COMPLETED) 
			{
				printf("Connection terminated, couldn't get body info\n");
				b3wDisconnectSharedMemory(self->sm);
				return -1;
			}

			command = b3wInitSyncUserDataCommand(self->sm);
			statusHandle = b3wSubmitClientCommandAndWaitStatus(self->sm, command);
			statusType = b3wGetStatusType(statusHandle);

			if (statusType != CMD_SYNC_USER_DATA_COMPLETED)
			{
				printf("Connection terminated, couldn't get user data\n");
				b3wDisconnectSharedMemory(self->sm);
				return -1;
			}
		}
		else
		{
			b3wDisconnectSharedMemory(self->sm);
			return -1;
		}
	}
	return 1;
}

void bullet_disconnectPhysicsServer(c_bullet_t *self)
{
	b3wDisconnectSharedMemory(self->sm);
	self->sm = 0;
}

bool_t bullet_isConnected(c_bullet_t *self)
{
	return self->sm != 0 && b3wCanSubmitCommand(self->sm);
}

// Reset the simulation to remove all loaded objects
void bullet_resetSimulation(c_bullet_t *self)
{
	b3wSubmitClientCommandAndWaitStatus(self->sm, b3wInitResetSimulationCommand(self->sm));
}

void bullet_setRealTimeSimulation(c_bullet_t *self, int enableRealTimeSimulation)
{
	b3SharedMemoryCommandHandle command = b3wInitPhysicsParamCommand(self->sm);

	b3wPhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

	b3wSubmitClientCommandAndWaitStatus(self->sm, command);
}

// Set the gravity of the world with (x, y, z) arguments
void bullet_setGravity(c_bullet_t *self, d3_t g)
{
	b3SharedMemoryCommandHandle command;

	command = b3wInitPhysicsParamCommand(self->sm);

	b3wPhysicsParamSetGravity(command, g.x, g.y, g.z);
	// ret = b3wPhysicsParamSetTimeStep(command,  timeStep);
	b3wSubmitClientCommandAndWaitStatus(self->sm, command);

}

void bullet_setTimeStep(c_bullet_t *self, double timeStep)
{
	/* double timeStep = 0.001; */

	b3SharedMemoryCommandHandle command = b3wInitPhysicsParamCommand(self->sm);

	b3wPhysicsParamSetTimeStep(command, timeStep);
	b3wSubmitClientCommandAndWaitStatus(self->sm, command);
}

int bullet_getAPIVersion()
{
	return SHARED_MEMORY_MAGIC_NUMBER;
}

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
	int flags /* -1 */)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int i;
	b3SharedMemoryCommandHandle commandHandle = b3wCreateMultiBodyCommandInit(self->sm);
	b3wCreateMultiBodyBase(commandHandle, baseMass,
			baseCollisionShapeIndex, baseVisualShapeIndex, (double*)&basePosition,
			(double*)&baseOrientation, (double*)&baseInertialFramePosition,
			(double*)&baseInertialFrameOrientation );

	for (i = 0; i < numLinkMasses; i++)
	{
		double linkMass = linkMasses[i];
		int linkCollisionShapeIndex = linkCollisionShapeIndices[i];
		int linkVisualShapeIndex = linkVisualShapeIndices[i];
		d3_t linkPosition;
		d4_t linkOrientation;
		d3_t linkJointAxis;
		d3_t linkInertialFramePosition;
		d4_t linkInertialFrameOrientation;
		int linkParentIndex;
		int linkJointType;

		linkInertialFramePosition = linkInertialFramePositions[i];
		linkInertialFrameOrientation = linkInertialFrameOrientations[i];
		linkPosition = linkPositions[i];
		linkOrientation = linkOrientations[i];
		linkJointAxis = linkJointAxes[i];
		linkParentIndex = linkParentIndices[i];
		linkJointType = linkJointTypes[i];

		b3wCreateMultiBodyLink(commandHandle, 
							linkMass, 
							linkCollisionShapeIndex, 
							linkVisualShapeIndex, 
							(double*)&linkPosition, 
							(double*)&linkOrientation,
							(double*)&linkInertialFramePosition,
							(double*)&linkInertialFrameOrientation,
							linkParentIndex,
							linkJointType,
							(double*)&linkJointAxis
		);

	}

	if (useMaximalCoordinates > 0)
	{
		b3wCreateMultiBodyUseMaximalCoordinates(commandHandle);
	}
	if (flags > 0)
	{
		b3wCreateMultiBodySetFlags(commandHandle, flags);
	}
	statusHandle = b3wSubmitClientCommandAndWaitStatus(self->sm, commandHandle);
	statusType = b3wGetStatusType(statusHandle);
	if (statusType == CMD_CREATE_MULTI_BODY_COMPLETED)
	{
		return b3wGetStatusBodyIndex(statusHandle);
	}

	puts("createMultiBody failed.");
	return -1;
}

void c_bullet_init(c_bullet_t *self)
{
	b3w_init();
	if (bullet_connectPhysicsServer(self) < 0)
		exit(1);
	self->time_scale = 1.0f;
	bullet_setGravity(self, d3(0, -10, 0));
	/* bullet_setRealTimeSimulation(1, 0); */
}

c_bullet_t *c_bullet_new()
{
	c_bullet_t *self = component_new(ct_bullet);
	self->running = 1;
	return self;
}

int c_bullet_update(c_bullet_t *self, float *dt)
{
	if(self->running && bullet_isConnected(self))
	{
		bullet_setTimeStep(self, *dt * self->time_scale);
		bullet_stepSimulation(self);
	}
	return CONTINUE;
}

int c_bullet_menu(c_bullet_t *self, void *ctx)
{
	return CONTINUE;
}

void ct_bullet(ct_t *self)
{
	ct_init(self, "bullet", sizeof(c_bullet_t));
	ct_set_init(self, (init_cb)c_bullet_init);

	ct_add_listener(self, WORLD, 0, ref("world_update"), c_bullet_update);

	ct_add_listener(self, WORLD, 0, ref("component_menu"), c_bullet_menu);
}

