#include "bullet.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <utils/mafs.h>
#include <PhysicsClientC_API.h>
#include <PhysicsDirectC_API.h>
#include <SharedMemoryInProcessPhysicsC_API.h>

#define MAX_PHYSICS_CLIENTS 1024
static b3PhysicsClientHandle sPhysicsClients1[MAX_PHYSICS_CLIENTS] = {0};
static int sPhysicsClientsGUI[MAX_PHYSICS_CLIENTS] = {0};
static int sNumPhysicsClients = 0;

struct point_pair
{
	d3_t a, b;
};

static b3PhysicsClientHandle getPhysicsClient(int physicsClientId)
{
	if ((physicsClientId < 0) || (physicsClientId >= MAX_PHYSICS_CLIENTS) || (0 == sPhysicsClients1[physicsClientId]))
	{
		return 0;
	}
	b3PhysicsClientHandle sm = sPhysicsClients1[physicsClientId];
	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			return sm;
		}
		//broken connection?
		b3DisconnectSharedMemory(sm);
		sPhysicsClients1[physicsClientId] = 0;
		sPhysicsClientsGUI[physicsClientId] = 0;

		sNumPhysicsClients--;
	}
	return 0;
}

void *c_bullet_sm(c_bullet_t *self)
{
	return (void*)getPhysicsClient(0);
}


// Step through one timestep of the simulation
void bullet_stepSimulation(int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	if (b3CanSubmitCommand(sm))
	{
		statusHandle = b3SubmitClientCommandAndWaitStatus(
			sm, b3InitStepSimulationCommand(sm));
		statusType = b3GetStatusType(statusHandle);
	}
}

int bullet_connectPhysicsServer(int method, int key, const char *hostName, int port)
{
	int freeIndex = -1;
	int i;

	b3PhysicsClientHandle sm = 0;

	if (sNumPhysicsClients >= MAX_PHYSICS_CLIENTS)
	{
		puts("Exceeding maximum number of physics connections.");
		return 0;
	}

	/* int key = SHARED_MEMORY_KEY; */
	int udpPort = 1234;
	int tcpPort = 6667;

	if (port>=0)
	{
		udpPort = port;
		tcpPort = port;
	}

	switch (method)
	{
		case eCONNECT_DIRECT:
		{
			sm = b3ConnectPhysicsDirect();
			break;
		}
		case eCONNECT_SHARED_MEMORY:
		{
			sm = b3ConnectSharedMemory(key);
			break;
		}
		case eCONNECT_UDP:
		{
#ifdef BT_ENABLE_ENET

			sm = b3ConnectPhysicsUDP(hostName, udpPort);
#else
			puts("UDP is not enabled in this bullet build");
			return 0;
#endif  //BT_ENABLE_ENET

			break;
		}
		case eCONNECT_TCP:
		{
#ifdef BT_ENABLE_CLSOCKET

			sm = b3ConnectPhysicsTCP(hostName, tcpPort);
#else
			puts("TCP is not enabled in this bullet build");
			return 0;
#endif  //BT_ENABLE_CLSOCKET

			break;
		}

		default:
		{
			puts("connectPhysicsServer unexpected argument");
			return 0;
		}
	};

	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			for (i = 0; i < MAX_PHYSICS_CLIENTS; i++)
			{
				if (sPhysicsClients1[i] == 0)
				{
					freeIndex = i;
					break;
				}
			}

			if (freeIndex >= 0)
			{
				b3SharedMemoryCommandHandle command;
				b3SharedMemoryStatusHandle statusHandle;
				int statusType;

				sPhysicsClients1[freeIndex] = sm;
				sPhysicsClientsGUI[freeIndex] = method;
				sNumPhysicsClients++;

				command = b3InitSyncBodyInfoCommand(sm);
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
				statusType = b3GetStatusType(statusHandle);

				if (statusType != CMD_SYNC_BODY_INFO_COMPLETED) 
				{
					printf("Connection terminated, couldn't get body info\n");
					b3DisconnectSharedMemory(sm);
							sm = 0;
					sPhysicsClients1[freeIndex] = 0;
								sPhysicsClientsGUI[freeIndex] = 0;
								sNumPhysicsClients++;
					return -1;
				}

				command = b3InitSyncUserDataCommand(sm);
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
				statusType = b3GetStatusType(statusHandle);

				if (statusType != CMD_SYNC_USER_DATA_COMPLETED)
				{
					printf("Connection terminated, couldn't get user data\n");
					b3DisconnectSharedMemory(sm);
							sm = 0;
					sPhysicsClients1[freeIndex] = 0;
								sPhysicsClientsGUI[freeIndex] = 0;
								sNumPhysicsClients++;
					return -1;
				}
			}
		} else
		{
			b3DisconnectSharedMemory(sm);
		}
	}
	return freeIndex;
}

void bullet_disconnectPhysicsServer(int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	{
		b3DisconnectSharedMemory(sm);
		sm = 0;
	}

	sPhysicsClients1[physicsClientId] = 0;
	sPhysicsClientsGUI[physicsClientId] = 0;
	sNumPhysicsClients--;
}

///to avoid memory leaks, disconnect all physics servers explicitly
void b3bulletExitFunc(void)
{
	int i;
	for (i=0;i<MAX_PHYSICS_CLIENTS;i++)
	{
		if (sPhysicsClients1[i])
		{
			b3DisconnectSharedMemory(sPhysicsClients1[i]);
			sPhysicsClients1[i] = 0;
			sNumPhysicsClients--;
		}
	}
}

int bullet_isConnected(int physicsClientId)
{
	int isConnected = 0;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	if (sm != 0)
	{
		if (b3CanSubmitCommand(sm))
		{
			isConnected = 1;
		}
	}
	return isConnected;
}



int bullet_getConnectionMethod(int physicsClientId)
{
	int method = 0;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	if (sm != 0)
	{
		if (b3CanSubmitCommand(sm))
		{
			method = sPhysicsClientsGUI[physicsClientId];
		}
	}
	return method;
}


void bullet_syncBodyInfo(int physicsClientId)
{
    
    b3SharedMemoryCommandHandle command;
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;

    b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
    
    command = b3InitSyncBodyInfoCommand(sm);
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    statusType = b3GetStatusType(statusHandle);
    
    if (statusType != CMD_SYNC_BODY_INFO_COMPLETED)
    {
        puts("Error in syncBodyzInfo command.");
    }
}

void bullet_syncUserData(int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3InitSyncUserDataCommand(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_SYNC_USER_DATA_COMPLETED)
	{
		puts("Error in syncUserInfo command.");
	}

}

int bullet_addUserData(int bodyUniqueId, int linkIndex, const char *key, const char *value, int physicsClientId)
{
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int userDataId;
	int valueLen=-1;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	valueLen = strlen(value)+1;
	command = b3InitAddUserDataCommand(sm, bodyUniqueId, linkIndex, key, USER_DATA_VALUE_TYPE_STRING, valueLen, value);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_ADD_USER_DATA_COMPLETED)
	{
		puts("Error in addUserData command.");
		return -1;
	}

	userDataId = b3GetUserDataIdFromStatus(statusHandle);
	return userDataId;
}

void bullet_removeUserData(int bodyUniqueId, int linkIndex, int userDataId, int physicsClientId)
{

	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3InitRemoveUserDataCommand(sm, bodyUniqueId, linkIndex, userDataId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_REMOVE_USER_DATA_COMPLETED)
	{
		puts("Error in removeUserData command.");
	}
}


int bullet_getUserDataId(int bodyUniqueId, int linkIndex, const char *key, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetUserDataId(sm, bodyUniqueId, linkIndex, key);
}

int bullet_getUserData(int bodyUniqueId, int linkIndex, int userDataId, int physicsClientId, struct b3UserDataValue *value)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);


	if (!b3GetUserData(sm, bodyUniqueId, linkIndex, userDataId, value)) {
		return 0;
	}
	if (value->m_type != USER_DATA_VALUE_TYPE_STRING) 
	{
		puts("User data value has unknown type");
		return 0;
	}

	/* return strdup((const char *)value.m_data1); */
	return 1;
}

int bullet_getNumUserData(int bodyUniqueId, int linkIndex, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetNumUserData(sm, bodyUniqueId, linkIndex);
}

int bullet_getUserDataInfo(int bodyUniqueId, int linkIndex, int userDataIndex, int physicsClientId, const char **key, int *userDataId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	/* const char* key = 0; */
	/* int userDataId = -1; */

	b3GetUserDataInfo(sm, bodyUniqueId, linkIndex, userDataIndex, key, userDataId);
	if (*key == 0 || *userDataId == -1) {
		puts("Could not get user data info.");
		return 0;
	}

	/* return {userDataId, key}; */
	return 1;
}

void bullet_saveWorld(const char *worldFileName, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3SaveWorldCommandInit(sm, worldFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_SAVE_WORLD_COMPLETED)
	{
		puts("saveWorld command execution failed.");
	}
}

int *bullet_loadBullet(const char *bulletFileName, int physicsClientId, int *num_bodies)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	int i, numBodies;
	int bodyIndicesOut[MAX_SDF_BODIES];

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3LoadBulletCommandInit(sm, bulletFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_LOADING_COMPLETED)
	{
		puts("Couldn't load .bullet file.");
		return NULL;
	}

	numBodies =
		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
	if (numBodies > MAX_SDF_BODIES)
	{
		puts("loadBullet exceeds body capacity");
		return NULL;
	}
	int *indices = malloc(sizeof(int) * numBodies);

	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES)
	{
		for (i = 0; i < numBodies; i++)
		{
			indices[i] = bodyIndicesOut[i];
		}
	}
	if(numBodies) *num_bodies = numBodies;
	return indices;
}

void bullet_saveBullet(const char *bulletFileName, int physicsClientId)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3SaveBulletCommandInit(sm, bulletFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_SAVING_COMPLETED)
	{
		puts("Couldn't save .bullet file.");
	}
}


void bullet_restoreState(int stateId, const char *filename, int physicsClientId)
{
	const char* fileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3LoadStateCommandInit(sm);
	if (stateId >= 0)
	{
		b3LoadStateSetStateId(command, stateId);
	}
	if (fileName)
	{
		b3LoadStateSetFileName(command, fileName);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_RESTORE_STATE_COMPLETED)
	{
		puts("Couldn't restore state.");
	}
}

int bullet_saveState(int physicsClientId)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3SaveStateCommandInit(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_SAVE_STATE_COMPLETED)
	{
		puts("Couldn't save state");
		return -1;
	}

	return b3GetStatusGetStateId(statusHandle);
}

/* LIST bullet_loadMJCF(const char *mjcfFileName, int flags, int physicsClientId) */
/* { */
/* 	b3SharedMemoryStatusHandle statusHandle; */
/* 	int statusType; */
/* 	b3SharedMemoryCommandHandle command; */
/* 	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId); */
/* 	int numBodies = 0; */
/* 	int i; */
/* 	int bodyIndicesOut[MAX_SDF_BODIES]; */
/* 	LIST* pylist = 0; */
/* 	/1* int flags = -1; *1/ */

/* 	command = b3LoadMJCFCommandInit(sm, mjcfFileName); */
/* 	if (flags >= 0) */
/* 	{ */
/* 		b3LoadMJCFCommandSetFlags(command,flags); */
/* 	} */
/* 	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command); */
/* 	statusType = b3GetStatusType(statusHandle); */
/* 	if (statusType != CMD_MJCF_LOADING_COMPLETED) */
/* 	{ */
/* 		puts("Couldn't load .mjcf file."); */
/* 		return NULL; */
/* 	} */

/* 	numBodies = */
/* 		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES); */
/* 	if (numBodies > MAX_SDF_BODIES) */
/* 	{ */
/* 		char str[1024]; */
/* 		sprintf(str,"SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES); */
/* 		puts(str); */
/* 		return NULL; */
/* 	} */

/* 	pylist = PyTuple_New(numBodies); */

/* 	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES) */
/* 	{ */
/* 		for (i = 0; i < numBodies; i++) */
/* 		{ */
/* 			PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i])); */
/* 		} */
/* 	} */
/* 	return pylist; */
/* } */

/* TODO(pudds) */
/* void bullet_changeDynamicsInfo(PyObject* self, PyObject* args, PyObject* keywds) */
/* { */
/* 	int bodyUniqueId = -1; */
/* 	int linkIndex = -2; */
/* 	double mass = -1; */
/* 	double lateralFriction = -1; */
/* 	double spinningFriction= -1; */
/* 	double rollingFriction = -1; */
/* 	double restitution = -1; */
/* 	double linearDamping = -1; */
/* 	double angularDamping = -1; */
/* 	double contactStiffness = -1; */
/* 	double contactDamping = -1; */
/* 	double ccdSweptSphereRadius=-1; */
/* 	int frictionAnchor = -1; */
/* 	double contactProcessingThreshold = -1; */
/* 	int activationState = -1; */

/* 	PyObject* localInertiaDiagonalObj=0; */

/* 	b3PhysicsClientHandle sm = 0; */
	
/* 	int physicsClientId = 0; */
/* 	static char* kwlist[] = {"bodyUniqueId", "linkIndex", "mass", */
/* 		"lateralFriction", "spinningFriction", "rollingFriction","restitution", */
/* 		"linearDamping", "angularDamping", "contactStiffness", */
/* 		"contactDamping", "frictionAnchor", "localInertiaDiagonal", */
/* 		"ccdSweptSphereRadius", "contactProcessingThreshold", */
/* 		"activationState", "physicsClientId", NULL}; */
/* 	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|dddddddddiOddii", */
/* 				kwlist, &bodyUniqueId, &linkIndex,&mass, &lateralFriction, */
/* 				&spinningFriction, &rollingFriction, */
/* 				&restitution,&linearDamping, &angularDamping, */
/* 				&contactStiffness, &contactDamping, &frictionAnchor, */
/* 				&localInertiaDiagonalObj, &ccdSweptSphereRadius, */
/* 				&contactProcessingThreshold,&activationState, */
/* 				&physicsClientId)) */
/* 	{ */
/* 		return; */
/* 	} */
	
/* 	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId); */
	
/* 	if ((contactStiffness>=0 && contactDamping <0)||(contactStiffness<0 && contactDamping >=0)) */
/* 	{ */
/* 		puts("Both contactStiffness and contactDamping needs to be set together."); */
/* 		return; */
/* 	} */

/* 	{ */
/* 		b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(sm); */
/* 		b3SharedMemoryStatusHandle statusHandle; */
		
/* 		if (mass >= 0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetMass(command, bodyUniqueId, linkIndex, mass); */
/* 		} */
/* 		if (localInertiaDiagonalObj) */
/* 		{ */
/* 			double localInertiaDiagonal[3]; */
/* 			bullet_internalSetVectord(localInertiaDiagonalObj, localInertiaDiagonal); */
/* 			b3ChangeDynamicsInfoSetLocalInertiaDiagonal(command, bodyUniqueId, linkIndex, localInertiaDiagonal); */
/* 		} */
/* 		if (lateralFriction >= 0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetLateralFriction(command, bodyUniqueId, linkIndex, lateralFriction); */
/* 		} */
/* 		if (spinningFriction>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetSpinningFriction(command, bodyUniqueId, linkIndex,spinningFriction); */
/* 		} */
/* 		if (rollingFriction>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetRollingFriction(command, bodyUniqueId, linkIndex,rollingFriction); */
/* 		} */

/* 		if (linearDamping>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetLinearDamping(command,bodyUniqueId, linearDamping); */
/* 		} */
/* 		if (angularDamping>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetAngularDamping(command,bodyUniqueId,angularDamping); */
/* 		} */

/* 		if (restitution>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetRestitution(command, bodyUniqueId, linkIndex, restitution); */
/* 		} */
/* 		if (contactStiffness>=0 && contactDamping >=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetContactStiffnessAndDamping(command,bodyUniqueId,linkIndex,contactStiffness, contactDamping); */
/* 		} */
/* 		if (frictionAnchor>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetFrictionAnchor(command,bodyUniqueId,linkIndex, frictionAnchor); */
/* 		} */
/* 		if (ccdSweptSphereRadius>=0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetCcdSweptSphereRadius(command,bodyUniqueId,linkIndex, ccdSweptSphereRadius); */
/* 		} */
/* 		if (activationState >= 0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetActivationState(command, bodyUniqueId, activationState); */
/* 		} */
/* 		if (contactProcessingThreshold >= 0) */
/* 		{ */
/* 			b3ChangeDynamicsInfoSetContactProcessingThreshold(command, bodyUniqueId, linkIndex, contactProcessingThreshold); */
/* 		} */
/* 		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command); */
/* 	} */
/* } */

int bullet_getDynamicsInfo(int bodyUniqueId, int linkIndex, int physicsClientId, struct b3DynamicsInfo *info)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	{
		int status_type = 0;
		b3SharedMemoryCommandHandle cmd_handle;
		b3SharedMemoryStatusHandle status_handle;

		if (bodyUniqueId < 0)
		{
			puts("getDynamicsInfo failed; invalid bodyUniqueId");
			return 0;
		}
		if (linkIndex < -1)
		{
			puts("getDynamicsInfo failed; invalid linkIndex");
			return 0;
		}
		cmd_handle = b3GetDynamicsInfoCommandInit(sm, bodyUniqueId, linkIndex);
		status_handle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
		status_type = b3GetStatusType(status_handle);
		if (status_type != CMD_GET_DYNAMICS_INFO_COMPLETED)
		{
			puts("getDynamicsInfo failed; invalid return status");
			return 0;
		}

		if (b3GetDynamicsInfo(status_handle, info))
		{
			return 1;
		}
	}
	puts("Couldn't get dynamics info");
	return 0;
}

int bullet_getPhysicsEngineParameters(int physicsClientId, struct b3PhysicsSimulationParameters *params)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	b3SharedMemoryCommandHandle command = b3InitRequestPhysicsParamCommand(sm);
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType!=CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED)
	{
		puts("Couldn't get physics simulation parameters.");
		return 0;
	}
	b3GetStatusPhysicsSimulationParameters(statusHandle, params);

	return 1;
}

/* TODO(pudds) */
/* static PyObject* bullet_setPhysicsEngineParameter(PyObject* self, PyObject* args, PyObject* keywds) */
/* { */
/* 	double fixedTimeStep = -1; */
/* 	int numSolverIterations = -1; */
/* 	int useSplitImpulse = -1; */
/* 	double splitImpulsePenetrationThreshold = -1; */
/* 	int numSubSteps = -1; */
/* 	int collisionFilterMode = -1; */
/* 	double contactBreakingThreshold = -1; */
/* 	int maxNumCmdPer1ms = -2; */
/* 	int enableFileCaching = -1; */
/* 	double restitutionVelocityThreshold=-1; */
/* 	double erp = -1; */
/* 	double contactERP = -1; */
/* 	double frictionERP = -1; */
/* 	double allowedCcdPenetration = -1; */

/* 	int enableConeFriction = -1; */
/* 	b3PhysicsClientHandle sm = 0; */
/* 	int deterministicOverlappingPairs = -1; */
/* 	int jointFeedbackMode =-1; */
/* 	double solverResidualThreshold = -1; */
/* 	double contactSlop = -1; */
/* 	int enableSAT = -1; */

/* 	int physicsClientId = 0; */
/* 	static char* kwlist[] = {"fixedTimeStep", "numSolverIterations", "useSplitImpulse", "splitImpulsePenetrationThreshold", "numSubSteps", "collisionFilterMode", "contactBreakingThreshold", "maxNumCmdPer1ms", "enableFileCaching","restitutionVelocityThreshold", "erp", "contactERP", "frictionERP", "enableConeFriction", "deterministicOverlappingPairs", "allowedCcdPenetration", "jointFeedbackMode", "solverResidualThreshold", "contactSlop", "enableSAT", "physicsClientId", NULL}; */

/* 	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|diidiidiiddddiididdii", kwlist, &fixedTimeStep, &numSolverIterations, &useSplitImpulse, &splitImpulsePenetrationThreshold, &numSubSteps, */
/* 									 &collisionFilterMode, &contactBreakingThreshold, &maxNumCmdPer1ms, &enableFileCaching, &restitutionVelocityThreshold, &erp, &contactERP, &frictionERP, &enableConeFriction, &deterministicOverlappingPairs, &allowedCcdPenetration, &jointFeedbackMode, &solverResidualThreshold, &contactSlop, &enableSAT, &physicsClientId)) */
/* 	{ */
/* 		return NULL; */
/* 	} */

/* 	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId); */

/* 	{ */
/* 		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm); */
/* 		b3SharedMemoryStatusHandle statusHandle; */

/* 		if (numSolverIterations >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetNumSolverIterations(command, numSolverIterations); */
/* 		} */

/* 		if (solverResidualThreshold>=0) */
/* 		{ */
/* 			b3PhysicsParamSetSolverResidualThreshold(command, solverResidualThreshold); */
/* 		} */

/* 		if (collisionFilterMode >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetCollisionFilterMode(command, collisionFilterMode); */
/* 		} */
/* 		if (numSubSteps >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetNumSubSteps(command, numSubSteps); */
/* 		} */
/* 		if (fixedTimeStep >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetTimeStep(command, fixedTimeStep); */
/* 		} */
/* 		if (useSplitImpulse >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetUseSplitImpulse(command, useSplitImpulse); */
/* 		} */
/* 		if (splitImpulsePenetrationThreshold >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetSplitImpulsePenetrationThreshold(command, splitImpulsePenetrationThreshold); */
/* 		} */
/* 		if (contactBreakingThreshold >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetContactBreakingThreshold(command, contactBreakingThreshold); */
/* 		} */
/* 		if (contactSlop >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetContactSlop(command, contactSlop); */
/* 		} */


/* 		//-1 is disables the maxNumCmdPer1ms feature, allow it */
/* 		if (maxNumCmdPer1ms >= -1) */
/* 		{ */
/* 			b3PhysicsParamSetMaxNumCommandsPer1ms(command, maxNumCmdPer1ms); */
/* 		} */

/* 		if (restitutionVelocityThreshold>=0) */
/* 		{ */
/* 			b3PhysicsParamSetRestitutionVelocityThreshold(command, restitutionVelocityThreshold); */
/* 		} */
/* 		if (enableFileCaching>=0) */
/* 		{ */
/* 			b3PhysicsParamSetEnableFileCaching(command, enableFileCaching); */
/* 		} */

/* 		if (erp>=0) */
/* 		{ */
/* 			b3PhysicsParamSetDefaultNonContactERP(command,erp); */
/* 		} */
/* 		if (contactERP>=0) */
/* 		{ */
/* 			b3PhysicsParamSetDefaultContactERP(command,contactERP); */
/* 		} */
/* 		if (frictionERP >=0) */
/* 		{ */
/* 			b3PhysicsParamSetDefaultFrictionERP(command,frictionERP); */
/* 		} */
/* 		if (enableConeFriction >= 0) */
/* 		{ */
/* 			b3PhysicsParamSetEnableConeFriction(command, enableConeFriction); */
/* 		} */
/* 		if (deterministicOverlappingPairs>=0) */
/* 		{ */
/* 			b3PhysicsParameterSetDeterministicOverlappingPairs(command,deterministicOverlappingPairs); */
/* 		} */

/* 		if (allowedCcdPenetration>=0) */
/* 		{ */
/* 			b3PhysicsParameterSetAllowedCcdPenetration(command,allowedCcdPenetration); */
/* 		} */
/* 		if (jointFeedbackMode>=0) */
/* 		{ */
/* 			b3PhysicsParameterSetJointFeedbackMode(command,jointFeedbackMode); */
/* 		} */

/* 		if (enableSAT>=0) */
/* 		{ */
/* 			b3PhysicsParameterSetEnableSAT(command, enableSAT); */
/* 		} */
/* 		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command); */
/* 	} */


/* 	Py_INCREF(Py_None); */
/* 	return Py_None; */
/* } */

/* static PyObject* bullet_loadSDF(PyObject* self, PyObject* args, PyObject* keywds) */
/* { */
/* 	const char* sdfFileName = ""; */
/* 	int numBodies = 0; */
/* 	int i; */
/* 	int bodyIndicesOut[MAX_SDF_BODIES]; */
/* 	int useMaximalCoordinates = -1; */
/* 	PyObject* pylist = 0; */
/* 	b3SharedMemoryStatusHandle statusHandle; */
/* 	int statusType; */
/* 	b3SharedMemoryCommandHandle commandHandle; */
/* 	b3PhysicsClientHandle sm = 0; */
/* 	double globalScaling = -1; */

/* 	int physicsClientId = 0; */
/* 	static char* kwlist[] = {"sdfFileName", "useMaximalCoordinates", "globalScaling", "physicsClientId", NULL}; */
/* 	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|idi", kwlist, &sdfFileName, &useMaximalCoordinates, &globalScaling, &physicsClientId)) */
/* 	{ */
/* 		return NULL; */
/* 	} */
/* 	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId); */

/* 	commandHandle = b3LoadSdfCommandInit(sm, sdfFileName); */
/* 	if (useMaximalCoordinates>0) */
/* 	{ */
/* 		b3LoadSdfCommandSetUseMultiBody(commandHandle,0); */
/* 	} */
/* 	if (globalScaling > 0) */
/* 	{ */
/* 		b3LoadSdfCommandSetUseGlobalScaling(commandHandle,globalScaling); */
/* 	} */
/* 	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle); */
/* 	statusType = b3GetStatusType(statusHandle); */
/* 	if (statusType != CMD_SDF_LOADING_COMPLETED) */
/* 	{ */
/* 		puts("Cannot load SDF file."); */
/* 		return NULL; */
/* 	} */

/* 	numBodies = */
/* 		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES); */
/* 	if (numBodies > MAX_SDF_BODIES) */
/* 	{ */
/* 		char str[1024]; */
/*                 sprintf(str,"SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES); */
/* 		puts(str); */
/* 		return NULL; */
/* 	} */

/* 	pylist = PyTuple_New(numBodies); */

/* 	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES) */
/* 	{ */
/* 		for (i = 0; i < numBodies; i++) */
/* 		{ */
/* 			PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i])); */
/* 		} */
/* 	} */
/* 	return pylist; */
/* } */

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
// Load a softbody from an obj file
long bullet_loadSoftBody(const char *fileName, double scale, double mass, double collisionMargin, int physicsClientId)
{
	int bodyUniqueId= -1;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if (strlen(fileName))
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		b3SharedMemoryCommandHandle command = b3LoadSoftBodyCommandInit(sm, fileName);

		if (scale>0)
		{
			b3LoadSoftBodySetScale(command,scale);
		}
		if (mass>0)
		{
			b3LoadSoftBodySetMass(command,mass);
		}
		if (collisionMargin>0)
		{
			b3LoadSoftBodySetCollisionMargin(command,collisionMargin);
		}
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_LOAD_SOFT_BODY_COMPLETED)
		{
			puts("Cannot load soft body.");
			return -1;
		}
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	return bodyUniqueId;
}
#endif

// Reset the simulation to remove all loaded objects
void bullet_resetSimulation(int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryStatusHandle statusHandle;
	statusHandle = b3SubmitClientCommandAndWaitStatus(
			sm, b3InitResetSimulationCommand(sm));
}

void bullet_setJointMotorControlArray(int bodyUniqueId,
		int numControlledDofs,
		int *jointIndices, int controlMode, double *targetPositions, double *targetVelocities,
		double *forces, double *positionGains, double *velocityGains, int physicsClientId)
{

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int numJoints;
	int i;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3JointInfo info;

	numJoints = b3GetNumJoints(sm, bodyUniqueId);

	if ((controlMode != CONTROL_MODE_VELOCITY) &&
			(controlMode != CONTROL_MODE_TORQUE) &&
			(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD))
	{
		puts("Illegral control mode.");
		return;
	}

	if (jointIndices==0)
	{
		puts("expected a sequence of joint indices");
		return;
	}

	for (i = 0; i < numControlledDofs; i++)
	{
		int jointIndex = jointIndices[i];
		if ((jointIndex >= numJoints) || (jointIndex < 0))
		{
			puts("Joint index out-of-range.");
			return;
		}
	}

	commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

	for (i = 0; i < numControlledDofs; i++)
	{
		double targetVelocity = 0.0;
		double targetPosition = 0.0;
		double force = 100000.0;
		double potitionGain = 0.1;
		double velocityGain = 1.0;
		int jointIndex;

		if (targetVelocities)
		{
			targetVelocity = targetVelocities[i];
		}

		if (targetPositions)
		{
			targetPosition = targetPositions[i];
		}


		if (forces)
		{
			force = forces[i];
		}

		if (positionGains)
		{
			potitionGain = positionGains[i];
		}

		if (velocityGains)
		{
			velocityGain = velocityGains[i];
		}

		jointIndex = jointIndices[i];
		b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

		switch (controlMode)
		{
			case CONTROL_MODE_VELOCITY:
				{
					b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
							targetVelocity);
					b3JointControlSetKd(commandHandle, info.m_uIndex, velocityGain);
					b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
					break;
				}

			case CONTROL_MODE_TORQUE:
				{
					b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
							force);
					break;
				}

			case CONTROL_MODE_POSITION_VELOCITY_PD:
				{
					b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
							targetPosition);
					b3JointControlSetKp(commandHandle, info.m_uIndex, potitionGain);
					b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
							targetVelocity);
					b3JointControlSetKd(commandHandle, info.m_uIndex, velocityGain);
					b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
					break;
				}
			default:
				{
				}
		};
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
}

int bullet_setJointMotorControl(int bodyUniqueId, int jointIndex,
		int controlMode, double targetPosition, double targetVelocity, double force,
		double positionGain, double velocityGain, double maxVelocity, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int numJoints;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3JointInfo info;

	numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if ((jointIndex >= numJoints) || (jointIndex < 0))
	{
		puts("Joint index out-of-range.");
		return 0;
	}

	if ((controlMode != CONTROL_MODE_VELOCITY) &&
		(controlMode != CONTROL_MODE_TORQUE) &&
		(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD) &&
		(controlMode != CONTROL_MODE_PD))
	{
		puts("Illegral control mode.");
		return 0;
	}

	commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

	b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

	switch (controlMode)
	{
		case CONTROL_MODE_VELOCITY:
		{
			b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
											 targetVelocity);
			b3JointControlSetKd(commandHandle, info.m_uIndex, velocityGain);
			b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
			break;
		}

		case CONTROL_MODE_TORQUE:
		{
			b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
												force);
			break;
		}

		case CONTROL_MODE_POSITION_VELOCITY_PD:
		case CONTROL_MODE_PD:
		{
			if (maxVelocity>0)
			{
				b3JointControlSetMaximumVelocity(commandHandle, info.m_uIndex, maxVelocity);
			}
			b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
											 targetPosition);
			b3JointControlSetKp(commandHandle, info.m_uIndex, positionGain);
			b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
											 targetVelocity);
			b3JointControlSetKd(commandHandle, info.m_uIndex, velocityGain);
			b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
			break;
		}
		default:
		{
		}
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	return 1;
}

void bullet_setRealTimeSimulation(int enableRealTimeSimulation, int physicsClientId)
{
	int ret;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
	b3SharedMemoryStatusHandle statusHandle;

	ret = b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	// ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
}

void bullet_setInternalSimFlags(int flags, int physicsClientId)
{
	int ret;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
	b3SharedMemoryStatusHandle statusHandle;

	ret = b3PhysicsParamSetInternalSimFlags(command, flags);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	// ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);

}

// Set the gravity of the world with (x, y, z) arguments
void bullet_setGravity(d3_t g, int physicsClientId)
{
	int ret;
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3InitPhysicsParamCommand(sm);

	ret = b3PhysicsParamSetGravity(command, g.x, g.y, g.z);
	// ret = b3PhysicsParamSetTimeStep(command,  timeStep);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	// ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);

}

void bullet_setTimeStep(double timeStep, int physicsClientId)
{
	/* double timeStep = 0.001; */
	int ret;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
	b3SharedMemoryStatusHandle statusHandle;

	ret = b3PhysicsParamSetTimeStep(command, timeStep);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
}
void bullet_setDefaultContactERP(double defaultContactERP, int physicsClientId)
{
	/* double defaultContactERP = 0.005; */
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	int ret;

	b3SharedMemoryStatusHandle statusHandle;

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
	ret = b3PhysicsParamSetDefaultContactERP(command, defaultContactERP);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
}

int bullet_getBaseVelocity(int bodyUniqueId, d3_t *baseLinearVelocity, d3_t *baseAngularVelocity, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	baseLinearVelocity->x = 0.;
	baseLinearVelocity->y = 0.;
	baseLinearVelocity->z = 0.;

	baseAngularVelocity->x = 0.;
	baseAngularVelocity->y = 0.;
	baseAngularVelocity->z = 0.;

	b3SharedMemoryCommandHandle cmd_handle =
		b3RequestActualStateCommandInit(sm, bodyUniqueId);
	b3SharedMemoryStatusHandle status_handle =
		b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

	const int status_type = b3GetStatusType(status_handle);
	const double* actualStateQdot;
	// const double* jointReactionForces[];

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		puts("getBaseVelocity failed.");
		return 0;
	}

	b3GetStatusActualState(
		status_handle, 0 /* body_unique_id */,
		0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
		0 /*root_local_inertial_frame*/, 0,
		&actualStateQdot, 0 /* joint_reaction_forces */);

	// printf("joint reaction forces=");
	// for (i=0; i < (sizeof(jointReactionForces)/sizeof(double)); i++) {
	//   printf("%f ", jointReactionForces[i]);
	// }
	// now, position x,y,z = actualStateQ[0],actualStateQ[1],actualStateQ[2]
	// and orientation x,y,z,w =
	// actualStateQ[3],actualStateQ[4],actualStateQ[5],actualStateQ[6]
	baseLinearVelocity->x = actualStateQdot[0];
	baseLinearVelocity->y = actualStateQdot[1];
	baseLinearVelocity->z = actualStateQdot[2];

	baseAngularVelocity->x = actualStateQdot[3];
	baseAngularVelocity->y = actualStateQdot[4];
	baseAngularVelocity->z = actualStateQdot[5];
	return 1;
}

// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
// values for the base link of your object
// Object is retrieved based on body index, which is the order
// the object was loaded into the simulation (0-based)
int bullet_getBasePositionAndOrientation(int bodyUniqueId, d3_t *basePosition,
		d4_t *baseOrientation, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	basePosition->x = 0.;
	basePosition->y = 0.;
	basePosition->z = 0.;

	baseOrientation->x = 0.;
	baseOrientation->y = 0.;
	baseOrientation->z = 0.;
	baseOrientation->w = 1.;

	b3SharedMemoryCommandHandle cmd_handle =
		b3RequestActualStateCommandInit(sm, bodyUniqueId);
	b3SharedMemoryStatusHandle status_handle =
		b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

	const int status_type = b3GetStatusType(status_handle);
	const double* actualStateQ;
	// const double* jointReactionForces[];

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		puts("getBasePositionAndOrientation failed.");
		return 0;
	}

	b3GetStatusActualState(
		status_handle, 0 /* body_unique_id */,
		0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
		0 /*root_local_inertial_frame*/, &actualStateQ,
		0 /* actual_state_q_dot */, 0 /* joint_reaction_forces */);

	basePosition->x = actualStateQ[0];
	basePosition->y = actualStateQ[1];
	basePosition->z = actualStateQ[2];

	baseOrientation->x = actualStateQ[3];
	baseOrientation->y = actualStateQ[4];
	baseOrientation->z = actualStateQ[5];
	baseOrientation->w = actualStateQ[6];
	return 1;
}

int bullet_getAABB(int bodyUniqueId, int linkIndex, int physicsClientId, struct point_pair *result)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int status_type = 0;
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle status_handle;

	if (bodyUniqueId < 0)
	{
		puts("getAABB failed; invalid bodyUniqueId");
		return 0;
	}

	if (linkIndex < -1)
	{
		puts("getAABB failed; invalid linkIndex");
		return 0;
	}

	cmd_handle =
		b3RequestCollisionInfoCommandInit(sm, bodyUniqueId);
	status_handle =
		b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

	status_type = b3GetStatusType(status_handle);
	if (status_type != CMD_REQUEST_COLLISION_INFO_COMPLETED)
	{
		puts("getAABB failed.");
		return 0;
	}

	{
		d3_t aabbMin;
		d3_t aabbMax;
		int i=0;
		if (b3GetStatusAABB(status_handle, linkIndex, (double*)&aabbMin, (double*)&aabbMax))
		{
			result->a = aabbMin;
			result->b = aabbMax;
			return 1;
		}
	}
	puts("getAABB failed.");
	return 0;
}

int bullet_getNumBodies(int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetNumBodies(sm);
}

int bullet_getBodyUniqueId(int serialIndex, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetBodyUniqueId(sm, serialIndex);
}

void bullet_removeBody(int bodyUniqueId, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	if (bodyUniqueId>=0)
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		if (b3CanSubmitCommand(sm))
		{
			statusHandle = b3SubmitClientCommandAndWaitStatus( sm, b3InitRemoveBodyCommand(sm,bodyUniqueId));
			statusType = b3GetStatusType(statusHandle);
		}
	}
}

int bullet_getBodyInfo(int bodyUniqueId, int physicsClientId, struct b3BodyInfo *info)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if (b3GetBodyInfo(sm, bodyUniqueId, info))
	{
		return 1;
	}
	return 0;
}

int bullet_getConstraintInfo(int constraintUniqueId, int physicsClientId, struct b3UserConstraint *constraintInfo)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if (b3GetUserConstraintInfo(sm, constraintUniqueId, constraintInfo))
	{
		return 1;
	}

	puts("Couldn't get user constraint info");
	return 0;
}

int bullet_getConstraintState(int constraintUniqueId, int physicsClientId, struct b3UserConstraintState *constraintState)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	if (b3CanSubmitCommand(sm))
	{
		return 1;
	}
	puts("Couldn't getConstraintState.");
	return 0;
}

int bullet_getConstraintUniqueId(int serialIndex, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetUserConstraintId(sm, serialIndex);
}

int bullet_getNumConstraints(int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetNumUserConstraints(sm);
}

int bullet_getAPIVersion()
{
	return SHARED_MEMORY_MAGIC_NUMBER;
}

// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
int bullet_getNumJoints(int bodyUniqueId, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	return b3GetNumJoints(sm, bodyUniqueId);
}

// Initalize all joint positions given a list of values
void bullet_resetJointState(int bodyUniqueId, int jointIndex, double targetValue, double targetVelocity, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int numJoints;

	numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if ((jointIndex >= numJoints) || (jointIndex < 0))
	{
		puts("Joint index out-of-range.");
		return;
	}

	commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

	b3CreatePoseCommandSetJointPosition(sm, commandHandle, jointIndex,
			targetValue);

	b3CreatePoseCommandSetJointVelocity(sm, commandHandle, jointIndex,
			targetVelocity);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
}

void bullet_resetBaseVelocity(int objectUniqueId, d3_t *linearVelocity, d3_t *angularVelocity, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if (linearVelocity || angularVelocity)
	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;

		commandHandle = b3CreatePoseCommandInit(sm, objectUniqueId);

		if (linearVelocity)
		{
			b3CreatePoseCommandSetBaseLinearVelocity(commandHandle, (double*)linearVelocity);
		}

		if (angularVelocity)
		{
			b3CreatePoseCommandSetBaseAngularVelocity(commandHandle, (double*)angularVelocity);
		}

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		return;
	}
	else
	{
		puts("expected at least linearVelocity and/or angularVelocity.");
		return;
	}
	puts("error in resetJointState.");
}

// Reset the position and orientation of the base/root link, position [x,y,z]
// and orientation quaternion [x,y,z,w]
void bullet_resetBasePositionAndOrientation(int bodyUniqueId, d3_t pos, d4_t orn, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;

	commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

	b3CreatePoseCommandSetBasePosition(commandHandle, pos._[0], pos._[1], pos._[2]);
	b3CreatePoseCommandSetBaseOrientation(commandHandle, orn._[0], orn._[1], orn._[2], orn._[3]);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
}

// Get the a single joint info for a specific bodyUniqueId
//
// Args:
//  bodyUniqueId - integer indicating body in simulation
//  jointIndex - integer indicating joint for a specific body
//
// Joint information includes:
//  index, name, type, q-index, u-index,
//  flags, joint damping, joint friction
//
// The format of the returned list is
// [int, str, int, int, int, int, float, float]
//
// TODO(hellojas): get joint positions for a body
int bullet_getJointInfo(int bodyUniqueId, int jointIndex, int physicsClientId, struct b3JointInfo *info)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if (b3GetJointInfo(sm, bodyUniqueId, jointIndex, info))
	{
		return 1;
	}
	return 0;
}

// Returns the state of a specific joint in a given bodyUniqueId
//
// Args:
//  bodyUniqueId - integer indicating body in simulation
//  jointIndex - integer indicating joint for a specific body
//
// The state of a joint includes the following:
//  position, velocity, force torque (6 values), and motor torque
// The returned pylist is an array of [float, float, float[6], float]

// TODO(hellojas): check accuracy of position and velocity
// TODO(hellojas): check force torque values

int bullet_getJointState(int bodyUniqueId, int jointIndex, int physicsClientId, struct b3JointSensorState *sensorState)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int status_type = 0;
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle status_handle;

	if (bodyUniqueId < 0)
	{
		puts("getJointState failed; invalid bodyUniqueId");
		return 0;
	}
	if (jointIndex < 0)
	{
		puts("getJointState failed; invalid jointIndex");
		return 0;
	}

	cmd_handle = b3RequestActualStateCommandInit(sm, bodyUniqueId);
	status_handle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
	status_type = b3GetStatusType(status_handle);

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		puts("getJointState failed.");
		return 0;
	}

	if (b3GetJointState(sm, status_handle, jointIndex, sensorState))
	{
		return 1;
	}
	return 0;
}

int bullet_getLinkState(int bodyUniqueId, int linkIndex,
		int computeLinkVelocity, int computeForwardKinematics,
		struct b3LinkState *linkState, int physicsClientId)
{
	int i;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int status_type = 0;
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle status_handle;

	if (bodyUniqueId < 0)
	{
		puts("getLinkState failed; invalid bodyUniqueId");
		return -1;
	}
	if (linkIndex < 0)
	{
		puts("getLinkState failed; invalid linkIndex");
		return -1;
	}

	cmd_handle = b3RequestActualStateCommandInit(sm, bodyUniqueId);

	if (computeLinkVelocity)
	{
		b3RequestActualStateCommandComputeLinkVelocity(cmd_handle,computeLinkVelocity);
	}

	if (computeForwardKinematics)
	{
		b3RequestActualStateCommandComputeForwardKinematics(cmd_handle,computeForwardKinematics);
	}

	status_handle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

	status_type = b3GetStatusType(status_handle);
	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		puts("getLinkState failed.");
		return 0;
	}

	if (b3GetLinkState(sm, status_handle, linkIndex, linkState))
	{
		return 1;
	}
	return 0;
}

int bullet_startStateLogging(
		int loggingType,
		const char *fileName,
		int size_objectUniqueIds,
		int *objectUniqueIds,
		int maxLogDof,
		int bodyUniqueIdA,
		int bodyUniqueIdB,
		int linkIndexA,
		int linkIndexB,
		int deviceTypeFilter,
		int logFlags,
		int physicsClientId
)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	b3SharedMemoryCommandHandle commandHandle;
	commandHandle = b3StateLoggingCommandInit(sm);

	b3StateLoggingStart(commandHandle, loggingType, fileName);

	if (objectUniqueIds)
	{
		int i;
		for (i = 0; i < size_objectUniqueIds; i++)
		{
			b3StateLoggingAddLoggingObjectUniqueId(commandHandle, objectUniqueIds[i]);
		}
	}

	if (maxLogDof > 0)
	{
		b3StateLoggingSetMaxLogDof(commandHandle, maxLogDof);
	}
	
	if (bodyUniqueIdA > -1)
	{
		b3StateLoggingSetBodyAUniqueId(commandHandle, bodyUniqueIdA);
	}
	if (bodyUniqueIdB > -1)
	{
		b3StateLoggingSetBodyBUniqueId(commandHandle, bodyUniqueIdB);
	}
	if (linkIndexA > -2)
	{
		b3StateLoggingSetLinkIndexA(commandHandle, linkIndexA);
	}
	if (linkIndexB > -2)
	{
		b3StateLoggingSetLinkIndexB(commandHandle, linkIndexB);
	}

	if (deviceTypeFilter>=0)
	{
		b3StateLoggingSetDeviceTypeFilter(commandHandle,deviceTypeFilter);
	}

	if (logFlags >0)
	{
		b3StateLoggingSetLogFlags(commandHandle, logFlags);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_STATE_LOGGING_START_COMPLETED)
	{
		return b3GetStatusLoggingUniqueId(statusHandle);
	}
	return -1;
}

void bullet_submitProfileTiming(const char *eventName, int duraction, int physicsClientId)
{
	int duractionInMicroSeconds=-1;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	if (eventName)
	{
		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3ProfileTimingCommandInit(sm, eventName);
		if (duractionInMicroSeconds>=0)
		{
			b3SetProfileTimingDuractionInMicroSeconds(commandHandle, duractionInMicroSeconds);
		}
		b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
}


void bullet_stopStateLogging(int loggingId, int physicsClientId)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	if (loggingId >= 0)
	{
		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3StateLoggingCommandInit(sm);
		b3StateLoggingStop(commandHandle, loggingId);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
	}
}


void bullet_setAdditionalSearchPath(const char *path, int physicsClientId)
{
	if (path)
	{
		b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;

		commandHandle = b3SetAdditionalSearchPath(sm, path);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
}

void bullet_setTimeOut(double timeOutInSeconds, int physicsClientId)
{
	if (timeOutInSeconds >= 0)
	{
		b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
		b3SetTimeOut(sm, timeOutInSeconds);
	}
}

int bullet_rayTestObsolete(d3_t rayFromPosition, d3_t rayToPosition, int physicsClientId, struct b3RaycastInformation *raycastInfo)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3CreateRaycastCommandInit(sm, rayFromPosition.x, rayFromPosition.y, rayFromPosition.z,
											   rayToPosition.x, rayToPosition.y, rayToPosition.z);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
	{
		b3GetRaycastInformation(sm, raycastInfo);
		return 1;
	}
	return 0;
}

int bullet_rayTestBatch(int num_rays, d3_t *rayFromPositions, d3_t *rayToPositions, int numThreads, int physicsClientId,
		struct b3RaycastInformation *raycastInfo)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	
	commandHandle = b3CreateRaycastBatchCommandInit(sm);
	b3RaycastBatchSetNumThreads(commandHandle, numThreads);

	if (rayFromPositions && rayToPositions)
	{
		if (num_rays > MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING)
		{
			puts("Number of rays exceed the maximum batch size.");
			return 0;
		}
		b3PushProfileTiming(sm, "extractthonFromToSequenceToC");
		b3RaycastBatchAddRays(sm, commandHandle, (double*)rayFromPositions, (double*)rayToPositions, num_rays);	
		b3PopProfileTiming(sm);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
	{
		int i;
		b3PushProfileTiming(sm, "convertRaycastInformationTothon");
		b3GetRaycastInformation(sm, raycastInfo);
		b3PopProfileTiming(sm);
		return 1;
	}
	return 0;
}

int bullet_getCollisionShapeData(int objectUniqueId, int linkIndex, int physicsClientId, struct b3CollisionShapeInformation *collisionShapeInfo)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int i;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3InitRequestCollisionShapeInformation(sm, objectUniqueId, linkIndex);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_COLLISION_SHAPE_INFO_COMPLETED)
	{
		b3GetCollisionShapeInformation(sm, collisionShapeInfo);
		return 1;
	}
	else
	{
		puts("Error receiving collision shape info");
	}
	return 0;
}

int bullet_getOverlappingObjects(d3_t aabbMin, d3_t aabbMax, int physicsClientId)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3AABBOverlapData overlapData;


	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3InitAABBOverlapQuery(sm, (double*)&aabbMin, (double*)&aabbMax);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	b3GetAABBOverlapResults(sm, &overlapData);

	if (overlapData.m_numOverlappingObjects)
	{
		return 1;
	}
	return 0;
}

int bullet_getClosestPointData(int bodyA, int bodyB,
		double distanceThreshold,
		int linkIndexA, int linkIndexB,
		int physicsClientId,
		struct b3ContactInformation *contactPointData)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3InitClosestDistanceQuery(sm);
	b3SetClosestDistanceFilterBodyA(commandHandle, bodyA);
	b3SetClosestDistanceFilterBodyB(commandHandle, bodyB);
	b3SetClosestDistanceThreshold(commandHandle, distanceThreshold);
	if (linkIndexA >= -1)
	{
		b3SetClosestDistanceFilterLinkA(commandHandle, linkIndexA);
	}
	if (linkIndexB >= -1)
	{
		b3SetClosestDistanceFilterLinkB(commandHandle, linkIndexB);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
	{
		b3GetContactPointInformation(sm, contactPointData);

		return 1;
	}

	return 0;
}

void bullet_changeUserConstraint(int userConstraintUniqueId,
		d3_t *jointChildPivot, d4_t *jointChildFrameOrn, double maxForce,
		double gearRatio, int gearAuxLink, double relativePositionTarget, double erp,
		int physicsClientId)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	/* int gearAuxLink = -1; */
	/* double maxForce = -1; */
	/* double gearRatio = 0; */
	/* double relativePositionTarget=1e32; */
	/* double erp=-1; */
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3InitChangeUserConstraintCommand(sm, userConstraintUniqueId);

	if (jointChildPivot)
	{
		b3InitChangeUserConstraintSetPivotInB(commandHandle, (double*)jointChildPivot);
	}

	if (jointChildFrameOrn)
	{
		b3InitChangeUserConstraintSetFrameInB(commandHandle, (double*)jointChildFrameOrn);
	}

	if (relativePositionTarget<1e10)
	{
		b3InitChangeUserConstraintSetRelativePositionTarget(commandHandle, relativePositionTarget);
	}
	if (erp>=0)
	{
		b3InitChangeUserConstraintSetERP(commandHandle, erp);
	}

	if (maxForce >= 0)
	{
		b3InitChangeUserConstraintSetMaxForce(commandHandle, maxForce);
	}
	if (gearRatio!=0)
	{
		b3InitChangeUserConstraintSetGearRatio(commandHandle,gearRatio);
	}
	if (gearAuxLink>=0)
	{
		b3InitChangeUserConstraintSetGearAuxLink(commandHandle,gearAuxLink);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
}

void bullet_removeUserConstraint(int userConstraintUniqueId, int physicsClientId)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3InitRemoveUserConstraintCommand(sm, userConstraintUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
}

int bullet_enableJointForceTorqueSensor(int bodyUniqueId, int jointIndex, int enableSensor, int physicsClientId)
{
	int numJoints = -1;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if (bodyUniqueId < 0)
	{
		puts("Error: invalid bodyUniqueId");
		return 0;
	}
	numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if ((jointIndex < 0) || (jointIndex >= numJoints))
	{
		puts("Error: invalid jointIndex.");
		return 0;
	}

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3CreateSensorCommandInit(sm, bodyUniqueId);
	b3CreateSensorEnable6DofJointForceTorqueSensor(commandHandle, jointIndex, enableSensor);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CLIENT_COMMAND_COMPLETED)
	{
		return 1;
	}

	puts("Error creating sensor.");
	return 0;
}

int bullet_createCollisionShape(int shapeType, double radius,
		d3_t halfExtents, double height, const char *fileName,
		d3_t meshScale, d3_t planeNormal, int flags,
		d3_t collisionFramePosition, d4_t collisionFrameOrientation,
		int physicsClientId)
{
	/* double radius=0.5; */
	/* double halfExtents[3] = {1,1,1}; */
	/* double height = 1; */
	/* double meshScale[3] = {1,1,1}; */
	/* double planeNormal[3] = {0,0,1}; */
	/* double collisionFramePosition[3]={0,0,0}; */
	/* double collisionFrameOrientation[4]={0,0,0,1}; */
	/* int flags = 0; */

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);
	if (shapeType>=GEOM_SPHERE)
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		int shapeIndex = -1;

		b3SharedMemoryCommandHandle commandHandle = b3CreateCollisionShapeCommandInit(sm);
		if (shapeType==GEOM_SPHERE && radius>0)
		{
			shapeIndex = b3CreateCollisionShapeAddSphere(commandHandle,
					radius);
		}
		if (shapeType==GEOM_BOX)
		{
			shapeIndex = b3CreateCollisionShapeAddBox(commandHandle,
					(double*)&halfExtents);
		}
		if (shapeType==GEOM_CAPSULE && radius>0 && height>=0)
		{
			shapeIndex = b3CreateCollisionShapeAddCapsule(commandHandle,
					radius, height);
		}
		if (shapeType==GEOM_CYLINDER && radius>0 && height>=0)
		{
			shapeIndex = b3CreateCollisionShapeAddCylinder(commandHandle,
					radius, height);
		}
		if (shapeType==GEOM_MESH && fileName)
		{
			shapeIndex = b3CreateCollisionShapeAddMesh(commandHandle, fileName,
					(double*)&meshScale);
		}
		if (shapeType==GEOM_PLANE)
		{
			double planeConstant=0;
			shapeIndex = b3CreateCollisionShapeAddPlane(commandHandle,
					(double*)&planeNormal, planeConstant);
		}
		if (shapeIndex>=0 && flags)
		{
			b3CreateCollisionSetFlag(commandHandle,shapeIndex,flags);
		}
		if (shapeIndex>=0)
		{
			b3CreateVisualShapeSetChildTransform(commandHandle, shapeIndex,
					(double*)&collisionFramePosition, (double*)&collisionFrameOrientation);
		}
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CREATE_COLLISION_SHAPE_COMPLETED)
		{
			return b3GetStatusCollisionShapeUniqueId(statusHandle);
		}
	}
	puts("createCollisionShape failed.");
	return -1;
}

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
	int physicsClientId)
{

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int i;
	b3SharedMemoryCommandHandle commandHandle = b3CreateMultiBodyCommandInit(sm);
	int baseIndex;
	baseIndex = b3CreateMultiBodyBase(commandHandle, baseMass,
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

		b3CreateMultiBodyLink(commandHandle, 
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
		b3CreateMultiBodyUseMaximalCoordinates(commandHandle);
	}
	if (flags > 0)
	{
		b3CreateMultiBodySetFlags(commandHandle, flags);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CREATE_MULTI_BODY_COMPLETED)
	{
		return b3GetStatusBodyIndex(statusHandle);
	}

	puts("createMultiBody failed.");
	return -1;
}


int bullet_createUserConstraint(int parentBodyUniqueId, int parentLinkIndex,
		int childBodyUniqueId, int childLinkIndex, int jointType,
		d3_t jointAxis, d3_t parentFramePosition, d3_t childFramePosition,
		d4_t parentFrameOrientation, d4_t childFrameOrientation,
		int physicsClientId)
{
	b3SharedMemoryCommandHandle commandHandle;
	/* int parentBodyUniqueId = -1; */
	/* int parentLinkIndex = -1; */
	/* int childBodyUniqueId = -1; */
	/* int childLinkIndex = -1; */
	/* int jointType = ePoint2PointType; */
	/* d3_t jointAxis = {0, 0, 0}; */
	/* d3_t parentFramePosition[3] = {0, 0, 0}; */
	/* d3_t childFramePosition[3] = {0, 0, 0}; */
	/* d4_t parentFrameOrientation[4] = {0, 0, 0, 1}; */
	/* d4_t childFrameOrientation[4] = {0, 0, 0, 1}; */

	struct b3JointInfo jointInfo;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	jointInfo.m_jointType = jointType;
	jointInfo.m_parentFrame[0] = parentFramePosition.x;
	jointInfo.m_parentFrame[1] = parentFramePosition.y;
	jointInfo.m_parentFrame[2] = parentFramePosition.z;
	jointInfo.m_parentFrame[3] = parentFrameOrientation.x;
	jointInfo.m_parentFrame[4] = parentFrameOrientation.y;
	jointInfo.m_parentFrame[5] = parentFrameOrientation.z;
	jointInfo.m_parentFrame[6] = parentFrameOrientation.w;

	jointInfo.m_childFrame[0] = childFramePosition.x;
	jointInfo.m_childFrame[1] = childFramePosition.y;
	jointInfo.m_childFrame[2] = childFramePosition.z;
	jointInfo.m_childFrame[3] = childFrameOrientation.x;
	jointInfo.m_childFrame[4] = childFrameOrientation.y;
	jointInfo.m_childFrame[5] = childFrameOrientation.z;
	jointInfo.m_childFrame[6] = childFrameOrientation.w;

	jointInfo.m_jointAxis[0] = jointAxis.x;
	jointInfo.m_jointAxis[1] = jointAxis.y;
	jointInfo.m_jointAxis[2] = jointAxis.z;

	commandHandle = b3InitCreateUserConstraintCommand(sm, parentBodyUniqueId, parentLinkIndex, childBodyUniqueId, childLinkIndex, &jointInfo);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_USER_CONSTRAINT_COMPLETED)
	{
		return b3GetStatusUserConstraintUniqueId(statusHandle);
	}

	puts("createConstraint failed.");
	return -1;
}

int bullet_getContactPointData(int bodyA, int bodyB, int linkIndexA, int linkIndexB, int physicsClientId, struct b3ContactInformation *contactPointData)
{
	
	b3SharedMemoryCommandHandle commandHandle;
	;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	commandHandle = b3InitRequestContactPointInformation(sm);
	if (bodyA>=0)
	{
		b3SetContactFilterBodyA(commandHandle, bodyA);
	}
	if (bodyB>=0)
	{
		b3SetContactFilterBodyB(commandHandle, bodyB);
	}

	if (linkIndexA >= -1)
	{
		b3SetContactFilterLinkA(commandHandle, linkIndexA);
	}
	if (linkIndexB >= -1)
	{
		b3SetContactFilterLinkB(commandHandle, linkIndexB);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
	{
		b3GetContactPointInformation(sm, contactPointData);

		return 1;
	}
	return 0;
}

void bullet_applyExternalForce(int objectUniqueId, int linkIndex, d3_t force, d3_t position, int flags, int physicsClientId)
{
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
	{
		puts("flag has to be either WORLD_FRAME or LINK_FRAME");
		return;
	}
	command = b3ApplyExternalForceCommandInit(sm);
	b3ApplyExternalForce(command, objectUniqueId, linkIndex, (double*)&force,
			(double*)&position, flags);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
}

int bullet_applyExternalTorque(int objectUniqueId, int linkIndex, d3_t torque, int flags, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

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
	b3SharedMemoryStatusHandle statusHandle;
	b3SharedMemoryCommandHandle command =
		b3ApplyExternalForceCommandInit(sm);
	b3ApplyExternalTorque(command, objectUniqueId, linkIndex, (double*)&torque,
			flags);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

	return 1;
}

int bullet_loadPlugin(const char *pluginPath, const char *postFix, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	b3SharedMemoryCommandHandle command = 0;
	b3SharedMemoryStatusHandle 	statusHandle = 0;
	int statusType = -1;

	command = b3CreateCustomCommand(sm);
	b3CustomCommandLoadPlugin(command, pluginPath);
	if (postFix)
	{
		b3CustomCommandLoadPluginSetPostFix(command, postFix);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusPluginUniqueId(statusHandle);
	return statusType;
}

void bullet_unloadPlugin(int pluginUniqueId, int physicsClientId)
{
	b3SharedMemoryCommandHandle command = 0;
	b3SharedMemoryStatusHandle 	statusHandle = 0;
	int statusType = -1;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	command = b3CreateCustomCommand(sm);
	b3CustomCommandUnloadPlugin(command, pluginUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
}

//createCustomCommand for executing commands implemented in a plugin system
int bullet_executePluginCommand(int pluginUniqueId, const char *textArgument, int intArgs_len, int *intArgs, int floatArgs_len, float *floatArgs, int physicsClientId)
{
	b3SharedMemoryCommandHandle command=0;
	b3SharedMemoryStatusHandle 	statusHandle=0;
	int statusType = -1;

	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);


	command = b3CreateCustomCommand(sm);
	b3CustomCommandExecutePluginCommand(command, pluginUniqueId, textArgument);

	{
		int i;
		for (i=0;i<intArgs_len;i++)
		{
			b3CustomCommandExecuteAddIntArgument(command, intArgs[i]);
		}
		

		for (i=0;i<floatArgs_len;i++)
		{
			b3CustomCommandExecuteAddFloatArgument(command, floatArgs[i]);
		}
	}
	
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusPluginCommandResult(statusHandle);
	return statusType;
}

///Inverse Kinematics binding
double *bullet_calculateInverseKinematics(
		int bodyUniqueId,
		int endEffectorLinkIndex,
		d3_t targetPosition, d4_t *targetOrientation,
		double *lowerLimits, double *upperLimits,
		double *jointRanges,
		double *restPoses,
		double *jointDamping,
		int solver,
		d3_t *currentPositions,
		int maxNumIterations,
		double residualThreshold, int physicsClientId)

{

	/* int solver = 0; // the default IK solver is DLS */

	static char* kwlist[] = {};
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int numJoints = b3GetNumJoints(sm, bodyUniqueId);
	int dofCount = b3ComputeDofCount(sm, bodyUniqueId);

	b3SharedMemoryStatusHandle statusHandle;
	int numPos = 0;
	int resultBodyIndex;
	int result;

	b3SharedMemoryCommandHandle command =
		b3CalculateInverseKinematicsCommandInit(sm, bodyUniqueId);
	b3CalculateInverseKinematicsSelectSolver(command, solver);

	if (currentPositions)
	{
		b3CalculateInverseKinematicsSetCurrentPositions(command, dofCount,
				(double*)&currentPositions);
	}
	if (maxNumIterations>0)
	{
		b3CalculateInverseKinematicsSetMaxNumIterations(command,maxNumIterations);
	}
	if (residualThreshold>=0)
	{
		b3CalculateInverseKinematicsSetResidualThreshold(command,residualThreshold);
	}

	if (dofCount)
	{
		if (targetOrientation)
		{
			b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command,
					dofCount, endEffectorLinkIndex, (double*)&targetPosition,
					(double*)targetOrientation, lowerLimits, upperLimits, jointRanges,
					restPoses);
		}
		else
		{
			b3CalculateInverseKinematicsPosWithNullSpaceVel(command, dofCount,
					endEffectorLinkIndex, (double*)&targetPosition, lowerLimits,
					upperLimits, jointRanges, restPoses);
		}
	}
	else
	{
		if (targetOrientation)
		{
			b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command,
					endEffectorLinkIndex, (double*)&targetPosition,
					(double*)targetOrientation);
		}
		else
		{
			b3CalculateInverseKinematicsAddTargetPurePosition(command,
					endEffectorLinkIndex, (double*)&targetPosition);
		}
	}

	if (jointDamping)
	{
		b3CalculateInverseKinematicsSetJointDamping(command, dofCount, jointDamping);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

	result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
			&resultBodyIndex, &numPos, 0);
	if (result && numPos)
	{
		int i;
		double* ikOutPutJointPos = (double*)malloc(numPos * sizeof(double));
		result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
				&resultBodyIndex, &numPos, ikOutPutJointPos);
		return ikOutPutJointPos;
	}
	else
	{
		puts("Error in calculateInverseKinematics");
	}
	return NULL;
}

/// Given an object id, joint positions, joint velocities and joint
/// accelerations,
/// compute the joint forces using Inverse Dynamics
int bullet_calculateInverseDynamics(int bodyUniqueId,
		d3_t *jointPositionsQ,
		d3_t *jointVelocitiesQdot,
		d3_t *jointAccelerations,
		int physicsClientId,
		double **jointForcesOutput)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int dofCountOrg = b3ComputeDofCount(sm, bodyUniqueId);

	if (dofCountOrg)
	{
		int szInBytes = sizeof(double) * dofCountOrg;
		double* jointForcesOutput = (double*)malloc(szInBytes);


		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		b3SharedMemoryCommandHandle commandHandle =
			b3CalculateInverseDynamicsCommandInit(
					sm, bodyUniqueId, (double*)jointPositionsQ,
					(double*)jointVelocitiesQdot, (double*)jointAccelerations);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

		statusType = b3GetStatusType(statusHandle);

		if (statusType == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED)
		{
			int bodyUniqueId;
			int dofCount;

			b3GetStatusInverseDynamicsJointForces(statusHandle, &bodyUniqueId,
					&dofCount, 0);


			if (dofCount)
			{
				b3GetStatusInverseDynamicsJointForces(statusHandle, 0, 0,
						jointForcesOutput);
				return dofCount;
			}
		}
		else
		{
			puts("Internal error in calculateInverseDynamics");
		}
	}
	else
	{
		puts("calculateInverseDynamics numDofs needs to be "
				"positive and [joint positions], [joint velocities], "
				"[joint accelerations] need to match the number of "
				"degrees of freedom.");
	}
	return 0;
}

/// Given an object id, joint positions, joint velocities and joint
/// accelerations, compute the Jacobian
int bullet_calculateJacobian(int bodyUniqueId,
		int linkIndex, d3_t localPosition, d3_t *jointPositions,
		d3_t *jointVelocities, d3_t *jointAccelerations, int physicsClientId,
		mat3_t *linearJacobian, mat3_t *angularJacobian)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int numJoints = b3GetNumJoints(sm, bodyUniqueId);

	int j=0;
	int dofCountOrg = 0;
	for (j=0;j<numJoints;j++)
	{
		struct b3JointInfo info;
		b3GetJointInfo(sm, bodyUniqueId, j, &info);
		switch (info.m_jointType)
		{
			case eRevoluteType:
				{
					dofCountOrg+=1;
					break;
				}
			case ePrismaticType:
				{
					dofCountOrg+=1;
					break;
				}
			case eSphericalType:
				{
					puts("Spherirical joints are not supported in the bullet binding");
					return -1;
				}
			case ePlanarType:
				{
					puts("Planar joints are not supported in the bullet binding");
					return -1;
				}
			default:
				{
					//fixed joint has 0-dof and at the moment, we don't deal with planar, spherical etc
				}
		}
	}

	if (dofCountOrg)
	{
		b3SharedMemoryStatusHandle statusHandle;	
		int statusType;
		b3SharedMemoryCommandHandle commandHandle =
			b3CalculateJacobianCommandInit(sm, bodyUniqueId, linkIndex,
					(double*)&localPosition, (double*)jointPositions,
					(double*)jointVelocities, (double*)jointAccelerations);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CALCULATED_JACOBIAN_COMPLETED)
		{
			int dofCount;
			b3GetStatusJacobian(statusHandle, &dofCount, NULL, NULL);
			if (dofCount)
			{
				int byteSizeDofCount = sizeof(double) * dofCount;
				b3GetStatusJacobian(statusHandle, NULL, (double*)linearJacobian, (double*)angularJacobian);
			}
		}
		else
		{
			puts("Internal error in calculateJacobian");
		}
	}
	else
	{
		puts("calculateJacobian [numDof] needs to be "
				"positive, [local position] needs to be of "
				"size 3 and [joint positions], "
				"[joint velocities], [joint accelerations] "
				"need to match the number of DoF.");
	}
	return 1;
}

/// Given an object id, joint positions, joint velocities and joint
/// accelerations, compute the Jacobian
double *bullet_calculateMassMatrix(int bodyUniqueId, double *jointPositions, int physicsClientId)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physicsClientId);

	int numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if (numJoints)
	{
		double *massMatrix = NULL;
		int i;
		b3SharedMemoryStatusHandle statusHandle;	
		int statusType;
		b3SharedMemoryCommandHandle commandHandle = b3CalculateMassMatrixCommandInit(sm, bodyUniqueId, jointPositions);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CALCULATED_MASS_MATRIX_COMPLETED)
		{
			int dofCount;
			b3GetStatusMassMatrix(sm, statusHandle, &dofCount, NULL);
			if (dofCount)
			{
				int byteSizeDofCount = sizeof(double) * dofCount;

				massMatrix = (double*)malloc(dofCount * byteSizeDofCount);
				b3GetStatusMassMatrix(sm, statusHandle, NULL, massMatrix);
			}
		}
		else
		{
			puts("Internal error in calculateJacobian");
		}
		return massMatrix;
	}
	else
	{
		puts("No joints");
	}
	return NULL;
}

int test(int argc, const char *argv[])
{

	int c1 = bullet_createCollisionShape(GEOM_PLANE, 0.0f, d3(0,0,0), -1, NULL, d3(1,1,1),
			d3(0,0,1), 0, d3(0,0,0), d4(0,0,0,1), 0);

	int c2 = bullet_createCollisionShape(GEOM_SPHERE, 0.5f, d3(0,0,0), -1, NULL, d3(1,1,1),
			d3(0,0,0), 0, d3(0,0,0), d4(0,0,0,1), 0);

	bullet_createMultiBody( 0, c1, -1,
			d3(0, 0, 0), d4(0,0,0,1), d3(0, 0, 0), d4(0,0,0,1), 0, NULL, NULL, NULL,
			NULL, NULL, NULL, NULL, NULL, NULL, NULL, 1, 0, 0);
	bullet_createMultiBody( 1, c2, -1,
			d3(0, 0, 1), d4(0,0,0,1), d3(0, 0, 0), d4(0,0,0,1), 0, NULL, NULL, NULL,
			NULL, NULL, NULL, NULL, NULL, NULL, NULL, 1, 0, 0);

	return 0;
}

void c_bullet_init(c_bullet_t *self)
{
	bullet_connectPhysicsServer(eCONNECT_DIRECT, SHARED_MEMORY_KEY,
			"localhost", -1);
	self->time_scale = 1.0f;
	bullet_setGravity(d3(0, -10, 0), 0);
	/* bullet_setRealTimeSimulation(1, 0); */
}

c_bullet_t *c_bullet_new()
{
	c_bullet_t *self = component_new("bullet");
	self->running = 1;
	return self;
}

int c_bullet_update(c_bullet_t *self, float *dt)
{
	if(self->running && bullet_isConnected(0))
	{
		int i;
		bullet_setTimeStep(*dt * self->time_scale, 0);
		bullet_stepSimulation(0);
		/* d3_t p; */
		/* d4_t q; */
		/* for(i = 0; i < bullet_getNumBodies(0); i++) */
		/* { */
		/* } */
	}
}

int c_bullet_menu(c_bullet_t *self, void *ctx)
{
	return CONTINUE;
}

REG()
{
	ct_t *ct = ct_new("bullet", sizeof(c_bullet_t), c_bullet_init, NULL, 0);

	ct_listener(ct, WORLD, sig("world_update"), c_bullet_update);

	ct_listener(ct, WORLD, sig("component_menu"), c_bullet_menu);

	signal_init(sig("collider_callback"), 0);
}

