
#include <PhysicsDirectC_API.h>
#include <SharedMemoryInProcessPhysicsC_API.h>
#include "blw.h"

#ifdef _WIN32
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <unistd.h>
#include <dlfcn.h>
#endif
#include "../candle/systems/sauces.h"

b3PhysicsClientHandle (*b3wConnectPhysicsDirect)(void);
void (*b3wDisconnectSharedMemory)(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle (*b3wInitStepSimulationCommand)(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle (*b3wInitSyncBodyInfoCommand)(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle (*b3wInitSyncUserDataCommand)(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle (*b3wInitResetSimulationCommand)(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle (*b3wInitPhysicsParamCommand)(b3PhysicsClientHandle physClient);
int (*b3wPhysicsParamSetRealTimeSimulation)(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
int (*b3wPhysicsParamSetGravity)(b3SharedMemoryCommandHandle commandHandle, double gravx, double gravy, double gravz);
int (*b3wPhysicsParamSetTimeStep)(b3SharedMemoryCommandHandle commandHandle, double timeStep);
b3SharedMemoryCommandHandle (*b3wCreateMultiBodyCommandInit)(b3PhysicsClientHandle physClient);
int (*b3wCreateMultiBodyBase)(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, const double basePosition[/*3*/], const double baseOrientation[/*4*/], const double baseInertialFramePosition[/*3*/], const double baseInertialFrameOrientation[/*4*/]);
int (*b3wCreateMultiBodyLink)(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex,
 								double linkVisualShapeIndex,
 								const double linkPosition[/*3*/],
 								const double linkOrientation[/*4*/],
 								const double linkInertialFramePosition[/*3*/],
 								const double linkInertialFrameOrientation[/*4*/],
 								int linkParentIndex,
 								int linkJointType,
 								const double linkJointAxis[/*3*/]);
void (*b3wCreateMultiBodyUseMaximalCoordinates)(b3SharedMemoryCommandHandle commandHandle);
void (*b3wCreateMultiBodySetFlags)(b3SharedMemoryCommandHandle commandHandle, int flags);
int (*b3wGetStatusBodyIndex)(b3SharedMemoryStatusHandle statusHandle);


b3SharedMemoryCommandHandle(*b3wCreateCollisionShapeCommandInit)(b3PhysicsClientHandle physClient);
b3SharedMemoryStatusHandle(*b3wSubmitClientCommandAndWaitStatus)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);
int (*b3wCanSubmitCommand)(b3PhysicsClientHandle physClient);
int (*b3wGetStatusType)(b3SharedMemoryStatusHandle statusHandle);
int (*b3wGetStatusCollisionShapeUniqueId)(b3SharedMemoryStatusHandle statusHandle);
int (*b3wCreateCollisionShapeAddSphere)(b3SharedMemoryCommandHandle commandHandle, double radius);
int (*b3wCreateCollisionShapeAddBox)(b3SharedMemoryCommandHandle commandHandle, const double halfExtents[/*3*/]);
int (*b3wCreateCollisionShapeAddCapsule)(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
int (*b3wCreateCollisionShapeAddCylinder)(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
int (*b3wCreateCollisionShapeAddHeightfield)(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/], double textureScaling);
int (*b3wCreateCollisionShapeAddHeightfield2)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], double textureScaling, float* heightfieldData, int numHeightfieldRows, int numHeightfieldColumns, int replaceHeightfieldIndex);
int (*b3wCreateCollisionShapeAddPlane)(b3SharedMemoryCommandHandle commandHandle, const double planeNormal[/*3*/], double planeConstant);
int (*b3wCreateCollisionShapeAddMesh)(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/]);
int (*b3wCreateCollisionShapeAddConvexMesh)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices);
int (*b3wCreateCollisionShapeAddConcaveMesh)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices, const int* indices, int numIndices);
void (*b3wCreateCollisionSetFlag)(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, int flags);
void (*b3wCreateCollisionShapeSetChildTransform)(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double childPosition[/*3*/], const double childOrientation[/*4*/]);
b3SharedMemoryCommandHandle (*b3wCreatePoseCommandInit)(b3PhysicsClientHandle physClient, int bodyUniqueId);
int (*b3wCreatePoseCommandSetBasePosition)(b3SharedMemoryCommandHandle commandHandle, double startPosX, double startPosY, double startPosZ);
int (*b3wCreatePoseCommandSetBaseOrientation)(b3SharedMemoryCommandHandle commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);
int (*b3wCreatePoseCommandSetBaseLinearVelocity)(b3SharedMemoryCommandHandle commandHandle, const double linVel[/*3*/]);
int (*b3wCreatePoseCommandSetBaseAngularVelocity)(b3SharedMemoryCommandHandle commandHandle, const double angVel[/*3*/]);
int (*b3wCreatePoseCommandSetBaseScaling)(b3SharedMemoryCommandHandle commandHandle, double scaling[/* 3*/]);
b3SharedMemoryCommandHandle (*b3wRequestActualStateCommandInit)(b3PhysicsClientHandle physClient, int bodyUniqueId);
int (*b3wGetStatusActualState)(b3SharedMemoryStatusHandle statusHandle,
 									 int* bodyUniqueId,
 									 int* numDegreeOfFreedomQ,
 									 int* numDegreeOfFreedomU,
 									 const double* rootLocalInertialFrame[],
 									 const double* actualStateQ[],
 									 const double* actualStateQdot[],
 									 const double* jointReactionForces[]);
b3SharedMemoryCommandHandle (*b3wApplyExternalForceCommandInit)(b3PhysicsClientHandle physClient);
void (*b3wApplyExternalForce)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[/*3*/], const double position[/*3*/], int flag);
void (*b3wApplyExternalTorque)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[/*3*/], int flag);
b3SharedMemoryCommandHandle (*b3wInitRemoveBodyCommand)(b3PhysicsClientHandle physClient, int bodyUniqueId);
int (*b3wGetNumBodies)(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle (*b3wInitChangeDynamicsInfo)(b3PhysicsClientHandle physClient);
int (*b3wChangeDynamicsInfoSetActivationState)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int activationState);

void b3w_init(void)
{
#ifdef _WIN32
#define bllib(l) LoadLibrary(l)
#define blsym(v, type, l, s) v = (type)GetProcAddress(l, #s)
#define blclose(l)
#elif defined(__EMSCRIPTEN__)
#define bllib(l)
#define blsym(v, type, l, s) v = (type)s
#define blclose(l)
#else
#define bllib(l) dlopen(l, RTLD_NOW | RTLD_NODELETE)
#define blsym(v, type, l, s) v = (type)dlsym(l, #s)
#define blclose(l)
#endif

	FILE *fp;
#ifdef _WIN32
	HINSTANCE obllib;
	char lib_filename[MAX_PATH];  
	strcpy(lib_filename, "libBulletRobotics.dll");
	fp = fopen(lib_filename, "r");
	if (fp == NULL)
	{
		char temp_path[MAX_PATH];
		HANDLE fd, file_map;
		LPVOID address;
		size_t bytes_num;

		resource_t *sauce = c_sauces_get_sauce(c_sauces(&SYS), sauce_handle(lib_filename));
		char *bytes = c_sauces_get_bytes(c_sauces(&SYS), sauce, &bytes_num);

		GetTempPath(MAX_PATH, temp_path);
		GetTempFileName(temp_path, TEXT("tempobl"), 0, lib_filename);

		fd = CreateFile(lib_filename, GENERIC_READ | GENERIC_WRITE, 0, NULL,
				CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
		file_map = CreateFileMapping(fd, NULL, PAGE_READWRITE, 0, bytes_num, NULL);	    
		address = MapViewOfFile(file_map, FILE_MAP_WRITE, 0, 0, 0);	    
		CopyMemory(address, bytes, bytes_num);	    
		UnmapViewOfFile(address);
		CloseHandle(file_map);
		CloseHandle(fd);
	}
	obllib = bllib(lib_filename);
#elif !defined(__EMSCRIPTEN__)
	void *obllib;
	char lib_filename[PATH_MAX];  
	strcpy(lib_filename, "libBulletRobotics.so");
	fp = fopen(lib_filename, "r");
	if (fp == NULL)
	{
		char temp_name[] = "/tmp/XXXXXXX.so";
		int fd = mkstemps(temp_name, 3);
		size_t bytes_num;

		resource_t *sauce = c_sauces_get_sauce(c_sauces(&SYS), sauce_handle(lib_filename));
		char *bytes = c_sauces_get_bytes(c_sauces(&SYS), sauce, &bytes_num);

		if (write(fd, bytes, bytes_num) == -1)
		{
			printf("Failed to write to bl shared library temp file.\n");
			exit(1);
		}
		close(fd);
		obllib = bllib(temp_name);
	}
	else
	{
		obllib = bllib(lib_filename);
	}
	char *err = dlerror();
	if (err)
		puts(err);
#else
	void *obllib;
#endif


	blsym(b3wConnectPhysicsDirect, b3PhysicsClientHandle (*)(void), obllib, b3ConnectPhysicsDirect);
	blsym(b3wDisconnectSharedMemory, void (*)(b3PhysicsClientHandle physClient), obllib, b3DisconnectSharedMemory);
	blsym(b3wInitStepSimulationCommand, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3InitStepSimulationCommand);
	blsym(b3wInitSyncBodyInfoCommand, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3InitSyncBodyInfoCommand);
	blsym(b3wInitSyncUserDataCommand, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3InitSyncUserDataCommand);
	blsym(b3wInitResetSimulationCommand, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3InitResetSimulationCommand);
	blsym(b3wInitPhysicsParamCommand, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3InitPhysicsParamCommand);
	blsym(b3wPhysicsParamSetRealTimeSimulation, int (*)(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation), obllib, b3PhysicsParamSetRealTimeSimulation);
	blsym(b3wPhysicsParamSetGravity, int (*)(b3SharedMemoryCommandHandle commandHandle, double gravx, double gravy, double gravz), obllib, b3PhysicsParamSetGravity);
	blsym(b3wPhysicsParamSetTimeStep, int (*)(b3SharedMemoryCommandHandle commandHandle, double timeStep), obllib, b3PhysicsParamSetTimeStep);
	blsym(b3wCreateMultiBodyCommandInit, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3CreateMultiBodyCommandInit);
	blsym(b3wCreateMultiBodyBase, int (*)(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, const double basePosition[/*3*/], const double baseOrientation[/*4*/], const double baseInertialFramePosition[/*3*/], const double baseInertialFrameOrientation[/*4*/]), obllib, b3CreateMultiBodyBase);
	blsym(b3wCreateMultiBodyLink, int (*)(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex,
 								double linkVisualShapeIndex,
 								const double linkPosition[/*3*/],
 								const double linkOrientation[/*4*/],
 								const double linkInertialFramePosition[/*3*/],
 								const double linkInertialFrameOrientation[/*4*/],
 								int linkParentIndex,
 								int linkJointType,
 								const double linkJointAxis[/*3*/]), obllib, b3CreateMultiBodyLink);
	blsym(b3wCreateMultiBodyUseMaximalCoordinates, void (*)(b3SharedMemoryCommandHandle commandHandle), obllib, b3CreateMultiBodyUseMaximalCoordinates);
	blsym(b3wCreateMultiBodySetFlags, void (*)(b3SharedMemoryCommandHandle commandHandle, int flags), obllib, b3CreateMultiBodySetFlags);
	blsym(b3wGetStatusBodyIndex, int (*)(b3SharedMemoryStatusHandle statusHandle), obllib, b3GetStatusBodyIndex);
	blsym(b3wCreateCollisionShapeCommandInit, b3SharedMemoryCommandHandle(*)(b3PhysicsClientHandle physClient), obllib, b3CreateCollisionShapeCommandInit);
	blsym(b3wSubmitClientCommandAndWaitStatus, b3SharedMemoryStatusHandle(*)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle), obllib, b3SubmitClientCommandAndWaitStatus);
	blsym(b3wCanSubmitCommand, int (*)(b3PhysicsClientHandle physClient), obllib, b3CanSubmitCommand);
	blsym(b3wGetStatusType, int (*)(b3SharedMemoryStatusHandle statusHandle), obllib, b3GetStatusType);
	blsym(b3wGetStatusCollisionShapeUniqueId, int (*)(b3SharedMemoryStatusHandle statusHandle), obllib, b3GetStatusCollisionShapeUniqueId);
	blsym(b3wCreateCollisionShapeAddSphere, int (*)(b3SharedMemoryCommandHandle commandHandle, double radius), obllib, b3CreateCollisionShapeAddSphere);
	blsym(b3wCreateCollisionShapeAddBox, int (*)(b3SharedMemoryCommandHandle commandHandle, const double halfExtents[/*3*/]), obllib, b3CreateCollisionShapeAddBox);
	blsym(b3wCreateCollisionShapeAddCapsule, int (*)(b3SharedMemoryCommandHandle commandHandle, double radius, double height), obllib, b3CreateCollisionShapeAddCapsule);
	blsym(b3wCreateCollisionShapeAddCylinder, int (*)(b3SharedMemoryCommandHandle commandHandle, double radius, double height), obllib, b3CreateCollisionShapeAddCylinder);
	blsym(b3wCreateCollisionShapeAddHeightfield, int (*)(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/], double textureScaling), obllib, b3CreateCollisionShapeAddHeightfield);
	blsym(b3wCreateCollisionShapeAddHeightfield2, int (*)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], double textureScaling, float* heightfieldData, int numHeightfieldRows, int numHeightfieldColumns, int replaceHeightfieldIndex), obllib, b3CreateCollisionShapeAddHeightfield2);
	blsym(b3wCreateCollisionShapeAddPlane, int (*)(b3SharedMemoryCommandHandle commandHandle, const double planeNormal[/*3*/], double planeConstant), obllib, b3CreateCollisionShapeAddPlane);
	blsym(b3wCreateCollisionShapeAddMesh, int (*)(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/]), obllib, b3CreateCollisionShapeAddMesh);
	blsym(b3wCreateCollisionShapeAddConvexMesh, int (*)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices), obllib, b3CreateCollisionShapeAddConvexMesh);
	blsym(b3wCreateCollisionShapeAddConcaveMesh, int (*)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices, const int* indices, int numIndices), obllib, b3CreateCollisionShapeAddConcaveMesh);
	blsym(b3wCreateCollisionSetFlag, void (*)(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, int flags), obllib, b3CreateCollisionSetFlag);
	blsym(b3wCreateCollisionShapeSetChildTransform, void (*)(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double childPosition[/*3*/], const double childOrientation[/*4*/]), obllib, b3CreateCollisionShapeSetChildTransform);
	blsym(b3wCreatePoseCommandInit, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient, int bodyUniqueId), obllib, b3CreatePoseCommandInit);
	blsym(b3wCreatePoseCommandSetBasePosition, int (*)(b3SharedMemoryCommandHandle commandHandle, double startPosX, double startPosY, double startPosZ), obllib, b3CreatePoseCommandSetBasePosition);
	blsym(b3wCreatePoseCommandSetBaseOrientation, int (*)(b3SharedMemoryCommandHandle commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW), obllib, b3CreatePoseCommandSetBaseOrientation);
	blsym(b3wCreatePoseCommandSetBaseLinearVelocity, int (*)(b3SharedMemoryCommandHandle commandHandle, const double linVel[/*3*/]), obllib, b3CreatePoseCommandSetBaseLinearVelocity);
	blsym(b3wCreatePoseCommandSetBaseAngularVelocity, int (*)(b3SharedMemoryCommandHandle commandHandle, const double angVel[/*3*/]), obllib, b3CreatePoseCommandSetBaseAngularVelocity);
	blsym(b3wCreatePoseCommandSetBaseScaling, int (*)(b3SharedMemoryCommandHandle commandHandle, double scaling[/* 3*/]), obllib, b3CreatePoseCommandSetBaseScaling);
	blsym(b3wRequestActualStateCommandInit, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient, int bodyUniqueId), obllib, b3RequestActualStateCommandInit);
	blsym(b3wGetStatusActualState, int (*)(b3SharedMemoryStatusHandle statusHandle,
 									 int* bodyUniqueId,
 									 int* numDegreeOfFreedomQ,
 									 int* numDegreeOfFreedomU,
 									 const double* rootLocalInertialFrame[],
 									 const double* actualStateQ[],
 									 const double* actualStateQdot[],
 									 const double* jointReactionForces[]), obllib, b3GetStatusActualState);
	blsym(b3wApplyExternalForceCommandInit, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3ApplyExternalForceCommandInit);
	blsym(b3wApplyExternalForce, void (*)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[/*3*/], const double position[/*3*/], int flag), obllib, b3ApplyExternalForce);
	blsym(b3wApplyExternalTorque, void (*)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[/*3*/], int flag), obllib, b3ApplyExternalTorque);
	blsym(b3wInitRemoveBodyCommand, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient, int bodyUniqueId), obllib, b3InitRemoveBodyCommand);
	blsym(b3wGetNumBodies, int (*)(b3PhysicsClientHandle physClient), obllib, b3GetNumBodies);
	blsym(b3wInitChangeDynamicsInfo, b3SharedMemoryCommandHandle (*)(b3PhysicsClientHandle physClient), obllib, b3InitChangeDynamicsInfo);
	blsym(b3wChangeDynamicsInfoSetActivationState, int (*)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int activationState), obllib, b3ChangeDynamicsInfoSetActivationState);
}
