#ifndef BLW_H
#define BLW_H

#include <SharedMemoryPublic.h>

#define BLW_DECLARE_HANDLE(name) \
	typedef struct name##__     \
	{                           \
		int unused;             \
	} * name

#ifndef PHYSICS_CLIENT_C_API_H
BLW_DECLARE_HANDLE(b3PhysicsClientHandle);
BLW_DECLARE_HANDLE(b3SharedMemoryCommandHandle);
BLW_DECLARE_HANDLE(b3SharedMemoryStatusHandle);
#endif

extern b3PhysicsClientHandle (*b3wConnectPhysicsDirect)(void);
extern void (*b3wDisconnectSharedMemory)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryCommandHandle (*b3wInitStepSimulationCommand)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryCommandHandle (*b3wInitSyncBodyInfoCommand)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryCommandHandle (*b3wInitSyncUserDataCommand)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryCommandHandle (*b3wInitResetSimulationCommand)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryCommandHandle (*b3wInitPhysicsParamCommand)(b3PhysicsClientHandle physClient);
extern int (*b3wPhysicsParamSetRealTimeSimulation)(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
extern int (*b3wPhysicsParamSetGravity)(b3SharedMemoryCommandHandle commandHandle, double gravx, double gravy, double gravz);
extern int (*b3wPhysicsParamSetTimeStep)(b3SharedMemoryCommandHandle commandHandle, double timeStep);
extern b3SharedMemoryCommandHandle (*b3wCreateMultiBodyCommandInit)(b3PhysicsClientHandle physClient);
extern int (*b3wCreateMultiBodyBase)(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, const double basePosition[/*3*/], const double baseOrientation[/*4*/], const double baseInertialFramePosition[/*3*/], const double baseInertialFrameOrientation[/*4*/]);
extern int (*b3wCreateMultiBodyLink)(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex,
										double linkVisualShapeIndex,
										const double linkPosition[/*3*/],
										const double linkOrientation[/*4*/],
										const double linkInertialFramePosition[/*3*/],
										const double linkInertialFrameOrientation[/*4*/],
										int linkParentIndex,
										int linkJointType,
										const double linkJointAxis[/*3*/]);
extern void (*b3wCreateMultiBodyUseMaximalCoordinates)(b3SharedMemoryCommandHandle commandHandle);
extern void (*b3wCreateMultiBodySetFlags)(b3SharedMemoryCommandHandle commandHandle, int flags);
extern int (*b3wGetStatusBodyIndex)(b3SharedMemoryStatusHandle statusHandle);


extern b3SharedMemoryCommandHandle(*b3wCreateCollisionShapeCommandInit)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryStatusHandle(*b3wSubmitClientCommandAndWaitStatus)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);
extern int (*b3wCanSubmitCommand)(b3PhysicsClientHandle physClient);
extern int (*b3wGetStatusType)(b3SharedMemoryStatusHandle statusHandle);
extern int (*b3wGetStatusCollisionShapeUniqueId)(b3SharedMemoryStatusHandle statusHandle);
extern int (*b3wCreateCollisionShapeAddSphere)(b3SharedMemoryCommandHandle commandHandle, double radius);
extern int (*b3wCreateCollisionShapeAddBox)(b3SharedMemoryCommandHandle commandHandle, const double halfExtents[/*3*/]);
extern int (*b3wCreateCollisionShapeAddCapsule)(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
extern int (*b3wCreateCollisionShapeAddCylinder)(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
extern int (*b3wCreateCollisionShapeAddHeightfield)(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/], double textureScaling);
extern int (*b3wCreateCollisionShapeAddHeightfield2)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], double textureScaling, float* heightfieldData, int numHeightfieldRows, int numHeightfieldColumns, int replaceHeightfieldIndex);
extern int (*b3wCreateCollisionShapeAddPlane)(b3SharedMemoryCommandHandle commandHandle, const double planeNormal[/*3*/], double planeConstant);
extern int (*b3wCreateCollisionShapeAddMesh)(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/]);
extern int (*b3wCreateCollisionShapeAddConvexMesh)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices);
extern int (*b3wCreateCollisionShapeAddConcaveMesh)(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices, const int* indices, int numIndices);
extern void (*b3wCreateCollisionSetFlag)(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, int flags);
extern void (*b3wCreateCollisionShapeSetChildTransform)(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double childPosition[/*3*/], const double childOrientation[/*4*/]);
extern b3SharedMemoryCommandHandle (*b3wCreatePoseCommandInit)(b3PhysicsClientHandle physClient, int bodyUniqueId);
extern int (*b3wCreatePoseCommandSetBasePosition)(b3SharedMemoryCommandHandle commandHandle, double startPosX, double startPosY, double startPosZ);
extern int (*b3wCreatePoseCommandSetBaseOrientation)(b3SharedMemoryCommandHandle commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);
extern int (*b3wCreatePoseCommandSetBaseLinearVelocity)(b3SharedMemoryCommandHandle commandHandle, const double linVel[/*3*/]);
extern int (*b3wCreatePoseCommandSetBaseAngularVelocity)(b3SharedMemoryCommandHandle commandHandle, const double angVel[/*3*/]);
extern int (*b3wCreatePoseCommandSetBaseScaling)(b3SharedMemoryCommandHandle commandHandle, double scaling[/* 3*/]);
extern b3SharedMemoryCommandHandle (*b3wRequestActualStateCommandInit)(b3PhysicsClientHandle physClient, int bodyUniqueId);
extern int (*b3wGetStatusActualState)(b3SharedMemoryStatusHandle statusHandle,
											 int* bodyUniqueId,
											 int* numDegreeOfFreedomQ,
											 int* numDegreeOfFreedomU,
											 const double* rootLocalInertialFrame[],
											 const double* actualStateQ[],
											 const double* actualStateQdot[],
											 const double* jointReactionForces[]);
extern b3SharedMemoryCommandHandle (*b3wApplyExternalForceCommandInit)(b3PhysicsClientHandle physClient);
extern void (*b3wApplyExternalForce)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[/*3*/], const double position[/*3*/], int flag);
extern void (*b3wApplyExternalTorque)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[/*3*/], int flag);
extern b3SharedMemoryCommandHandle (*b3wInitRemoveBodyCommand)(b3PhysicsClientHandle physClient, int bodyUniqueId);
extern int (*b3wGetNumBodies)(b3PhysicsClientHandle physClient);
extern b3SharedMemoryCommandHandle (*b3wInitChangeDynamicsInfo)(b3PhysicsClientHandle physClient);
extern int (*b3wChangeDynamicsInfoSetActivationState)(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int activationState);

void b3w_init(void);

#endif /* !BLW_H */
