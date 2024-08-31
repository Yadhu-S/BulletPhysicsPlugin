#pragma once
#include "CoreMinimal.h"

// The most minimal include for Bullet as needed by headers for value types / subclasses

// This is needed to suppress some warnings that UE4 escalates that Bullet doesn't
THIRD_PARTY_INCLUDES_START
// This is needed to fix memory alignment issues
PRAGMA_PUSH_PLATFORM_DEFAULT_PACKING

#define BT_USE_DOUBLE_PRECISION
// Value types
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

// Core things we override
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btIDebugDraw.h"


#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

// Forward decl for everything else we use
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btDynamicsWorld;
class btCollisionShape;
class btBoxShape;
class btCapsuleShape;
class btConvexHullShape;
class btCompoundShape;
class btSphereShape;
class btRigidBody;
class btCollisionObject;
class btIDebugDraw;

PRAGMA_POP_PLATFORM_DEFAULT_PACKING
THIRD_PARTY_INCLUDES_END
