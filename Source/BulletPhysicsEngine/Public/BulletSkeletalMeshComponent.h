
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PhysicsEngine/BodySetup.h"
#include "BulletPhysicsEngineLibrary/BulletMinimal.h"
#include "BulletPhysicsEngineLibrary/src/bthelper.h"
#include "BulletPhysicsEngineLibrary/src/motionstate.h"
#include "BulletPhysicsEngineLibrary/src/BulletMain.h"
#include "BulletPhysicsEngineLibrary/debug/btdebug.h"
#include "Components/ShapeComponent.h"
#include <functional>
#include "GameFramework/Actor.h"
#include "BulletSkeletalMeshComponent.generated.h"


UCLASS()
	class BULLETPHYSICSENGINE_API UBulletSkeletalMeshComponent: public USkeletalMeshComponent
{
	GENERATED_BODY()


	public:

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddStaticBody(AActor* player, float Friction, float Restitution,int &ID);

		btDiscreteDynamicsWorld* GetBulletWorld(){return BtWorld;};

	protected:

		virtual void BeginPlay() override;

	private:

		typedef const std::function<void(btCollisionShape* /*SingleShape*/, const FTransform& /*RelativeXform*/)>& PhysicsGeometryCallback;

		btCollisionObject* AddStaticCollision(btCollisionShape* Shape, const FTransform& Transform, float Friction, float Restitution, AActor* Actor);

		void ExtractPhysicsGeometry(AActor* Actor, PhysicsGeometryCallback CB);

		void ExtractPhysicsGeometry(UStaticMeshComponent* SMC, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

		void ExtractPhysicsGeometry(UShapeComponent* Sc, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

		void ExtractPhysicsGeometry(const FTransform& XformSoFar, UBodySetup* BodySetup, PhysicsGeometryCallback CB);

		btCollisionShape* GetBoxCollisionShape(const FVector& Dimensions);

		btCollisionShape* GetSphereCollisionShape(float Radius);

		btCollisionShape* GetCapsuleCollisionShape(float Radius, float Height);

		btCollisionShape* GetConvexHullCollisionShape(UBodySetup* BodySetup, int ConvexIndex, const FVector& Scale);

		TArray<btBoxShape*> BtBoxCollisionShapes;

		TArray<btSphereShape*> BtSphereCollisionShapes;

		TArray<btCapsuleShape*> BtCapsuleCollisionShapes;

		btSequentialImpulseConstraintSolver* mt;
		// Structure to hold re-usable ConvexHull shapes based on origin BodySetup / subindex / scale
		struct ConvexHullShapeHolder
		{
			UBodySetup* BodySetup;
			int HullIndex;
			FVector Scale;
			btConvexHullShape* Shape;
		};

		TArray<ConvexHullShapeHolder> BtConvexHullCollisionShapes;

		UBulletSkeletalMeshComponent();

		btCollisionConfiguration* BtCollisionConfig;

		btCollisionDispatcher* BtCollisionDispatcher;

		btBroadphaseInterface* BtBroadphase;

		btConstraintSolver* BtConstraintSolver;
		// Bullet Globals
		btDiscreteDynamicsWorld* BtWorld;
		// Dynamic Bodies
		TArray<btRigidBody*> BtRigidBodies;
		// Static colliders
		TArray<btCollisionObject*> BtStaticObjects;

};
