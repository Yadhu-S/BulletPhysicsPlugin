// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PhysicsEngine/BodySetup.h"
#include "BulletPhysicsEngine/bthelper.h"
#include "BulletPhysicsEngine/motionstate.h"
#include "BulletPhysicsEngine/BulletMain.h"
#include "BulletPhysicsEngine/btdebug.h"
#include "Components/ShapeComponent.h"
#include <functional>
#include "GameFramework/Actor.h"
#include "Subsystems/SubsystemCollection.h"
#include "Templates/Function.h"

#include "BulletSubsystem.generated.h"


DECLARE_DYNAMIC_DELEGATE_ThreeParams(FRayTestSingleCallback, const FVector&, To, const FVector&, From, bool&, HasHit);


UCLASS()
	class BULLETPHYSICSENGINE_API UBulletSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

	public:	

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddStaticBody(AActor* player, float Friction, float Restitution,int &ID);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddProcBody(AActor* Body,  float Friction, TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d, float Restitution, int& ID);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void UpdateProcBody(AActor* Body, float Friction, TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d, float Restitution, int& ID, int PrevID);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddRigidBody(AActor* Body, float Friction, float Restitution, int& ID,float mass);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void UpdatePlayertransform(AActor* player, int ID);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddImpulse(int ID, FVector Impulse, FVector Location);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddForce(int ID, FVector Impulse, FVector Location);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void StepPhysics(float deltaSeconds, int maxSubSteps = 1, float fixedTimeStep = 0.016666667f);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void SetPhysicsState(int ID, FTransform transforms, FVector Velocity, FVector AngularVelocity,FVector& Force);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void GetPhysicsState(int ID, FTransform& transforms, FVector& Velocity, FVector& AngularVelocity, FVector& Force);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void ResetSim();

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			bool DebugEnabled=true;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			FVector Gravity=FVector(0, 0, -9.8);

		// Input the fixed frame rate to calculate physics
		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			float PhysicsRefreshRate =60.0f;

		// This is independent of the frame rate in UE
		UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Bullet Physics|Objects")
			float PhysicsDeltaTime;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			int SubSteps=1;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			float RandVar;

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			float GetGravity(){ return Gravity.Z; };

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|RayCast")
			void RayTestSingle(FVector start, FVector end, int CheckObjectID, const FRayTestSingleCallback HitCallback);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Settings")
			void EnableDebugDrawer();

		void RayTestSingle(FVector Start, FVector End, int CheckObjectID, std::function<void(const FVector&, const FVector&, const bool&)> HitCallback);

		void RayTest(FVector Start, FVector End,std::function<void(const FVector&, const FVector&, const bool&)> HitCallback);

		virtual void Initialize(FSubsystemCollectionBase& Collection) override;

		virtual void Deinitialize() override;

	private:
		// Bullet section
		// Global objects

		btCollisionConfiguration* BtCollisionConfig;
		btCollisionDispatcher* BtCollisionDispatcher;
		btBroadphaseInterface* BtBroadphase;
		btConstraintSolver* BtConstraintSolver;
		btDiscreteDynamicsWorld* BtWorld;
		BulletHelpers* BulletHelpers;
		BulletDebugDraw* btdebugdraw;
		btStaticPlaneShape* plane;
		// Custom debug interface
		btIDebugDraw* BtDebugDraw;
		// Dynamic bodies
		// Static colliders
		TArray<btCollisionObject*> BtStaticObjects;
		btCollisionObject* procbody;
		// Re-usable collision shapes
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
		// These shapes are for *potentially* compound rigid body shapes
		struct CachedDynamicShapeData
		{
			FName ClassName; // class name for cache
			btCollisionShape* Shape;
			bool bIsCompound; // if true, this is a compound shape and so must be deleted
			btScalar Mass;
			btVector3 Inertia; // because we like to precalc this
		};
		TArray<CachedDynamicShapeData> CachedDynamicShapes;

		TArray<btRigidBody*> BtRigidBodies;

		float Accumulator = 0.0f;
	public:
		btDiscreteDynamicsWorld* GetBulletWorld(){return BtWorld;};

		void RayTestSingle(FVector Start, FVector End, int CheckObjectID, void (*HitCallback)(const FVector&, const FVector&));

		btCollisionShape* GetBoxCollisionShape(const FVector& Dimensions);

		btCollisionShape* GetSphereCollisionShape(float Radius);

		btCollisionShape* GetCapsuleCollisionShape(float Radius, float Height);

		btCollisionShape* GetTriangleMeshShape(TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d);

		btCollisionShape* GetConvexHullCollisionShape(UBodySetup* BodySetup, int ConvexIndex, const FVector& Scale);

		btRigidBody* AddRigidBody(AActor* Actor, const UBulletSubsystem::CachedDynamicShapeData& ShapeData, float Friction, float Restitution);

		btRigidBody* AddRigidBody(AActor* Actor, btCollisionShape* CollisionShape, btVector3 Inertia, float Mass, float Friction, float Restitution);

		btRigidBody* AddRigidBody(USkeletalMeshComponent* skel,FTransform localTransform, btCollisionShape* collisionShape, float Mass, float Friction, float Restitution);

		btCollisionObject* GetStaticObject(int ID);

	private:
		typedef const std::function<void(btCollisionShape* /*SingleShape*/, const FTransform& /*RelativeXform*/)>& PhysicsGeometryCallback;

		void SetupStaticGeometryPhysics(TArray<AActor*> Actors, float Friction, float Restitution);

		void ExtractPhysicsGeometry(AActor* Actor, PhysicsGeometryCallback CB);

		btCollisionObject* AddStaticCollision(btCollisionShape* Shape, const FTransform& Transform, float Friction, float Restitution, AActor* Actor);

		void ExtractPhysicsGeometry(UStaticMeshComponent* SMC, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

		void ExtractPhysicsGeometry(UShapeComponent* Sc, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

		void ExtractPhysicsGeometry(const FTransform& XformSoFar, UBodySetup* BodySetup, PhysicsGeometryCallback CB);

		const UBulletSubsystem::CachedDynamicShapeData& GetCachedDynamicShapeData(AActor* Actor, float Mass);

};
