#include "BulletSkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Math/Rotator.h"
#include "PhysicsEngine/PhysicsAsset.h"

// TODO: Lots of duplication here, have to move most of the code here into a base class and inherit

UBulletSkeletalMeshComponent::UBulletSkeletalMeshComponent()
{
	// Disable physics simulation
	// We don't want UE to mess with the physics
	SetSimulatePhysics(false);

	// Set the physics simulation mode to None
	SetPhysicsLinearVelocity(FVector::ZeroVector);
	SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	SetPhysicsBlendWeight(0.0f);
	SetPhysicsMaxAngularVelocityInRadians(0.0f);
	SetAllPhysicsLinearVelocity(FVector::ZeroVector);

	// Optionally, disable collision if needed
	SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void UBulletSkeletalMeshComponent::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::BeginPlay"));
}



void UBulletSkeletalMeshComponent::AddOwnPhysicsAsset()
{
	if (BulletActor==nullptr) {
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: loaded wihout a global bulletActor, physics won't work"));
		return;
	}
	// Get the physics asset associated with the skeletal mesh
	UPhysicsAsset* PhysicsAsset = GetPhysicsAsset();
	if (!PhysicsAsset)
	{
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: could not get physics asset, configure physics asset in your editor"));
		return;
	}
	btCompoundShape* compoundShape = nullptr;
	btCollisionShape* Shape = nullptr;
	btVector3 inertia;
	FTransform LocalTransform = FTransform::Identity;
	FTransform FinalTransform = FTransform::Identity;
	// Iterate over the bodies in the physics asset
	for (const USkeletalBodySetup* bodySetup : PhysicsAsset->SkeletalBodySetups)
	{
		// if the total makes up more than 1, we have a compound shape configured in skeletalmeshcomponent
		if( bodySetup->AggGeom.BoxElems.Num() + bodySetup->AggGeom.SphereElems.Num() + bodySetup->AggGeom.SphylElems.Num() > 1){
			compoundShape = new btCompoundShape();
		}

		for (FKBoxElem box : bodySetup->AggGeom.BoxElems){
			FVector Dimensions = FVector(box.X, box.Y, box.Z) * box.GetTransform().GetScale3D();
			Shape = BulletActor -> GetBoxCollisionShape(Dimensions);
			if (compoundShape) {
				compoundShape->addChildShape(BulletHelpers::ToBt(box.GetTransform(),GetComponentLocation()), Shape);
				continue;
			}
			LocalTransform = box.GetTransform();
			FinalTransform = LocalTransform * GetComponentTransform();
		}

		// Why are capsules called "SphylElemes", I don't know, flies over my head I guess.
		for (FKSphylElem capsule : bodySetup->AggGeom.SphylElems){
			FVector scale = capsule.GetTransform().GetScale3D();
			// Capsules are in Z in UE, in Y in Bullet, so roll -90
			FTransform shapeXform(capsule.Rotation + FRotator(0, 0, -90), capsule.Center);
			capsule.SetTransform(shapeXform);
			Shape =BulletActor->GetCapsuleCollisionShape(capsule.Radius * scale.X, capsule.Length * scale.Z);
			if (compoundShape) {
				compoundShape->addChildShape(BulletHelpers::ToBt(capsule.GetTransform(),GetComponentLocation()), Shape);
				continue;
			}
			LocalTransform = capsule.GetTransform();
			FinalTransform = LocalTransform * GetComponentTransform();
		}


		for (FKSphereElem sphere : bodySetup->AggGeom.SphereElems){
			FVector scale = sphere.GetTransform().GetScale3D();
			Shape = BulletActor->GetSphereCollisionShape(sphere.Radius * scale.X) ;
			if (compoundShape) {
				compoundShape->addChildShape(BulletHelpers::ToBt(sphere.GetTransform(),GetComponentLocation()), Shape);
				continue;
			}
			LocalTransform = sphere.GetTransform();
			FinalTransform = LocalTransform * GetComponentTransform();
		}

		Shape->calculateLocalInertia(Mass, inertia);
		if (compoundShape) {
			Shape = compoundShape;
			FinalTransform = LocalTransform = GetComponentTransform();
		}
		BulletOwnerRigidBody = BulletActor->AddRigidBody(this, FinalTransform, LocalTransform, Shape, inertia, Mass, Friction, Restitution);
		UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
	}
}

void UBulletSkeletalMeshComponent::BulletAddForce(FVector Force, FVector Location)
{
	if (BulletOwnerRigidBody){
		BulletOwnerRigidBody->applyForce(BulletHelpers::ToBtDir(Force, true), BulletHelpers::ToBtPos(Location, GetComponentLocation()));
	}
}

void UBulletSkeletalMeshComponent::GetPhysicsState(FTransform& Transform, FVector& Velocity, FVector& AngularVelocity,FVector& Force)
{
	if (BulletOwnerRigidBody) {
		Transform= BulletHelpers::ToUE( BulletOwnerRigidBody->getWorldTransform(),GetComponentLocation()) ;
		Velocity = BulletHelpers::ToUEPos(BulletOwnerRigidBody->getLinearVelocity(), FVector(0,0,0));
		AngularVelocity = BulletHelpers::ToUEPos(BulletOwnerRigidBody->getAngularVelocity(), FVector(0));
		Force = BulletHelpers::ToUEPos(BulletOwnerRigidBody->getTotalForce(), GetComponentLocation());
	}
}
