#include "BulletSkeletalMeshComponent.h"
#include "Math/Rotator.h"
#include "PhysicsEngine/PhysicsAsset.h"

// TODO: Lots of duplication here, have to move most of the code here into a base class and inherit

UBulletSkeletalMeshComponent::UBulletSkeletalMeshComponent()
{
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
	if (PhysicsAsset)
	{
		// Iterate over the bodies in the physics asset
		for (const USkeletalBodySetup* bodySetup : PhysicsAsset->SkeletalBodySetups)
		{
			for (FKBoxElem box : bodySetup->AggGeom.BoxElems){
				FVector Dimensions = FVector(box.X, box.Y, box.Z) * box.GetTransform().GetScale3D();
				btCollisionShape* Shape = BulletActor -> GetBoxCollisionShape(Dimensions);
				btVector3 inertia;
				Shape->calculateLocalInertia(2000, inertia);
				BulletOwnerRigidBody = BulletActor->AddRigidBody(
						this,
						box.GetTransform() * GetComponentTransform(),
						box.GetTransform(),
						Shape,
						inertia,
						2000,
						2,
						0);

				// For now, I need only 1 box. TODO: Make this general purpose
				break;
			}
			UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
		}
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset:: got empty physics asset"));
}

void UBulletSkeletalMeshComponent::BulletAddForce(FVector Force, FVector Location)
{
	if (BulletOwnerRigidBody){
		BulletOwnerRigidBody->applyForce(BulletHelpers::ToBtDir(Force, true), BulletHelpers::ToBtPos(Location, GetComponentLocation()));
	}
}

float UBulletSkeletalMeshComponent::BulletGetHorizontalVelocity()
{
	if (BulletOwnerRigidBody){
		FVector linearVelocity = BulletHelpers::ToUEDir(BulletOwnerRigidBody->getLinearVelocity(),true);
		linearVelocity.Z=0.0f;
		return linearVelocity.Size();
	}
	return 0.0f;
}
