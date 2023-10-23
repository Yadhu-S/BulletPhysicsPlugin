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
		for (const USkeletalBodySetup* BodySetup : PhysicsAsset->SkeletalBodySetups)
		{
			// Check if there are no skeletal body setups
			if (BodySetup->AggGeom.BoxElems.Num() == 0)
			{
				continue;
			}
			FKBoxElem Box = BodySetup->AggGeom.BoxElems[0];;
			FVector Dimensions = FVector(Box.X, Box.Y, Box.Z) * Box.GetTransform().GetScale3D();
			btCollisionShape* Shape = BulletActor -> GetBoxCollisionShape(Dimensions);
			btVector3 inertia;
			Shape->calculateLocalInertia(2000, inertia);
			BulletActor->AddRigidBody(
					this,
					Box.GetTransform() * GetComponentTransform(),
					Box.GetTransform(),
					Shape,
					inertia,
					2000,
					2,
					0);

			UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
			break;
		}
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset:: got empty physics asset"));
}
