#include "BulletSkeletalMeshComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"

// TODO: Lots of duplication here, have to move most of the code here into a base class and inherit

UBulletSkeletalMeshComponent::UBulletSkeletalMeshComponent()
{
}

void UBulletSkeletalMeshComponent::BeginPlay()
{
	Super::BeginPlay();
	AddOwnPhysicsAsset();
	UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::BeginPlay"));
}



void UBulletSkeletalMeshComponent::AddOwnPhysicsAsset()
{
	if (!BulletActor) {
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: Loaded wihout a bulletActor. Physics won't work."));
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
			// Access the information of the first box element
			FTransform ActorInverseTransform = GetOwner()->GetActorTransform().Inverse(); // Transform of the actor
			FTransform ComponentTransform = GetComponentTransform(); // Transform of the component
			FTransform LocalComponentTransform = ActorInverseTransform * ComponentTransform;
			FKBoxElem Box = BodySetup->AggGeom.BoxElems[0];;
			FVector Dimensions = FVector(Box.X, Box.Y, Box.Z) * LocalComponentTransform.GetScale3D();
			btCollisionShape* Shape = BulletActor -> GetBoxCollisionShape(Dimensions);
			BulletActor->AddRigidBody(GetOwner(),Shape,btVector3(),10,10,10);
			UE_LOG(LogTemp, Warning, TEXT("GOT"));
		}
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset:: Got empty physics asset"));
}
