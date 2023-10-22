#include "BulletSkeletalMeshComponent.h"
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
			// Access the information of the first box element
			FTransform ActorInverseTransform = GetOwner()->GetActorTransform().Inverse(); // Transform of the actor
			FTransform ComponentTransform = GetComponentTransform(); // Transform of the component
			FTransform LocalComponentTransform =  ComponentTransform* ActorInverseTransform ;
			FKBoxElem Box = BodySetup->AggGeom.BoxElems[0];;
			FVector Dimensions = FVector(Box.X, Box.Y, Box.Z) * LocalComponentTransform.GetScale3D();
			FTransform ShapeXForm(Box.Rotation,Box.Center);
			FTransform Xform = ShapeXForm * LocalComponentTransform;
			btCollisionShape* Shape = BulletActor -> GetBoxCollisionShape(Dimensions);
			if (Xform.EqualsNoScale(FTransform::Identity)) {
			
				UE_LOG(LogTemp, Warning, TEXT("IDENTITY"));
				
			}
			BulletActor->AddRigidBody(GetOwner(),Shape,btVector3(),1000,10,10);
			UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
		}
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset:: got empty physics asset"));
}
