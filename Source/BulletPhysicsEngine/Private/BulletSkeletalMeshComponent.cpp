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

	if (bManualInitialization)
	{
		UE_LOG(LogTemp, Log, TEXT("Using manual initialization"));
		return;
	}

	UBulletSubsystem* BulletSubsystem = GetWorld()->GetSubsystem<UBulletSubsystem>();
	if (BulletSubsystem == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("BulletSubSystem null, manual init required)"));
		return;
	}
	LoadBulletSubsystem(BulletSubsystem);

}



void UBulletSkeletalMeshComponent::AddOwnPhysicsAsset()
{
	if (BulletSubSystem==nullptr) {
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
	btCollisionShape* shape = nullptr;
	FTransform physicsAssetLocalTransform = FTransform::Identity;
	// Iterate over the bodies in the physics asset
	for (const USkeletalBodySetup* bodySetup : PhysicsAsset->SkeletalBodySetups)
	{
		// if the total makes up more than 1, we have a compound shape configured in skeletalmeshcomponent
		if( bodySetup->AggGeom.BoxElems.Num() + bodySetup->AggGeom.SphereElems.Num() + bodySetup->AggGeom.SphylElems.Num() > 1){
			compoundShape = new btCompoundShape();
		}

		for (FKBoxElem box : bodySetup->AggGeom.BoxElems){
			FVector dimensions = FVector(box.X, box.Y, box.Z) * box.GetTransform().GetScale3D();
			shape = BulletSubSystem -> GetBoxCollisionShape(dimensions);
			if (compoundShape) {
				compoundShape->addChildShape(BulletHelpers::ToBt(box.GetTransform(),GetComponentLocation()), shape);
				continue;
			}
			physicsAssetLocalTransform = box.GetTransform();
		}

		// Why are capsules called "SphylElemes", I don't know, flies over my head I guess.
		for (FKSphylElem capsule : bodySetup->AggGeom.SphylElems){
			FVector scale = capsule.GetTransform().GetScale3D();
			// Capsules are in Z in UE, in Y in Bullet, so roll -90
			FTransform shapeXform(capsule.Rotation + FRotator(0, 0, -90), capsule.Center);
			capsule.SetTransform(shapeXform);
			shape =BulletSubSystem->GetCapsuleCollisionShape(capsule.Radius * scale.X, capsule.Length * scale.Z);
			if (compoundShape) {
				compoundShape->addChildShape(BulletHelpers::ToBt(capsule.GetTransform(),GetComponentLocation()), shape);
				continue;
			}
			physicsAssetLocalTransform = capsule.GetTransform();
		}


		for (FKSphereElem sphere : bodySetup->AggGeom.SphereElems){
			FVector scale = sphere.GetTransform().GetScale3D();
			shape = BulletSubSystem->GetSphereCollisionShape(sphere.Radius * scale.X) ;
			if (compoundShape) {
				compoundShape->addChildShape(BulletHelpers::ToBt(sphere.GetTransform(),GetComponentLocation()), shape);
				continue;
			}
			physicsAssetLocalTransform = sphere.GetTransform();
		}

		if (compoundShape) {
			shape = compoundShape;
		}
		BulletOwnerRigidBody = BulletSubSystem->AddRigidBody(this, physicsAssetLocalTransform, shape, Mass, Friction, Restitution);
		UE_LOG(LogTemp, Log, TEXT("UBulletSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
	}
}

bool UBulletSkeletalMeshComponent::BulletIsReady(){
	if (!BulletOwnerRigidBody) {
		return false;
	}
	return true;
}

void UBulletSkeletalMeshComponent::BulletAddForceAtLocation(FVector Force, FVector Location)
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletAddForce: Owning body is null. Unable to add force"));
		return;
	}
	BulletOwnerRigidBody->applyForce(BulletHelpers::ToBtDir(Force, true), BulletHelpers::ToBtPos(Location, FVector(0)));
}

void UBulletSkeletalMeshComponent::BulletAddImpulseAtLocation(FVector Impulse, FVector Location)
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletAddImpulseAtLocation: Owning body is null. Unable to add impulse"));
		return;
	}

	BulletOwnerRigidBody->applyImpulse(BulletHelpers::ToBtDir(Impulse, true), BulletHelpers::ToBtPos(Location, FVector(0)));
}

FVector UBulletSkeletalMeshComponent::BulletGetCentreOfMass()
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletGetCentreOfMass: Owning body is null.Unable to get Center of mass"));
		return FVector(0);
	}
	return BulletHelpers::ToUEPos(BulletOwnerRigidBody->getCenterOfMassPosition(),FVector(0));
}

void UBulletSkeletalMeshComponent::BulletAddCentralImpulse(FVector Impulse)
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletAddCentralImpulse: Owning body is null. Unable to add impulse"));
		return;
	}
	return BulletOwnerRigidBody->applyCentralImpulse(BulletHelpers::ToBtDir(Impulse, true));
}

void UBulletSkeletalMeshComponent::BulletSetCenterOfMass(FTransform CentreOfMass)
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletSetCenterOfMass: Owning body is null. Unable to set Centre of mass"));
		return;
	}
	BulletOwnerRigidBody->setCenterOfMassTransform(BulletHelpers::ToBt(CentreOfMass,GetComponentLocation()));
}

void UBulletSkeletalMeshComponent::BulletSetDamping(float LinearDamping, float AngularDamping)
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletSetDamping: Owning body is null. Unable to set damping"));
		return;
	}
	BulletOwnerRigidBody->setDamping(LinearDamping, AngularDamping);
}

void UBulletSkeletalMeshComponent::BulletSetWorldTransform(FTransform WorldTransform)
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletSetCenterOfMass: Owning body is null. Unable to set world transform"));
		return;
	}
	BulletOwnerRigidBody->setWorldTransform(BulletHelpers::ToBt(WorldTransform,FVector(0)));
}

float UBulletSkeletalMeshComponent::BulletGetBodyMass() 
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::GetMass: Owning body is null."));
		return 0.0f;
	}
	return BulletOwnerRigidBody->getMass();
}

FVector UBulletSkeletalMeshComponent::BulletGetVelocityAt(FVector LocationLS) 
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletGetVelocityAt: Owning body is null."));
		return FVector(0);
	}
	return BulletHelpers::ToUEPos(
			BulletOwnerRigidBody->getVelocityInLocalPoint(BulletHelpers::ToBtPos(LocationLS, FVector(0))),
			FVector(0));
}

void UBulletSkeletalMeshComponent::BulletSetMass(float NewMass) 
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletSetMass: Owning body is null."));
		return ;
	}
	btVector3 inertia(0,0,0);
	BulletOwnerRigidBody->getCollisionShape()->calculateLocalInertia(NewMass, inertia);
	BulletOwnerRigidBody->setMassProps(NewMass,inertia);
}

void UBulletSkeletalMeshComponent::BulletApplyTorqueImpulse(FVector torque) 
{
	if (!BulletOwnerRigidBody){
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::BulletApplyTorqueTurnImpulse: Owning body is null."));
		return ;
	}
	BulletOwnerRigidBody->applyTorqueImpulse(BulletHelpers::ToBtDir(torque, true));
}

void UBulletSkeletalMeshComponent::GetPhysicsState(FTransform& Transform, FVector& Velocity, FVector& AngularVelocity,FVector& Force)
{
	if (!BulletOwnerRigidBody) {
		UE_LOG(LogTemp, Warning, TEXT("UBulletSkeletalMeshComponent::GetPhysicsState: Owning body is null. Unable to fetch physics state"));
		return;
	}
	Transform= BulletHelpers::ToUE( BulletOwnerRigidBody->getWorldTransform(),FVector(0)) ;
	Velocity = BulletHelpers::ToUEPos(BulletOwnerRigidBody->getLinearVelocity(), FVector(0));
	AngularVelocity = BulletHelpers::ToUEPos(BulletOwnerRigidBody->getAngularVelocity(), FVector(0));
	Force = BulletHelpers::ToUEPos(BulletOwnerRigidBody->getTotalForce(), GetComponentLocation());
}


