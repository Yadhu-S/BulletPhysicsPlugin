
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "BulletSubsystem.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "Delegates/DelegateCombinations.h"

#include "BulletSkeletalMeshComponent.generated.h"


UCLASS(Blueprintable, meta=(BlueprintSpawnableComponent))
	class BULLETPHYSICSENGINE_API UBulletSkeletalMeshComponent: public USkeletalMeshComponent
{
	GENERATED_BODY()


	public:
		UBulletSkeletalMeshComponent();

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void LoadBulletSubsystem(UBulletSubsystem* bulletSubsystem){BulletSubSystem = bulletSubsystem; AddOwnPhysicsAsset();};

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletAddForceAtLocation(FVector Force, FVector Location);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletAddCentralImpulse(FVector Impulse);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletAddImpulseAtLocation(FVector Impulse, FVector Location);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletApplyTorqueImpulse(FVector torque);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void GetPhysicsState(FTransform& Transform, FVector& Velocity, FVector& AngularVelocity,FVector& Force);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletSetWorldTransform(FTransform WorldTransform);

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Skeletal Mesh")
			float Mass = 2000.0f;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Skeletal Mesh")
			float Friction=2.0f;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Skeletal Mesh")
			float Restitution=0.5f;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			int bManualInitialization=false;

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletSetCenterOfMass(FTransform CentreOfMass);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			FVector BulletGetCentreOfMass();

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			bool BulletIsReady();

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			float BulletGetBodyMass();

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			FVector BulletGetVelocityAt(FVector LocationLS);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletSetDamping(float LinearDamping, float AngularDamping);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Skeletal Mesh")
			void BulletSetMass(float NewMass);
	private:

		btRigidBody* BulletOwnerRigidBody;

		UBulletSubsystem* BulletSubSystem;

		void AddOwnPhysicsAsset();
	protected:

		virtual void BeginPlay() override;


};
