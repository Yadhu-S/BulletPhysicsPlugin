
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "BulletActor.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletSkeletalMeshComponent.generated.h"


UCLASS()
	class BULLETPHYSICSENGINE_API UBulletSkeletalMeshComponent: public USkeletalMeshComponent
{
	GENERATED_BODY()


	public:
		UBulletSkeletalMeshComponent();

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void LoadBulletActor(ABulletActor* bulletActor){BulletActor = bulletActor; AddOwnPhysicsAsset();};

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void BulletAddForce(FVector Force, FVector Location);

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void GetPhysicsState(FTransform& Transform, FVector& Velocity, FVector& AngularVelocity,FVector& Force);

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			float Mass = 2000.0f;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			float Friction=2.0f;

		UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
			float Restitution=0.5f;

	private:
		btRigidBody* BulletOwnerRigidBody;

		UPROPERTY()
			ABulletActor* BulletActor;

		void AddOwnPhysicsAsset();
	protected:

		virtual void BeginPlay() override;

	private:



};
