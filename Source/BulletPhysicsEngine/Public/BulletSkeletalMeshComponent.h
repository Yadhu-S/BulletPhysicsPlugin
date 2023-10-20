
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "BulletActor.h"
#include "BulletSkeletalMeshComponent.generated.h"


UCLASS()
	class BULLETPHYSICSENGINE_API UBulletSkeletalMeshComponent: public USkeletalMeshComponent
{
	GENERATED_BODY()


	public:
		UBulletSkeletalMeshComponent();

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void SetBulletActor(ABulletActor* bulletActor){BulletActor = bulletActor;};

		UFUNCTION(BlueprintCallable, Category = "Bullet Physics|Objects")
			void AddOwnPhysicsAsset();

	private:
		UPROPERTY()
			ABulletActor* BulletActor;

	protected:

		virtual void BeginPlay() override;

	private:



};
