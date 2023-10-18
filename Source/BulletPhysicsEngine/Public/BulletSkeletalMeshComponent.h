
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PhysicsEngine/BodySetup.h"
#include "BulletPhysicsEngineLibrary/BulletMinimal.h"
#include "BulletPhysicsEngineLibrary/src/bthelper.h"
#include "BulletPhysicsEngineLibrary/src/motionstate.h"
#include "BulletPhysicsEngineLibrary/src/BulletMain.h"
#include "BulletPhysicsEngineLibrary/debug/btdebug.h"
#include "Components/ShapeComponent.h"
#include <functional>
#include "BulletActor.h"
#include "GameFramework/Actor.h"
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
