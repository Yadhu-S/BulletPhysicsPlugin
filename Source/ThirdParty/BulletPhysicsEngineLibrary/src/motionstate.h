#pragma once

#include "BulletPhysicsEngineLibrary/BulletMinimal.h"
#include "BulletPhysicsEngineLibrary/src/bthelper.h"
#include "CoreMinimal.h"
#include "Engine/Classes/Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

/**
 * Customised MotionState which propagates motion to linked Actor & tracks when sleeping
 */
class BULLETPHYSICSENGINE_API BulletCustomMotionState : public btMotionState
{
	protected:
		TWeakObjectPtr<AActor> Parent;
		// Bullet is made local so that all sims are close to origin
		// This world origin must be in *UE dimensions*
		FVector WorldOrigin;
		btTransform CenterOfMassTransform;


	public:
		BulletCustomMotionState()
		{

		}
		BulletCustomMotionState(AActor* ParentActor, const FVector& WorldCentre, const btTransform& CenterOfMassOffset = btTransform::getIdentity())
			: Parent(ParentActor), WorldOrigin(WorldCentre), CenterOfMassTransform(CenterOfMassOffset)

		{
		}

		///synchronizes world transform from UE to physics (typically only called at start)
		void getWorldTransform(btTransform& OutCenterOfMassWorldTrans) const override
		{
			if (Parent.IsValid())
			{
				auto&& Xform = Parent->GetActorTransform();
				OutCenterOfMassWorldTrans = BulletHelpers::ToBt(Parent->GetActorTransform(), WorldOrigin) * CenterOfMassTransform.inverse();
			}

		}

		///synchronizes world transform from physics to UE
		void setWorldTransform(const btTransform& CenterOfMassWorldTrans) override
		{// send this to actor
			if (Parent.IsValid(false))
			{
				btTransform GraphicTrans = CenterOfMassWorldTrans * CenterOfMassTransform;
				Parent->SetActorTransform(BulletHelpers::ToUE(GraphicTrans, WorldOrigin));
			}
		}
};


class BULLETPHYSICSENGINE_API BulletUEMotionState: public btMotionState
{
	protected:
		TWeakObjectPtr<USkeletalMeshComponent> Parent;
		// Bullet is made local so that all sims are close to origin
		// This world origin must be in *UE dimensions*
		FVector WorldOrigin;
		FTransform LocalTransform;
		btTransform CenterOfMassTransform;


	public:
		BulletUEMotionState()
		{

		}
		BulletUEMotionState(
				USkeletalMeshComponent* ParentActor,
				const FVector& WorldCentre,
				const FTransform& localTransform,
				const btTransform& CenterOfMassOffset = btTransform::getIdentity()
				):
			Parent(ParentActor),
			WorldOrigin(WorldCentre),LocalTransform(localTransform),
			CenterOfMassTransform(CenterOfMassOffset)
		{}

		///synchronizes world transform from UE to physics (typically only called at start)
		void getWorldTransform(btTransform& OutCenterOfMassWorldTrans) const override
		{
			if (Parent.IsValid())
			{
				OutCenterOfMassWorldTrans = BulletHelpers::ToBt(Parent->GetComponentTransform(), WorldOrigin)*CenterOfMassTransform.inverse();
			}
		}

		///synchronizes world transform from physics to UE
		void setWorldTransform(const btTransform& CenterOfMassWorldTrans) override
		{// send this to actor
			if (Parent.IsValid(false))
			{
				btTransform GraphicTrans = CenterOfMassWorldTrans * CenterOfMassTransform;
				Parent->SetWorldTransform(LocalTransform.Inverse()* BulletHelpers::ToUE(GraphicTrans, WorldOrigin));
			}
		}
};
