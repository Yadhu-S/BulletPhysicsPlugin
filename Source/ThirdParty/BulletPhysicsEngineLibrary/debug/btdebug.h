#pragma once
#include "CoreMinimal.h"
#include "BulletPhysicsEngineLibrary/BulletMinimal.h"
#include "BulletPhysicsEngineLibrary/debug/btdebug.h"
#include "BulletPhysicsEngineLibrary/src/bthelper.h"
#include "DrawDebugHelpers.h"

class BulletDebugDraw : public btIDebugDraw
{
	protected:
		UWorld* World;
		FVector WorldOrigin;
		int DebugMode;

	public:
		BulletDebugDraw(UWorld* world, const FVector& worldOrigin)
			: World(world), WorldOrigin(worldOrigin), DebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE)
		{}

		virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override
		{
			DrawDebugLine(World,
					BulletHelpers::ToUEPos(from, WorldOrigin),
					BulletHelpers::ToUEPos(to, WorldOrigin),
					BulletHelpers::ToUEColour(color));
		}

		virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance,
				int lifeTime, const btVector3& color) override
		{
			drawLine(PointOnB, PointOnB + normalOnB * distance, color);
			btVector3 ncolor(3, 0, 0);
			drawLine(PointOnB, PointOnB + normalOnB * 0.01, ncolor);
		}

		virtual void reportErrorWarning(const char* warningString) override
		{
			UE_LOG(LogTemp, Warning, TEXT("BulletDebugDraw: %hs"), warningString);
		}

		virtual void draw3dText(const btVector3& location, const char* textString) override
		{
			// Your implementation for draw3dText goes here
		}

		virtual void setDebugMode(int debugMode) override
		{
			DebugMode = debugMode;
		}

		virtual int getDebugMode() const override
		{
			return DebugMode;
		}
};

