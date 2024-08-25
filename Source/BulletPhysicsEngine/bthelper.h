#pragma once
#include "BulletMain.h"
// Bullet scale is 1=1m, UE is 1=1cm
// So x100
#define BULLET_TO_WORLD_SCALE 100.f
#define WORLD_TO_BULLET_SCALE (1.f/BULLET_TO_WORLD_SCALE)

/**
 *
 */
class BulletHelpers
{

	public:
		static float ToUESize(btScalar Sz)
		{
			return Sz * BULLET_TO_WORLD_SCALE;
		}
		static btScalar ToBtSize(float Sz)
		{
			return Sz * WORLD_TO_BULLET_SCALE;
		}
		static btVector3 ToBtSize(FVector Sv)
		{
			// For clarity; this is for box sizes so no offset
			return ToBtDir(Sv);
		}
		static FVector ToUEPos(const btVector3& V, const FVector& WorldOrigin)
		{
			return FVector(V.x(), V.y(), V.z()) * BULLET_TO_WORLD_SCALE + WorldOrigin;
		}
		static btVector3 ToBtPos(const FVector& V, const FVector& WorldOrigin)
		{
			return btVector3(V.X - WorldOrigin.X, V.Y - WorldOrigin.Y, V.Z - WorldOrigin.Z) * WORLD_TO_BULLET_SCALE;
		}
		static FVector ToUEDir(const btVector3& V, bool AdjustScale = true)
		{
			if (AdjustScale)
				return FVector(V.x(), V.y(), V.z()) * BULLET_TO_WORLD_SCALE;
			else
				return FVector(V.x(), V.y(), V.z());
		}
		static btVector3 ToBtDir(const FVector& V, bool AdjustScale = true)
		{
			if (AdjustScale)
				return btVector3(V.X, V.Y, V.Z) * WORLD_TO_BULLET_SCALE;
			else
				return btVector3(V.X, V.Y, V.Z);
		}
		static FQuat ToUE(const btQuaternion& Q)
		{
			return FQuat(Q.x(), Q.y(), Q.z(), Q.w());
		}
		static btQuaternion ToBt(const FQuat& Q)
		{
			return btQuaternion(Q.X, Q.Y, Q.Z, Q.W);
		}
		static btQuaternion ToBt(const FRotator& r)
		{
			return ToBt(r.Quaternion());
		}
		static FColor ToUEColour(const btVector3& C)
		{
			return FLinearColor(C.x(), C.y(), C.z()).ToFColor(true);
		}

		static FTransform ToUE(const btTransform& T, const FVector& WorldOrigin)
		{
			const FQuat Rot = ToUE(T.getRotation());
			const FVector Pos = ToUEPos(T.getOrigin(), WorldOrigin);
			return FTransform(Rot, Pos);
		}
		static btTransform ToBt(const FTransform& T, const FVector& WorldOrigin)
		{
			return btTransform(
					ToBt(T.GetRotation()),
					ToBtPos(T.GetLocation(), WorldOrigin));
		}
};
