// Copyright Epic Games, Inc. All Rights Reserved.

#include "BulletPhysicsEngine.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FBulletPhysicsEngineModule"

void FBulletPhysicsEngineModule::StartupModule()
{
	
}

void FBulletPhysicsEngineModule::ShutdownModule()
{
	
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FBulletPhysicsEngineModule, BulletPhysicsEngine)
