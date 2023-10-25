// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class BulletPhysicsEngineLibrary : ModuleRules
{
	public BulletPhysicsEngineLibrary(ReadOnlyTargetRules Target) : base(Target)
	{

		Type = ModuleType.External;

		bool bDebug = Target.Configuration == UnrealTargetConfiguration.Debug || Target.Configuration == UnrealTargetConfiguration.DebugGame;
		bool bDevelopment = Target.Configuration == UnrealTargetConfiguration.Development;

		string BuildFolder;
		string BuildSuffix;

		if (bDebug)
		{
			BuildFolder = "Debug";
			BuildSuffix = "_Debug";
		}
		else if (bDevelopment)
		{
			BuildFolder = "RelWithDebugInfo";
			BuildSuffix = "_RelWithDebugInfo";
		}
		else
		{
			BuildFolder = "Release";
			BuildSuffix = "";
		}

		string BuildPlatForm = "win64";
		string LibExtension = ".lib";
		string BuildPrefix="";

		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			BuildPlatForm = "linux";
			LibExtension = ".so";
			BuildPrefix = "lib";
			BuildSuffix = "";
		}

		// Library path
		string LibrariesPath = Path.Combine( ModuleDirectory, "lib", BuildPlatForm, BuildFolder);

		string[] libraryNames = { "BulletCollision", "BulletDynamics", "LinearMath" };

		foreach (string libraryName in libraryNames)
		{
			PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, BuildPrefix + libraryName + BuildSuffix + LibExtension));
		}

		// Include path (I'm just using the source here since Bullet has mixed src & headers)
		PublicIncludePaths.Add( Path.Combine( ModuleDirectory, "src" ) );
		PublicDefinitions.Add("WITH_BULLET_BINDING=1");

	}
}
