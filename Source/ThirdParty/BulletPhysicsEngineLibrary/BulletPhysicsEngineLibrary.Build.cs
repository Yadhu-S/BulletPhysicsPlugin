// Fill out your copyright notice in the Description page of Project Settings.

using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using UnrealBuildTool;


// A bit messy, but it works

public class BulletPhysicsEngineLibrary : ModuleRules
{

	private bool BuildBullet(string BuildType)
	{

		string ThirdPartyBulletPath = Path.Combine(ModuleDirectory, "bullet3");
		string ModulePath = Path.Combine( ModuleDirectory, "bullet3");
		string BulletBuildDir = BuildUtils.GetBulletBuildDir(ModuleDirectory, Target.Platform);
		string LibOutputPath = Path.Combine(BulletBuildDir, BuildUtils.GetBuildType(BuildType));


		System.Console.WriteLine("Bullet thirdparty directory: " + ThirdPartyBulletPath);

		var cmakeOptions = "";
		cmakeOptions += " -DUSE_DOUBLE_PRECISION=1 "; 
		//Don't forget to take out the definition
		// #define BT_USE_DOUBLE_PRECISION in BulletMain.h if you disable DOUBLE_PRECISION
		// TODO: Too lazy to add it as a definition here.
		cmakeOptions += " -DINSTALL_LIBS=0 "; 
		cmakeOptions += " -DINSTALL_EXTRA_LIBS=0 "; 
		cmakeOptions += " -DLIBRARY_OUTPUT_PATH=\""+LibOutputPath + "\""; 
		cmakeOptions += " -DCMAKE_BUILD_TYPE="+BuildUtils.GetBuildType(BuildType);

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			cmakeOptions += " -DUSE_MSVC_RUNTIME_LIBRARY_DLL=1";
			cmakeOptions += " -DUSE_MSVC_RELEASE_RUNTIME_ALWAYS=1";
		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			//?
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			cmakeOptions += " -DCMAKE_POSITION_INDEPENDENT_CODE=1 ";
			cmakeOptions += " -DBUILD_SHARED_LIBS=0 "; 
			cmakeOptions += " -DCMAKE_CXX_COMPILER=/usr/bin/clang++ "; 
			cmakeOptions += " -DCMAKE_C_COMPILER=/usr/bin/clang "; 

		}else {
			System.Console.WriteLine("[ERROR] You are trying to build Bullet for a not yet supported platform: `" +
					Target.Platform);
			return false;
		}


		var generateCommand = "";
		generateCommand += BuildUtils.GetCMakeExe() + " ";
		generateCommand += " -S\"" + ThirdPartyBulletPath + "\" ";
		generateCommand += "-B\"" + BulletBuildDir + "\" ";
		generateCommand += cmakeOptions;
		var configureCode = BuildUtils.ExecuteCommandSync(generateCommand,Path.GetFullPath(ModulePath));
		if (configureCode != 0)
		{
			System.Console.WriteLine("Bullet lib configure CMake project failed with code: " + configureCode);
			return false;
		}

		var buildCommand = "";
		buildCommand += BuildUtils.GetCMakeExe() + " ";
		buildCommand += " --build \"" + BulletBuildDir + "\" ";
		buildCommand += " --target ";
		string[] libraryNames = { "BulletCollision", "BulletDynamics", "LinearMath" };
		foreach (string libraryName in libraryNames)
		{
			buildCommand += "" + libraryName + " ";
		}

		buildCommand += " -j " + System.Environment.ProcessorCount + " ";


		var buildExitCode = BuildUtils.ExecuteCommandSync (buildCommand, Path.GetFullPath(ModulePath));
		if (buildExitCode != 0)
		{
			System.Console.WriteLine("Bullet lib build failed with code: " + buildExitCode);
			return false;
		}

		return true;

	}


	public BulletPhysicsEngineLibrary(ReadOnlyTargetRules Target) : base(Target)
	{

		Type = ModuleType.External;

		bool bDebug = Target.Configuration == UnrealTargetConfiguration.Debug || Target.Configuration == UnrealTargetConfiguration.DebugGame;
		bool bDevelopment = Target.Configuration == UnrealTargetConfiguration.Development;

		string BuildFolder="";
		string BuildSuffix="";

		if (bDebug)
		{
			BuildFolder = "Debug";
			BuildSuffix = "_Debug";
			BuildBullet("Debug");
		}
		else if (bDevelopment)
		{
			BuildSuffix = "_RelWithDebInfo";
			BuildFolder = "RelWithDebInfo";
			BuildBullet("Development");
			if (Target.Platform == UnrealTargetPlatform.Win64)
			{
				//FIXME: I don't know, maybe...
				BuildFolder = Path.Combine("RelWithDebInfo","Debug");
				BuildSuffix = "_Debug";
			}
		}
		else
		{
			BuildFolder = "Release";
			BuildSuffix = "";
			BuildBullet("Release");
		}

		string BuildPlatForm = "Win64";
		string LibExtension = ".lib";
		string BuildPrefix = "";

		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			BuildPlatForm = "Linux";
			LibExtension = ".a";
			BuildPrefix = "lib";
			BuildSuffix = "";
		}

		// Library path
		string LibrariesPath = Path.Combine(ModuleDirectory, "lib", BuildPlatForm, BuildFolder);

		string[] libraryNames = { "BulletCollision", "BulletDynamics", "LinearMath" };

		foreach (string libraryName in libraryNames)
		{
			PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, BuildPrefix + libraryName + BuildSuffix + LibExtension));
		}

		// Include path (I'm just using the source here since Bullet has mixed src & headers)
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "bullet3/src"));
		PublicDefinitions.Add("WITH_BULLET_BINDING=1");

	}
}


// from UE4CMAKE:  https://github.com/caseymcc/UE4CMake/blob/main/Source/CMakeTarget.Build.cs
public class BuildUtils
{
	public static Tuple<string, string> GetExecuteCommandSync()
	{
		string cmd = "";
		string options = "";

		if ((BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64)
#if !UE_5_0_OR_LATER
				|| (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win32)
#endif//!UE_5_0_OR_LATER
		  )
		{
			cmd = "cmd.exe";
			options = "/c ";
		}
		else if (IsUnixPlatform(BuildHostPlatform.Current.Platform))
		{
			cmd = "bash";
			options = "-c ";
		}
		return Tuple.Create(cmd, options);
	}

	public static int ExecuteCommandSync(string Command, string MModulePath)
	{
		var cmdInfo = GetExecuteCommandSync();

		if (IsUnixPlatform(BuildHostPlatform.Current.Platform))
		{
			Command = " \"" + Command.Replace("\"", "\\\"") + " \"";
		}

		Console.WriteLine("Calling: " + cmdInfo.Item1 + " " + cmdInfo.Item2 + Command);

		var processInfo = new ProcessStartInfo(cmdInfo.Item1, cmdInfo.Item2 + Command)
		{
			CreateNoWindow = true,
			UseShellExecute = false,
			RedirectStandardError = true,
			RedirectStandardOutput = true,
			WorkingDirectory = MModulePath
		};

		StringBuilder outputString = new StringBuilder();
		Process p = Process.Start(processInfo);

		p.OutputDataReceived += (sender, args) => { outputString.Append(args.Data); Console.WriteLine(args.Data); };
		p.ErrorDataReceived += (sender, args) => { outputString.Append(args.Data); Console.WriteLine(args.Data); };
		p.BeginOutputReadLine();
		p.BeginErrorReadLine();
		p.WaitForExit();

		if (p.ExitCode != 0)
		{
			Console.WriteLine(outputString);
		}
		return p.ExitCode;
	}

	private static bool IsUnixPlatform(UnrealTargetPlatform Platform)
	{
		return Platform == UnrealTargetPlatform.Linux || Platform == UnrealTargetPlatform.Mac;
	}

	public static string GetCMakeExe()
	{
		string program = "cmake";

		if ((BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64)
#if !UE_5_0_OR_LATER
				|| (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win32)
#endif//!UE_5_0_OR_LATER
		  )
		{
			program += ".exe";
		}
		return program;
	}

	public static string GetBulletBuildDir(string ModuleDirectory, UnrealBuildTool.UnrealTargetPlatform Platform)
	{

		if (Platform == UnrealTargetPlatform.Win64)
		{
			return Path.Combine(ModuleDirectory, "lib", "Win64");
		}
		else if (Platform == UnrealTargetPlatform.Mac)
		{
			return Path.Combine(ModuleDirectory, "lib", "Mac");
		}
		else if (Platform == UnrealTargetPlatform.Linux)
		{
			return Path.Combine(ModuleDirectory, "lib", "Linux");
		}
		return "invalid platform";
	}

	public static string GetBuildType(string BuildType)
	{
		switch (BuildType)
		{
			case "Debug":
				return "Debug";
			case "Development":
				return "RelWithDebInfo";
		}
		return "Release";
	}

}

