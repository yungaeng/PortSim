// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class PortSim : ModuleRules
{
	public PortSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "ProceduralMeshComponent" });

		PrivateDependencyModuleNames.AddRange(new string[] { "Json", "UMG", "Slate", "SlateCore", "DesktopPlatform", "PhysicsCore", "Chaos" });

        // One authoritative reference in Document/STS; copy/stage it for packaged runs.
        string ProjectRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "../.."));
        string Reference = Path.GetFullPath(Path.Combine(ProjectRoot, "../Document/STS/STS_ReferenceData.json"));
        RuntimeDependencies.Add("$(ProjectDir)/Content/STS/STS_ReferenceData.json", Reference, StagedFileType.UFS);
        RuntimeDependencies.Add("$(ProjectDir)/Config/STS_Simulation.json", StagedFileType.NonUFS);
        RuntimeDependencies.Add("$(ProjectDir)/Config/Stowage_Planning.json", StagedFileType.NonUFS);
        RuntimeDependencies.Add("$(ProjectDir)/Config/Weather_Friction.json", StagedFileType.NonUFS);
        RuntimeDependencies.Add("$(ProjectDir)/Config/AGV_PhysicsBench.json", StagedFileType.NonUFS);
        RuntimeDependencies.Add("$(ProjectDir)/Config/AGV_EmergencyBench.json", StagedFileType.NonUFS);

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
