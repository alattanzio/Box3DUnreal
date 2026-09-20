// Author: Antonio Lattanzio - emptyvessel

#include "Box3DUnreal.h"
#include "Box3DLog.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogBox3D);

#define LOCTEXT_NAMESPACE "FBox3DUnrealModule"

void FBox3DUnrealModule::StartupModule()
{
	if (FParse::Param(FCommandLine::Get(), TEXT("DisableBox3D")))
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("box3d.Enabled")))
		{
			CVar->Set(TEXT("0"), ECVF_SetByCommandline);
		}
		UE_LOG(LogBox3D, Log, TEXT("box3d: -DisableBox3D on command line; simulation off (box3d.Enabled=0)."));
	}
}

void FBox3DUnrealModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FBox3DUnrealModule, Box3DUnreal)
