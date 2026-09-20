// Author: Antonio Lattanzio - emptyvessel

#include "Box3DCollisionData.h"
#include "Box3DStaticGeometry.h"
#include "Misc/PackageName.h"

#if WITH_EDITOR
#include "Engine/Level.h"
#include "HAL/FileManager.h"
#endif

FString UBox3DCollisionData::DeriveAssetPackageName(const FString& MapPackageName)
{
	const FString Path = FPackageName::GetLongPackagePath(MapPackageName); // /Game/Maps
	const FString Name = FPackageName::GetShortName(MapPackageName);       // Foo
	return FString::Printf(TEXT("%s/BC_%s"), *Path, *Name);
}

#if WITH_EDITOR

FString UBox3DCollisionData::ComputeSourceFingerprint(const FString& MapPackageName)
{
	FString MapFilename;
	if (!FPackageName::TryConvertLongPackageNameToFilename(
			MapPackageName, MapFilename, FPackageName::GetMapPackageExtension()))
	{
		return FString();
	}

	IFileManager& Files = IFileManager::Get();
	if (!Files.FileExists(*MapFilename))
	{
		return FString();
	}

	FDateTime Newest = Files.GetTimeStamp(*MapFilename);
	int64 TotalSize = Files.FileSize(*MapFilename);
	int32 FileCount = 1;

	const FString ExternalActorsPath = ULevel::GetExternalActorsPath(MapPackageName);
	FString ExternalActorsDir;
	if (!ExternalActorsPath.IsEmpty()
		&& FPackageName::TryConvertLongPackageNameToFilename(ExternalActorsPath, ExternalActorsDir)
		&& Files.DirectoryExists(*ExternalActorsDir))
	{
		Files.IterateDirectoryStatRecursively(*ExternalActorsDir,
			[&Newest, &TotalSize, &FileCount](const TCHAR*, const FFileStatData& Stat) -> bool
			{
				if (!Stat.bIsDirectory)
				{
					Newest = FMath::Max(Newest, Stat.ModificationTime);
					TotalSize += Stat.FileSize;
					++FileCount;
				}
				return true;
			});
	}

	return FString::Printf(TEXT("%lld|%lld|%d"), Newest.GetTicks(), TotalSize, FileCount);
}

bool UBox3DCollisionData::IsStale(FString& OutReason) const
{
	const FString CurrentVersion = Box3D::StaticGeometry::GetBox3DVersionString();
	if (!Box3DVersion.IsEmpty() && Box3DVersion != CurrentVersion)
	{
		OutReason = FString::Printf(TEXT("baked against box3d %s, now running %s"), *Box3DVersion, *CurrentVersion);
		return true;
	}

	if (SourceFingerprint.IsEmpty())
	{
		OutReason = TEXT("no source fingerprint recorded (baked before staleness checks existed)");
		return true;
	}

	const FString Current = ComputeSourceFingerprint(SourceLevel);
	if (Current.IsEmpty())
	{
		OutReason = FString::Printf(TEXT("source level '%s' not found on disk"), *SourceLevel);
		return true;
	}
	if (Current != SourceFingerprint)
	{
		OutReason = FString::Printf(TEXT("source level '%s' changed since the bake"), *SourceLevel);
		return true;
	}

	return false;
}

#endif // WITH_EDITOR
