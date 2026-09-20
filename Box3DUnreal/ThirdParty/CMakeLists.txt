cmake_minimum_required(VERSION 3.22)

# Thin wrapper that lets Unreal's build pipeline (UnrealBuildTool ->
# Box3DUnreal.Build.cs) compile the vendored `box3d` submodule without editing
# any of its files. Box3DUnreal.Build.cs configures and builds this project on
# demand and links the resulting static library.
project(Box3DUnrealThirdParty LANGUAGES C CXX)

# Static library only. box3d gates its samples/tests/benchmarks/docs on
# PROJECT_IS_TOP_LEVEL, so adding it as a subdirectory here builds *only* the
# `box3d` library target.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# Double precision: translation is stored as double so coordinates stay accurate
# far from the origin (open-world sim), while rotation stays float. Determinism is
# preserved (IEEE-754 with no FMA contraction). box3d marks this PUBLIC on its
# target, but our consumer is UnrealBuildTool, not CMake, so Box3DUnreal.Build.cs
# defines BOX3D_DOUBLE_PRECISION on the module too. Both MUST agree or box3d's
# ABI-mismatch link guard (the b3CreateWorld macro) trips on purpose.
# Default only, not FORCE: configuring this project by hand with
# -DBOX3D_DOUBLE_PRECISION=OFF produces a single-precision library for a pre-built
# drop (tell the module with BOX3D_PREBUILT_PRECISION=float).
set(BOX3D_DOUBLE_PRECISION ON CACHE BOOL "Store box3d translation as double")

add_subdirectory(box3d)

if (MSVC)
	# box3d forces the static CRT (/MT) at its top level. Unreal links its
	# modules against the dynamic CRT (/MD), so override the runtime on the
	# imported target to avoid LNK4098 warnings and mismatched-CRT crashes.
	# Setting the target property after add_subdirectory() wins over the value
	# box3d seeded at target-creation time. The generator expression keeps the
	# debug/release CRT aligned with the build configuration.
	set_property(TARGET box3d PROPERTY
		MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()
