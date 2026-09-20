using System;
using System.Diagnostics;
using System.IO;
using UnrealBuildTool;

public class Box3DUnreal : ModuleRules
{
	private const string PrebuiltDirEnvVar = "BOX3D_PREBUILT_DIR";

	// Precision of a pre-built library ("double" or "float").
	private const string PrebuiltPrecisionEnvVar = "BOX3D_PREBUILT_PRECISION";

	// Escape hatch: full path to a cmake executable, ahead of every other candidate.
	private const string CMakeEnvVar = "BOX3D_CMAKE";

	public Box3DUnreal(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new[] { "Core" });
		PrivateDependencyModuleNames.AddRange(new[] { "CoreUObject", "Engine", "PhysicsCore", "InputCore" });

		// ThirdParty/ holds the wrapper CMakeLists.txt and the box3d submodule.
		string ThirdPartyPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty"));
		string Box3DPath = Path.Combine(ThirdPartyPath, "box3d");
		string SubmoduleIncludePath = Path.Combine(Box3DPath, "include");

		string Platform = GetPlatformFolder(Target);
		string LibFileName = (Target.Platform == UnrealTargetPlatform.Win64) ? "box3d.lib" : "libbox3d.a";

		string LibPath;
		string IncludePath;
		bool bDoublePrecision;

		string PrebuiltRoot = FindPrebuilt(ThirdPartyPath, Platform, LibFileName, out LibPath);
		if (PrebuiltRoot != null)
		{
			string PrebuiltIncludePath = Path.Combine(PrebuiltRoot, "include");
			IncludePath = Directory.Exists(PrebuiltIncludePath) ? PrebuiltIncludePath : SubmoduleIncludePath;
			if (!Directory.Exists(IncludePath))
			{
				throw new BuildException(
					"Box3DUnreal: pre-built box3d at {0} has no include/ folder and the box3d submodule is " +
					"missing. Add headers to the drop, or run: git submodule update --init --recursive",
					PrebuiltRoot);
			}

			bDoublePrecision = IsPrebuiltDoublePrecision();
			Console.WriteLine("Box3DUnreal: linking pre-built box3d ({0}, {1} precision)",
				LibPath, bDoublePrecision ? "double" : "single");
		}
		else
		{
			if (!Directory.Exists(SubmoduleIncludePath))
			{
				throw new BuildException(
					"Box3DUnreal: the box3d submodule is not checked out at {0}. Run " +
					"\"git submodule update --init --recursive\", or point {1} at a pre-built box3d.",
					Box3DPath, PrebuiltDirEnvVar);
			}

			IncludePath = SubmoduleIncludePath;
			bDoublePrecision = true;

			string BuildDir = Path.Combine(ThirdPartyPath, "Intermediate", Platform, "build");
			string InstallDir = Path.Combine(ThirdPartyPath, "Intermediate", Platform, "install");
			LibPath = Path.Combine(InstallDir, "lib", LibFileName);

			if (!File.Exists(LibPath))
			{
				BuildBox3D(ThirdPartyPath, BuildDir, InstallDir);
			}

			if (!File.Exists(LibPath))
			{
				throw new BuildException("Box3DUnreal: box3d build did not produce the expected library at " + LibPath);
			}
		}

		PublicIncludePaths.Add(IncludePath);

		PublicDefinitions.Add("BOX3D_DOUBLE_PRECISION=" + (bDoublePrecision ? "1" : "0"));

		PublicAdditionalLibraries.Add(LibPath);
	}

	private static string GetPlatformFolder(ReadOnlyTargetRules Target)
	{
		if (Target.Platform == UnrealTargetPlatform.Win64) return "Win64";
		if (Target.Platform == UnrealTargetPlatform.Mac)   return "Mac";
		if (Target.Platform == UnrealTargetPlatform.Linux) return "Linux";
		throw new BuildException("Box3DUnreal: unsupported platform " + Target.Platform);
	}

	private static string FindPrebuilt(string ThirdPartyPath, string Platform, string LibFileName, out string LibPath)
	{
		string FromEnv = Environment.GetEnvironmentVariable(PrebuiltDirEnvVar);
		if (!string.IsNullOrWhiteSpace(FromEnv))
		{
			string Root = Path.GetFullPath(FromEnv.Trim().Trim('"'));
			if (!TryFindLibrary(Root, LibFileName, out LibPath))
			{
				throw new BuildException(
					"Box3DUnreal: {0} is set to \"{1}\" but no {2} was found there (looked in <root>/lib and <root>).",
					PrebuiltDirEnvVar, Root, LibFileName);
			}
			return Root;
		}

		string LocalRoot = Path.Combine(ThirdPartyPath, "Prebuilt", Platform);
		if (TryFindLibrary(LocalRoot, LibFileName, out LibPath))
		{
			return LocalRoot;
		}

		LibPath = null;
		return null;
	}

	private static bool TryFindLibrary(string Root, string LibFileName, out string LibPath)
	{
		foreach (string Candidate in new[] { Path.Combine(Root, "lib", LibFileName), Path.Combine(Root, LibFileName) })
		{
			if (File.Exists(Candidate))
			{
				LibPath = Candidate;
				return true;
			}
		}

		LibPath = null;
		return false;
	}

	private static bool IsPrebuiltDoublePrecision()
	{
		string Value = Environment.GetEnvironmentVariable(PrebuiltPrecisionEnvVar);
		if (string.IsNullOrWhiteSpace(Value))
		{
			return true;
		}

		Value = Value.Trim().ToLowerInvariant();
		if (Value == "double") return true;
		if (Value == "float" || Value == "single") return false;

		throw new BuildException("Box3DUnreal: {0} must be \"double\" or \"float\", got \"{1}\".",
			PrebuiltPrecisionEnvVar, Value);
	}

	private void BuildBox3D(string SourceDir, string BuildDir, string InstallDir)
	{
		string CMake = FindCMake();

		Console.WriteLine("Box3DUnreal: building box3d via CMake ({0})", CMake);

		string ConfigureArgs = string.Format(
			"-S \"{0}\" -B \"{1}\" -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=\"{2}\"",
			SourceDir, BuildDir, InstallDir);

		Directory.CreateDirectory(BuildDir);
		if (!RunCMake(CMake, ConfigureArgs, SourceDir, bAllowFailure: true))
		{
			Console.WriteLine("Box3DUnreal: CMake configure failed; retrying with a clean build tree.");
			SafeDeleteDirectory(BuildDir);
			Directory.CreateDirectory(BuildDir);
			RunCMake(CMake, ConfigureArgs, SourceDir);
		}

		string BuildArgs = string.Format("--build \"{0}\" --config Release --target install", BuildDir);
		RunCMake(CMake, BuildArgs, SourceDir);
	}

	private static void SafeDeleteDirectory(string Dir)
	{
		try
		{
			if (Directory.Exists(Dir))
			{
				Directory.Delete(Dir, true);
			}
		}
		catch (Exception Ex)
		{
			Console.WriteLine("Box3DUnreal: could not clean {0}: {1}", Dir, Ex.Message);
		}
	}

	private bool RunCMake(string CMake, string Arguments, string WorkingDir, bool bAllowFailure = false)
	{
		ProcessStartInfo Info = new ProcessStartInfo(CMake, Arguments)
		{
			WorkingDirectory = WorkingDir,
			UseShellExecute = false,
			RedirectStandardOutput = true,
			RedirectStandardError = true,
			CreateNoWindow = true,
		};

		using (Process Proc = new Process())
		{
			Proc.StartInfo = Info;
			Proc.OutputDataReceived += (s, e) => { if (e.Data != null) Console.WriteLine("[box3d] " + e.Data); };
			Proc.ErrorDataReceived  += (s, e) => { if (e.Data != null) Console.WriteLine("[box3d] " + e.Data); };
			Proc.Start();
			Proc.BeginOutputReadLine();
			Proc.BeginErrorReadLine();
			Proc.WaitForExit();

			if (Proc.ExitCode != 0)
			{
				if (bAllowFailure)
				{
					return false;
				}
				throw new BuildException("Box3DUnreal: CMake failed (exit {0}) for: cmake {1}", Proc.ExitCode, Arguments);
			}
		}

		return true;
	}

	private string FindCMake()
	{
		string FromEnv = Environment.GetEnvironmentVariable(CMakeEnvVar);
		if (!string.IsNullOrWhiteSpace(FromEnv))
		{
			string Explicit = Path.GetFullPath(FromEnv.Trim().Trim('"'));
			if (!File.Exists(Explicit))
			{
				throw new BuildException("Box3DUnreal: {0} points at \"{1}\", which does not exist.", CMakeEnvVar, Explicit);
			}
			return Explicit;
		}

		foreach (string Candidate in GetEngineCMakeCandidates())
		{
			if (File.Exists(Candidate))
			{
				return Candidate;
			}
		}

		if (TryResolveFromPath("cmake", out string OnPath))
		{
			return OnPath;
		}

		if (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64)
		{
			string[] ProgramFilesRoots =
			{
				Environment.GetEnvironmentVariable("ProgramFiles") ?? @"C:\Program Files",
				Environment.GetEnvironmentVariable("ProgramFiles(x86)") ?? @"C:\Program Files (x86)",
			};
			string[] VsYears = { "2026", "2022" };
			string[] VsEditions = { "Community", "Professional", "Enterprise", "BuildTools" };

			foreach (string Root in ProgramFilesRoots)
			{
				foreach (string Year in VsYears)
				{
					foreach (string Edition in VsEditions)
					{
						string Candidate = Path.Combine(Root, "Microsoft Visual Studio", Year, Edition,
							"Common7", "IDE", "CommonExtensions", "Microsoft", "CMake", "CMake", "bin", "cmake.exe");
						if (File.Exists(Candidate))
						{
							return Candidate;
						}
					}
				}

				string Standalone = Path.Combine(Root, "CMake", "bin", "cmake.exe");
				if (File.Exists(Standalone))
				{
					return Standalone;
				}
			}
		}

		throw new BuildException(
			"Box3DUnreal: could not find CMake. A source build of the engine ships one at " +
			"Engine/Extras/ThirdPartyNotUE/CMake; launcher installs do not, so install CMake and add it " +
			"to PATH, install the \"C++ CMake tools\" component in Visual Studio, set " + CMakeEnvVar +
			", or link a pre-built box3d via " + PrebuiltDirEnvVar + ".");
	}

	// Where the engine keeps its bundled CMake, per host platform.
	private string[] GetEngineCMakeCandidates()
	{
		string Bundled = Path.Combine(EngineDirectory, "Extras", "ThirdPartyNotUE", "CMake");

		if (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Mac)
		{
			return new[]
			{
				Path.Combine(Bundled, "CMake.app", "Contents", "bin", "cmake"),
				Path.Combine(Bundled, "bin", "cmake"),
			};
		}

		if (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64)
		{
			return new[]
			{
				Path.Combine(Bundled, "bin", "cmake.exe"),
				Path.Combine(EngineDirectory, "Binaries", "ThirdParty", "CMake", "Win64", "bin", "cmake.exe"),
			};
		}

		return new[] { Path.Combine(Bundled, "bin", "cmake") };
	}

	private static bool TryResolveFromPath(string Executable, out string ResolvedPath)
	{
		ResolvedPath = null;
		bool bWindows = BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64;
		string[] Extensions = bWindows ? new[] { ".exe", ".cmd", ".bat", "" } : new[] { "" };

		string PathEnv = Environment.GetEnvironmentVariable("PATH");
		if (string.IsNullOrEmpty(PathEnv))
		{
			return false;
		}

		foreach (string Dir in PathEnv.Split(Path.PathSeparator))
		{
			if (string.IsNullOrWhiteSpace(Dir))
			{
				continue;
			}

			foreach (string Ext in Extensions)
			{
				string Candidate = Path.Combine(Dir.Trim(), Executable + Ext);
				if (File.Exists(Candidate))
				{
					ResolvedPath = Candidate;
					return true;
				}
			}
		}

		return false;
	}
}
