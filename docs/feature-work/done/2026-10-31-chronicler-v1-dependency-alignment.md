# FMS-Issue-037: Chronicler v1 dependency alignment

## Resolution

Aligned the remaining Chronicler source identities and the math core's Lean shim
package reference to v1.0.0. Core, companion, and test projects now request the
same `Chronicler` and shim assemblies. Runtime C# code, recorded schemas, hash
ordering, CI dependency selection, and the immutable sibling pin are unchanged.

## Failure and root cause

At FixedMathSharp revision `b4a489de5ae5ba178a5970499e7d90f7847fe0f9`,
[develop CI run 41227903957](https://github.com/mrdav30/FixedMathSharp/actions/runs/41227903957)
built successfully on Windows and Linux in Release and ReleaseLean. Each lane
then failed 27 of the 49 companion tests with:

```text
System.IO.FileNotFoundException: Could not load file or assembly
'Chronicler, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null'.
```

The version bump updated `src/FixedMathSharp.Chronicler` to 1.0.0, but its test
project still explicitly built Chronicler with
`SemVer=0.4.0;AssemblySemVer=0.4.0.0`. The two references compiled the same
source with different assembly identities into shared sibling output paths. The
test output contained `Chronicler.dll` version **0.4.0.0**, while the companion
required **1.0.0.0**. Finding a DLL with the right filename did not satisfy that
reference. A warning-free build therefore did not establish that the test host
could load its dependencies.

The same incomplete migration left the core and core tests explicitly building
the Lean shim as 0.4.0, conflicting with the companion's 1.0.0 reference. The
core's package-mode shim dependency also remained at 0.4.0. The fix changes
these four values across three project files; it retains the existing direct
source references and global transitive-reference settings.

The older
[main run 41213557027](https://github.com/mrdav30/FixedMathSharp/actions/runs/41213557027)
failed earlier during restore with `NU1101` for `Chronicler.Diagnostics`. All
four Chronicler 1.0.0 packages were available on nuget.org during this
investigation. That earlier publication-order failure is distinct from the
reproduced source assembly mismatch.

## Reproducer

Use the failing FixedMathSharp revision above with a sibling Chronicler checkout
at `c45f9ab53913e5b1692f0717733425e0f4defeb2` (the `v1.0.0` release and CI pin).
From the FixedMathSharp repository root:

```powershell
dotnet build FixedMathSharp.slnx -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false
dotnet test tests/FixedMathSharp.Chronicler.Tests/FixedMathSharp.Chronicler.Tests.csproj -c Release --no-build --no-restore -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 --filter FullyQualifiedName~WriteFixed64_UsesRawFixedPointPayload
[System.Reflection.AssemblyName]::GetAssemblyName((Join-Path (Get-Location) 'tests/FixedMathSharp.Chronicler.Tests/bin/Release/net10.0/Chronicler.dll')).FullName
```

Before the fix, the existing hash regression failed with the same load error and
the identity inspection returned 0.4.0.0. After the fix, the full companion
suite passes and the copied identity is 1.0.0.0. No new test of build machinery
was added.

## Verification

Validated on Windows x64 with .NET SDK 10.0.302 and runtime 10.0.10. All four
solution builds completed with zero warnings and errors; all tests passed:

| Dependency mode         | Configuration | Core tests | Companion tests |
| ----------------------- | ------------- | ---------: | --------------: |
| Pinned sibling source   | Release       |      7,257 |              49 |
| Pinned sibling source   | ReleaseLean   |      7,236 |              49 |
| Released NuGet packages | Release       |      7,257 |              49 |
| Released NuGet packages | ReleaseLean   |      7,236 |              49 |

Source validation used the build flags in the reproducer and full-solution
`dotnet test` with the same configuration/properties and
`--no-build --no-restore`. For package validation, both Boolean properties were
`false`. Restore selected only nuget.org, disabled the HTTP cache, and used a
fresh `artifacts/chronicler-v1-release-packages` directory through `--packages`;
build/test used that directory through `-p:RestorePackagesPath`.

The restored metadata confirmed that Core, Core.Lean, MemoryPackShim, and
Diagnostics 1.0.0 came from nuget.org. Both package-mode test outputs carried
Chronicler and Diagnostics assembly version 1.0.0.0; the source Lean output also
carried shim version 1.0.0.0. All six generated math package IDs, their two
target frameworks, symbol packages, portable PDBs, and Chronicler dependency
versions were checked. These validation packages use the normal local 0.0.0
fallback; no FixedMathSharp release version was selected or published.

Independent review found no actionable issue. Post-fix hosted Linux/Windows CI
has not run; no push was made. Coverage, benchmarks, and DocFX were not rerun
for dependency metadata changes. Ignored logs and package caches are
supplemental; this record contains the failure, revisions, commands, and
observations needed to reproduce the investigation.

Reopen if any lane still produces mismatched Chronicler/shim assembly versions
after all explicit references agree, or if the released package graph fails with
a clean nuget.org restore. Preserve the failing assembly identity and resolved
reference path before changing caches or runtime code.
