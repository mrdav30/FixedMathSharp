# Feature Work Issue Tracker

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before implementing fixes, use
> superpowers:test-driven-development for runtime behavior changes, and use
> superpowers:verification-before-completion before claiming an issue is fixed.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** Active

**Goal:** Keep bugs, correctness risks, documentation defects, and
feature-work-discovered issues separate from feature design plans so each fix
can be triaged, tested, and committed independently.

**Architecture:** This document is intentionally undated. Each tracked item has
its own discovery date, source, status, affected files, and recommended
verification. Feature plans may reference this tracker instead of carrying bug
fixes inside API or design phases.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, xUnit,
BenchmarkDotNet when performance evidence is needed, FixedMathSharp core
runtime and tests.

---

## Tracker Rules

- Add new items when feature work uncovers a suspected bug, stale doc, test
  smell, performance anomaly, or correctness risk.
- Keep each item scoped tightly enough to fix and verify independently.
- Record the date on the item, not in this filename.
- Move an item to `Resolved Issues` only after the fix has tests or documented
  verification evidence.
- Do not use this tracker as a substitute for tests, benchmarks, or release
  notes.

## Active Issues

### FMS-Issue-008: `--no-restore` can reuse stale NuGet assets across OS or configuration changes

**Discovered:** 2026-06-04

**Status:** Active

**Source:** Phase 5 benchmark coverage verification.

**Observed behavior:**

A focused `dotnet test --no-restore` run failed before test execution because
`tests/FixedMathSharp.Tests/obj/project.assets.json` and generated NuGet props
referenced Windows fallback package folders such as
`C:\Program Files (x86)\Microsoft Visual Studio\Shared\NuGetPackages`.
Running the same focused command without `--no-restore` regenerated the assets
for WSL and the tests passed.

A later `Release --no-restore` benchmark build also failed after a
`ReleaseLean` restore because the benchmark project conditionally includes the
MemoryPack package only outside `ReleaseLean`. Running the `Release` build with
restore regenerated the package graph and succeeded.

Running `Release` and `ReleaseLean` benchmark builds in parallel can hit the
same shared `obj` assets race and leave one configuration seeing the other
configuration's package graph. Sequential forced restores/builds for both
configurations succeeded.

**Impact:**

Multi-OS or multi-configuration workspaces can produce noisy local verification
failures when a restore from one environment/configuration is followed by a
different `--no-restore` build/test loop. This is not a runtime defect, but it
can mislead agents or contributors during focused verification.

**Recommended next steps:**

- [ ] Reproduce from a clean Windows restore followed by WSL `dotnet test
  --no-restore`.
- [ ] Decide whether local verification docs should avoid `--no-restore` after
  switching OS environments or benchmark configurations with conditional
  package references.
- [ ] Check whether repo-level NuGet configuration can prevent generated assets
  from carrying machine-local fallback folders.
- [ ] Preserve fast `--no-restore` workflows when the restore and test happen
  under the same OS/environment.
- [ ] Avoid parallel restore/build invocations for configurations with
  different conditional package references unless the outputs/intermediate
  paths are isolated.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore -p:RunSettingsFilePath= --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug -p:RunSettingsFilePath= --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
```

### FMS-Issue-005: Coverlet can fail to restore instrumented assemblies on WSL

**Discovered:** 2026-06-03

**Status:** Active

**Source:** Phase 2 scalar/trigonometry allocation test verification.

**Observed behavior:**

Focused `dotnet test` runs passed, but the configured XPlat Code Coverage data
collector intermittently reported `UnauthorizedAccessException` while restoring
an instrumented `FixedMathSharp.dll` under `tests/FixedMathSharp.Tests/bin` on
WSL.

**Impact:**

The tests themselves pass, but coverage instrumentation can add noisy false
negatives or scary output during tight focused verification loops.

**Recommended next steps:**

- [ ] Reproduce with a clean bin/obj state and the default
  `coverlet.runsettings`.
- [ ] Decide whether focused local verification docs should recommend
  `-p:RunSettingsFilePath=` when coverage is not the goal.
- [ ] If the collector failure is repeatable, isolate whether it is caused by
  WSL file locking, generated packages, or concurrent build/test output.
- [ ] Preserve normal coverage behavior for CI-oriented runs.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore -p:RunSettingsFilePath=
```

### FMS-Issue-006: `ReleaseLean` `netstandard2.1` build emits MemoryPack shim conflict warnings

**Discovered:** 2026-06-03

**Status:** Active

**Source:** Phase 2 final verification of Unity-supporting `netstandard2.1`
builds.

**Observed behavior:**

`dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration
ReleaseLean --framework netstandard2.1 --no-restore` succeeds, but emits many
`CS0436` warnings where `MemoryPack.Disable.Shim.cs` types conflict with
imported `MemoryPack.Core` attribute types.

**Impact:**

The build exits successfully, but the lean build is noisy and may hide more
important warnings. The warning pattern also suggests the lean configuration is
still seeing MemoryPack assets in a target where the local shim is intended to
own those symbols.

**Recommended next steps:**

- [ ] Confirm whether the warning existed before the Phase 2 runtime changes.
- [ ] Inspect the `DisableMemoryPack` and package-reference conditions for
  `ReleaseLean` across `netstandard2.1` and `net8.0`.
- [ ] Preserve the public serialization attributes and package layout while
  removing duplicate MemoryPack type visibility.
- [ ] Verify both `Release` and `ReleaseLean` builds for `netstandard2.1` and
  `net8.0`.

Verification:

```bash
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release --framework netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean --framework netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release --framework net8.0 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean --framework net8.0 --no-restore
```

## Performance Investigation Queue

Performance issues should stay in the benchmark plan unless they become a
confirmed runtime defect. Current queue:

- None currently. The 2026-06-03 vector normalization and residual quaternion
  allocation investigation was completed in
  `docs/feature-work/done/2026-06-03-benchmark-hot-path-followups.md` Phase 3. The
  residual `1 B` signal did not reproduce in the expanded short-run
  diagnostics, so no runtime defect is currently tracked.

## Resolved Issues

### FMS-Issue-007: `FixedThrowHelper` was easy to misuse with eager message construction

**Discovered:** 2026-06-03

**Resolved:** 2026-06-03

**Source:** Phase 2 allocation fixes and follow-up review of throw-helper call
sites.

**Resolution:**

`FixedThrowHelper` has been removed. Runtime guard call sites now use direct
`if (...) throw ...` branches, so formatted diagnostics are only constructed on
exception paths. This keeps the code `netstandard2.1` friendly without adding a
custom interpolated string handler or a helper abstraction that can be
accidentally misused.

**Completed work:**

- [x] Added allocation regression tests for `FixedBoundFrustum.GetCorners`,
  `FixedBoundFrustum.GetPlanes`, and `FixedQuaternion.FromEulerAngles` valid
  paths.
- [x] Replaced helper calls with direct guard branches while preserving existing
  exception types and messages.
- [x] Removed `src/FixedMathSharp/Support/FixedThrowHelper.cs`.
- [x] Verified no `FixedThrowHelper` or `FixedMathSharp.Support` references
  remain.
- [x] Re-ran focused allocation tests.

Verification:

```bash
rg -n "FixedThrowHelper|FixedMathSharp\\.Support" src tests
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore -p:RunSettingsFilePath= --filter "FullyQualifiedName~GetPlanes_ArrayOverloadWithValidDestination_DoesNotAllocate|FullyQualifiedName~GetCorners_ArrayOverloadWithValidDestination_DoesNotAllocate|FullyQualifiedName~FixedQuaternion_FromEulerAngles_ValidInput_DoesNotAllocate"
```

Result on 2026-06-03: source scan found no helper references, and the focused
allocation tests passed.

### FMS-Issue-001: `FixedQuaternion.FromDirection(Vector3d.Backward)` returned identity

**Discovered:** 2026-06-03

**Resolved:** 2026-06-03

**Source:** Coordinate-convention review of `Vector3d.Forward`/`Backward` and
quaternion direction helpers.

**Resolution:**

`FixedQuaternion.FromDirection` now detects the anti-parallel zero-cross-axis
case and returns a deterministic 180-degree rotation around `Vector3d.Up`
instead of `FixedQuaternion.Identity`.

**Completed work:**

- [x] Added a failing test for `FixedQuaternion.FromDirection(Vector3d.Backward)`.
- [x] Added near-forward and near-backward tests so edge behavior is documented.
- [x] Preserved `FixedQuaternion.FromDirection(Vector3d.Forward) ==
  FixedQuaternion.Identity`.
- [x] Used a deterministic `Vector3d.Up` fallback axis for the anti-parallel
  case.
- [x] Re-ran focused quaternion tests.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedQuaternionTests"
```

Result on 2026-06-03: passed, 72 tests.

### FMS-Issue-002: `Vector3d.Direction` docs swapped pitch and yaw semantics

**Discovered:** 2026-06-03

**Resolved:** 2026-06-03

**Source:** Coordinate-convention review of vector direction helpers.

**Resolution:**

`Vector3d.Direction` behavior was confirmed as pitch on `X` and yaw on `Y`.
The XML documentation now matches that convention and calls out the existing
positive-pitch sign, which rotates canonical forward toward `Vector3d.Down`.
No additive API was needed for this pass.

**Completed work:**

- [x] Added tests for positive yaw, negative yaw, positive pitch, and negative
  pitch using canonical `+Z` forward expectations.
- [x] Preserved the existing zero-angle behavior test.
- [x] Confirmed the right fix was XML documentation only.
- [x] Preserved the existing public API.
- [x] Skipped benchmarking because no hot-path runtime behavior changed.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Vector3dTests"
```

Result on 2026-06-03: passed, 100 tests.

### FMS-Issue-003: Valid scalar/trigonometry fast paths allocated through eager guard messages

**Discovered:** 2026-06-03

**Resolved:** 2026-06-03

**Source:** Phase 2 scalar/trigonometry benchmark diagnostics.

**Resolution:**

`Fixed64.operator /`, `FixedMath.Asin`, and `FixedMath.Acos` now construct
diagnostic exception messages only after their guard condition is true. This
preserves existing error-path behavior while removing valid-path string
allocation inherited by scalar arithmetic, trigonometry, logarithm, and
quaternion/matrix callers.

**Completed work:**

- [x] Split scalar/trigonometry benchmarks so allocation sources could be
  isolated by operation.
- [x] Added allocation regression tests for valid division, `Asin`, and `Acos`.
- [x] Preserved existing exception types and diagnostic messages for invalid
  inputs.
- [x] Re-ran focused tests and short fixed64 arithmetic benchmarks.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Divide_ValidDivisor_DoesNotAllocate|FullyQualifiedName~Asin_ValidInput_DoesNotAllocate|FullyQualifiedName~Acos_ValidInput_DoesNotAllocate"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic -j Short -i
```

Result on 2026-06-03: focused allocation tests passed, and the short focused
benchmark reported no managed allocations for all 13 scalar/trigonometry
benchmarks.

### FMS-Issue-004: `FixedMath.Pow10Lookup` allocated a new array per access

**Discovered:** 2026-06-03

**Resolved:** 2026-06-03

**Source:** Phase 2 lookup/span allocation review.

**Resolution:**

`FixedMath.Pow10Lookup` keeps the public `ReadOnlySpan<int>` API but now returns
a span over a single static backing array instead of creating a fresh array on
every property access.

**Completed work:**

- [x] Added an allocation regression test for `Pow10Lookup` access.
- [x] Preserved the existing lookup values, length, and public API shape.
- [x] Re-ran the focused allocation test.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Pow10Lookup_Access_DoesNotAllocate"
```

Result on 2026-06-03: focused allocation test passed.
