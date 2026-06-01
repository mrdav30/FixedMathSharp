# Coverage 100 Battle Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:test-driven-development before production or test changes, and use superpowers:verification-before-completion before claiming a phase is complete.

**Goal:** Raise the `Debug` coverage run for `FixedMathSharp.slnx` to 100% line coverage and 100% branch coverage without hollow tests or band-aid coverage exclusions.

**Architecture:** Use the fresh Cobertura baseline from `tests/FixedMathSharp.Tests/coverlet.runsettings` as the target map. Add focused public-behavior tests for reachable branches, remove truly unreachable deterministic hot-path code only when the code path is mathematically or structurally dead, and keep the complexity register in sync after coverage/CRAP changes.

**Tech Stack:** .NET 8, xUnit v3, coverlet collector, ReportGenerator, `tests/FixedMathSharp.Tests/Support/FixedMathTestHelper.cs`.

---

## Baseline

Fresh command:

```bash
dotnet test FixedMathSharp.slnx --configuration Debug --collect:"XPlat Code Coverage" --results-directory tests/FixedMathSharp.Tests/TestResults/coverage-analysis/raw --settings tests/FixedMathSharp.Tests/coverlet.runsettings --verbosity normal
```

Result on 2026-06-01:

- Tests: 841 passed, 0 failed.
- Line coverage: 99.1% (4833 / 4875 lines, 42 uncovered).
- Branch coverage: 98.3% (1406 / 1430 branches, 24 missed).
- Method coverage: 98.8% (860 / 870 methods).
- CRAP hotspots above 30: 0.
- Generated report: `tests/FixedMathSharp.Tests/TestResults/coverage-analysis/reports/index.html`.

Final result on 2026-06-01 after Phases 1-5:

- Tests: 852 passed, 0 failed.
- Line coverage: 100% (4867 / 4867 lines).
- Branch coverage: 100% (1410 / 1410 branches).
- Method coverage: 100% (870 / 870 methods).
- CRAP hotspots above 30: 0.
- Raw coverage: `tests/FixedMathSharp.Tests/TestResults/coverage-analysis/raw/396aea61-1f28-4973-b3cd-55ea308271e6/coverage.cobertura.xml`.

## Phase 1: Reachable Low-Risk API Surface

**Files:**
- Modify: `tests/FixedMathSharp.Tests/Bounds/BoundingBox.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Bounds/BoundingSphere.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Fixed3x3.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Fixed4x4.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Vector2d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Vector3d.Tests.cs`

- [x] Add assertions for `BoundingBox.Scope`, default-struct `Vertices`, and contained-box intersection.
- [x] Add `BoundingSphere.Deconstruct` and `BoundingSphere.ToString` assertions in the existing sphere test file.
- [x] Add scalar overload coverage for `Fixed3x3.CreateScale(Fixed64)` and `Fixed4x4.CreateScale(Fixed64)`.
- [x] Add non-uniform scalar overload coverage for `Fixed4x4.CreateScale(Fixed64, Fixed64, Fixed64)`.
- [x] Add `Fixed3x3` inequality operator coverage.
- [x] Add static `Vector2d.CrossProduct` coverage.
- [x] Add `Vector3d` scalar subtraction and integer multiply operator coverage.

Target uncovered lines:

- `Geometry/Bounds/BoundingBox.cs`: 175, 497.
- `Geometry/Bounds/BoundingSphere.cs`: 427-430, 511-513.
- `Numerics/Matrices/Fixed3x3.cs`: 358-360, 731-733.
- `Numerics/Matrices/Fixed4x4.cs`: 542-544, 551-553.
- `Numerics/Vectors/Vector2d.cs`: 808-810.
- `Numerics/Vectors/Vector3d.cs`: 1184-1186, 1256-1258.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~BoundingBoxTests|FullyQualifiedName~BoundingSphereTests|FullyQualifiedName~Fixed3x3Tests|FullyQualifiedName~Fixed4x4Tests|FullyQualifiedName~Vector2dTests|FullyQualifiedName~Vector3dTests"
```

## Phase 2: Reachable Branch Hardening

**Files:**
- Modify: `tests/FixedMathSharp.Tests/Bounds/BoundingFrustum.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Bounds/FixedPlane.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Bounds/FixedRay.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Vector3d.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/DeterministicRandom.Tests.cs`

- [x] Add `BoundingFrustum.Contains(BoundingArea)` cases for fully contained and crossing areas.
- [x] Add `BoundingFrustum.Intersects(FixedRay)` case where the whole clipped interval is behind the ray.
- [x] Add invalid custom-plane frustum construction that triggers non-unique plane intersections.
- [x] Add `FixedPlane`/frustum classification where a frustum corner lies exactly on the plane.
- [x] Add ray-box cases for parallel axes below/above range and for an all-behind slab interval.
- [x] Add `FixedRay.Equals(FixedRay)` false-path assertion.
- [x] Add closest-line cases that drive `SolveClosestLineParameters` and `ClampSegmentParameter` through both clamp directions.
- [x] Add `Vector3d.ProjectOnPlane` zero-normal behavior.
- [x] Add a deterministic public RNG range case that exercises the `NextBounded` rejection loop without reflection.

Target partial branches:

- `Geometry/Bounds/BoundingFrustum.cs`: 240, 355, 532, 574, 590.
- `Geometry/Primitives/FixedPlane.cs`: 164.
- `Geometry/Primitives/FixedRay.cs`: 140, 153, 217.
- `Numerics/Vectors/Vector3d.cs`: 890, 898, 901, 1019.
- `Random/DeterministicRandom.cs`: 200.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~BoundingFrustumTests|FullyQualifiedName~FixedPlaneTests|FullyQualifiedName~FixedRayTests|FullyQualifiedName~Vector3dTests|FullyQualifiedName~DeterministicRandomTests"
```

## Phase 3: Dead-Code and Hot-Path Cleanup

**Files:**
- Modify: `src/FixedMathSharp/Core/FixedMath.Trigonometry.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/BoundingSphere.cs`
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- Modify: `src/FixedMathSharp/Numerics/Curves/FixedCurve.cs`
- Modify: `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- Modify: `src/FixedMathSharp/Random/DeterministicRandom.cs`
- Modify matching test files from Phases 1 and 2 as behavioral guardrails.

- [x] Remove the `Fixed64` multiplication `SHIFT_AMOUNT_I` runtime guard if confirmed unreachable because `SHIFT_AMOUNT_I` is a compile-time constant equal to 32.
- [x] Remove the `FixedMath.Tan` denominator convergence branch if confirmed ineffective because each continued-fraction iteration changes the term index by 2.
- [x] Remove the `BoundingSphere.CreateFromPointList` `distance == Fixed64.Zero` check if confirmed unreachable after `sqDistance > sqRadius`.
- [x] Replace the `FixedRay.Intersects(BoundingSphere)` negative-`t` fallback if confirmed unreachable after the outside/pointing-away guards.
- [x] Refactor `FixedCurve.Evaluate` to avoid the structurally unreachable fallback return while preserving sorted-keyframe behavior.
- [x] Remove the `DeterministicRandom` all-zero state repair if confirmed mathematically unreachable for two consecutive SplitMix64 outputs from one seed.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug
```

## Phase 4: Hollow-Test and Dead-Code Audit

**Files:**
- Review: `tests/FixedMathSharp.Tests/**/*.cs`
- Review: `src/FixedMathSharp/**/*.cs`

- [x] Check new tests for real assertions against observable public behavior.
- [x] Combine tests only when they exercise the same setup and adjacent branches without hiding the failure signal.
- [x] Avoid reflection-based coverage unless no public deterministic path exists and the branch is still worth preserving.
- [x] Prefer removing unreachable guards over adding synthetic coverage for impossible states.

## Phase 5: Full Verification and Documentation

**Files:**
- Modify: `docs/complexity-exceptions.md`
- Keep generated: `tests/FixedMathSharp.Tests/TestResults/coverage-analysis/coverage-analysis.md`

- [x] Re-run full coverage using the baseline command.
- [x] Re-run ReportGenerator and CRAP scripts.
- [x] Confirm line coverage is 100%.
- [x] Confirm branch coverage is 100%.
- [x] Confirm CRAP hotspots above 30 remain at 0.
- [x] Update `docs/complexity-exceptions.md` coverage percentages for changed methods.
- [x] Run the CI-equivalent release configurations:

```bash
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
```
