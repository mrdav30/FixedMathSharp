# Capsule side/vertex contact witness

Date: 2026-09-21. Issue: FMS-Issue-022. Baseline: `d53bf28`.

Resolved after reproducing the defect, correcting the shared contact owner,
and validating core and downstream consumers. Removed from the active tracker.

## Reproduction and cause

The issue reproduced immediately against the committed baseline. For a capsule
centered at `(0,0)`, Right axis, axis length `4`, radius `1`, and triangle
`(1,1),(2,3),(0,3)`, `TryGetCenteredCapsuleConvexContacts` correctly returned
contact, normal `(0,1)`, and zero depth. Its capsule anchor was `(0,1)` while
the polygon anchor was `(1,1)`. Both should identify `(1,1)`.

The capsule support helper correctly chooses the side midpoint when a support
direction is perpendicular to the capsule axis: every point on that side is a
valid support. The contact fallback incorrectly treated that arbitrary support
as the paired witness. It projected the midpoint onto the polygon feature but
never resolved the capsule's axial position from that feature. This can give a
physics consumer the wrong contact moment arm.

Four permanent regressions failed before the runtime edit: positive and
negative axial offsets, each at tangency and at penetration depth `1/4`.
The failure was deterministic and did not require repeated crash investigations.

## Correction

`WideCenteredCapsule2dRelations.TryGetContacts` now recognizes a side support
using the exact selected contact axis. After selecting the polygon witness,
it projects that point onto the capsule's finite side. The parameter is clamped
against the conceptual endpoints before nearest-even narrowing. The resulting
anchor reuses the existing separate axial/radial terms and sub-lattice residual.
No world endpoint or contact point must be representable.

For `S = 2^32`, let `A` and `N` be the capsule-frame rotated local axis and
normal numerators over `S²`, and `P` the opposing point relative to the capsule
center, also over `S²`. With raw radius `r`, define:

```text
delta = S*P - r*N
signedLengthRaw = 2*dot(delta,A) / dot(A,A)
```

`delta` has denominator `S³`. The ratio gives twice the local axial distance in
raw units, matching the existing centered-axis anchor representation. Comparing
the numerator to `+/-axisLength.Raw * dot(A,A)` clamps without first rounding
half-lengths. The normalized input axis guarantees a nonzero denominator.
Relative point components are below `2^97`, rotated normalized-axis components
below `2^66`, and delta components below `2^131`; the doubled projection remains
below `2^200`. The existing Signed192/320/576 operations suffice.

Only the single-contact side case takes the added projection. End-cap, circle,
and existing two-contact paths retain their behavior. Classification, normal,
and penetration-depth selection are unchanged. The additional work is constant
time and allocation-free.

## Verification

The public regressions are in
[`CenteredCapsuleConvexWitness.Tests.cs`](../../../tests/FixedMathSharp.Tests/Geometry/Primitives/CenteredCapsuleConvexWitness.Tests.cs).
They cover both axial signs, tangency and penetration, both side endpoints,
common rotations, different origins/frames, unrepresentable world coordinates,
sub-epsilon axis lengths, and warmed allocation behavior.

Core Release and ReleaseLean coverage both remain **100% line and branch**,
including every branch of the new projection. A separate source review found
no defect in its scaling, width bounds, clamping, or non-side behavior.

Final full-suite results:

| Repository | Release | ReleaseLean |
| --- | ---: | ---: |
| FixedMathSharp core | 2,824 passed | 2,803 passed |
| FixedMathSharp Chronicler | 8 passed | 8 passed |
| Gravitas with the local stack | 4,062 passed | 4,007 passed |

Gravitas has a test-only regression in
`tests/Gravitas.Tests/Physics2D/CenteredCapsuleExactRelationTests.cs` checking that
both collider orders produce `(1,1)` on both shapes. Its runtime code and
dependency versions are unchanged; local-stack validation uses
`-p:UseLocalLsfStack=true`. Release the FixedMathSharp correction before validating
the published dependency graph.

Portable verification commands:

```sh
dotnet restore FixedMathSharp.slnx -p:Configuration=Release
dotnet test FixedMathSharp.slnx -c Release --no-restore
dotnet restore FixedMathSharp.slnx -p:Configuration=ReleaseLean
dotnet test FixedMathSharp.slnx -c ReleaseLean --no-restore

# From Gravitas:
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true
```

Core coverage adds `--collect "XPlat Code Coverage"` and
`--settings tests/FixedMathSharp.Tests/coverlet.runsettings` to its test command.
The original repro, source tests, arithmetic, and commands are durable evidence;
temporary reports and logs are supplemental.

## Benchmark

`CenteredCapsuleConvexBenchmarks.SideVertexContact` measures one full contact
query and returns the capsule anchor. Final setup asserts the correct witness.
The baseline used `d53bf28` plus this benchmark before that setup assertion,
because the old implementation returns the wrong point.

```sh
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll centered-capsule-convex --exporters json
```

The full out-of-process DefaultJob completed all five cases; the launcher and
all five child processes exited zero. MemoryDiagnoser reported **0 B/op** in
every case. Environment: Windows 11 build 26200.9457, Intel i7-9700K, SDK
10.0.302, .NET 8.0.29 x64 RyuJIT, workstation concurrent GC, BenchmarkDotNet
0.15.8.

| Query | Final mean |
| --- | ---: |
| Side overlap | 9.065 us |
| Rotated overlap | 10.240 us |
| Corner tangency | 8.213 us |
| Corner miss | 7.061 us |
| Side/vertex contact anchors | 8.327 us |

The side/vertex baseline was 7.460 us with an incorrect midpoint witness. The
correction adds about 0.867 us (12%) for this case on this host, with no managed
allocations. This is the measured cost of resolving the paired witness; the
minimum-translation path does not perform the new projection. These focused
query timings are not end-to-end physics measurements. No additional benchmark
replays were needed to validate this deterministic defect.
