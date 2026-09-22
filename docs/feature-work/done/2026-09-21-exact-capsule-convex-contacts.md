# Exact capsule/convex contact axes

Date: 2026-09-21. Issue: FMS-Issue-020.

Resolved: exact closed-contact classification and depth ordering are repaired,
with core, downstream consumer, coverage, and repeated benchmark validation.

## Reproduction and cause

For polygon `(-1,-1),(0,-1),(0,0),(-1,0)`, capsule center `(3,4)`, Forward
axis, zero axis length, and radius `5`, the nearest corner is exactly tangent:
`3² + 4² = 5²`. The previous minimum-translation API returned false at scales
1, 10, 100, and 1000. The permanent
[`MinimumTranslation_ExactCornerTangencyReturnsZeroDepth`](../../../tests/FixedMathSharp.Tests/Geometry/Primitives/CenteredCapsuleConvexRelations.Tests.cs)
regression failed at all four scales before the runtime edit.

`WideCenteredCapsule2dRelations` normalized each candidate axis to `Vector2d`
before projecting the shapes, then treated that rounded vector as exactly unit
length when adding radial support. The rounded 3–4–5 direction projects the
center beyond the assumed radius. Wider products after normalization cannot
restore the discarded direction information. Polygon-edge axes had the same
problem, and closest-point clamping separately assumed the supplied capsule
direction had an exactly unit squared length.

The shared owner serves all centered capsule/polygon minimum-translation
overloads, capsule/polygon contacts, circle/polygon contacts, and the initial
overlap check in capsule/polygon sweeps. The independent upright-capsule
classification API was not the repair point.

## Exact classification and depth

The corrected owner retains unnormalized integer axis components throughout
classification, minimum-depth ordering, and polygon support-feature selection.
It considers polygon-edge axes, the nonzero capsule segment's perpendicular,
and the closest polygon-vertex/segment axis. Polygon edges retain priority on
exact depth ties, followed by the capsule side and the first closest authored
vertex.

Let `S = 2^32`, `D` be the supplied capsule direction's raw components, `L` its
raw axis length, and `A` an unnormalized candidate axis. Relative polygon points
are exact numerators over `S²`, including the fixed-point rotation products.
If their projection extrema on `A` are `min` and `max`, define:

```text
axialExtent = L * abs(dot(D, A))
p = min(axialExtent - 2*min, axialExtent + 2*max)
rawDepth = radius.Raw + p / (2*S*sqrt(dot(A, A)))
```

The sign of this depth is tested with signed squared comparisons. Negative
depth rejects separation before rounding; zero retains closed tangency. The
radius cancels when comparing two candidate depths, leaving a signed normalized
overlap comparison. No tolerance, radius inflation, or normalization-dependent
clamp participates in classification.

Closest-point clamping uses `dot(D,D)`, rather than assuming `S²`. Endpoint
distances and perpendicular interior distances share a rational denominator,
so selecting the closest vertex requires no rounded parameter or endpoint.

After selecting the winning axis, the normal is normalized once. Adjacent
integer bounds around the scaled square root bound the final depth; exact
midpoint comparisons resolve any straddled rounding boundary. The radius is
included before nearest-even rounding. Exact depth above the scalar maximum
sets `depthIsClamped`; exact `MaxValue` remains unclamped.

The full-domain width bounds are explicit: relative components are below
`2^97`, candidate components below `2^99`, signed overlaps below `2^199`, and
squared-overlap/axis comparison products below `2^597`. Existing `Signed192`,
`Signed320`, `Signed576`, and `Signed832` operations suffice. Runtime work remains
allocation-free and `O(n²)` for `n` polygon vertices. One rotation frame is
constructed per query and reused in the projection loops.

## Related corrections within the same axis path

Two independently reproduced gaps were repaired alongside the rounding cause:

- A horizontal capsule of length `2`, radius `0.5`, centered at zero against
  polygon `(0,-1),(1.5,0.25),(0,1),(-1.5,-0.25)` previously omitted its own side
  axis. It reported depth about `1.8416407865`; the true minimum is `1.5` along
  the vertical axis. The mandatory capsule perpendicular supplies this missing
  edge of the polygon's expansion by the capsule segment.
- A circle at `(0,2)`, radius `2`, against triangle `(-1,-1),(1,-1),(0,1)`
  selected an edge-derived axis whose opposite support is the vertex `(0,1)`.
  Choosing an adjacent edge produced a point near `(-0.4,0.2)`. A neighboring
  vertex joins the support feature only when its exact projection ties the
  extreme; otherwise the feature remains a point.

Both regressions were observed failing before their corrections.

## Validation

The
[`exactness tests`](../../../tests/FixedMathSharp.Tests/Geometry/Primitives/CenteredCapsuleConvexExactness.Tests.cs)
cover corner and oblique-face tangency, one-raw-unit radius misses and overlaps,
rotated capsule endpoints and polygon frames, unrepresentable world vertices,
nearest-even depth ties, irrational midpoint refinement, clamped versus exact
maximum depth, authored ties, and warmed zero-allocation execution.

An independent `BigInteger` oracle uses segment intersection and point-to-segment
distance inequalities, rather than the runtime SAT implementation. It agrees
with 1,200 seeded fixtures and 72 extreme-coordinate/rotation fixtures.

- FixedMathSharp Release solution: **2,810 core + 8 Chronicler tests passed**.
- FixedMathSharp ReleaseLean solution: **2,789 core + 8 Chronicler tests passed**.
- Core coverage: **100% line and branch coverage in both configurations**.
- Both runtime target frameworks, `netstandard2.1` and `net8.0`, build through
  the solution validation.
- Gravitas Release with `UseLocalLsfStack=true`: **4,060 tests passed**, with
  **100% line and branch coverage**.
- Gravitas ReleaseLean with `UseLocalLsfStack=true`: **4,005 tests passed**.

Seven downstream regressions were added to
`Gravitas/tests/Gravitas.Tests/Physics2D/CenteredCapsuleExactRelationTests.cs`.
Five fail against its published FixedMathSharp 7.1.0 dependency: exact circle
and capsule tangency, one-raw-unit overlap, and the query contact path. All seven
pass against the local correction; the two nearby-miss controls remain false.
Gravitas runtime code and dependency versions were not changed. Release the
FixedMathSharp correction before validating/updating the published Gravitas
dependency graph.

Re-run the normal solution commands from the relevant checkout:

```sh
dotnet restore FixedMathSharp.slnx -p:Configuration=Release
dotnet test FixedMathSharp.slnx -c Release --no-restore
dotnet restore FixedMathSharp.slnx -p:Configuration=ReleaseLean
dotnet test FixedMathSharp.slnx -c ReleaseLean --no-restore

# From the sibling Gravitas checkout:
dotnet test Gravitas.slnx -c Release -p:UseLocalLsfStack=true
dotnet test Gravitas.slnx -c ReleaseLean -p:UseLocalLsfStack=true
```

For core coverage, run the core test project with `--collect "XPlat Code Coverage"`
and `--settings tests/FixedMathSharp.Tests/coverlet.runsettings` in each
configuration. Gravitas uses its corresponding test project's runsettings.

## Performance verification

The focused `centered-capsule-convex` benchmark contains side overlap, rotated
overlap, exact corner tangency, and a one-raw-unit corner miss. Fixtures are
deterministic and use `MemoryDiagnoser`.

The pre-fix baseline used `8e03b14` with the benchmark added before its final
correctness setup assertion. That baseline incorrectly rejected tangency;
timing that row is not an equal-semantics speed comparison. The final setup
requires the corrected classifications and intentionally cannot pass against
the old implementation.

```sh
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll centered-capsule-convex --exporters json
```

The full out-of-process DefaultJob ran on Windows 11 (build 26200.9457),
Intel i7-9700K, SDK 10.0.302, .NET 8.0.29 x64 RyuJIT, workstation concurrent
GC, and BenchmarkDotNet 0.15.8. Each row measures one public query. Two fresh
final captures completed all four cases with launcher exit zero and all eight
child exits zero. MemoryDiagnoser reported **0 B/op** for every row in the
baseline and both final runs.

| Query | Before mean | Final mean, run 1 | Final mean, run 2 |
| --- | ---: | ---: | ---: |
| Side overlap | 13.761 us | 9.044 us | 9.084 us |
| Rotated overlap | 20.496 us | 10.315 us | 10.374 us |
| Exact corner tangency | 14.149 us (incorrect rejection) | 8.197 us | 8.382 us |
| One-raw-unit corner miss | 14.129 us | 7.126 us | 7.062 us |

Side overlap took about 34% less time and rotated overlap about 50% less time
in these captures. Reusing one rotation frame and projecting rotated offsets
directly removes repeated trigonometry and zero-origin world transformations.
These are focused query measurements on one host, not an end-to-end simulation
claim. The tangency row demonstrates the corrected behavior and is not used
as a like-for-like speed comparison.

## Separate follow-ups

Two separate findings were recorded during this correction:

- **FMS-Issue-021:** one intermediate out-of-process benchmark process threw a
  managed NRE through the value-type convex transform path. That failed run is
  excluded from performance evidence. Its input, source distinction, environment,
  managed stack, and command are retained in the [active tracker](../issue-tracker.md);
  the cause is unattributed.
  Neither final capture reproduced the crash, which does not establish its cause.
- **FMS-Issue-022:** a preexisting capsule side/vertex fallback can pair different
  tangential anchor positions even when classification and depth are correct.
  This is contact-witness construction, separate from the repaired axis classifier.
  It is now [resolved and validated](2026-09-21-capsule-side-contact-witness.md).

Temporary benchmark exports, logs, and coverage reports are supplemental. The
regression sources, arithmetic invariants, commands, and observations above do
not depend on those artifacts surviving cleanup.
