# Feature Work Issue Tracker

## Tracker Rules

- Add new items when feature work uncovers a suspected bug, stale doc, test
  smell, performance anomaly, or correctness risk.
- Keep each item scoped tightly enough to fix and verify independently.
- Record the date on the item, not in this filename.
- Move an item to `Resolved Issues` only after the fix has tests or documented
  verification evidence.
- Do not use this tracker as a substitute for tests, benchmarks, or release
  notes.
- Performance issues should stay in
  [`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md)
  unless they become a confirmed runtime defect. Do not add performance issues
  here until they have been investigated and confirmed as runtime defects.

## Active Issues

None.

## Performance Investigation Queue

Performance issues should stay in the benchmark plan unless they become a
confirmed runtime defect. Current queue:

- Benchmark any proposed single-limb-denominator path for the general
  `Fixed64.TryGetSignedRawRatio(Signed576, Signed576, ...)` contract before
  replacing its fixed-limb divider. Exact coordinate interpolation now owns a
  narrower `Signed320 / Signed192` path whose positive single-word denominator
  and representable quotient are construction invariants; it reduced the
  existing segment reconstruction benchmark from roughly 514/759 nanoseconds
  to about 121/118 nanoseconds at unit/100,000 scale. Do not generalize those
  invariants to arbitrary signed 576-bit callers without separate evidence.

## Resolved Issues

### FMS-Issue-016: Derived area and box center/extent paths were not full-domain

**Discovered:** 2026-07-20

**Resolved:** 2026-07-20

**Source:** sphere-construction full-domain audit and downstream bounds-consumer audit

`FixedBoundArea` and `FixedBoundBox` now derive centers with exact nearest-even
midpoints, derive scopes with a conservative ceiling after the exact endpoint
difference, and expose exact full sizes only when that public `Fixed64` result
is representable. An inherently unrepresentable size or scope throws instead
of silently returning a saturated under-estimate. Centered construction,
resize, and recenter operations validate both endpoints before committing, so
failed changes are atomic; odd-raw centered sizes expand outward by one raw
unit when necessary to preserve the requested lattice center without
under-bounding. Explicit `FromCenterAndSizeClippedToDomain` and
`FromCenterAndScopeClippedToDomain` factories cover the separate spatial-proxy
contract where the intended shape may extend outside the representable
coordinate domain.

The same audit hardened `FixedRange.Length` and `MidPoint`, reduced
SwiftCollections' `FixedBoundVolume` to one canonical `FixedBoundBox`, taught
`SwiftFixedOctree` to compare child spans in unsigned raw arithmetic without
materializing an unrepresentable full size, and moved fixed-BVH insertion cost
to exact 192-bit union-volume growth. `FixedBoundCircle.Bounds` now explicitly
clips its representable-domain intersection. GridForge grid centers and
Gravitas mesh/slab reference centers use the shared exact midpoint, while
Gravitas broad-phase proxies opt into the explicit clipped contract. Focused
regressions cover same-sign scalar faces, opposite faces, raw midpoint ties,
unrepresentable sizes with representable scopes, full-domain intervals and BVH
unions, atomic failures, circle/proxy clipping, serialization, octree/mesh
behavior, and mixed mesh-slab hit points.

The final FixedMathSharp Debug coverage run passed 1,655 tests and reported
13,855/13,855 lines, 3,952/3,952 branches, and 1,808/1,808 methods. The
allocation-free bounds benchmark improved centered area construction from
about 63.16 to 35.72 nanoseconds, centered box construction from about 72.50
to 45.94 nanoseconds, and box min/max mutation from about 36.65 to 2.80
nanoseconds; direct min/max construction remained effectively flat. The
downstream canonical `FixedBoundVolume` layout improved copy-plus-center access
from about 44.24 to 2.37 nanoseconds. Repeated center/size/volume metadata reads
increased from about 2.84 to 8.83 nanoseconds, still with zero managed
allocation; the smaller single-source struct was retained because query trees
copy volumes far more often than they repeatedly rematerialize all metadata.
The new exact volume-expansion cost measured about 10.46 nanoseconds versus
17.15 nanoseconds for the reconstructed chained/saturating formula; its
full-domain clamped case measured about 10.27 nanoseconds, all with zero managed
allocation.

### FMS-Issue-015: Sphere construction and merge paths were not full-domain

**Discovered:** 2026-07-18

**Resolved:** 2026-07-20

**Source:** exact radial predicate migration and downstream Gravitas release audit

**Resolution:**

`FixedBoundSphere.CreateFromBoundingBox`, `CreateFromFrustum`,
`CreateFromPoints`, and `CreateMerged` now retain midpoint, exact squared-
distance ordering, Euclidean roots, radius sums, and coordinate interpolation
without narrowing or saturation before the final Q32.32 result. Required radii
round outward, every successful construction is revalidated for containment,
and an unrepresentable deterministic result throws `OverflowException` instead
of returning a saturated under-bound sphere. Point/frustum construction remains
deterministic Ritter-style rather than a minimum-enclosing-sphere solve, and
merge centers remain constrained to the Q32.32 lattice, so the public contract
promises containment rather than mathematical minimality.

Extreme-pair seeding compares the exact squared distances from the midpoint to
the two actual endpoints. Reversed secondary axes therefore cannot select the
nearer endpoint, and combining coordinate-wise extrema cannot create a
fictional corner that falsely requires an unrepresentable radius.

The shared exact coordinate interpolation path now preserves final-result
half-even parity, including odd-origin ties, and uses a specialized
single-word-denominator ratio only where its internal invariants prove the
quotient representable. Its 128/64 division retains the shifted remainder's
carry for denominators across the complete unsigned raw domain.
`FixedSegment2d` and `FixedSegment` reuse that helper, removing their duplicate
weighted-coordinate implementations. `FixedBoundCircle` has no point-cloud or
merge factory, so no 2D construction API required a parallel implementation.

Verification passed 1,640 core and 8 Chronicler tests in `Release`, 1,619 core
and 8 Chronicler tests in `ReleaseLean`, and restored exact 100% coverage:
13,826/13,826 lines, 3,930/3,930 branches, and 1,801/1,801 methods. Ordinary
1,024-point array/span construction measured about 8.77/8.80 microseconds with
zero managed allocation versus the former saturated approximation's roughly
4.84/4.60 microseconds. Asymmetric full-domain merge measured about 321
nanoseconds with zero allocation. The existing segment reconstruction rows
improved from about 514/759 nanoseconds to about 121/118 nanoseconds at
unit/100,000 scale, also with zero allocation. The audit additionally exposed
the separate full-domain
derived-center/extent gap tracked as `FMS-Issue-016`.

### FMS-Issue-014: Finite-segment capsule/cylinder projections need dedicated wide ownership

**Discovered:** 2026-07-18

**Resolved:** 2026-07-19

**Source:** full-domain radial interval consumer audit

**Resolution:**

`FixedSegment2d` and `FixedSegment` now own allocation-free finite-axis capsule
intervals, and `FixedSegment` additionally owns finite-cylinder and separately
expanded affine-cylinder intervals. Endpoint differences, radial projection,
quadratic roots, and axial clipping remain in exact wide integer arithmetic
until one final half-to-even Q32.32 conversion. Bounded rays return parameters;
authored segments return physical distances and reconstruct points directly from
the original chord. Authored radius and sweep expansion remain separate, exact
inclusive-start and strict-end containment are available to downstream reducers,
collapsed capsule axes use the sphere/circle limit, and collapsed cylinder axes
are rejected because they cannot retain a flat-cap normal.

Bounded `FixedRay2d` and `FixedRay` overloads solve directly in
`[0, maxParameter]`, while authored-segment physical-distance overloads preserve
the original chord without normalization and narrow only once into
`[0, totalDistance]`. `GetPoint` uses fused multiply-add reconstruction and
`GetPointAtDistance` reconstructs against that same exact authored chord. A zero
distance budget is valid only for a zero-length segment, so point-query
classification remains explicit without accepting an invalid nondegenerate
sweep.

Gravitas migrated 2D capsule raycasts and sweeps, 3D capsule/cylinder raycasts,
swept-sphere capsule and mesh-edge projections, and the corresponding mixed
finite-axis reducers to these segment-owned contracts. The migration also
removed the downstream normalized-axis quadratics that originally lost full-
domain information.

Verification reached 100% FixedMathSharp coverage (13,691/13,691 lines,
3,908/3,908 branches, and 1,777/1,777 methods). Release passed 1,623 core and 8
Chronicler tests; ReleaseLean passed 1,602 core and 8 Chronicler tests. Coverage
closure also retained the public normalization repair path after deterministic
raw-boundary regressions proved ordinary rounded division can miss the squared-
magnitude tolerance immediately above the scale-safe cutoff. Common finite-axis
benchmarks measured approximately 2.3-2.9 microseconds and the arbitrary-raw
cylinder stress row measured approximately 8.9 microseconds. Fresh warmed
centered-capsule point-distance rows measured approximately 4.0-4.8 microseconds
across normal and 100,000-unit inputs. A downstream reconstruction review also
removed exact wide division for coordinates that do not change along a segment.
The existing axis-aligned reconstruction benchmark improved from 569.6 to 513.7
nanoseconds at unit scale and from 828.8 to 758.3 nanoseconds at 100,000-unit
scale, with no allocation. A separate Dry cold-start smoke run exercised all
eight scale variants of the new authored- segment distance-interval and
reconstruction rows; its timings are not used as throughput evidence, but
`MemoryDiagnoser` reported zero managed allocation for every row. Complexity
exceptions and migration/geometry documentation record the resulting contracts.

### FMS-Issue-013: Full-domain radial predicates and bounded query intervals

**Discovered:** 2026-07-18

**Resolved:** 2026-07-18

**Source:** relative CCD exact-root migration and Gravitas interval-consumer
audit

**Resolution:**

Circle/sphere, vector-distance, and centered-extent predicates now compare in
wide integer space across the complete finite endpoint domain. `FixedRay2d` and
`FixedRay` expose exact closed bounded entry/exit intervals with separate radius
expansion, and `FixedMath.TryGetCircleCrossSectionRadius` owns exact
sphere-slice reduction. `FixedMath.TryGetSphereSlabCrossSectionRadius`
additionally keeps opposite-domain center separation exact before selecting the
nearest plane in a finite slab. The misleading public `RadiusSquared` properties
were removed.

The corresponding Gravitas sphere-segment and mixed circle-slab/cross-section
consumers now retain actual radius ownership and use these APIs. Finite-axis
capsule/cylinder/mesh-edge projection remains `FMS-Issue-014`; full-domain
sphere construction/merge remains `FMS-Issue-015`; conic quadratics remain a
Gravitas issue.

Verification reached 100% FixedMathSharp coverage (9,408/9,408 lines,
3,064/3,064 branches, and 1,528/1,528 methods), with 1,460 standard and 1,439
Lean tests plus 8 Chronicler tests in both configurations passing. First-hit and
interval benchmark rows remained allocation free, and an independent review
reported no findings. Full execution detail is retained in
[`2026-07-18-full-domain-radial-query-plan.md`](done/2026-07-18-full-domain-radial-query-plan.md).

### FMS-Issue-012: Full in-process bounds benchmark can crash in frustum segment after prior rows

**Discovered:** 2026-06-30

**Source:** Phase 5 validation for
`docs/feature-work/2026-06-28-bounds-and-2d-geometry-hardening-plan.md`.

**Status:** Resolved on 2026-06-30

**Affected files:**

- `tests/FixedMathSharp.Benchmarks/README.md`
- `AGENTS.md`
- BenchmarkDotNet in-process runner path invoked by `-i`

**Resolution:**

The crash was classified as a broad in-process BenchmarkDotNet harness/toolchain
instability, not a confirmed `FixedBoundFrustum` or `Fixed64` runtime defect.
The stack landed in `FixedBoundFrustum.IntersectionPoint`, but that path is pure
managed fixed-point/vector math and no isolated runtime repro was found.

Evidence:

- A direct `dotnet-script` repro over 2,000,000 frustum constructions using the
  same perspective-matrix fixture completed successfully.
- A temporary Release console repro over 2,000,000 frustum constructions using
  the same fixture completed successfully.
- The isolated `BoundsBenchmarks.FrustumCreateFromMatrix` in-process benchmark
  completed successfully with no managed allocation.
- The full `bounds -j Short -i` benchmark completed successfully in a later
  rerun, so the original failure was not deterministic.
- The focused out-of-process frustum benchmark group completed all 9 rows with
  no managed allocation in the frustum hot rows.
- The full out-of-process `bounds -j Short` benchmark completed all 64 rows and
  passed through `FrustumCreateFromMatrix` after the preceding bounds rows.

Workflow change:

- Broad/full benchmark smoke guidance now uses the normal out-of-process
  BenchmarkDotNet path so each row keeps process isolation.
- In-process `-i` remains useful for narrow filtered diagnostics, but it should
  not be used as the broad/full-suite validation path or release evidence.
- No runtime geometry changes were made because the investigation did not
  produce a non-BenchmarkDotNet repro or focused failing unit test.

### FMS-Issue-011: Floating-point conversion paths rely on unchecked raw casts

**Discovered:** 2026-06-07

**Source:** Diagnostics and formatting surface implementation while hardening
`Fixed64.Parse`, `TryParse`, and decimal conversion behavior.

**Status:** Resolved on 2026-06-08

**Affected files:**

- `src/FixedMathSharp/Numerics/Scalars/Fixed64.Conversions.cs`
- `src/FixedMathSharp/Numerics/Scalars/FixedRange.cs`
- `src/FixedMathSharp/Numerics/Vectors/Vector2d.cs`
- `src/FixedMathSharp/Numerics/Vectors/Vector3d.cs`
- `src/FixedMathSharp/Numerics/Vectors/Vector4d.cs`
- `src/FixedMathSharp/Numerics/Curves/FixedCurveKey.cs`

**Concern:**

The diagnostics pass made value-space decimal parsing explicit and rejected
invalid, non-finite, and out-of-range text. A follow-up scan found older
floating-point conversion paths that still convert through
`Math.Round(value * FixedMath.ONE_L)` and direct `long` casts. That shape may
produce surprising behavior for `NaN`, infinities, and out-of-range doubles, and
similar logic appears in `FixedRange.InRange(double)`.

This is separate from diagnostic formatting because it affects public numeric
conversion semantics and should be resolved with focused tests, explicit
overflow policy, and a small benchmark check if any conversion helper is used in
setup-heavy or editor-facing paths.

**Recommended work:**

- [x] Add tests for `Fixed64.FromDouble`, explicit `float`/`double` conversions,
      `FromFraction`, vector `FromDouble` factories, and curve-key `FromDouble`
      factories using finite values, `NaN`, infinities, and out-of-range values.
      Removed the `FixedRange.InRange(double)` surface instead of keeping a
      cross-domain overload.
- [x] Decide whether floating-point value-space conversion should throw,
      saturate, or route through a shared checked helper. Prefer one documented
      policy rather than per-call-site behavior.
- [x] Keep raw Q32.32 construction explicit through `FromRaw`.
- [x] Verify that deterministic runtime code does not depend on ambient
      floating-point conversion in hot paths.
- [x] Update XML docs and `docs/wiki/fixed64-representation.md` after the
      conversion policy is settled.

**Resolution:**

`Fixed64.FromDouble` now performs checked value-space conversion directly.
Non-finite inputs throw `ArgumentOutOfRangeException`, while finite values
outside the Q32.32 range throw `OverflowException`. `FromFraction`, explicit
`float`/`double` conversions, vector `FromDouble` factories, and
`FixedCurveKey.FromDouble` inherit the same scalar policy.

`FixedRange.InRange(double)` was removed so callers convert to `Fixed64` at the
boundary and keep range checks in fixed-point space.
`DeterministicRandom.NextDouble` was also removed in favor of deterministic
`Fixed64` random helpers.

Raw payload construction remains explicit through `Fixed64.FromRaw(long)`. The
source scan found no remaining production `Math.Round(value * FixedMath.ONE_L)`
plus direct `long` cast sites outside `Fixed64.FromDouble`. The remaining
production `FromDouble` usages are public boundary/factory conveniences and
static matrix initialization, not deterministic runtime math loops.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Fixed64Tests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~Vector2dTests|FullyQualifiedName~Vector3dTests|FullyQualifiedName~Vector4dTests|FullyQualifiedName~FixedCurveTests"
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj --configuration Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed-range -j Short -i --filter "*InRange*"
rg -n "InRange\\(double|NextDouble|ToRawFromDouble|TryFromDecimal|Math\\.Round\\([^\\n]*ONE_L|\\(long\\)Math\\.Round" src/FixedMathSharp tests/FixedMathSharp.Benchmarks
```

Verification result on 2026-06-08: focused conversion, range, vector, and curve
tests first failed against the unchecked conversion behavior, then passed with
339 tests after the fix. The full Debug solution passed with 975 tests, Release
and ReleaseLean `netstandard2.1` builds passed with zero warnings and errors,
and the benchmark runner built in Release `net8.0`. The refactor then removed
the `FixedRange.InRange(double)` and `DeterministicRandom.NextDouble` surfaces
entirely. The source scan confirmed no remaining `InRange(double)`,
`NextDouble`, `ToRawFromDouble`, or `TryFromDecimal` runtime/test surfaces.

### FMS-Issue-010: `Fixed64` arithmetic operators with `long` appear to treat long operands as raw fixed values

**Discovered:** 2026-06-06

**Source:** Phase 1 public API inventory for
`docs/feature-work/done/2026-06-05-public-api-hardening-track.md`.

**Status:** Resolved on 2026-06-06

**Affected files:**

- `src/FixedMathSharp/Numerics/Scalars/Fixed64.cs`
- `tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`

**Concern:**

The explicit conversion operator from `long` to `Fixed64` shifts the integer
operand into Q32.32 value space, but the arithmetic overloads for
`Fixed64 + long`, `Fixed64 - long`, `long - Fixed64`, `Fixed64 * long`,
`long * Fixed64`, `Fixed64 / long`, `long / Fixed64`, `Fixed64 % long`, and
`long % Fixed64` construct `new Fixed64(long)` directly. That constructor is
internal and raw-value based, so these overloads likely treat user-facing long
integers as raw fixed increments.

This is larger than a public API cleanup because it may be a correctness bug
with silent numerical drift. Resolve separately with focused tests before the
scalar API cleanup phase decides whether long operator overloads should stay in
v5.0.0.

**Recommended work:**

- [x] Add tests proving long-to-`Fixed64` conversion uses integer-value
      semantics, not raw-value semantics.
- [x] Add tests covering positive, negative, and out-of-range long conversion
      behavior.
- [x] Decide whether long overloads should mirror int overload semantics, route
      through explicit conversion, or be removed for public API clarity.
- [x] Preserve saturating behavior by keeping explicit long conversion bounded
      to the representable Q32.32 range.
- [x] Add XML docs that distinguish raw construction via `FromRaw` from
      integer-value construction.

**Resolution:**

Removed the public `Fixed64` arithmetic operators that accepted `long` operands.
Callers now need to choose `(Fixed64)longValue` for integer-value semantics or
`Fixed64.FromRaw(rawValue)` for raw Q32.32 payload semantics. The explicit long
conversion now saturates to `Fixed64.MinValue` or `Fixed64.MaxValue` when the
source integer is outside the representable Q32.32 integer range, avoiding
unchecked left-shift overflow.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Fixed64Tests"
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
```

Verification result on 2026-06-06: focused `Fixed64Tests` passed with 52 tests,
the full Debug solution test run passed with 945 tests, Release and ReleaseLean
`netstandard2.1` builds passed with zero warnings/errors, the source scan found
no remaining public `Fixed64` arithmetic operators accepting `long`, and
`git diff --check` passed.

### FMS-Issue-009: `Fixed4x4` point transforms and matrix composition used different affine conventions

**Discovered:** 2026-06-05

**Resolved:** 2026-06-05

**Source:** Phase 4 matrix hotspot optimization work while validating affine
inversion with near-singular transforms.

**Resolution:**

FixedMathSharp now consistently applies `Fixed3x3` and `Fixed4x4` transforms
with an explicit row-vector convention: vectors and points are transformed as
`value * matrix`, translation lives in `M41/M42/M43`, and affine matrix
composition remains left-to-right. Quaternion matrix conversion and rotation
factory matrices were transposed to preserve existing `+Z` forward and
positive-angle rotation semantics under the row-vector convention.

**Completed work:**

- [x] Added regression tests proving `Fixed4x4.TransformPoint` uses row-vector
      affine math.
- [x] Added regression tests proving `Fixed4x4` multiplication composes
      left-to-right row-vector transforms.
- [x] Added regression tests proving `Fixed3x3.TransformDirection` and
      `Vector3d * Fixed3x3` use the same row-vector convention.
- [x] Updated `Fixed3x3`, `Fixed4x4`, `Vector3d`, and `Vector4d` transform
      formulas to use the same row-vector convention.
- [x] Updated affine inverse translation for row-vector matrix inversion.
- [x] Updated quaternion matrix conversion, matrix-to-quaternion extraction,
      Euler extraction, and `LookRotation` basis construction to match
      row-vector rotation matrices.
- [x] Added XML remarks to the core matrix transform APIs that call out the
      row-vector convention.
- [x] Verified matrix, quaternion, vector, and bounds/frustum-focused tests.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~Fixed3x3|FullyQualifiedName~Fixed4x4|FullyQualifiedName~Vector3d|FullyQualifiedName~Vector4d|FullyQualifiedName~FixedQuaternion|FullyQualifiedName~FixedBoundFrustum|FullyQualifiedName~FixedBoundBox|FullyQualifiedName~FixedRay|FullyQualifiedName~FixedPlane"
dotnet test FixedMathSharp.slnx --configuration Debug --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release -f netstandard2.1 --no-restore
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean -f netstandard2.1 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll matrix3x3 matrix4x4 -j Short -i --filter "*Transform*" "*Invert*" "*CreateRotation*" "*ScaleRotateTranslate*" "*TranslateRotateScale*" --exporters json
```

Result on 2026-06-05: focused matrix/vector/quaternion/bounds tests passed with
516 tests, full solution tests passed with 911 tests, release builds passed with
zero warnings/errors, and the matrix benchmark smoke completed with no managed
allocations.

### FMS-Issue-008: `--no-restore` can reuse stale NuGet assets across OS or configuration changes

**Discovered:** 2026-06-04

**Resolved:** 2026-06-04

**Source:** Phase 5 benchmark coverage verification.

**Resolution:**

`Directory.Build.props` now isolates generated files by host platform using
`BaseIntermediateOutputPath` and isolates normal project extension/intermediate
paths by configuration using `MSBuildProjectExtensionsPath` and
`IntermediateOutputPath`, while explicitly excluding generated `bin/**` and
`obj/**` files from SDK source globs. This preserves fast `--no-restore` loops
after a matching restore, but prevents `Release`, `ReleaseLean`, Windows, and
WSL builds from sharing the same conditional `project.assets.json`.

BenchmarkDotNet generated projects are intentionally excluded from the
configuration-scoped project extension/intermediate path override. Its generated
restore can omit the `Release` configuration even when the generated build later
uses it; keeping `BenchmarkDotNet.Autogenerated` on the platform-only
intermediate path lets its restore and build agree without weakening isolation
for the repository projects.

**Completed work:**

- [x] Added host/configuration-specific intermediate output paths.
- [x] Kept `BenchmarkDotNet.Autogenerated` on a platform-only generated-project
      path so BenchmarkDotNet restore/build agree during full benchmark exports.
- [x] Excluded all generated `bin/**` and `obj/**` files from source globs so
      old generated assembly-info files are never compiled as source.
- [x] Verified focused tests after restore and with `--no-restore`.
- [x] Verified `Release` and `ReleaseLean` benchmark builds can switch
      configurations and still succeed with `--no-restore` after matching
      restores.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug -p:RunSettingsFilePath= --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore -p:RunSettingsFilePath= --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --force
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0 --force
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c ReleaseLean -f net8.0 --no-restore
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i --exporters json
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

Result on 2026-06-04: focused tests passed, and benchmark builds passed with
zero warnings/errors in both configurations. A short `bounds` benchmark smoke
run and the full 136-benchmark `Release` export both completed after the
BenchmarkDotNet generated-project exception was added.

### FMS-Issue-005: Coverlet can fail to restore instrumented assemblies on WSL

**Discovered:** 2026-06-03

**Resolved:** 2026-06-04

**Source:** Phase 2 scalar/trigonometry allocation test verification.

**Resolution:**

`tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj` no longer sets
`RunSettingsFilePath` by default. Normal local `dotnet test` runs now avoid
coverlet instrumentation unless coverage is explicitly requested with
`--settings tests/FixedMathSharp.Tests/coverlet.runsettings`, while CI coverage
continues to request the runsettings file directly.

**Completed work:**

- [x] Removed the default test-project `RunSettingsFilePath`.
- [x] Verified focused tests without coverage still pass.
- [x] Verified explicit coverage collection still works and writes a Cobertura
      attachment.
- [x] Preserved CI coverage behavior because `.github/workflows/coverage.yml`
      already passes
      `--settings tests/FixedMathSharp.Tests/coverlet.runsettings`.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --no-restore --settings tests/FixedMathSharp.Tests/coverlet.runsettings --filter "FullyQualifiedName~FixedCurveTests|FullyQualifiedName~FixedRangeTests|FullyQualifiedName~DeterministicRandomTests"
```

Result on 2026-06-04: both focused runs passed; only the explicit coverage run
produced a `coverage.cobertura.xml` attachment.

### FMS-Issue-006: `ReleaseLean` `netstandard2.1` build emits MemoryPack shim conflict warnings

**Discovered:** 2026-06-03

**Resolved:** 2026-06-04

**Source:** Phase 2 final verification of Unity-supporting `netstandard2.1`
builds.

**Resolution:**

The warning pattern did not reproduce after a forced `ReleaseLean` restore. The
root cause was stale conditional NuGet assets from a previous MemoryPack-enabled
configuration, not duplicate runtime source. The same host/configuration
intermediate-output isolation added for `FMS-Issue-008` prevents lean builds
from seeing MemoryPack package assets restored for standard builds.

**Completed work:**

- [x] Confirmed a forced `ReleaseLean netstandard2.1` restore/build succeeds
      without shim conflict warnings.
- [x] Preserved `FIXEDMATHSHARP_DISABLE_MEMORYPACK` shim behavior for lean
      builds.
- [x] Verified `Release` and `ReleaseLean` builds for `netstandard2.1` and
      `net8.0`.

Verification:

```bash
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release --framework netstandard2.1 --force
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean --framework netstandard2.1 --force
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration Release --framework net8.0 --force
dotnet build src/FixedMathSharp/FixedMathSharp.csproj --configuration ReleaseLean --framework net8.0 --force
```

Result on 2026-06-04: all four builds passed with zero warnings/errors.

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

- [x] Added a failing test for
      `FixedQuaternion.FromDirection(Vector3d.Backward)`.
- [x] Added near-forward and near-backward tests so edge behavior is documented.
- [x] Preserved
      `FixedQuaternion.FromDirection(Vector3d.Forward) == FixedQuaternion.Identity`.
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

`Vector3d.Direction` behavior was confirmed as pitch on `X` and yaw on `Y`. The
XML documentation now matches that convention and calls out the existing
positive-pitch sign, which rotates canonical forward toward `Vector3d.Down`. No
additive API was needed for this pass.

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
