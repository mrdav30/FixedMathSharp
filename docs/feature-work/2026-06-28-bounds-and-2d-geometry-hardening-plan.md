# Bounds And 2D Geometry Hardening Battle Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:test-driven-development before production or test changes, and use superpowers:verification-before-completion before claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** In Progress - Phase 7 complete.

**Goal:** Make FixedMathSharp's bounds and geometry APIs explicit, allocation-free on hot paths, and reusable by Gravitas, GridForge, Trailblazer, and future deterministic LSF packages without pulling physics-specific behavior into the math layer.

**Architecture:** Split dimensional ownership cleanly: `FixedBoundBox` owns 3D AABB behavior, while `FixedBoundArea` becomes a true `Vector2d` AABB. Remove the old 3D `FixedBoundArea` meaning instead of patching it with more 3D parity. Add 2D circle, segment, ray, and triangle primitives around the new area type, add explicit 3D segment and triangle parity for shared spatial math, and remove hidden mutable array state from 3D bounds structs.

**Tech Stack:** `netstandard2.1`, `net8.0`, `Fixed64`, `Vector2d`, `Vector3d`, MemoryPack standard package support, ReleaseLean shims, xUnit v3, BenchmarkDotNet.

---

## Context

Gravitas now has first-class 2D, 3D, and mixed-dimension physics. That exposed several lower-stack geometry gaps:

- The former 3D-shaped `FixedBoundArea` was described as lightweight area math, but it stored `Vector3d` corners and overlapped heavily with `FixedBoundBox`.
- 3D callers that need volume bounds should use `FixedBoundBox`. Flat 3D footprints should be modeled by higher-level systems as a 2D area plus explicit layer or elevation state.
- The prior `FixedBoundBox` public `(center, size)` constructor made construction intent harder to read at call sites that needed explicit min/max or center/scope behavior.
- The prior `FixedBoundBox.Vertices` property hid a mutable cached `Vector3d[]` inside a struct, which was surprising for value semantics and allocated when callers only needed corner enumeration.
- There is no true `Vector2d`-based AABB/circle/segment/ray/triangle geometry layer for pure 2D packages to share.
- There are no explicit 3D segment or triangle primitives even though downstream geometry code already needs reusable finite-segment and triangle projection helpers.
- Gravitas, GridForge, and Trailblazer should not duplicate planar math just because the current FixedMathSharp bounds surface is 3D-shaped.

## Design Rules

- `FixedBoundBox` is the 3D AABB type.
- `FixedBoundArea` is the 2D AABB type.
- There should be no 3D `FixedBoundArea` compatibility layer after this plan.
- Prefer named factories over ambiguous overloads. A call site should read as min/max, center/size, or center/scope without checking constructor docs.
- Normalize min/max inputs at API boundaries unless an explicitly internal raw path is required for a measured hot path.
- Treat `Intersects` as inclusive closed-bound overlap unless the method name says otherwise. Add strict variants where contact-at-boundary must be distinguishable.
- Do not put physics concepts such as colliders, layers, body state, response, or inertia in FixedMathSharp.
- Do not add a generic shape hierarchy unless benchmarks prove it is faster and clearer than explicit value-type overloads.
- Keep new geometry structs small, readonly where practical, and compatible with Release and ReleaseLean.
- Keep hot methods allocation-free. Public APIs that allocate should make that cost obvious in the name.
- Preserve deterministic ordering for generated corners, triangle edges, segment endpoints, and benchmark fixtures.

## Risks To Avoid

- A compatibility adapter that keeps old 3D `FixedBoundArea` semantics alive beside better APIs.
- A `FixedBoundTriangle` name that implies a triangle is an axis-aligned bound. Use `FixedTriangle2d` for triangle geometry.
- Hidden array caches in structs. They behave poorly under copies and are easy to mutate through public accessors.
- 2D APIs that accidentally mean X/Z, Y-up, screen-space, or engine-space. FixedMathSharp 2D geometry is plain `Vector2d` plane math.
- A 3D segment API that duplicates or subtly diverges from `Vector3d.ClosestPointOnLineSegment(...)`.
- Intersection semantics that differ silently between area, box, circle, sphere, ray, and triangle types.

## Phase 1: Inventory And Guardrail Tests

**Files:**

- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundSphere.cs`
- Review: `src/FixedMathSharp/Geometry/Bounds/FixedBoundFrustum.cs`
- Review: `src/FixedMathSharp/Geometry/Primitives/FixedPlane.cs`
- Review: `src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundBox.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundSphere.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundFrustum.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedPlane.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedRay.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/SerializationBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/Support/BenchmarkCatalog.cs` if a bounds benchmark selector is missing.

- [x] Add tests that document current inclusive boundary behavior for `FixedBoundBox.Contains` and `FixedBoundBox.Intersects`.
- [x] Move existing 3D `FixedBoundArea` tests that represent actual 3D volumes to `FixedBoundBox` tests.
- [x] Move existing 3D `FixedBoundArea` tests that represent flat footprints into new 2D `FixedBoundArea` test names that will be implemented in Phase 4.
- [x] Add tests for swapped min/max inputs so the desired `FixedBoundBox` normalization behavior is locked before implementation.
- [x] Add tests that expose the current `FixedBoundBox.Vertices` allocation/reference-mutability risk. The test should be removed or replaced when `Vertices` is deleted in Phase 3.
- [x] Add or confirm a focused `bounds` benchmark catalog entry so later phases can measure construction, containment, intersection, corner copy, and union paths.
- [x] Capture a short benchmark baseline before changing the hot-path implementation.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundBoxTests|FullyQualifiedName~FixedBoundSphereTests|FullyQualifiedName~FixedBoundFrustumTests|FullyQualifiedName~FixedPlaneTests|FullyQualifiedName~FixedRayTests"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll list
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
```

**Phase 1 Notes:**

- `FixedBoundBox.Contains(Vector3d)` is boundary-inclusive for all corners.
  `FixedBoundBox.Intersects(FixedBoundBox)` is currently strict for
  face/edge/corner touch unless the other box is fully contained. That behavior
  is now explicit in tests and should be resolved deliberately during the
  Phase 8 intersection semantics sweep.
- Swapped `SetMinMax` and serialized state inputs currently preserve inverted
  bounds. Phase 1 captured this as current-risk guardrail coverage so Phase 2
  can replace it with desired normalization behavior without mistaking the
  change for incidental breakage.
- Phase 1 captured that `FixedBoundBox.Vertices` exposed a mutable backing array
  and default structs allocated that array on first access. Phase 3 replaced
  those guardrails with `GetCorner`/`CopyCorners` coverage.
- The benchmark catalog already exposes the `bounds` selector. Phase 1 added
  box construction, `SetMinMax`, vertex-read, and union baselines. Short
  in-process baseline on 2026-06-29 completed 37 bounds benchmarks; key new
  baselines: `BoxConstructCenterSize` 20,385.9 ns, `BoxSetMinMax` 9,670.0 ns,
  `BoxReadVertices` 1,155.0 ns, `BoxUnion` 1,525.0 ns.

## Phase 2: 3D Bounds Cleanup And Old Area Removal

**Files:**

- Delete after migration: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Delete after migration: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundArea.Tests.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundSphere.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundFrustum.cs`
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedPlane.cs`
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedRay.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundBox.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundSphere.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundFrustum.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedPlane.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedRay.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/SerializationBenchmarks.cs`
- Review downstream after release: Gravitas, GridForge, Trailblazer, and any Unity adapter call sites that construct bounds directly.

- [x] Add `FixedBoundBox.FromMinMax(Vector3d min, Vector3d max)`.
- [x] Add `FixedBoundBox.FromCenterAndSize(Vector3d center, Vector3d size)`.
- [x] Add `FixedBoundBox.FromCenterAndScope(Vector3d center, Vector3d scope)`.
- [x] Replace the public `FixedBoundBox(Vector3d center, Vector3d size)` constructor with named factories in source, tests, benchmarks, and known local consumers. Keep the state constructor needed by serialization.
- [x] Normalize min/max inputs in `FromMinMax`, `SetMinMax`, and state population so callers cannot create inverted boxes through public APIs.
- [x] Normalize size/scope through absolute component values where negative extents are accepted. XML docs must state that behavior.
- [x] Replace every `Contains(FixedBoundArea)` and `Intersects(FixedBoundArea)` overload in 3D types with `FixedBoundBox`-based coverage.
- [x] Replace every `FixedRay.Intersects(FixedBoundArea)` and `FixedPlane.Intersects(FixedBoundArea)` path with `FixedBoundBox` coverage.
- [x] Delete the old `FixedBoundArea` source file after all 3D call sites have moved to `FixedBoundBox`.
- [x] Delete or migrate old `FixedBoundArea` serialization benchmarks. The new 2D `FixedBoundArea` serialization coverage is added in Phase 4.
- [x] Update complexity exception entries that reference `FixedBoundFrustum.Contains(FixedBoundArea)` so they refer to the surviving `FixedBoundBox` path or are removed.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundBoxTests|FullyQualifiedName~FixedBoundSphereTests|FullyQualifiedName~FixedBoundFrustumTests|FullyQualifiedName~FixedPlaneTests|FullyQualifiedName~FixedRayTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
rg -n "FixedBoundArea\\b" src tests
```

Expected after Phase 2 and before Phase 4: `rg` should find no production
references to the old 3D `FixedBoundArea`. Test or benchmark references should
exist only if they are intentionally waiting for the new 2D type in Phase 4.

**Phase 2 Notes:**

- The old 3D `FixedBoundArea` source and tests were deleted rather than kept as
  compatibility wrappers. Phase 4 reintroduced `FixedBoundArea` as a true
  `Vector2d` AABB instead of preserving the old 3D semantics.
- `FixedMathSharp.Chronicler.WriteBoundArea` was removed with the old 3D area.
  The companion package should add a new deterministic `Vector2d` area hash
  writer in Phase 4 when the replacement type exists.
- `FixedBoundBox` keeps the serialization state constructor, but the public
  center/size construction path now uses named factories.

## Phase 3: Allocation-Free FixedBoundBox Corners

**Files:**

- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundBox.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`

- [x] Remove the `_vertices` field and `_isDirty` corner-cache behavior from `FixedBoundBox`.
- [x] Remove the public `Vertices` array property. Do not replace it with another hidden allocation path.
- [x] Add `public Vector3d GetCorner(int index)` with deterministic index ordering matching the current documented near/far corner order.
- [x] Add `public void CopyCorners(Span<Vector3d> destination)` and throw `ArgumentException` when `destination.Length < 8`.
- [x] Update `FindClosestPointsBetweenBoxes` to use `GetCorner` or a stack/local span path instead of `b.Vertices[i]`.
- [x] Add tests for all eight corner indexes, copy ordering, short span validation, and no externally mutable corner storage.
- [x] Add a benchmark comparing `GetCorner`, `CopyCorners`, and the old baseline behavior captured in Phase 1.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundBoxTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
```

**Phase 3 Notes:**

- `FixedBoundBox` is now field-only and no longer caches corner arrays. The
  MemoryPack constructor attribute was removed because MemoryPack does not call
  constructors for unmanaged structs.
- `BoxReadVertices` benchmark coverage was replaced by `BoxGetCorner` and
  `BoxCopyCorners`.

## Phase 4: True 2D Area Bounds

**Files:**

- Create or rewrite: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundArea.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/SerializationBenchmarks.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/Support/BenchmarkCatalog.cs` if needed.

- [x] Add a `FixedBoundArea` value type with `Vector2d Min`, `Vector2d Max`, `Vector2d Center`, `Vector2d Size`, and `Vector2d Scope`.
- [x] Add named factories: `FromMinMax`, `FromCenterAndSize`, and `FromCenterAndScope`.
- [x] Normalize min/max and extents consistently with the 3D bounds factories.
- [x] Add `Contains(Vector2d point)`, `Contains(FixedBoundArea area)`, `Intersects(FixedBoundArea area)`, `IntersectsStrict(FixedBoundArea area)`, `ClampPoint`, `ProjectPoint`, and `Union`.
- [x] Add `Deconstruct(out Vector2d min, out Vector2d max)` so consumers can unpack normalized bounds without reaching into internal storage.
- [x] Add Release and ReleaseLean compatible serialization attributes following existing bounds patterns.
- [x] Add tests for construction, normalization, boundary containment, strict vs inclusive intersection, union, clamp, projection, equality, hash behavior, JSON roundtrip, and MemoryPack roundtrip when MemoryPack is enabled.
- [x] Add benchmark cases for 2D construction, contains, intersects, union, clamp, and serialization.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundAreaTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
```

**Phase 4 Notes:**

- `FixedBoundArea` is a true `Vector2d` AABB with mutable `Min`/`Max`
  ownership, named factories, normalized state, and no 3D compatibility
  surface.
- The 2D API uses `Size` for total extent and `Scope` for half extent. A
  `Proportions` alias was intentionally not added to avoid duplicate public
  names for the same value.
- `Contains(FixedBoundArea)` returns `FixedEnclosureType`, matching 3D bounds
  classification. `Intersects` is boundary-inclusive; `IntersectsStrict`
  requires positive area overlap on both axes.
- `FixedMathSharp.Chronicler.WriteBoundArea` was restored for the new 2D area
  shape and writes canonical `Min` then `Max` as `Vector2d` values.
- Phase 4 also tightened `FixedBoundBox.GetHashCode()` away from
  `HashCode.Combine` to an explicit deterministic component hash, matching the
  new `FixedBoundArea` hash behavior.

## Phase 5: 2D Circle Bounds

**Files:**

- Create: `src/FixedMathSharp/Geometry/Bounds/FixedBoundCircle.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundCircle.Tests.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Modify: `tests/FixedMathSharp.Tests/Geometry/Bounds/FixedBoundArea.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`

- [x] Add a `FixedBoundCircle` value type with `Vector2d Center`, `Fixed64 Radius`, `Fixed64 RadiusSquared`, and `FixedBoundArea Bounds`.
- [x] Normalize negative radius inputs to absolute radius, or reject them with `ArgumentOutOfRangeException`. Choose one policy and keep it consistent with `FixedBoundSphere`.
- [x] Add `Contains(Vector2d point)`, `Contains(FixedBoundCircle circle)`, `Intersects(FixedBoundCircle circle)`, `Intersects(FixedBoundArea area)`, `ClampPoint`, and `ProjectPoint`.
- [x] Add matching `FixedBoundArea.Contains(FixedBoundCircle circle)` and `FixedBoundArea.Intersects(FixedBoundCircle circle)` only if those overloads remove duplicated downstream code.
- [x] Add strict overlap variants only where boundary-touch behavior matters to downstream callers. Do not add duplicate convenience APIs without a known use case.
- [x] Add tests for zero-radius circles, touching circles, circle-area touch, containment, projection, equality, hash behavior, JSON roundtrip, and MemoryPack roundtrip when MemoryPack is enabled.
- [x] Add benchmark cases for point containment, circle intersection, area intersection, and clamp/projection.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundCircleTests|FullyQualifiedName~FixedBoundAreaTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
```

**Phase 5 Notes:**

- `FixedBoundCircle` is a true `Vector2d` circular bound with normalized radius,
  a derived `FixedBoundArea Bounds`, boundary-inclusive containment and
  intersection, deterministic equality/hash behavior, and JSON/MemoryPack
  roundtrip coverage.
- Negative circle radii are normalized by absolute value, matching the
  `FixedBoundArea` size/scope normalization style. During this phase,
  `FixedBoundSphere` was also tightened so construction and radius assignment
  cannot retain a negative radius.
- Review follow-up moved `FixedBoundSphere` to a matching
  `BoundingSphereState` payload so sphere serialization uses the same explicit
  state shape as `FixedBoundBox`, `FixedBoundArea`, and `FixedBoundCircle`
  instead of mixing a serialized center field with a radius-backed property.
- `FixedBoundArea.Contains(FixedBoundCircle)` and
  `FixedBoundArea.Intersects(FixedBoundCircle)` were added because downstream
  planar broad-phase/query callers naturally ask the area-side question and the
  overloads centralize the closest-point circle/area math.
- Strict circle overlap methods were intentionally not added. The known
  downstream need is boundary-inclusive broad-phase/query behavior; strict
  variants can be added in Phase 8 only if a measured caller needs them.
- `FixedMathSharp.Chronicler.WriteBoundCircle` hashes canonical `Center` then
  normalized `Radius`.
- While scanning for adjacent radius/hash issues, the remaining active
  `HashCode.Combine` uses in `FixedBoundFrustum` and `FixedCurveKey` were
  replaced with explicit deterministic component hashes under focused tests.
- Focused short-run circle benchmark rows completed with no managed allocation:
  `CircleContainsPoint` 3.373 us, `CircleIntersectsCircle` 3.741 us,
  `CircleIntersectsArea` 3.780 us, `CircleClampPoint` 58.747 us, and
  `CircleProjectPoint` 56.495 us.
- A full in-process bounds benchmark smoke hit an unrelated BenchmarkDotNet
  crash in the frustum row after prior bounds rows had completed. Direct
  two-million-iteration frustum construction repros and the isolated frustum
  benchmark passed, so this is tracked separately as `FMS-Issue-012` in
  `docs/feature-work/issue-tracker.md` instead of changing production frustum
  math without a runtime repro.

## Phase 6: 2D Segment And Ray Primitives

**Files:**

- Create: `src/FixedMathSharp/Geometry/Primitives/FixedSegment2d.cs`
- Create: `src/FixedMathSharp/Geometry/Primitives/FixedRay2d.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedSegment2d.Tests.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedRay2d.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs` or create `tests/FixedMathSharp.Benchmarks/Geometry2dBenchmarks.cs`.
- Modify: `tests/FixedMathSharp.Benchmarks/Support/BenchmarkCatalog.cs` if a new geometry benchmark group is added.

- [x] Add `FixedSegment2d` with `Start`, `End`, `Delta`, `Length`, `LengthSquared`, `Bounds`, `ClosestPoint(Vector2d point)`, and `DistanceSquared(Vector2d point)`.
- [x] Add `FixedRay2d` with `Position`, `Direction`, `GetPoint(Fixed64 distance)`, and intersection helpers against `FixedBoundArea` and `FixedBoundCircle`.
- [x] Match `FixedRay` normalization semantics where practical. If `FixedRay` permits unnormalized direction, document whether `FixedRay2d` does the same or intentionally normalizes.
- [x] Guard zero-length ray directions and zero-length segments with explicit deterministic behavior covered by tests.
- [x] Add tests for horizontal, vertical, diagonal, reversed, zero-length, and boundary-touch cases.
- [x] Add benchmark coverage for segment closest-point, segment distance, ray-area intersection, and ray-circle intersection.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedSegment2dTests|FullyQualifiedName~FixedRay2dTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
```

**Phase 6 Notes:**

- `FixedSegment2d` is a simple ordered-endpoint primitive with derived length,
  squared length, bounds, closest-point, and squared-distance helpers. Reversed
  endpoints keep their order for equality and deconstruction, while `Bounds`
  normalizes through `FixedBoundArea`.
- `Vector2d.ClosestPointOnLineSegment(...)` was added for parity with the
  existing `Vector3d` segment helper, and `FixedSegment2d` delegates to that
  low-level projection math. Distance and bounds ownership stay on the segment
  primitive instead of expanding `Vector2d` into a shape API.
- `FixedRay2d` intentionally does not normalize `Direction`, matching
  `FixedRay`. Ray intersections return the first forward ray parameter; that
  value is a physical distance only when the direction is normalized.
- Zero-length segments return `Start` for closest-point projection.
  Zero-direction rays return `0` only when the origin is inside or on the
  queried area/circle, otherwise `null`.
- Short in-process benchmark rows completed with no managed allocation in the
  summary: `Segment2dClosestPoint` 14.01 us,
  `Segment2dDistanceSquared` 15.95 us, `Ray2dIntersectsArea` 31.26 us, and
  `Ray2dIntersectsCircle` 12.18 us.

## Phase 6b: 3D Segment Primitive

**Files:**

- Create: `src/FixedMathSharp/Geometry/Primitives/FixedSegment.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedSegment.Tests.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs` only if existing segment-helper coverage needs naming or edge-case cleanup.
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs` or `tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs` for segment primitive benchmark coverage.

- [x] Add `FixedSegment` with `Start`, `End`, `Delta`, `Length`, `LengthSquared`, `Bounds`, `ClosestPoint(Vector3d point)`, and `DistanceSquared(Vector3d point)`.
- [x] Use `FixedBoundBox.FromMinMax(Start, End)` for `Bounds` so reversed endpoints produce normalized 3D AABB coverage without resurrecting 3D `FixedBoundArea`.
- [x] Delegate closest-point projection to the existing `Vector3d.ClosestPointOnLineSegment(...)` helper so 3D segment primitive behavior stays aligned with current `Vector3d` math.
- [x] Preserve ordered endpoint equality and deconstruction. Reversed endpoints should have the same `Bounds` but should not compare equal unless a measured caller needs unordered segment identity.
- [x] Guard zero-length segments with explicit deterministic behavior: `ClosestPoint(point)` returns `Start`, `Length` and `LengthSquared` are zero, and `DistanceSquared(point)` equals squared point-to-start distance.
- [x] Add JSON and MemoryPack roundtrip tests following `FixedSegment2d` and `FixedRay` primitive patterns.
- [x] Add tests for horizontal, vertical, depth-axis, diagonal, reversed, zero-length, bounds, equality/hash, deconstruction, and serialization behavior.
- [x] Add benchmark coverage for `FixedSegment.ClosestPoint` and `FixedSegment.DistanceSquared`. Keep the benchmark fixture deterministic and allocation-free after setup.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedSegmentTests|FullyQualifiedName~Vector3dTests"
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds --filter "*Segment3d*" -j Short -i
```

**Phase 6b Notes:**

- `FixedSegment` mirrors the 2D segment value model with ordered endpoints,
  derived length data, normalized `FixedBoundBox` coverage, deconstruction,
  deterministic equality/hash behavior, and JSON/MemoryPack-compatible state.
- Closest-point projection is intentionally delegated to
  `Vector3d.ClosestPointOnLineSegment(...)` so the primitive does not fork
  lower-level segment math. Zero-length segments inherit the existing
  deterministic `Start` fallback and are covered through the primitive tests.
- Existing `Vector3d` segment helper tests did not need changes; the focused
  Phase 6b test run covered both `FixedSegmentTests` and `Vector3dTests`.
- Short in-process benchmark rows completed with no managed allocation in the
  summary: `Segment3dClosestPoint` 18.75 us and
  `Segment3dDistanceSquared` 22.31 us.

## Phase 7: Triangle Primitives

**Files:**

- Create: `src/FixedMathSharp/Geometry/Primitives/FixedTriangle2d.cs`
- Create: `src/FixedMathSharp/Geometry/Primitives/FixedTriangle.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedTriangle2d.Tests.cs`
- Create: `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedTriangle.Tests.cs`
- Modify: `src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs` or `tests/FixedMathSharp.Benchmarks/Geometry2dBenchmarks.cs`.

- [x] Add `FixedTriangle2d` with vertices `A`, `B`, `C`, `SignedArea`, `Area`, `Bounds`, and deterministic edge access.
- [x] Add `FixedTriangle` with vertices `A`, `B`, `C`, derived normal/area/bounds, and deterministic edge access.
- [x] Add `Vector2d.BarycentricCoordinates(...)` parity with `Vector3d.BarycentricCoordinates(...)`.
- [x] Add `GetPoint(Fixed64 weightB, Fixed64 weightC)` to both triangle primitives so known barycentric weights can reconstruct points through the vector helpers.
- [x] Add `TryGetBarycentricWeights(...)` to `FixedTriangle2d` for exact planar weight solving.
- [x] Add `TryGetProjectedBarycentricWeights(...)` to `FixedTriangle` so the public name makes the 3D off-plane projection semantics explicit.
- [x] Add `Contains(Vector2d point)` using fixed-point same-side tests with explicit boundary-inclusive behavior.
- [x] Add `Contains(Vector3d point)` using closest-point distance so containment remains plane-aware and boundary-inclusive.
- [x] Add `ClosestPoint(Vector2d point)` using segment closest-point helpers for edge fallback.
- [x] Add `ClosestPoint(Vector3d point)` using deterministic triangle-region projection with segment closest-point fallback for degenerate triangles.
- [x] Add `Intersects(FixedBoundArea area)` and `Intersects(FixedBoundCircle circle)` only if Gravitas, GridForge, or Trailblazer will consume them immediately. Otherwise keep the first version focused on triangle ownership, area, bounds, and point containment.
- [x] Add tests for clockwise, counter-clockwise, degenerate, edge-touch, vertex-touch, outside-near-edge, inside, barycentric interpolation, and barycentric weight solving cases.
- [x] Add benchmarks for area, bounds, point containment, closest-point, point interpolation, and barycentric weight-solving behavior.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~Vector2dTests.BarycentricCoordinates|FullyQualifiedName~FixedTriangle2dTests|FullyQualifiedName~FixedTriangleTests"
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds --filter "*Triangle*" -j Short -i
```

**Phase 7 Notes:**

- `FixedTriangle2d` owns ordered vertices, signed and absolute area,
  centroid, normalized `FixedBoundArea` coverage, stable vertex/edge access,
  boundary-inclusive point containment, closest-point projection, squared
  distance, and JSON/MemoryPack-compatible state.
- `FixedTriangle` mirrors the 3D naming convention instead of using a
  `FixedTriangle3d` suffix. It derives normal, area, centroid, and
  `FixedBoundBox` coverage from ordered vertices so mutable triangle state
  cannot leave a stale cached normal beside changed vertices.
- `Vector2d.BarycentricCoordinates(...)` now mirrors the existing 3D vector
  helper and delegates to `FixedMath.BarycentricCoordinate(...)` per component.
  The second-order barycentric product helpers remain in `FixedMath` and
  `Fixed3x3.CreateBarycentricProductSums(...)` because they serve integration
  and inertia-style math rather than triangle point queries.
- `FixedTriangle2d.GetPoint(...)` and `FixedTriangle.GetPoint(...)` reconstruct
  points from known B/C barycentric weights. `FixedTriangle2d` also solves
  planar barycentric weights directly, while `FixedTriangle` exposes
  `TryGetProjectedBarycentricWeights(...)` so callers cannot mistake 3D
  off-plane points for strict on-plane containment.
- 3D containment is plane-aware through closest-point distance, and degenerate
  triangles fall back to deterministic edge projection. The face-region closest
  point path guards a zero denominator and falls back to edges rather than
  dividing through a rounded-degenerate case.
- Area/circle intersection overloads were intentionally not added yet because
  no immediate downstream caller needs them. The reusable core surface is
  triangle state, bounds, point containment, closest point, and distance.
- Future Gravitas migration can replace local 3D `TriangleData` and
  `MixedTriangle` value structs with `FixedTriangle`, adding physics-specific
  cached normals or bounds only if profiling proves the cache is worth the
  extra state.
- Short in-process benchmark rows completed with no managed allocation in the
  summary: `Triangle2dArea` 9.012 us, `Triangle2dBounds` 1.470 us,
  `Triangle2dContainsPoint` 20.117 us, `Triangle2dClosestPoint` 73.144 us,
  `Triangle2dGetPoint` 15.807 us, `Triangle2dBarycentricWeights` 32.073 us,
  `Triangle3dArea` 36.493 us, `Triangle3dBounds` 3.048 us,
  `Triangle3dContainsPoint` 35.925 us, `Triangle3dClosestPoint` 32.170 us,
  `Triangle3dGetPoint` 18.921 us, and
  `Triangle3dProjectedBarycentricWeights` 61.086 us.

## Phase 8: Intersection Semantics Sweep

**Files:**

- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundBox.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundSphere.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundArea.cs`
- Modify: `src/FixedMathSharp/Geometry/Bounds/FixedBoundCircle.cs`
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedRay.cs` only if ray semantics are inconsistent.
- Modify: `src/FixedMathSharp/Geometry/Primitives/FixedRay2d.cs`
- Modify: `tests/FixedMathSharp.Benchmarks/BoundsBenchmarks.cs`
- Modify matching tests under `tests/FixedMathSharp.Tests/Geometry`.

- [x] Audit every `Contains` and `Intersects` overload touched by this plan.
- [x] Make inclusive closed-bound overlap the default for `Intersects`.
- [x] Add `IntersectsStrict` only for bounds pairs where downstream systems need to distinguish touching from overlapping volume/area.
- [x] Ensure strict methods use `<`/`>` and inclusive methods use `<=`/`>=` consistently across 2D and 3D.
- [x] Add cross-type tests for box-sphere, 2D area-circle, and 2D ray-boundary cases.
- [x] Update XML docs so boundary-touch behavior is stated on public APIs.

Notes:

- `FixedBoundBox.Intersects(FixedBoundBox)` now follows the same closed-bound
  inclusive model as 2D areas, circles, spheres, and ray-boundary checks.
- `IntersectsStrict` was added for box-box, box-sphere, sphere-box,
  sphere-sphere, area-circle, circle-area, and circle-circle pairs. These
  methods reject boundary-only contact and zero-area/zero-volume inputs.
- Strict frustum overloads were intentionally left out of this sweep because
  frustum plane classification needs a separate positive-volume contract if a
  downstream consumer proves it needs one.

Verification completed:

- `dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundBoxTests|FullyQualifiedName~FixedBoundSphereTests|FullyQualifiedName~FixedBoundAreaTests|FullyQualifiedName~FixedBoundCircleTests|FullyQualifiedName~FixedRayTests|FullyQualifiedName~FixedRay2dTests"` passed.
- `dotnet test FixedMathSharp.slnx --configuration Debug` passed.
- `dotnet test FixedMathSharp.slnx --configuration Release` passed.
- `dotnet test FixedMathSharp.slnx --configuration ReleaseLean` passed.
- `dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i` completed all 64 bounds benchmarks before the benchmark-reporting fast-follow. Relevant intersection rows remained allocation-free as whole fixture-batch measurements (`BoxIntersectsBox` 697.8 ns, `BoxIntersectsSphere` 5.39 us, `SphereIntersectsSphere` 5.65 us, `SphereIntersectsBox` 10.95 us, `CircleIntersectsCircle` 3.98 us, `CircleIntersectsArea` 3.90 us).
- Fast-follow benchmark cleanup added `SampledBenchmarkAttribute` so loop-style bounds benchmarks report per fixture operation through `OperationsPerInvoke`; point-cloud `CreateFromPoints` benchmarks intentionally remain end-to-end API measurements. Focused short-run verification passed with no managed allocation for sampled rows (`AreaIntersectsArea` 2.239 ns/op, `AreaIntersectsAreaStrict` 2.468 ns/op, `BoxIntersectsBox` 2.746 ns/op) and preserved `SphereCreateFromPointsArray` as an end-to-end 5.012 us row.

Verification:

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj --configuration Debug --filter "FullyQualifiedName~FixedBoundBoxTests|FullyQualifiedName~FixedBoundSphereTests|FullyQualifiedName~FixedBoundAreaTests|FullyQualifiedName~FixedBoundCircleTests|FullyQualifiedName~FixedRayTests|FullyQualifiedName~FixedRay2dTests"
```

## Phase 9: Docs, Downstream Migration Notes, And Release Validation

**Files:**

- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `docs/complexity-exceptions.md`
- Create or modify: `docs/wiki/bounds-and-geometry.md`
- Modify: `docs/feature-work/2026-06-28-bounds-and-2d-geometry-hardening-plan.md`
- Review after package release: Gravitas bounds/query/collision call sites.
- Review after package release: GridForge blocker and traversal call sites.
- Review after package release: Trailblazer path, steering, or controller call sites.

- [ ] Document the public geometry model: 3D `FixedBoundBox`, 3D `FixedBoundSphere`, `FixedSegment`, `FixedTriangle`, 2D `FixedBoundArea`, 2D `FixedBoundCircle`, `FixedSegment2d`, `FixedRay2d`, and `FixedTriangle2d`.
- [ ] Document named factory usage and remove examples that call ambiguous constructors.
- [ ] Document boundary semantics for inclusive and strict intersection methods.
- [ ] Document that flat world footprints should be represented as `FixedBoundArea` plus explicit layer/elevation state in higher-level packages, not as a 3D area in FixedMathSharp.
- [ ] Document that physics-specific mesh, collider, shape-cast, and material behavior belongs in Gravitas, not FixedMathSharp.
- [ ] Run Debug, Release, and ReleaseLean tests.
- [ ] Run the bounds/geometry benchmark group and confirm no allocation regressions in hot paths.
- [ ] Search for stale direct `FixedBoundBox` construction, stale `Vertices` usage, stale 3D `FixedBoundArea` docs, and stale `Vector2d.Down` references.
- [ ] Mark this plan done only after downstream migration notes are either completed or moved into the consuming repo's feature-work docs.

Verification:

```bash
dotnet restore
dotnet build FixedMathSharp.slnx --configuration Debug --no-restore
dotnet test FixedMathSharp.slnx --configuration Debug
dotnet test FixedMathSharp.slnx --configuration Release --no-restore
dotnet test FixedMathSharp.slnx --configuration ReleaseLean --no-restore
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll bounds -j Short -i
rg -n "new FixedBoundBox\\(|\\.Vertices\\b|Vector2d\\.Down|FixedBoundTriangle" src tests README.md AGENTS.md docs
```

## Downstream Migration Shape

Gravitas:

- 3D collider bounds and query bounds should use `FixedBoundBox`.
- Pure 2D collider bounds should use the new `FixedBoundArea` directly.
- Mixed 2D/3D embedded slabs should keep using `FixedBoundBox` for mixed broad-phase identity, with `FixedBoundArea` used only for pure planar shape math.

GridForge:

- `BoundsBlocker` should store a `FixedBoundArea` plus explicit layer/elevation data instead of a 3D `FixedBoundArea` with Y locked to a layer.
- 3D or volumetric blocker/query work should use `FixedBoundBox`.
- Tests that only used `Vector3d` because the old area type required it should migrate to `Vector2d`.

Trailblazer:

- Planar path footprints, avoidance radii, steering bounds, and character-controller support casts should prefer `FixedBoundArea`, `FixedBoundCircle`, `FixedSegment2d`, and `FixedRay2d`.
- Any future 3D controller volume should use `FixedBoundBox`, not a resurrected 3D area type. 3D path probes, ledge traces, and steering/debug line math should prefer `FixedSegment` where a finite segment is the actual domain object.

## Recommended First Implementation Slice

Start with Phases 1 through 3 as one reviewable 3D cleanup slice. That locks down current 3D behavior, introduces explicit `FixedBoundBox` factories, removes old 3D `FixedBoundArea`, and removes the allocation-prone `FixedBoundBox.Vertices` shape before new 2D APIs are layered on top.

Then implement Phases 4 through 7 as true planar geometry, with Phase 6b as the adjacent 3D segment parity pass. These phases are independent enough to review by type, but they should ship together if Gravitas, GridForge, or Trailblazer is waiting on a coherent geometry surface.

Finish with Phase 8 and Phase 9 as a semantics and validation pass. The final package should leave downstream consumers with clearer APIs, fewer local geometry helpers, and no ambiguity about boundary-touch behavior.
