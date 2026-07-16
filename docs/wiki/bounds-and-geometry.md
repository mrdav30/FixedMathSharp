# Bounds And Geometry

FixedMathSharp geometry is dimension-explicit. The core package owns reusable
fixed-point shape math only; physics concepts such as colliders, materials,
shape casts, contact manifolds, body state, and broad-phase layers belong in
higher-level.

## Dimensional Ownership

Use 3D types for volume and spatial math:

- `FixedBoundBox`: 3D axis-aligned bounding box.
- `FixedBoundSphere`: 3D sphere bound.
- `FixedBoundFrustum`: 3D frustum bound.
- `FixedRay`: 3D ray intersection primitive.
- `FixedPlane`: 3D plane classification primitive.
- `FixedSegment`: finite 3D segment with closest-point, closest-pair, distance,
  and bounds.
- `FixedTriangle`: ordered 3D triangle with area, normal, bounds, closest-point,
  containment, interpolation, and projected barycentric helpers.

Use 2D types for plane math:

- `FixedBoundArea`: 2D `Vector2d` axis-aligned bounding area.
- `FixedBoundCircle`: 2D circular bound.
- `FixedRay2d`: 2D ray intersection primitive.
- `FixedSegment2d`: finite 2D segment with full-domain closest-point,
  unique-intersection, closest-pair, distance, and bounds.
- `FixedTriangle2d`: ordered 2D triangle with signed area, bounds,
  closest-point, containment, interpolation, and barycentric helpers.

There is no 3D `FixedBoundArea` compatibility model. A flat world footprint
should be represented as `FixedBoundArea` plus explicit layer, elevation, or
height state in the consuming package. A volumetric query or collider bound
should use `FixedBoundBox`.

## Construction

`FixedBoundBox` and `FixedBoundArea` use named factories so call sites state the
meaning of their extents:

```csharp
FixedBoundBox box = FixedBoundBox.FromMinMax(min3d, max3d);
FixedBoundBox room = FixedBoundBox.FromCenterAndSize(center3d, size3d);
FixedBoundBox influence = FixedBoundBox.FromCenterAndScope(center3d, halfExtents3d);

FixedBoundArea area = FixedBoundArea.FromMinMax(min2d, max2d);
FixedBoundArea footprint = FixedBoundArea.FromCenterAndSize(center2d, size2d);
FixedBoundArea sensorArea = FixedBoundArea.FromCenterAndScope(center2d, halfExtents2d);
```

`FromMinMax` normalizes swapped inputs. `FromCenterAndSize` and
`FromCenterAndScope` normalize negative extents by absolute component value.
This keeps public bounds state canonical without asking every caller to sort or
sanitize the inputs first.

`FixedBoundCircle` and `FixedBoundSphere` normalize radius by absolute value
through construction, assignment, and serialized state load. `FixedRay` and
`FixedRay2d` do not normalize direction; returned ray parameters are physical
distances only when the direction is normalized by the caller.

## Boundary Semantics

Default containment and intersection methods are boundary-inclusive:

- `Contains(point)` returns `true` for points on edges, faces, or surfaces.
- `Intersects(...)` treats touching edges, faces, corners, and tangent contact
  as intersections.
- Ray-bound intersections can return `Fixed64.Zero` when the ray starts inside
  or on the queried shape.

Strict overlap methods exist only where downstream systems need to distinguish
touching contact from positive area or volume overlap:

```csharp
bool touchesOrOverlaps = area.Intersects(otherArea);
bool hasPositiveArea = area.IntersectsStrict(otherArea);

bool boxTouchesOrOverlaps = box.Intersects(otherBox);
bool hasPositiveVolume = box.IntersectsStrict(otherBox);
```

`IntersectsStrict` rejects boundary-only contact and zero-area or zero-volume
inputs. Strict frustum overloads are not part of the public surface; frustum
classification uses plane tests and should grow a separate contract only if a
measured caller needs positive-volume frustum semantics.

## Primitives

Segments preserve ordered endpoint identity:

```csharp
FixedSegment segment = new(start3d, end3d);
Vector3d closest = segment.ClosestPoint(point3d);
Fixed64 distanceSquared = segment.DistanceSquared(point3d);
FixedBoundBox bounds = segment.Bounds;
(Vector3d firstPoint, Vector3d secondPoint) = segment.GetClosestPoints(other3d);

FixedSegment2d segment2d = new(start2d, end2d);
bool hasUniqueIntersection = segment2d.TryGetUniqueIntersection(
    other2d,
    out Fixed64 segmentParameter);
(Vector2d firstPoint2d, Vector2d secondPoint2d) = segment2d.GetClosestPoints(other2d);
```

Reversed endpoints produce the same bounds but are not equal. This keeps
directed segment use cases deterministic without hiding identity policy inside
the primitive.

`FixedSegment2d.TryGetUniqueIntersection` uses closed finite segments. A single
shared endpoint is unique, while disjoint segments and collinear
positive-length overlap return `false` with a default parameter. A zero-length
segment is a point: an identical point or a point on the other segment is a
unique intersection. Exact zero, rather than a physics epsilon, classifies
parallel and collinear inputs.

The 2D closest-pair order is the first segment's start, its end, the other
segment's start, then its end. Exact distance ties keep the first candidate,
and candidate distances are compared before public `Fixed64` saturation.
`FixedMath.Lerp` and both `Vector2d.ClosestPointOnLineSegment` and
`Vector3d.ClosestPointOnLineSegment` accept endpoint differences spanning the
complete raw `Fixed64` domain.

The 3D `FixedSegment` closest-point, closest-pair, and squared-distance queries
use exact fixed-width endpoint differences and products across that same raw
domain. An exact Q64.64 squared-length total at or below 2^31 raw units rounds
to zero in Q32.32 and classifies the segment as a point at its start. The
closest-pair solver compares its exact determinant magnitude with
`Fixed64.Epsilon` before choosing the established near-parallel policy, then
rounds parameters half-to-even and clamps them deterministically to the closed
interval [0, 1]. A mathematical zero-separation contact that is already an
endpoint is returned bit-for-bit in both segment orders. `DistanceSquared`
performs one final round-half-to-even conversion of the exact squared sum and
saturates positive results outside the `Fixed64` range to `Fixed64.MaxValue`.

`FixedSegment.Delta`, `Length`, and `LengthSquared` retain ordinary public
saturating vector-arithmetic behavior. They are convenient value properties,
not aliases for the wider intermediate contract of the query methods.

Triangles also preserve ordered vertices:

```csharp
FixedTriangle triangle = new(a3d, b3d, c3d);
Vector3d point = triangle.GetPoint(weightB, weightC);
bool inside = triangle.Contains(point);
Vector3d closest = triangle.ClosestPoint(point);
```

`FixedTriangle2d.TryGetBarycentricWeights(...)` solves planar barycentric
weights directly. `FixedTriangle.TryGetProjectedBarycentricWeights(...)` names
the 3D projection behavior explicitly so callers do not confuse projected
weights with strict on-plane containment.

`FixedTriangle2d` evaluates endpoint differences, cross products, barycentric
interpolation, and distance ordering across the complete raw `Fixed64` domain.
`SignedArea` halves the exact doubled area and performs one final
round-half-to-even conversion, saturating only the final signed result; `Area`
is its nonnegative saturating magnitude. `Centroid` averages each component
without an intermediate three-value sum. `IsDegenerate` uses an inclusive
`Fixed64.Epsilon` area threshold, while `TryGetBarycentricWeights` preserves its
separate inclusive `Fixed64.Epsilon` doubled-area failure threshold and returns
three zero weights on failure. Successful A, B, and C weights come from direct
exact numerators and are rounded and saturated independently.

Containment is winding-independent and includes epsilon-wide edges and
vertices. Exact orientations decide signs and tolerances before public scalar
saturation; collapsed line and point triangles retain their edge-distance
behavior. Closest-point candidates are visited in AB, BC, CA order, compared by
exact squared distance, and exact ties retain the first candidate.

`FixedTriangle` applies the same full-domain ownership to all three coordinate
components. It computes exact cross components and their exact squared sum
before converting public values. `UnnormalizedNormal` rounds each component
half to even and saturates components independently. `Normal` divides the exact
components by the exact magnitude and rounds each result half to even; it does
not normalize the already-saturated public normal. `Area` takes one exact
integer square root, halves at the final Q32.32 boundary, rounds half to even,
and saturates only the final nonnegative result. `Centroid` averages each
component without a potentially saturating three-value sum.

Projected barycentric weights use exact Gram numerators and denominator. A Gram
denominator at or below the inclusive `Fixed64.Epsilon` threshold returns
`false` and three zero weights; successful A, B, and C weights are rounded and
saturated independently. Closest-point Voronoi predicates also remain exact,
and degenerate edge candidates preserve stable AB, BC, CA tie order.
`Contains` remains the inclusive squared-distance epsilon predicate.

This full-domain triangle contract does not change general `Vector3d` cross,
dot, magnitude, or distance operations, and it does not extend to ray
discriminants or quadratic solvers. Those consumers require separate contracts
and evidence.
