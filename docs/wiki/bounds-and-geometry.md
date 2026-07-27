# Bounds And Geometry

FixedMathSharp geometry is dimension-explicit. The core package owns reusable
fixed-point shape math only; physics concepts such as colliders, materials,
shape casts, contact manifolds, body state, and broad-phase layers belong in
higher-level.

All bounds and geometry primitives on this page live in
`FixedMathSharp.Geometry`.

## Dimensional Ownership

Use 3D types for volume and spatial math:

- `FixedBoundBox`: 3D axis-aligned bounding box.
- `FixedBoundSphere`: 3D sphere bound.
- `FixedBoundFrustum`: 3D frustum bound.
- `FixedRay`: 3D ray intersection primitive.
- `FixedPlane`: 3D plane classification primitive.
- `FixedSegment`: finite 3D segment with closest-point, closest-pair, distance,
  finite-axis and finite-cone intervals, and bounds.
- `FixedTriangle`: ordered 3D triangle with area, normal, bounds, closest-point,
  containment, interpolation, projected barycentric helpers, and finite-cone
  intersection reduction.
- `FixedSlabProjection`: full-domain X/Z support for centered capsules,
  cylinders, and cones after intersection with a closed world-Y slab.

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

Derived centers use a full-domain nearest-even midpoint. Exact `Size` /
`Proportions` components are returned only when the endpoint span fits in a
positive `Fixed64`; wider spans throw `OverflowException` rather than reporting
a saturated under-size. `Scope` is the smallest representable half-extent that
conservatively contains both endpoints around the lattice center, so odd raw-
unit spans round outward. The complete scalar interval from `Fixed64.MinValue`
through `Fixed64.MaxValue` has neither a representable size nor scope and throws
for both derived properties.

Centered size construction follows the same conservative rule: an odd raw-unit
size expands by one raw unit. Recenter, resize, and centered factory operations
validate every endpoint before committing; an out-of-domain result throws
`OverflowException` and leaves an existing bound unchanged.

Use `FromCenterAndSizeClippedToDomain` or `FromCenterAndScopeClippedToDomain`
only when the desired result is explicitly the intersection between a
mathematical centered bound and the representable Q32.32 coordinate domain.
These named factories saturate only the out-of-domain endpoints and keep the
ordinary centered factories strict.

`FixedBoundBox.FromFiniteConeClippedToDomain` applies that same spatial-proxy
contract to an apex, base center, accepted normalized axis, and radius. It
retains the axis's exact fixed-point squared length, computes the least outward
raw disk extent per coordinate, and clips only conceptual coordinates outside
the representable domain. Cardinal axes use a constant-time fast path.

`FixedBoundCircle.Bounds` is an intentional clipped consumer: when a circle
crosses a scalar face, its derived area contains every representable point of
the circle instead of throwing or pretending to encode coordinates outside the
Q32.32 domain.

`FixedRange` follows the same scalar foundation: `MidPoint` is full-domain and
nearest-even, while `Length` returns the exact signed endpoint difference or
throws `OverflowException` when that difference is not representable.

## Oriented Boxes

`FixedOrientedBox` owns canonical oriented-box geometry without caching world
corners, normals, or axes:

```csharp
FixedOrientedBox box = new(
    center,
    normalizedOrientation,
    positiveHalfExtents);

FixedBoundBox broadPhaseBounds = box.GetBoundsClippedToDomain();
Vector3d localSupport = box.GetLocalSupportPoint(worldDirection);

if (box.TryMaterializeLocalPoint(localSupport, out Vector3d worldSupport))
{
    // Use the representable world-space witness.
}
```

The constructor requires a normalized quaternion and strictly positive
half-extents. `default(FixedOrientedBox)` is invalid. Each operation derives one
scale-invariant rational rotation basis directly from the stored quaternion's
raw components. Classification, support, clamping, and bounds use that exact
conceptual basis. `GetAxes` rounds each rational coefficient to its nearest-even
`Fixed64` value only for callers that need representable vectors; those rounded
views are not fed back into geometry queries. Negating all four quaternion
components therefore leaves every geometric result unchanged. Value equality
remains structural, so quaternion sign variants are distinct stored states even
though their geometry is identical.

Corners and support points are center-relative local features. Corner index bits
select positive X, Y, and Z respectively, matching `FixedBoundBox`; support ties
retain the lower corner index. Closest-surface and nearest-normal ties select X,
then Y, then Z, with zero selecting the positive face. Outside normal queries
select the first violated axis in that same order, matching the point's nearest
clamped face, edge, or corner feature.

Projection, local clamping, face selection, analytical bounds, and
local-to-world materialization retain exact wide intermediates through their
final conversion. Bounds floor their exact minimum endpoints, ceil their exact
maximum endpoints, and clip only conceptual scalar endpoints outside the Q32.32
domain. Materialization rounds each conceptual coordinate once to the
nearest-even lattice value, is atomic, and returns `false` when any selected
world coordinate is not representable. A conceptual face or corner can lie
between lattice points, so materializing that boundary point is not a promise
that the rounded witness will classify as contained; inset local points should
be used when containment after materialization is required.

## Rigid Point Anchors

`FixedPointAnchor` and `FixedPointAnchor2d` retain a conceptual point in a rigid
local frame:

```text
origin + rotation * (localPoint + localDisplacement)
```

The two local terms remain separate. This matters for features such as a capsule
cap-center displacement plus radial support: each term and the final world point
may be representable even when adding the local terms first is not.

Use `TryGetPoint` only when an absolute point is required. Relative geometry
should use `TryGetOffsetFrom` or `TryGetLocalPointIn`; the 3D anchor also
provides scaled and projected-offset helpers. These operations let origin and
feature cancellation occur before the one final half-even narrowing. The 2D
inverse-frame operation divides by the exact squared norm of the represented
sine/cosine pair; it does not assume quantized trigonometric values still form a
mathematically unit basis. Re-expression in another non-cardinal frame therefore
returns the nearest representable local lattice point, while same-frame recovery
remains exact.

Full-domain contact relations return `FixedContactAnchors`. Multi-contact
relations return one primary anchor pair plus compact `FixedContactLocalPoints`
entries that reuse the primary rigid frames, normal, depth, and depth-clamping
state.

## Finite-Slab Projection

`FixedSlabProjection` returns the planar support point of a centered finite 3D
shape after clipping it to an inclusive world-Y interval. Use it when a
higher-level spatial system needs the exact X/Z silhouette of a capsule,
cylinder, or cone inside a finite vertical layer:

```csharp
bool intersectsLayer = FixedSlabProjection.TryGetCylinderSupport(
    center,
    normalizedAxis,
    axisLength,
    radius,
    new FixedRange(layerMinY, layerMaxY),
    Vector2d.Right,
    out Vector2d rightmostPoint);
```

Axes and planar support directions must be normalized. Lengths and radii must be
nonnegative, and cylinders and cones require positive length. The methods return
`false` when the clipped shape is empty or its winning support point is outside
the representable `Fixed64` coordinate domain. Intermediate candidate
construction, comparison, and selection remain exact across the full input
domain; only the final support point is narrowed.

`FixedSlabProjection` itself is a stateless support primitive rather than a
collider API. Physics packages remain responsible for candidate ownership, query
tolerances, and response.

`FixedTriangle.TryGetFiniteSlabProjectedCircleContact` and
`TryGetFiniteSlabProjectedCircleSweep` provide the matching exact
triangle-boundary relation. They retain rigid triangle vertices, finite-Y
clipping intersections, and planar edge crossings as rational values until the
final distance and triangle-local anchor conversions. This avoids deforming a
triangle when its conceptual world vertices cannot be materialized.

For static embedded-volume contact, `FixedTriangle.TryGetCircleSlabContact` and
`TryGetCenteredCapsuleSlabContact` test the complete finite extrusion rather
than only its X/Z projection. They return canonical rigid-frame anchors, an
oriented minimum-translation normal, and a depth with explicit clamping state.
The slab support is retained exactly; the triangle anchor uses the full-domain
relative closest-point relation so face contacts remain tangentially coherent
instead of selecting an arbitrary tied vertex.

`FixedBoundBox.GetVolumeExpansionCost` is the full-domain insertion heuristic
for spatial indexes. It compares the exact Q96.96 volume growth in unsigned
192-bit arithmetic, floors only the final integer result, and clamps that public
`long` metric at `long.MaxValue`. It does not require `Proportions` to be
representable.

`FixedBoundCircle` and `FixedBoundSphere` normalize radius by absolute value
through construction, assignment, and serialized state load. `FixedRay` and
`FixedRay2d` do not normalize direction; returned ray parameters are physical
distances only when the direction is normalized by the caller.
`GetPoint(parameter)` uses a fused multiply-add per coordinate, so
reconstruction does not saturate or round the direction product before adding
the origin. `TryGetPoint(parameter, out point)` uses the same fused calculation
but returns `false` instead of saturating when any final coordinate is not
representable.

`FixedBoundSphere.CreateFromBoundingBox`, `CreateFromFrustum`,
`CreateFromPoints`, and `CreateMerged` retain endpoint differences, distance
ordering, roots, radius sums, and center interpolation in exact wide arithmetic.
Successful construction always returns a sphere that contains the supplied
geometry; radii round outward when the exact distance lies between raw values.
The point and frustum factories retain deterministic Ritter-style construction,
and merge centers must lie on the Q32.32 coordinate lattice, so these APIs do
not promise a mathematically minimum sphere. They throw `OverflowException` when
the selected deterministic construction requires an unrepresentable radius
instead of returning a saturated under-bound sphere. `FixedBoundCircle` has no
corresponding point-cloud or merge factory, so there is no 2D construction
contract to mirror.

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

bool crossesCapsule2d = segment2d.TryGetCapsuleIntersectionInterval(
    capsuleAxis2d,
    capsuleRadius,
    out Fixed64 capsuleEntry2d,
    out Fixed64 capsuleExit2d);

bool crossesCylinder = segment.TryGetFiniteCylinderIntersectionInterval(
    cylinderAxis,
    cylinderRadius,
    out Fixed64 cylinderEntry,
    out Fixed64 cylinderExit);

bool crossesCenteredCylinder = segment.TryGetFiniteCylinderIntersectionInterval(
    cylinderCenter,
    normalizedCylinderAxis,
    cylinderAxisLength,
    cylinderRadius,
    radialExpansion,
    axialExpansion,
    out Fixed64 centeredCylinderEntry,
    out Fixed64 centeredCylinderExit);
```

Reversed endpoints produce the same bounds but are not equal. This keeps
directed segment use cases deterministic without hiding identity policy inside
the primitive.

`FixedSegment2d.TryGetUniqueIntersection` uses closed finite segments. A single
shared endpoint is unique, while disjoint segments and collinear positive-length
overlap return `false` with a default parameter. A zero-length segment is a
point: an identical point or a point on the other segment is a unique
intersection. Exact zero, rather than a physics epsilon, classifies parallel and
collinear inputs.

The 2D closest-pair order is the first segment's start, its end, the other
segment's start, then its end. Exact distance ties keep the first candidate, and
candidate distances are compared before public `Fixed64` saturation.
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
saturating vector-arithmetic behavior. They are convenient value properties, not
aliases for the wider intermediate contract of the query methods.

Finite-axis interval queries likewise own endpoint differences, perpendicular
projection, radial roots, and axial clipping before narrowing. Capsule methods
exist on `FixedSegment2d` and `FixedSegment`; finite-cylinder methods exist on
`FixedSegment`. They return the closed query-parameter interval in `[0, 1]`,
with final parameters rounded half to even. Expanded overloads keep the authored
radius and nonnegative radius expansion separate so callers do not saturate a
combined radius first.

The endpoint-classification overloads report inclusive start containment and
strict end containment from the same wide inputs, independently of rounded
parameters. A zero-length capsule axis reduces to a circle or sphere. A
zero-length cylinder axis is rejected because an endpoint pair cannot retain a
flat-cap normal. Centered cylinder overloads accept the positive full axis
length plus separate radial and axial expansions. They expand both cap planes
without first constructing potentially saturated endpoints.

`FixedSegment.TryGetSweptSphereFiniteCylinderIntersectionDistance` and its
interval overload instead solve the exact Minkowski sum of a centered finite
cylinder and a sphere. The boundary keeps the expanded cylindrical side and cap
faces, but rounds each cap rim rather than filling the corners of an
independently expanded radius and height. Use this contract for an exact
swept-sphere-versus-cylinder query; use affine expansion only when a sharp-rim
cylinder is the intended volume. The solver retains the accepted axis's exact
squared raw length, wide side/cap/rim arithmetic, repeated-root tangencies, and
full-domain chord interpolation through one final round-half-to-even physical-
distance conversion. The entry-only overload skips refining the toroidal-rim
exit root when the caller needs only the first contact.

`FixedSegment.TryGetSweptSphereBoxIntersectionDistance` provides the matching
first-contact query for the exact spherical dilation of a `FixedBoundBox`.
Unlike expanding each box extent by the sphere radius, the represented boundary
keeps planar faces and rounds its edges and corners. The allocation-free solver
retains full-domain authored chord differences, exact feature-transition
ordering, and wide squared-distance quadratics until one final
round-half-to-even conversion into the caller-supplied physical-distance range.

Finite-cone methods on `FixedSegment` accept either an apex plus normalized
apex-to-base direction and parametric height, or a center plus normalized
base-to-apex direction and full parametric height. The conceptual endpoint is
formed by scaling the supplied near-unit fixed axis; the solver carries that
axis's exact squared raw length instead of pretending every accepted normalized
vector has a mathematically exact unit length. The centered form also doubles
its axial coordinate in wide arithmetic, so an odd raw-unit height is not
rounded away. They return the closed segment interval across the side, apex,
flat base, and rim as one convex-volume result. Endpoint classification and
point-containment overloads use the same inclusive-boundary and strict-interior
contract as the finite-axis families. Axial clipping, conic coefficients,
discriminant evaluation, and root selection remain in fixed-width wide
arithmetic until the final half-even conversion. Use the physical-distance
overload when the segment length is available and distinct spatial hits must not
be collapsed by a very long chord's Q32.32 parameter. Point-interval overloads
instead return deterministic high-resolution lattice witnesses through exact
authored-chord interpolation, which is preferable when the consumer needs hit
positions rather than segment parameters.

`Vector3d.ProjectNonNegativeDifferenceParameter` and
`GetNormalizedProjectionOnPlane` provide the corresponding q-aware consumer
operations. They retain exact endpoint differences and the supplied axis norm,
so an accepted near-unit fixed vector is not silently treated as a
mathematically exact unit vector before the final projection result is narrowed.

The same capsule families are available directly on `FixedRay2d` and `FixedRay`;
finite-cylinder families are available on `FixedRay`. Ray methods require an
explicit nonnegative maximum parameter and solve directly in the closed interval
`[0, maxParameter]`. They do not convert a long ray to a unit segment parameter,
so a normalized direction returns physical-distance values without losing
raw-unit ordering during a later rescale. Direction is otherwise unconstrained,
and returned values retain ordinary ray-parameter semantics. Advanced overloads
report inclusive origin containment and strict containment at the bounded
maximum independently of rounded interval endpoints.

When a finite authored path—not a ray—is the source of truth, use the matching
segment physical-distance interval APIs. They retain the exact original chord
components and map parameter `[0, 1]` to caller-supplied `[0, totalDistance]`
only at the final half-to-even conversion. This avoids the information loss of
normalizing a long chord whose small transverse component is still physically
meaningful. `GetPointAtDistance(distance, totalDistance)` reconstructs a
returned hit with the same exact chord contract, rejects values outside that
closed range, and returns exact authored endpoints at zero and the total
distance. A zero total distance is valid only when both authored endpoints are
equal; this lets overlap workers classify and reconstruct a point query without
a separate downstream branch.

For centers near the scalar-domain boundary, prefer the centered capsule and
cylinder overloads. Their axis direction must already be normalized, and their
full physical axis length remains separate from the center. Capsule length may
be zero and then uses the circle/sphere limit; cylinder length must stay
positive. The wide solvers define conceptual endpoints and cap planes
parametrically as `center +/- direction * (axisLength / 2)` and never construct
them as `Fixed64` coordinates, so a saturated coordinate cannot shorten or
rotate the axis.

The matching centered-axis helpers keep closest-feature selection exact as well.
`ContainsPointInCenteredCapsule` compares an optional radial expansion in wide
arithmetic and accepts explicit strict mode when surface contact must be
excluded. `ContainsPointInCenteredFiniteCylinder` provides the same exact
inclusive/strict choice for the radial side and flat caps.
`GetDirectionFromCenteredAxis` returns the normalized radial direction, or zero
for a point on the axis. Callers that need a surface point can supply that
direction (or an explicit deterministic on-axis direction) to
`TryGetSurfacePointOnCenteredCapsule`; it fuses the conceptual axis point and
radial offset before one final half-to-even coordinate conversion. It returns
`false` only when the final surface point itself is outside the scalar domain.
`GetDistanceToCenteredCapsule` and `TryGetDistanceToCenteredCapsule` instead
return the exact closest gap without requiring that surface point to be
representable. Inside and boundary points return zero; positive gaps use one
final half-to-even conversion. An unrepresentable result saturates to
`Fixed64.MaxValue`, and the `Try` form additionally returns `false`.
`TryGetClosestPointsBetweenCenteredAxes` performs the same full-domain
closest-feature solve for two 2D or 3D centered axes and narrows only the two
selected world points. It returns `false` with zero outputs if either witness is
outside the scalar domain.

`TryGetCenteredAxisEndpoint` materializes one explicitly selected conceptual
endpoint only when a caller needs a world point. The matching capsule support
overloads exist in 2D and 3D; finite-cylinder and finite-cone support overloads
are 3D. They accept any search-direction magnitude, retain the center, axial,
and radial terms until one final half-even conversion, and return `false` with a
zero output when the selected point is outside the scalar domain. Axial ties
select the negative endpoint, cylinder directions parallel to the axis select
the cap center, and cone apex/base ties select the base.

The `TryGetCentered*CapsuleSlabAxisPenetration` family compares a 3D capsule,
finite cylinder, or finite cone with a planar capsule extruded through a world-Y
slab on one normalized projection axis. Center, axial, radial, and slab terms
remain separate through the overlap decision, so scalar-face placement cannot
saturate before separation or oriented-depth selection. The methods return the
minimum oriented overlap along that axis, clamp only an unrepresentable final
positive depth, and return `false` for separation.

`FixedBoundArea.FromCenteredCapsuleClippedToDomain` and
`FixedBoundBox.FromCenteredCapsuleClippedToDomain` derive tight analytical 2D
and 3D axis-aligned bounds from the full axis length.
`FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain` provides the matching
3D finite-cylinder bound. These factories round extents outward and clip only
the final bounds to the representable coordinate domain.

Triangles also preserve ordered vertices:

```csharp
FixedTriangle triangle = new(a3d, b3d, c3d);
Vector3d point = triangle.GetPoint(weightB, weightC);
bool inside = triangle.Contains(point);
bool projectionInside = triangle.ContainsProjection(offPlanePoint);
Vector3d closest = triangle.ClosestPoint(point);
bool intersectsCone = triangle.TryGetFiniteConeIntersectionMinimumAxialPoint(
    apex,
    normalizedApexToBaseDirection,
    coneHeight,
    baseRadius,
    out Vector3d conePoint);
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

Containment is winding-independent and includes epsilon-wide edges and vertices.
Exact orientations decide signs and tolerances before public scalar saturation;
collapsed line and point triangles retain their edge-distance behavior.
Closest-point candidates are visited in AB, BC, CA order, compared by exact
squared distance, and exact ties retain the first candidate.

`FixedTriangle` applies the same full-domain ownership to all three coordinate
components. It computes exact cross components and their exact squared sum
before converting public values. `UnnormalizedNormal` rounds each component half
to even and saturates components independently. `Normal` divides the exact
components by the exact magnitude and rounds each result half to even; it does
not normalize the already-saturated public normal. `Area` takes one exact
integer square root, halves at the final Q32.32 boundary, rounds half to even,
and saturates only the final nonnegative result. `Centroid` averages each
component without a potentially saturating three-value sum.

Projected barycentric weights use exact Gram numerators and denominator. A Gram
denominator at or below the inclusive `Fixed64.Epsilon` threshold returns
`false` and three zero weights; successful A, B, and C weights are rounded and
saturated independently. `ContainsProjection` classifies those exact numerators
directly, includes projected edges, and returns `false` for a degenerate
projected face. Closest-point Voronoi predicates also remain exact, and
degenerate edge candidates preserve stable AB, BC, CA tie order. `Contains`
remains the inclusive squared-distance epsilon predicate.

`TryGetFiniteConeIntersectionMinimumAxialPoint` reduces the three stable edges
and the triangle face against an apex-authored finite cone. The normalized axis
keeps its exact fixed-point squared length; plane, conic, and half-space
predicates stay in fixed-width wide arithmetic. The returned point is the
deterministic maximum-scale lattice witness for the earliest admitted candidate,
rounded only at the public coordinate boundary. AB, BC, CA, then face order
resolves exact ties. Degenerate triangles retain edge-only behavior.

This full-domain triangle contract does not change general `Vector3d` cross,
dot, magnitude, or distance operations, and it does not extend to ray
discriminants or quadratic solvers. Those consumers require separate contracts
and evidence.
