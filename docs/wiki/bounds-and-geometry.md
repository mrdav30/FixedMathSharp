# Bounds and Geometry

FixedMathSharp geometry is dimension-explicit and engine-agnostic. It provides
reusable shape math; physics policy such as colliders, materials, broad-phase
layers, impulses, and contact response belongs in a higher-level simulation
package.

Add `using FixedMathSharp.Geometry;` for bounds and most primitives.
`FixedConvexPrismRelations` remains in the root `FixedMathSharp` namespace for
API compatibility.

## Choose a type

### Bounds

| Shape                | Type                | Typical use                                  |
| -------------------- | ------------------- | -------------------------------------------- |
| 2D axis-aligned area | `FixedBoundArea`    | Grids, footprints, planar broad-phase bounds |
| 2D circle            | `FixedBoundCircle`  | Radial planar containment and overlap        |
| 3D axis-aligned box  | `FixedBoundBox`     | Volumes and broad-phase bounds               |
| 3D sphere            | `FixedBoundSphere`  | Radial volume containment and overlap        |
| 3D oriented box      | `FixedOrientedBox`  | Rotated box geometry without cached corners  |
| 3D view frustum      | `FixedBoundFrustum` | Plane-based frustum classification           |

A flat world footprint is a `FixedBoundArea` plus explicit layer, elevation, or
height state in your application. Use `FixedBoundBox` when the query is truly
volumetric.

### Primitives

| Dimension           | Types                                                                         |
| ------------------- | ----------------------------------------------------------------------------- |
| 2D                  | `FixedRay2d`, `FixedSegment2d`, `FixedTriangle2d`, `FixedPointAnchor2d`       |
| 3D                  | `FixedRay`, `FixedPlane`, `FixedSegment`, `FixedTriangle`, `FixedPointAnchor` |
| Cross-shape helpers | `FixedSlabProjection`, convex relation helpers, contact-anchor values         |

Start with the simple bound or primitive that represents your data. Reach for a
centered-axis, sweep, slab, or anchor API only when materializing intermediate
world coordinates would lose information.

## Construct canonical bounds

Named factories keep extent meaning visible:

```csharp
FixedBoundBox room = FixedBoundBox.FromCenterAndSize(
    center,
    new Vector3d(10, 4, 10));

FixedBoundArea footprint = FixedBoundArea.FromCenterAndScope(
    center2d,
    new Vector2d(5, 2));

FixedBoundBox exactEndpoints = FixedBoundBox.FromMinMax(min, max);
```

- `FromMinMax` sorts swapped endpoint components.
- `FromCenterAndSize` takes total size.
- `FromCenterAndScope` takes half-extents.
- Negative size/scope components are normalized by absolute value.

Ordinary centered factories are strict: they throw when a required endpoint is
outside Q32.32. Use a factory named `*ClippedToDomain` only when you explicitly
want the intersection between the conceptual shape and the representable
coordinate domain.

Derived `Size`/`Proportions` values throw when the exact positive span does not
fit in `Fixed64`. `Scope` rounds outward when needed so it does not
underestimate the stored endpoints.

Circle and sphere radii are normalized by absolute value during construction,
assignment, and serialized-state load.

## Boundary rules

Default containment and intersection include the boundary:

- a point on an edge, face, or surface is contained;
- touching edges, faces, corners, and tangencies intersect; and
- a ray that starts inside or on a bound can report `Fixed64.Zero`.

Use `IntersectsStrict` where the API offers it when you need positive area or
volume rather than touch-inclusive overlap.

```csharp
bool touchesOrOverlaps = first.Intersects(second);
bool positiveVolume = first.IntersectsStrict(second);
```

There is no global geometry epsilon. Individual members document whether they
use exact comparison, `Fixed64.Epsilon`, inclusive boundaries, or strict
interiors.

For centered capsules, use `FixedSegment2d.DoCenteredCapsulesOverlapStrict` or
`FixedSegment.DoCenteredCapsulesOverlapStrict` when exact tangency must not
count. `FixedSegment2d.DoesCenteredCapsulePenetrateConvex` provides the
corresponding capsule/polygon decision. These methods classify the exact
geometry before rounding a normal or contact depth. A positive penetration
smaller than half a raw Q32.32 unit can produce a contact depth of zero; testing
`contact.Depth > Fixed64.Zero` is not a substitute for a strict query.

Use the overload that matches the geometry's authoritative representation: an
explicit world axis, a 2D scalar rotation, or a 3D quaternion and local axis. Do
not replace a rigid frame with a separately rounded or renormalized world axis
when exact boundary agreement matters. Zero-length capsule axes are points. Two
capsules with zero combined radius never have strict radial overlap; polygon
queries document their separate minimum-translation rules for zero-radius and
lower-dimensional inputs.

## Full-domain behavior

Geometry routinely compares products and squared distances that are larger than
their final public result. FixedMathSharp widens those intermediates before it
chooses a side, candidate, root, or witness.

The practical rules are:

1. Decisions are made from exact fixed-width intermediates where the member
   promises full-domain behavior.
2. A public point, distance, depth, or parameter is rounded only at the final
   boundary.
3. A `Try*` method reports its documented query/output failure atomically.
   Invalid arguments may still throw.
4. Clipped APIs clip only the conceptual final boundary; they do not silently
   shorten inputs before the query.

See [Full-Domain Arithmetic](full-domain-wide-arithmetic.md) for the numeric
model behind these contracts.

## Axis-aligned and oriented boxes

`FixedBoundBox` is the general 3D axis-aligned bound. It supports containment,
intersection, clamping, projection, merging, and cross-type relations with
spheres, frustums, planes, and rays.

`FixedOrientedBox` stores a center, a normalized quaternion, and strictly
positive half-extents:

```csharp
FixedOrientedBox box = new(center, normalizedRotation, halfExtents);

FixedBoundBox broadPhase = box.GetBoundsClippedToDomain();
Vector3d localSupport = box.GetLocalSupportPoint(direction);

if (box.TryMaterializeLocalPoint(localSupport, out Vector3d worldSupport))
{
    // The selected world point fits in Q32.32.
}
```

`default(FixedOrientedBox)` is invalid. Queries derive an exact rational basis
from the stored quaternion, so negating every quaternion component does not
change the represented geometry. Value equality remains structural, however; the
two stored quaternion signs are different values.

Support and corner features are center-relative. Materialization returns `false`
when the final world point is outside the coordinate domain.

## Rays and segments

Rays do not normalize their direction. A returned ray parameter is a physical
distance only when the caller supplied a normalized direction.

```csharp
FixedRay ray = new(origin, Vector3d.Right);
Fixed64? distance = ray.Intersects(box);

if (distance.HasValue)
{
    Vector3d hit = ray.GetPoint(distance.Value);
}
```

`GetPoint` uses a fused multiply-add per coordinate. `TryGetPoint` uses the same
calculation but returns `false` instead of saturating when a final coordinate is
not representable.

Segments preserve endpoint order. Reversing a segment keeps the same bounds but
produces a different value. Closest-point and closest-pair queries use stable
candidate order for exact ties.

```csharp
FixedSegment path = new(start, end);
Vector3d closest = path.ClosestPoint(point);
Fixed64 distanceSquared = path.DistanceSquared(point);
(Vector3d a, Vector3d b) = path.GetClosestPoints(otherPath);
```

For a long authored path, prefer physical-distance interval APIs when available.
They keep the original chord components until the final mapping into the
caller-supplied distance range. This preserves small transverse motion that
could be lost by normalizing the path first.

## Centered finite shapes

Centered capsule, cylinder, and cone helpers describe endpoints and cap planes
from a center, normalized axis, full axis length, and radius. The conceptual
endpoints do not have to fit in `Fixed64` as long as the requested final result
does.

Use them when a shape sits near a scalar-domain boundary or when combining the
center and half-axis first would saturate.

- Capsule axis length may be zero and then reduces to a circle or sphere.
- Cylinder and cone length must be positive.
- Expanded overloads keep authored dimensions and nonnegative expansion values
  separate to avoid saturating a combined radius or length.
- `FixedSegment2d` owns 2D capsule containment, support, endpoint, distance, and
  intersection helpers.
- `FixedSegment` owns the corresponding 3D capsule helpers plus cylinder, cone,
  swept-sphere, and rounded-box queries.

Capsule/polygon minimum-translation queries retain exact, unnormalized polygon
edge, capsule side, and closest-vertex axes through classification and depth
ordering. Exact tangency is closed contact with zero depth; a gap of one raw
unit remains separated. Only the selected public normal and depth are rounded,
with depth using nearest-even rounding. Contact overloads distinguish a depth
above `Fixed64.MaxValue` from an exactly representable maximum through
`depthIsClamped`. These rules also apply to circle/polygon contacts and the
initial-contact check in capsule/polygon sweeps.

For a single capsule-side contact, the opposing polygon feature determines the
axial position on the capsule. Projection and endpoint clamping use wide
coordinates before rounding the axial parameter, so off-center vertices do not
inherit an arbitrary side midpoint. Anchors retain separate axial and radial
terms even when their world positions are unrepresentable.

The exact overload names and argument preconditions are listed on the
[`FixedSegment2d`](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.Geometry.FixedSegment2d.html)
and
[`FixedSegment`](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.Geometry.FixedSegment.html)
API pages.

### Cylinder contacts

`FixedOrientedBox.TryGetCenteredCylinderContact` includes the box's faces, edges
and vertices against the finite cylinder's caps, side and rims. It finds the
minimum translation without replacing the cylinder with a capsule. A zero radius
is a finite axis segment. The manifold overload additionally clips up to four
matched contacts when a parallel box face and cylinder cap are selected; other
features use the primary contact with a zero manifold count.

`FixedTriangle.TryGetCenteredFiniteCylinderContact` includes the triangle's
face, edges and vertices against the finite cylinder's caps, side and rims. It
returns a triangle-to-cylinder normal, depth and shape-local contact anchors. A
triangle's closest point to the cylinder center is not sufficient to decide
contact: another part of the triangle can cross a flat cap or a curved rim.
Degenerate triangles return false, matching the triangle/slab contact APIs.
`TryGetCircleSlabContact` uses the same geometry for an upright finite cylinder,
while retaining the circle's authored yaw and half-thickness. Half-thickness is
not doubled in `Fixed64`, so a representable slab is not shortened when its full
height exceeds the scalar range. `TryGetCenteredCapsuleSlabContact` describes a
planar capsule (a rectangle with semicircular ends) extruded through a flat Y
slab, not a rounded 3D capsule. Its contact query covers both rounded ends, the
straight sides, and the flat caps, including oblique rim contacts. Contact
points are paired on the selected features; a straight-side or cap contact can
lie between the rounded ends. The query retains the authored local axis and yaw,
including half-raw endpoints from odd raw core lengths. A zero core uses the
cylinder path above, with the same contact anchors.

`FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact` includes the finite
cylinder's caps, side, and rims when finding the minimum translation. It also
handles a capsule core that enters the cylinder: depth is not necessarily just
the capsule radius. A zero-length capsule core is a sphere; a zero-radius
cylinder is its finite axis segment.

`FixedSegment.TryGetCenteredFiniteCylindersContact` similarly finds the minimum
translation between two finite cylinders, including side/rim and rim/rim
constraints. Testing only the axes and their cross product is insufficient for
general finite cylinders. Parallel pairs use a simpler exact path; general
interacting rims require substantially more work. Measure the contact workloads
that occur in your simulation, not only the ordinary parallel case.

Classification and depth ordering use the exact authored geometry. Exact
tangency is contact with zero depth, and a sub-raw separation remains a miss.
Only the selected normal and complete depth are rounded; `DepthIsClamped`
distinguishes conceptual overflow from an exactly representable maximum. Equal
minima follow stable feature order. Continuous radial ties use a representative
in the authored cylinder frame.

The returned anchors retain independently selected shape-local support features.
They are not a promise that two algebraic closest points were rounded together
or that the materialized anchors coincide at every tangency. Box anchors retain
the exact selected direction's support signs even if a tiny normal component
rounds to zero.

Curved oblique contacts require more work than ordinary cap/side contacts. The
query allocates no managed memory, but its exact fallback can use hundreds of
KiB of bounded stack scratch. Avoid deliberately small-stack worker threads;
allow at least a 1 MiB thread stack with adequate headroom for the caller.

Cylinder pairs use the same finite-segment contact path when either radius is
zero. The stack guidance above applies to both positive-radius and zero-radius
oblique pairs. A zero-radius shape's anchor can lie at the segment center when
its whole core supports the selected normal; callers should not require a
particular endpoint in a tie.

### Triangle/cone contacts

`FixedTriangle.TryGetCenteredFiniteConeContact` finds an inclusive minimum-depth
contact with a finite cone whose local axis is +Y: the base is at `-height/2`
and the apex at `+height/2`. It includes triangle face, edge and vertex features
against the base disk, curved rim, apex and lateral generators. The normal
points from triangle toward cone; depth and both anchors come from that same
selected feature.

Equal minima retain the earlier feature. When both axial exits share the minimum
for a horizontal face, the earlier +Up candidate retains the base witness.
Witness coordinates whose radical terms vanish or cancel use exact rational
nearest-even rounding, including signed half-raw ties and scalar extrema.

Both frame rotations must be normalized, height must be positive, and radius
must be nonnegative. A zero radius is the identical axial-segment relation used
by the cylinder contact query. A segment piercing a triangle can require a
positive exit distance even though the segment has no volume. Degenerate
triangles follow `FixedTriangle.IsDegenerate` and return false.

Classification precedes final rounding: exact touching is contact, a genuine
positive gap is not, and a tiny overlap can round to zero depth. Anchors retain
the authored frames even when the relative center or absolute world witness
cannot be materialized as a `Vector3d`. Curved contacts have the same bounded
stack-scratch considerations described above.

The internal cone contact owner can also certify a minimum face exit for a
borrower-owned connected coplanar patch. The borrower supplies trusted seam
topology and every exposed perimeter edge, including holes. Exact projection
membership in a seed triangle establishes membership in the actual patch;
unrounded boundary clearances then certify either a radius-depth tube around an
actual cone chord or the whole cone projection enlarged by that depth. The
center's signed side chooses each perimeter half-plane; exact cone support
clearance preserves concave patches without filling their holes. Rational
quaternion frames and complete cone support avoid rounded intermediate extrema.
A failed certificate leaves complete single-triangle feature selection
available.

For a trusted filled convex polygon, the internal owner also solves its complete
face, perimeter-edge and corner normal fan. An ordered strict-corner loop
excludes collinear subdivisions. Corner charts admit only globally supporting
features before classifying separation; their artificial diagonals never become
contacts. One unreduced rational frame and squared-value scale retain exact
ranking across analytic candidates and stationary rim roots. A complete polygon
fan admits supporting generator slices before materializing paired witnesses.
The borrower establishes convex topology and supplies an intersecting seed for
base-pole witnesses. Supplied local witness bounds may contain the whole patch,
allowing face coordinates outside the seed triangle to round without restricting
them to that triangle. This internal contract does not validate topology or
determine a whole noncoplanar surface union's minimum exit.

`TryGetCenteredFiniteConeSupportContact` is a different operation: it projects
one caller-selected cone support onto the triangle. A failed support projection
does not prove that the complete shapes are separated. Use the complete contact
query when deciding whether the shapes intersect.

## Sweeps and finite-shape intervals

`FixedTriangle.TryGetFiniteConeIntersectionMinimumAxialPoint` finds the first
intersecting surface point along an apex-authored cone's axis. This is not a
minimum-penetration contact. Its rigid-frame overload keeps the triangle origin
and rotation separate from the world-space apex and direction, returning a
`FixedPointAnchor` in the triangle's authored frame. Neither the apex in
triangle space nor transformed vertices in world space need to fit a scalar
coordinate. Classification uses exact rational rotation and a bounded
maximum-scale parameter lattice; the selected local point is rounded only at the
end. Equal candidates retain AB, BC, CA, face order. Axis direction and rotation
must be normalized, height positive, and radius nonnegative.

Interval queries return the closed portion of a bounded segment or ray that
intersects the requested shape. Advanced overloads can also report endpoint
containment independently of rounded interval parameters.

Choose the represented boundary carefully:

- A capsule is the spherical dilation of a segment.
- A finite cylinder has a side and flat caps.
- A swept sphere against a cylinder rounds the cylinder rim; independently
  expanding radius and height describes a different sharp-rim volume.
- A swept sphere against a box rounds edges and corners; expanding each box
  extent describes a larger box, not the same boundary.

This distinction is why the API exposes named sweep methods instead of treating
every query as an expanded axis-aligned bound.

`FixedConvexPrismRelations.IntersectsSweptUprightCylinderStrict` answers the
joint continuous relation between a translated upright cylinder and a rotated
vertical convex prism. It keeps one exact parameter domain for both footprint
and height overlap, so planar tangency, vertical tangency, and intervals that
meet only at one boundary parameter do not become false positive volume
intersections through independently rounded roots.

`FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict` tests the full
continuous planar sweep of a capsule with a Forward-parallel axis against a
convex polygon. Supply start/end **centers**, full axis length, and radius;
total height is axis length plus twice radius. Zero axis length is a circle.
Exact tangency is excluded, including at either endpoint. Zero radius asks
whether the swept axis enters the polygon's interior.

The query keeps conceptual half-axis endpoints exact, including odd-raw lengths,
and does not normalize the chord. Polygon vertices are origin-relative and must
be boundary-ordered and convex in either winding. Repeated vertices and
collinear edges are allowed; a wholly collinear polygon has no interior.
Negative dimensions and fewer than three vertices throw. Convexity remains an
authoring precondition; the hot query does not validate arbitrary polygons. This
is an overlap predicate, not a first-contact distance or a physics response.

For touch-inclusive admission of a single pose, use
`FixedConvex2dRelations.IntersectsUprightCapsule`. It shares the exact shape
math but includes tangency. Do not infer this classification from a rounded
contact normal or penetration depth.

## Triangles and contacts

Triangles preserve vertex order. `FixedTriangle2d` exposes planar barycentric
weights; `FixedTriangle` names its projected barycentric APIs explicitly so a
projection is not confused with strict on-plane containment.

`FixedTriangle.GetClosestPointAnchor` retains the query point's complete rigid
frame, independent local translation and exact support-rounding residual through
the relative-frame predicates. Only the selected triangle-local coordinates are
rounded. This preserves sub-raw edge distinctions and avoids requiring absolute
world points to fit a scalar coordinate.

```csharp
FixedTriangle triangle = new(a, b, c);

Vector3d closest = triangle.ClosestPoint(point);
bool onTriangle = triangle.Contains(point);
bool projectionInside = triangle.ContainsProjection(point);
```

Degenerate triangles retain deterministic edge/point behavior. Closest-feature
ties follow stable edge order.

Rigid triangle-pair queries return `FixedContactAnchors`:

```csharp
bool hit = first.TryGetContact(
    firstOrigin,
    firstRotation,
    secondOrigin,
    secondRotation,
    second,
    out FixedContactAnchors contact);
```

The normal points from the first triangle toward the second. Each anchor stays
in its input rigid frame, so the relation can remain valid even when an absolute
world witness cannot be materialized. Exact touching is included and reports
zero depth.

## Anchors and slab projection

`FixedPointAnchor` and `FixedPointAnchor2d` keep a point in a rigid local frame.
Use relative-offset or frame-reexpression methods when possible; call
`TryGetPoint` only when you actually need an absolute coordinate.

Anchors can compare squared distances exactly without materializing either
distance. Contact relations use them to retain stable local features across
large translations.

`FixedSlabProjection` returns X/Z support for a centered capsule, cylinder, or
cone clipped to an inclusive world-Y range. It is useful for layered spatial
systems that need a finite vertical silhouette, but it is not a collider or
physics-response API.

## Advanced API map

| Task                                                       | Start with                                                                |
| ---------------------------------------------------------- | ------------------------------------------------------------------------- |
| Cross-type bound relations                                 | `FixedBoundArea`, `FixedBoundBox`, `FixedBoundCircle`, `FixedBoundSphere` |
| Oriented-box support and relations                         | `FixedOrientedBox`                                                        |
| Segment/ray versus capsule, cylinder, or cone              | `FixedSegment2d`, `FixedSegment`, `FixedRay2d`, `FixedRay`                |
| Swept sphere versus cylinder or box                        | `FixedSegment`                                                            |
| Centered shape support, containment, or materialization    | `FixedSegment2d`, `FixedSegment` static helpers                           |
| Shape support inside a world-Y layer                       | `FixedSlabProjection`                                                     |
| Swept upright cylinder versus a vertical convex prism      | `FixedConvexPrismRelations`                                               |
| Strict planar sweep of an upright capsule or circle        | `FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict`              |
| Triangle contacts or finite-shape relations                | `FixedTriangle`                                                           |
| Relative witnesses outside ordinary world-coordinate range | `FixedPointAnchor`, `FixedPointAnchor2d`, contact-anchor types            |

Browse the
[`FixedMathSharp.Geometry` API](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.Geometry.html)
for exact overloads, exceptions, tie-breaking rules, and final-result behavior.
