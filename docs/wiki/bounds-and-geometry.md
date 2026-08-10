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

The exact overload names and argument preconditions are listed on the
[`FixedSegment2d`](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.Geometry.FixedSegment2d.html)
and
[`FixedSegment`](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.Geometry.FixedSegment.html)
API pages.

## Sweeps and finite-shape intervals

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

## Triangles and contacts

Triangles preserve vertex order. `FixedTriangle2d` exposes planar barycentric
weights; `FixedTriangle` names its projected barycentric APIs explicitly so a
projection is not confused with strict on-plane containment.

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
| Triangle contacts or finite-shape relations                | `FixedTriangle`                                                           |
| Relative witnesses outside ordinary world-coordinate range | `FixedPointAnchor`, `FixedPointAnchor2d`, contact-anchor types            |

Browse the
[`FixedMathSharp.Geometry` API](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.Geometry.html)
for exact overloads, exceptions, tie-breaking rules, and final-result behavior.
