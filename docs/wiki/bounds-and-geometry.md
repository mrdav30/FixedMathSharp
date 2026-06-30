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
- `FixedSegment`: finite 3D segment with closest-point, distance, and bounds.
- `FixedTriangle`: ordered 3D triangle with area, normal, bounds,
  closest-point, containment, interpolation, and projected barycentric helpers.

Use 2D types for plane math:

- `FixedBoundArea`: 2D `Vector2d` axis-aligned bounding area.
- `FixedBoundCircle`: 2D circular bound.
- `FixedRay2d`: 2D ray intersection primitive.
- `FixedSegment2d`: finite 2D segment with closest-point, distance, and bounds.
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
This keeps public bounds state canonical without asking every caller to sort
or sanitize the inputs first.

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
```

Reversed endpoints produce the same bounds but are not equal. This keeps
directed segment use cases deterministic without hiding identity policy inside
the primitive.

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
