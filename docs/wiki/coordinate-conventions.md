# Coordinate Conventions

FixedMathSharp defines its own conventions instead of inheriting them from a
game engine. Convert external values at the adapter boundary, then keep the
simulation on one canonical basis.

## Core convention

| Concept              | FixedMathSharp convention     |
| -------------------- | ----------------------------- |
| Right                | `+X`                          |
| Up                   | `+Y`                          |
| Forward              | `+Z`                          |
| 3D cross orientation | `Right × Up = Forward`        |
| 4x4 transform form   | Row vectors: `value * matrix` |
| 4x4 translation      | `M41`, `M42`, `M43`           |
| 2D forward           | `Vector2d.Forward == (0, 1)`  |

```csharp
Vector3d forward = Vector3d.Cross(Vector3d.Right, Vector3d.Up);
// forward == Vector3d.Forward
```

`Vector2d.ForwardDirection(Fixed64.Zero)` is a polar-angle helper and returns
`Vector2d.Right`. The zero angle starts on `+X`; that does not change the named
2D `Forward` direction.

`Vector4d` provides component and homogeneous-coordinate math. It does not have
its own forward/back convention.

## Rotations and angles

Quaternion angle factories accept the complete finite `Fixed64` domain.
Trigonometric functions reduce multi-turn radian inputs deterministically.

`FixedQuaternion.FromAxisAngle` and `AngleAxis` normalize any nonzero finite
axis with a scale-safe path. A zero axis returns `FixedQuaternion.Identity`.
Degree factories use the same rotation path after deterministic conversion to
radians.

```csharp
FixedQuaternion quarterTurn = FixedQuaternion.FromAxisAngle(
    Vector3d.Up,
    Fixed64.HalfPi);

Vector3d turned = quarterTurn.Rotate(Vector3d.Forward);
```

## Matrix composition

Because transforms use row vectors, a child-to-root hierarchy multiplies in that
same order:

```csharp
Fixed4x4 world = child.LocalMatrix
                 * parent.LocalMatrix
                 * grandParent.LocalMatrix;
```

Do not infer multiplication convention from field layout alone. When importing
another library's matrix, verify its vector side, composition order, handedness,
translation fields, and clip-space rules.

## `FixedTransform`

`FixedTransform` is an engine-neutral snapshot of local transform state. Its
authoritative values are:

- `LocalPosition`
- normalized `LocalRotation`
- exact signed or zero `LocalScale`

`LocalMatrix` rebuilds those components as scale, rotation, then translation.
World position, matrix, rotation, and lossy scale are derived by walking the
parent chain. The type does not own engine objects, child collections, dirty
flags, or a matrix cache.

This distinction matters with nonuniform scale:

- The composed world matrix may contain shear.
- `WorldRotation` composes the stored quaternion chain; it does not decompose
  the sheared matrix.
- `LossyScale` reports basis magnitudes and canonicalizes an odd reflection to
  negative X because a matrix cannot recover the originally negative axis.

`FixedTransform.TryCreateFromLocalMatrix` accepts only affine, nonsingular,
orthogonal TRS matrices that survive strict decomposition and recomposition.
Perspective, shear, zero scale, unrepresentable values, and non-round-trippable
inputs return `false`.

World-preserving mutation and reparenting are atomic. They fail without partial
changes when a required inverse is singular or cannot be verified. Reparenting
also rejects self-parenting and hierarchy cycles.

## X/Z planar helpers

The 2D-to-3D bridge maps `(x, y)` to `(x, 0, y)`. Planar scale maps to
`(x, 1, y)`. Explicit `*PointXZ` methods retain representable in-plane affine
shear and reject X/Z-to-Y coupling instead of silently projecting it away.

Use these helpers only when the hierarchy is intended to preserve the X/Z plane.

## Mapping external direction conventions

`CoordinateConvention3d` describes a signed right/up/forward basis and converts
direction components without global mutable state.

```csharp
CoordinateConvention3d external = CoordinateConvention3d.NegativeZForward;

Vector3d canonical = external.ToCanonicalDirection(Vector3d.Backward);
// canonical == Vector3d.Forward
```

Common starting points:

| External semantic basis           | Helper             |
| --------------------------------- | ------------------ |
| `+X` right, `+Y` up, `+Z` forward | `PositiveZForward` |
| `+X` right, `+Y` up, `-Z` forward | `NegativeZForward` |
| `+Y` right, `+Z` up, `+X` forward | `XForwardZUp`      |

Unity direction naming aligns with `+Z` forward. MonoGame uses `-Z` for
`Vector3.Forward`. Unreal-style coordinates commonly use `+X` forward and `+Z`
up. These helpers cover direction-basis mapping; they do not automatically
handle units, origins, quaternion storage, matrix layout, or projection depth.

## Adapter checklist

Before copying a vector, quaternion, or matrix across a boundary, verify:

1. right, up, and forward axes;
2. handedness and cross-product orientation;
3. row-vector versus column-vector multiplication;
4. matrix storage and translation fields;
5. angle units and quaternion component order;
6. world units, origin, and scale; and
7. projection and clip-space conventions.

Keep those conversions in the adapter. Core deterministic code should always be
able to treat `Vector3d.Forward` as semantic forward.
