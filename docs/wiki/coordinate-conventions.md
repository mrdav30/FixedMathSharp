# Coordinate Conventions

FixedMathSharp defines its own deterministic math convention instead of
inheriting one from any engine:

- 3D core basis: `+X` is right, `+Y` is up, and `+Z` is forward.
- 3D cross-product orientation: `Vector3d.Cross(Vector3d.Right, Vector3d.Up)`
  returns `Vector3d.Forward`.
- 4x4 transforms use row vectors: points and vectors are transformed as
  `value * matrix`, with translation stored in `M41`, `M42`, and `M43`.
- 2D plane math uses `Vector2d.Forward == (0, 1)`.
- 2D polar angle helpers are separate: `Vector2d.ForwardDirection(Fixed64.Zero)`
  returns `Vector2d.Right`.
- `Vector4d` is component math and homogeneous-coordinate math. It has no
  independent forward or backward semantics.

## Angle And Rotation Inputs

Quaternion angle constructors accept the complete finite `Fixed64` domain.
Radian inputs are reduced by the deterministic trigonometric functions, so
multi-turn angles represent the same rotation modulo quaternion sign. Degree
constructors convert with one fused round-half-to-even operation before using
the same radian path; `DegToRad` is representable for every input, while
`RadToDeg` saturates only when the final degree value is outside `Fixed64`.

`FixedQuaternion.FromAxisAngle` and `FixedQuaternion.AngleAxis` normalize any
nonzero finite axis scale-safely. A zero axis deterministically returns
`FixedQuaternion.Identity`.

## X/Z Planar Transforms

`FixedTransform` exposes an explicit bridge between `Vector2d` plane math and
the core X/Z ground plane. A planar position `(x, y)` embeds as `(x, 0, y)`, and
a planar scale `(x, y)` embeds as `(x, 1, y)`. The planar constructor accepts an
optional `Parent` reference, but `FixedTransform` does not compose hierarchy
state. `Scale` is therefore the authored component scale, not a lossy or
hierarchy-derived world scale.

`PositionXZ` and `ScaleXZ` use the existing `ToVector2d`/`ToVector3d` mapping.
Their setters replace X and Z while preserving the current Y elevation or Y
scale. Component construction and assignment preserve negative and zero scale
exactly. Rotation is stored as a normalized quaternion.

Planar rotation is measured in radians from `Vector2d.Right` toward
`Vector2d.Forward`, matching `Vector2d.Rotate`. Setting `RotationXZRadians`
replaces pitch and roll with a pure rotation around `Vector3d.Up`; the
quaternion uses the negated planar angle so positive planar rotation maps local
right toward `+Z`. Getting it rotates local right, projects that direction onto
X/Z, and returns `Atan2(z, x)`. This is deterministic and independent of scale
even when the quaternion also contains pitch or roll. A zero projected
local-right direction reports zero radians. Pure planar values round-trip
modulo `Fixed64.TwoPi`.

The matrix constructor calls `Fixed4x4.Decompose` once and stores its resulting
components. It deliberately inherits that method's existing canonicalization:
scale magnitudes are extracted from basis rows, an odd handedness change is
represented with a negative X scale, and zero scale magnitudes become one to
avoid division by zero. Multiple negative axes are not uniquely recoverable and
may be absorbed into the decomposed rotation. `Decompose` also does not reject
shear, perspective, or other non-TRS matrices, so those inputs receive its
established extraction result rather than a promised lossless TRS round trip.
Component-constructed transforms do not pass through matrix decomposition and
therefore do not inherit those ambiguities.

## Runtime Helpers

`CoordinateConvention3d` is a small, immutable helper for direction vectors at
adapter boundaries. It describes an explicit semantic basis with signed
right/up/forward axes.

```csharp
CoordinateConvention3d core = CoordinateConvention3d.PositiveZForward;
CoordinateConvention3d external = CoordinateConvention3d.NegativeZForward;

Vector3d externalForward = Vector3d.Backward;
Vector3d canonicalForward = external.ToCanonicalDirection(externalForward);

// canonicalForward == Vector3d.Forward
```

Use `CoordinateConvention3d.XForwardZUp` for adapter boundaries where semantic
forward is `+X`, semantic right is `+Y`, and semantic up is `+Z`.

```csharp
CoordinateConvention3d external = CoordinateConvention3d.XForwardZUp;

Vector3d externalForward = Vector3d.Right;
Vector3d canonicalForward = external.ToCanonicalDirection(externalForward);

// canonicalForward == Vector3d.Forward
```

Use these helpers for direction vectors and basis component mapping at adapter
boundaries. For positions in a coordinate system whose only differences are axis
signs or permutations, the same basis mapping may be appropriate, but keep that
conversion in adapter code so units, origins, scale, and storage semantics
remain explicit.

## Adapter Boundaries

Keep engine and toolchain conventions outside the core runtime. Convert inputs
before they enter deterministic simulation code, and convert outputs after they
leave it. Avoid global mutable convention settings; deterministic replay should
not depend on process-wide state.

Unity's direction naming aligns with FixedMathSharp's `+Z` forward convention,
but Unity-facing adapters still need to account for Unity's own transform,
matrix, and package semantics.

MonoGame is the maintained, open-source XNA-compatible framework. Its
`Vector3.Forward` is `(0, 0, -1)`, so use
`CoordinateConvention3d.NegativeZForward` for MonoGame direction semantics at
the adapter boundary. Treat XNA as legacy context when explaining why MonoGame
uses the `Microsoft.Xna.Framework` namespace and XNA-style API names.

Unreal-style coordinate spaces use `+X` forward, `+Y` right, and `+Z` up. Use
`CoordinateConvention3d.XForwardZUp` for direction basis mapping when an adapter
matches that convention.

Other engines, renderers, DCC tools, and file formats may differ by forward
axis, up axis, handedness, row-vector versus column-vector multiplication,
matrix storage layout, projection depth range, or clip-space handedness. Those
are adapter-specific basis conversions, not reasons to rename or flip the core
`Vector3d.Forward` constant.

## Practical Rules

- In core FixedMathSharp code, use `Vector3d.Forward` for semantic forward.
- Convert external direction vectors before calling
  `FixedQuaternion.FromDirection`, `FixedQuaternion.LookRotation`,
  `Fixed4x4.CreateWorld`, or similar convention-heavy APIs.
- Do not add Unity, MonoGame, Unreal, legacy XNA, or other engine conditionals
  to the core package.
- Do not treat a blind component copy as a semantic conversion for matrices or
  quaternions unless adapter tests prove basis, handedness, multiplication, and
  storage conventions match.
