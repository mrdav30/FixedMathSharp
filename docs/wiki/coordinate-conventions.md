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

## FixedTransform Contracts

`FixedTransform` is an engine-neutral transform snapshot shell. Its only
authoritative values are `LocalPosition`, normalized `LocalRotation`, and exact
signed or zero `LocalScale`. Adapters copy their host's local components into
those properties; FixedMathSharp does not retain engine objects or synchronize
live scene nodes.

`LocalMatrix` rebuilds those components as scale, rotation, then translation.
With row vectors, a hierarchy composes in child-to-root multiplication order:

```csharp
Fixed4x4 world = child.LocalMatrix * parent.LocalMatrix * grandParent.LocalMatrix;
```

`LocalToWorldMatrix`, `WorldPosition`, and `LossyScale` walk the parent chain
iteratively on every read. Traversal is allocation-free and linear in depth;
there is no child list, matrix cache, dirty propagation, or scene-graph
ownership. Nonuniform scale and rotation at different hierarchy levels may
produce a sheared world matrix. `WorldRotation` deliberately composes the stored
quaternion chain instead of decomposing that matrix, so scale, reflection, and
shear do not change the reported orientation.

`LocalScale` remains the exact authored component value. `LossyScale` is a
derived matrix view: basis magnitudes are preserved, including zero, and an odd
reflection is canonicalized to negative X because a matrix cannot recover which
authored axis originally carried the sign.

Matrix import is explicit through `FixedTransform.TryCreateFromLocalMatrix`. It
accepts only affine, nonsingular, orthogonal TRS matrices that pass strict
decomposition and recomposition checks. Perspective, zero scale, shear,
unrepresentable magnitudes, and non-round-trippable values return `false` and a
null transform. Component construction is the lossless adapter path for signed
and zero scale.

Parent-preserving and world-space mutation methods also fail explicitly when a
required parent inverse is singular, saturated, or does not verify in both
multiplication orders. These operations are atomic: failure does not partially
change local components or parent identity. Reparenting additionally rejects
self and hierarchy cycles.

`TransformPoint` and `InverseTransformPoint` convert through the complete
composed affine hierarchy, including representable shear. Their `Try*`
counterparts report singular, unrepresentable, or final-coordinate failures
atomically. Use the explicit `*PointXZ` variants only for hierarchies that
preserve the X/Z plane; they retain in-plane affine shear and reject any
X/Z-to-Y coupling instead of silently projecting it away.

### X/Z Planar Helpers

The X/Z bridge keeps the existing plane convention. A planar position `(x, y)`
embeds as `(x, 0, y)`, and planar scale `(x, y)` embeds as `(x, 1, y)`.
`LocalPositionXZ` and `LocalScaleXZ` replace X and Z while preserving Y. Planar
rotation is measured from `Vector2d.Right` toward `Vector2d.Forward`, matching
`Vector2d.Rotate`; the local setter uses the negated Y-axis angle. Local and
world getters rotate local right, project onto X/Z, and return `Atan2(z, x)`.
Pure planar values round-trip modulo `Fixed64.TwoPi`.

Chronicler hashes local position, local rotation, and local scale in that stable
order. Parent identity and every derived world view are excluded. Moving from
the earlier ambiguous component names to this authoritative local layout is an
intentional replay/hash compatibility boundary.

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
