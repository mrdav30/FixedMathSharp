# Coordinate Conventions

FixedMathSharp defines its own deterministic math convention instead of inheriting
one from any engine:

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

## Runtime Helpers

`CoordinateConvention3d` is a small, immutable helper for `+Y`-up conventions
whose semantic forward direction is either `+Z` or `-Z`.

```csharp
CoordinateConvention3d core = CoordinateConvention3d.PositiveZForward;
CoordinateConvention3d external = CoordinateConvention3d.NegativeZForward;

Vector3d externalForward = Vector3d.Backward;
Vector3d canonicalForward = external.ToCanonicalDirection(externalForward);

// canonicalForward == Vector3d.Forward
```

Use these helpers for direction vectors at adapter boundaries. For positions in
a coordinate system whose only semantic difference is signed Z forward, the same
Z sign flip may be appropriate, but keep that conversion in adapter code so
units, origins, scale, and storage semantics remain explicit.

## Adapter Boundaries

Keep engine and toolchain conventions outside the core runtime. Convert inputs
before they enter deterministic simulation code, and convert outputs after they
leave it. Avoid global mutable convention settings; deterministic replay should
not depend on process-wide state.

Unity's direction naming aligns with FixedMathSharp's `+Z` forward convention,
but Unity-facing adapters still need to account for Unity's own transform,
matrix, and package semantics.

XNA and MonoGame commonly use opposite forward naming for `+Y`-up, signed-Z
direction helpers. Use `CoordinateConvention3d.NegativeZForward` for those
direction semantics at the adapter boundary.

Other engines, renderers, DCC tools, and file formats may differ by forward axis,
up axis, handedness, row-vector versus column-vector multiplication, matrix
storage layout, projection depth range, or clip-space handedness. Those are
adapter-specific basis conversions, not reasons to rename or flip the core
`Vector3d.Forward` constant.

## Practical Rules

- In core FixedMathSharp code, use `Vector3d.Forward` for semantic forward.
- Convert external direction vectors before calling `FixedQuaternion.FromDirection`,
  `FixedQuaternion.LookRotation`, `Fixed4x4.CreateWorld`, or similar
  convention-heavy APIs.
- Do not add Unity, XNA, MonoGame, or other engine conditionals to the core
  package.
- Do not treat a blind component copy as a semantic conversion for matrices or
  quaternions unless adapter tests prove basis, handedness, multiplication, and
  storage conventions match.
