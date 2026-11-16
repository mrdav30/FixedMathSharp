# FMS-Issue-039 Matrix Normalization and Comparison Contracts

## Caller evidence

Reviewed the FixedMathSharp source at `24b0c47` and the local LSF and Unity
package sources. The observed uses have different requirements.

| Caller                                                                                                                                    | Contract in use                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| FixedMathSharp-Unity at `6b9b073`, `Build/Base/Runtime/Extensions/Fixed3x3.Extensions.cs`                                                 | Rotation-only transform helpers feed `Fixed3x3.GetNormalized` to `FixedQuaternion.FromMatrix`. Their sample derives the input from a Unity quaternion, and the EditMode test checks the three axes of a rotation-derived basis. The adapter exposes a separate raw-basis conversion that retains scale and shear. |
| Trailblazer at `c4485c9`, `tests/Trailblazer.Tests/Navigation/Motor/Locomotion/PlatformLocomotion.Tests.cs`                               | The rotation-inheritance test sets a rotation on its identity-seeded platform fixture before calling `NormalizeRotationMatrix`. The production attachment path reads the platform rotation from the transform.                                                                                                    |
| FixedMathSharp `Fixed4x4.Decompose` and `FixedTransform`, and FixedMathSharp-Unity `Build/Base/Runtime/Extensions/Fixed4x4.Extensions.cs` | Strict TRS, inverse and recomposition checks use explicitly bounded absolute per-entry error. Decomposition checks orthogonality separately and rejects shear.                                                                                                                                                    |
| Matrix `FuzzyEqual` calls in the audited source                                                                                           | Direct relative matrix comparisons appeared in tests. The FluentAssertions matrix adapter independently uses explicit absolute component tolerances.                                                                                                                                                              |

The 3×3 rotation path consumes a rotation basis. The known 4×4 normalization
call starts with a rotation as well. The audit found no caller that needs either
normalizer to repair a sheared basis or the relative comparison to use a 1%
default.

## Decision and characterization

Preserved the public contracts and runtime implementations. Row normalization,
strict TRS validation, and relative or absolute comparison remain distinct
operations.

- With scaled right, `(1, 1, 0)` as up, and a scaled negative Z row, both
  normalizers retain a dot product of about `0.7071` between the first two unit
  rows. The 3×3 operation rebuilds positive Z. The 4×4 operation preserves
  negative Z, clears translation and perspective, and produces an affine basis
  that strict decomposition rejects for shear.
- Adding `0.001` to identity's `M11` fails relative comparison with the default
  `Fixed64.Epsilon` fraction (2⁻²⁴) and passes with an explicit `0.01`. Both
  methods already used epsilon before the XML correction in `1a9a270`. The
  default accepts exactly epsilon at an entry of one and rejects one raw unit
  more. With tolerance `0.5`, changing one to two fails while changing two to
  one passes because each error budget is scaled by the first operand.
- The existing indexer regression covers column-major index 1 → `M21`. The new
  tests check that translation in the last row is affine, a nonzero `M14` is
  projective, and an X/Z inverse can succeed with zero Y scale even when the
  full 3D inverse fails.

Revisit orthonormalization if a concrete caller needs to repair a sheared basis
and can specify which axes to preserve, how to handle reflections, and when to
reject degenerate input. The rotation and tolerance contracts are recorded in
the
[coordinate guide](../../api/guides/coordinate-conventions.md#basis-normalization-and-matrix-comparisons).

## Verification

The focused matrix, transform, and assertion suites passed 358 tests in
`Release` and 356 in `ReleaseLean` on Windows with .NET SDK 10.0.302.

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter FullyQualifiedName~MatrixNormalizationAndComparisonTests
```

The new
[characterization tests](../../../tests/FixedMathSharp.Tests/Numerics/Matrices/MatrixNormalizationAndComparison.Tests.cs)
exercise the runtime entry points. An independent .NET 10 probe also ran the
Unity-shaped normalization and quaternion conversion on an authored rotation and
checked all three directions. The downstream audit used source and test
inspection.
