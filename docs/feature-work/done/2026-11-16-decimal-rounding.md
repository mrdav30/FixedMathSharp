# FMS-Issue-041 Decimal Rounding

At `723a05024765c7d11d135dc40364ce1c9df12ad0`, decimal rounding first scaled the
input in `Fixed64`, rounded that saturated value, added a raw bias, and divided
by the decimal factor. When the scaled value reached `MaxValue`, the raw
addition could wrap through the negative limit.

| One-decimal-place input      |   Previous raw result |   Corrected raw result |
| ---------------------------- | --------------------: | ---------------------: |
| `Fixed64.FromDecimal(0.2m)`  |           `858993460` |            `858993459` |
| `new Fixed64(1_000_000_000)` | `-922337203685477580` |  `4294967296000000000` |
| `Fixed64.MaxValue`           | `-922337203685477580` |  `9223372036854775807` |
| `Fixed64.MinValue`           | `-922337203685477580` | `-9223372036854775808` |

`RoundToPrecision` now scales the whole and fractional words separately to
select the exact decimal grid point. It encodes that point with the existing
nearest-even, saturating scalar conversion. Positive `MaxValue` at one decimal
place selects `2147483648`, which is one raw unit above `MaxValue`, so the
returned value saturates. At zero places the method delegates to
`FixedMath.Round`. The decimal midpoint rules and the supported precision range
stay the same.

The
[BigInteger regression oracle](../../../tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs)
uses independent signed rational arithmetic for both rounding steps. It checks
15 signed raw inputs across all ten precisions and both midpoint rules, through
both the static method and its extension. Nine of the fifteen cases failed
against the unmodified runtime, and six passed as controls.

Both solution builds succeeded for `netstandard2.1` and `net10.0`. The full
suites passed 7,468 tests in `Release` and 7,447 in `ReleaseLean`. Focused
coverage reached all 18 instrumented lines and 14 branch outcomes in the two
rounding entry points in each configuration.

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter FullyQualifiedName~Round_WithDecimalPlaces
```

## Related API decisions from triage

- Preserve the two integer conversions. `ToInt(-Fixed64.Half)` gives `-1` by
  flooring, and `(int)(-Fixed64.Half)` gives `0` by truncating toward zero. Both
  rounding policies are useful and already documented.
  [Conversion contracts](../../../src/FixedMathSharp/Numerics/Scalars/Fixed64.Conversions.cs)
- Preserve the reflection precondition and the cross-product signs.
  `Reflect(Right, (2, 0, 0))` gives `(-7, 0, 0)` under the supplied-normal
  formula, and a unit `Right` normal gives `(-1, 0, 0)`. The signed 2D
  determinant of `Forward, Right` is `-1`. In 3D, `Up × Right` is `-Forward` and
  its component sum is `-1`.
  [Reflection and 3D cross product](../../../src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs),
  [2D determinant](../../../src/FixedMathSharp/Numerics/Vectors/Vector2d.cs)
- **Direction API.** `GetForwardDirection()` returns local forward in world
  axes. `GetWorldForwardInLocalFrame()` returns canonical world forward in local
  axes. A positive quarter turn about `Up` gives `+X` and `-X`, respectively.
  The initial change retained `ToDirection()` as an obsolete forwarding alias.
  The v9 change removes the alias in favor of `GetWorldForwardInLocalFrame()`.
  [Direction contract and migration](../../api/guides/coordinate-conventions.md#rotations-and-angles)
- **Zero-quaternion proposal at triage.** The proposal treats `Identity` as the
  unchanged rotation and `Zero` and `default(FixedQuaternion)` as invalid
  rotations. Checked rotation and point transforms would return `false` with a
  default result for zero, and a throwing rotation would report the invalid
  receiver. Currently `Zero.Rotate(Forward)` gives `Forward`, `Zero.TryRotate`
  succeeds with a zero vector, and the single-origin `Zero.TryTransformPoint`
  succeeds with that origin. Reopen this as a separate compatibility
  investigation if the zero-quaternion contract is reconsidered. Start with
  default-initialized quaternions and callers that discard `Try*` results.
  [Rotation and transform methods](../../../src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.cs),
  [zero regression](../../../tests/FixedMathSharp.Tests/Numerics/Rotations/FixedQuaternion.Tests.cs)

### Zero-quaternion caller audit

Reviewed FixedMathSharp at `0eeec9e`, Gravitas at `c028f25d`, and Trailblazer at
`2f0fbfb`. The current checked forward transforms have a deliberate legacy
result for zero. `TryRotate` succeeds with a zero vector, the one-origin and
scaled point transforms succeed with the origin, and the two-origin point and
relative-offset overloads drop the rotated local term and check the remaining
sum or difference. The inverse scaled transform already fails with a zero
output. `Rotate` uses the identity fallback through normalization.

- **Gravitas body lifecycle.** Construct a `SolidBody` with a bound agent and
  collider, then read it before calling `Initialize`. `Rotation` is zero,
  `Forward` returns canonical forward, and `TryGetWorldCenterOfMass` succeeds
  with the body position. A throwing `Rotate` or a failing forward point
  transform would alter those results. Public initialization, reset, and
  rotation admission normalize zero to identity. Body restore also normalizes
  the loaded value. The rotation setter is private.
- **Trailblazer pose admission.** On two new, context-bound navigators, call
  `Setup` with the same valid position and profile. Omit the rotation for the
  first caller and supply `default(FixedQuaternion)` explicitly for the second.
  The first stores identity and the second stores zero. Both produce canonical
  forward. The restore path likewise assigns a stored rotation before deriving
  `Forward`. Setup was reproduced directly. The explicit zero restore path was
  traced in source.
- **Ignored status.** FixedMathSharp capsule and finite-surface code and
  Gravitas mesh, sweep, and mixed-cone code consume `TryRotate` outputs without
  checking the status. Returning `false` with the same zero output leaves these
  sites unable to distinguish a failed rotation from a valid zero result. The
  audited public capsule contact path validates both rotations. Gravitas
  `PhysicsMesh.UpdatePosition` with a zero quaternion was rejected by frame
  validation. Checked consumers include Gravitas joint anchor construction and
  the world's center-of-mass computation for a body.
- **Unity boundary.** The authored FixedMathSharp-Unity adapter copies all four
  quaternion components, so an all-zero Unity quaternion remains zero at
  conversion. This was checked in source.

Focused FixedMathSharp suites passed 178 tests in `Release` and 177 in
`ReleaseLean`. The local source-stack `Release` runs passed 110 Gravitas body,
joint, and mesh tests and 119 Trailblazer navigator and context tests.
Disposable public-entry-point probes confirmed the body lifecycle, mesh
admission, and navigator setup described above.

Before changing the shared contract, decide where pose owners should use
identity or reject explicit zero, and handle or prove away failures wherever the
caller consumes a checked rotation's output. Changing the `Try*` status alone
would leave ignored results ambiguous.

### Strict rotation rollout

Gravitas and Trailblazer moved pose validation to their admission boundaries
before FixedMathSharp changed. Gravitas uses identity for a fresh body and for
an omitted reset rotation. Schema 1 retains its existing interpretation of
legacy zero as identity. Trailblazer writes spatial navigator schema 11 and
migrates schema 10 zero poses to identity when loading. The migrated navigator
continues like an identity-seeded control from the restored frame. Historical
replays that applied later deltas to raw zero can differ after migration.
Invalid nested turn rotations reject before live state changes.

FixedMathSharp rotation operations reject zero. Checked rotation, point and
offset operations return false and a default output, and applying a zero world
rotation through `FixedTransform.TrySetWorldPose` leaves the existing pose
intact. Direct rotation, vector rotation operators, and direction or matrix
conversions report the invalid input. Explicit quaternion normalization still
offers its existing identity fallback. The audited quaternion `TryRotate` call
sites now handle or propagate failure.

A default Trailblazer `TrekRequest` supplies identity. Assigning zero to the
request explicitly fails. The request records retain their fields and read a
legacy stored zero as an omitted orientation.

On Windows with .NET SDK 10.0.302, the source-stack save and replay suites
passed in Release and ReleaseLean before the FixedMathSharp contract was
switched. Afterward, the full FixedMathSharp solution and the Gravitas,
Trailblazer, and Trailblazer-to-Gravitas adapter test projects passed in both
configurations.

| Suite                        | Release | ReleaseLean |
| ---------------------------- | ------: | ----------: |
| FixedMathSharp solution      |   7,487 |       7,466 |
| Gravitas                     |   8,516 |       8,439 |
| Trailblazer                  |   3,786 |       3,672 |
| Trailblazer Gravitas adapter |     118 |         111 |

JSON and MemoryPack tests confirmed that a legacy Gravitas body continues with
the identity control's replay hash after a complete simulation step. Trailblazer
tests confirmed that schema 10 retains a nonzero authored rotation, migrates a
zero pose and request to identity, and continues with the identity control's
next committed pose. Malformed schema 11 zero poses and zero turn targets fail
before replacing the live navigator state. DocFX builds and local site link
checks passed in all three repositories.

The existing `QuaternionBenchmarks.RotateVector` benchmark averaged 33.57 µs
before the change and 33.95 µs after it per batch of 256 ordinary rotations.
Both builds had two successful out-of-process launches and allocated zero bytes.
The difference is within the reported measurement uncertainty.
