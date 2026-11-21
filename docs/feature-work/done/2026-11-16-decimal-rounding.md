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

### Coverage follow-up (2026-11-21)

A clean package-mode `Release` coverage run at
`3da3a8ad9964d3e0413080737fec12ba107b48e4` passed all 7,442 core tests but
covered 68,858 of 68,860 lines and 22,881 of 22,884 branch outcomes.
ReportGenerator displayed both totals as 99.9%. The only misses were the
exception at `FixedSegment.CenteredAxis.cs:1385` and its two failure outcomes,
and the exception at `WideFiniteAxisIntersection.CenteredSurfaceAnchors.cs:319`
and its failure outcome. Both guards were introduced by the strict rotation
rollout in `30d76f1b9d7dcc7c5939574353da36e0492dff3c`; the coverage workflow and
exclusions had not changed.

These failures cannot occur for admitted inputs. Capsule contact validates both
rotations and unit axes before rotating them. Capsule, cylinder, and cone
surface queries supply bounded directions and nonzero admitted frame rotations,
including the committed collider frames used by Gravitas. An exact rational
rotation preserves vector length. Directions have component magnitudes below two
before rotation, including zero in degenerate cases, so the final components
remain far below the fixed-point representation limit.

The correction removes only the redundant failure branches and their throws. It
retains the `TryRotate` calls, their exact rational rounding, subsequent
normalization, and the existing public argument checks. Substituting `Rotate`
would use different arithmetic. Existing valid-input and invalid-input tests
remain the behavioral controls; no test fabricates invalid internal state to
reach an impossible exception. Revisit this proof if a caller starts supplying
unnormalized directions or bypassing frame admission.

The raw baseline and final configuration reports are under
`tests/FixedMathSharp.Tests/TestResults/coverage-analysis/`. Reproduce each
configuration with the existing CI collector settings:

```powershell
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false --collect:"XPlat Code Coverage" --settings tests/FixedMathSharp.Tests/coverlet.runsettings --results-directory tests/FixedMathSharp.Tests/TestResults/coverage-analysis/release/raw
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false --collect:"XPlat Code Coverage" --settings tests/FixedMathSharp.Tests/coverlet.runsettings --results-directory tests/FixedMathSharp.Tests/TestResults/coverage-analysis/lean/raw
```

Initial local validation on Windows with SDK 10.0.302, Coverlet 10.0.1, and
ReportGenerator 5.5.10 restored exact 100% line, branch, and method coverage in
both configurations. The CI coverage scope remains the core test project,
including the core and FluentAssertions assemblies.

| Configuration | Core tests passed |   Covered lines | Covered branch outcomes | Fully covered methods |
| ------------- | ----------------: | --------------: | ----------------------: | --------------------: |
| Release       |             7,442 | 68,858 / 68,858 |         22,878 / 22,878 |         5,358 / 5,358 |
| ReleaseLean   |             7,421 | 68,951 / 68,951 |         22,878 / 22,878 |         5,354 / 5,354 |

Raw reports contain zero uncovered line or branch points. Both solution builds
compiled `netstandard2.1` and `net10.0` with zero warnings and errors; all 49
Chronicler tests passed in each configuration. The .NET format verification,
repository-wide Prettier check, and independent code review passed. No coverage
exclusions changed. That initial pass did not rerun hosted Linux CI, downstream
suites, or benchmarks. The follow-up below verifies the downstream callers and
closes the missing CI gate.

#### Safety and enforcement verification

An independent follow-up checked both the final range and every intermediate:

- `FixedQuaternion.TryRotate` fails only for a zero quaternion or an
  unrepresentable rounded result. Its exact rational basis `B / D` satisfies
  `B^T B = D^2 I` for every nonzero quaternion, even when its magnitude is only
  within the normalization tolerance. The denominator is positive.
- `Vector3d.IsNormalized` uses nonnegative saturating squares and sums. A
  component with magnitude at least two cannot pass the near-one squared-length
  check; saturation cannot wrap it into the admitted range. Wide surface-normal
  builders also bound each component below two. Degenerate zero normals remain
  valid inputs to a nonzero rotation.
- Full-domain quaternion raw products are at most `2^126`; the denominator and
  basis coefficients are at most `2^128`, within `Signed192`. Multiplication by
  these bounded direction components and three-term accumulation stay below
  `2^163`, within `Signed320`, before exact extension to `Signed576`.
- Orthogonality bounds the final component magnitude below `sqrt(12)`, with
  rounding still below `2^34` raw units versus the scalar limit near `2^63`.
  Neither intermediate overflow nor final narrowing can trigger the removed
  failures. No production precondition was weakened.

The added quaternion test covers the exact admission cutoff
`One + FromRaw(128)`, rejects the next raw axis value, preserves zero and
bounded directions through positive/negative scalar quaternions from one raw
unit to both scalar limits, and checks a cyclic rotation with all quaternion
components at `MinValue`. Existing public invalid-rotation and full-domain
surface tests remain controls.

The Gravitas audit used source at `becdd5d61cd8608f508af4ef9933433a7cbf107c`:

- Sphere surface calls use identity, while capsule, cylinder and cone calls use
  committed collider rotation; all use canonical unit axis/fallback directions.
- `LSCollider.PublishPreparedShape` is the only writer of committed rotation.
  Requested body snapshots and reconfigurations must pass checked scaled-point
  transformation; bodyless snapshots require successful matrix decomposition;
  compound snapshots normalize their composed rotation.
- Fresh bodies and omitted resets use identity. Explicit zero pose rotations
  reject before publication. Schema 1 loads migrate legacy zero to identity
  before rebuilding colliders. Compound authored zero normalizes to identity.
  CCD and temporary pose handoffs still prepare valid snapshots before
  publication. `ChronicleGuard.ThrowIfArgument` throws on invalid input in both
  build profiles; it is not a debug-only check. The direct callers therefore
  never observe a zero frame.

The guard regression slipped through because `build-and-test.yml` ran tests
without collecting coverage, and `coverage.yml` only generated and published the
post-main report. There was no executable full-coverage assertion. The
complexity register contained no partial-coverage waivers: all 514 entries
claimed full coverage and exceeded complexity ten. Its "reachable" policy was
ambiguous, but neither removed guard was justified by a registered exception.

Following the SwiftCollections correction at
`e4f784ce623aa7e6777f11aef9fac394af6812aa`, the policy now requires all reported
lines, branches and executable methods. Complexity exceptions cannot waive
coverage. Every standard/Lean Windows/Linux build/test lane and the Pages
workflow use `.assets/scripts/assert-full-coverage.ps1` on raw Cobertura data.
The existing measured assemblies remain core and FluentAssertions; Chronicler
tests still run separately. No new coverage exclusion was added.

Disposable gate controls rejected the original two-line/three-outcome baseline,
missing reports, a missing expected assembly, invalid counts, an uncovered
method point, an inconsistent branch point and a package without executable
methods. Both previous full reports passed. These probes remain generated
diagnostic output, not maintained tests of test infrastructure.

Source-stack Gravitas checks passed 224 selected tests in `Release` and 201 in
`ReleaseLean`. The selected fixtures cover explicit zero rejection and atomic
failure, initialization/reset, legacy serialization, compound parts, scalar-face
surface queries, and sphere/capsule/cylinder/cone contacts:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 --filter "FullyQualifiedName~SolidBodyIntegrationTests|FullyQualifiedName~SolidBodySerializationTests|FullyQualifiedName~CompoundColliderTests|FullyQualifiedName~ColliderSurfaceDomainTests|FullyQualifiedName~CollisionDetectionShapePairTests"
```

Repeat for `ReleaseLean`. The test dependency manifests selected
`0.0.0-fms-local-lsf`; each test host's copied FixedMathSharp DLL matched the
corresponding local source output by SHA-256:

- Release: `3C1E0B9F6D8772CDDC3084284A5861E104CC29372B0D21D4E39E3D2CDA6748CB`.
- ReleaseLean:
  `422677D994E33187BF826099CA391B17B4FE33692E43ADBA23FF18C56EF72CF0`.

The stale Gravitas host guide was also corrected: explicit zero pose rotations
reject, while fresh/default resets and legacy schema 1 loading retain their
documented identity behavior. No Gravitas runtime code changed.

The refreshed package-mode core suites and the new raw-count gate passed:

| Configuration | Core tests passed |   Covered lines | Covered branch outcomes | Fully covered methods |
| ------------- | ----------------: | --------------: | ----------------------: | --------------------: |
| Release       |             7,443 | 68,858 / 68,858 |         22,878 / 22,878 |         5,358 / 5,358 |
| ReleaseLean   |             7,422 | 68,951 / 68,951 |         22,878 / 22,878 |         5,354 / 5,354 |

Fresh raw and generated reports are under the coverage-analysis directory's
`followup-release` and `followup-lean` subdirectories. Both multi-target
solution builds passed with zero warnings/errors, and all 49 Chronicler tests
passed in each profile. The repository-wide Prettier check passed. A no-restore
formatter check initially loaded sibling source references and proposed
unrelated import removals; explicit package-mode restore and verification
scanned 1,189 files with zero changes. No unrelated formatting edits were
applied. Independent reviews found no remaining actionable issues. Hosted
Linux/Windows CI, full downstream coverage suites, and performance benchmarks
remain unrun in this follow-up; the targeted downstream counts above are the
executed evidence.
