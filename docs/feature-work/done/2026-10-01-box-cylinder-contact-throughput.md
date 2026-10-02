# Box/Cylinder Contact Throughput Refinement

This records two refinements of the general rotated manifold and oblique
primary contact [benchmark signal](../benchmark-signal-hardening-backlog.md).
The bounded refinement is complete. Exact edge-root work still contributes
materially to both workloads; the remaining cost and reopen criteria are recorded
below. No observed simulation workload establishes a universal acceptance budget.

## Earlier Complete-Solver Baseline

The signal was measured on 2026-09-27 while repairing FMS-Issue-025, from working
tree revision `5059cb2`. The historical solver did not implement the complete
relation and is not an equivalent-correctness performance target. This does not
establish that its positive fixtures were wrong; the confirmed cap-clipped wrong
hits are marked below. The two positive targets use different rotations, so their
difference does not isolate manifold overhead.

The first complete solver cost 4.002 / 5.822 ms on the two general contacts.
Profiling placed roughly three quarters of time in edge/rim work, particularly
exact root admission/refinement. A proven cone-wide support lower bound skipped
noncompetitive quartics before construction, reusing the existing exact
candidate comparator. This reduced costs to roughly 1.97 / 3.04 ms. It introduced
no separate solver or approximation and preceded both refinements below.

The original seven-fixture capture used the same Windows/SDK/runtime and
two-launch, three-warmup, twelve-measurement setup recorded below. Means +/-
99.9% confidence half-width, microseconds per contact:

| Frozen fixture | Historical solver | Complete solver with cone pruning |
| --- | ---: | ---: |
| `CylinderManifold` | 539.790 +/- 9.907 | 1,969.743 +/- 32.357 |
| `CylinderObliquePrimary` | 581.907 +/- 9.562 | 3,037.938 +/- 62.071 |
| `CylinderParallelManifold` | 491.034 +/- 8.360 | 279.434 +/- 6.596 |
| `CylinderZeroRadiusPrimary` | 291.761 +/- 5.101 | 103.398 +/- 1.727 |
| `CylinderCapClippedSeparatedPrimary` | 577.393 +/- 10.087 (wrong hit) | 281.916 +/- 10.168 (miss) |
| `CylinderCapClippedSeparatedManifold` | 576.751 +/- 10.168 (wrong hit) | 285.832 +/- 8.114 (miss) |
| `CylinderStrict` | 0.320 +/- 0.006 | 0.322 +/- 0.006 |

Captures are in `artifacts/fms025-baseline`, `fms025-after`, `fms025-profile`,
`fms025-pruning` and `fms025-final-benchmarks`. All fourteen final children exited
zero with zero raw allocation and GC counters. The separated-primary distribution
was bimodal; use its broad trend rather than the last digits. Regression suites
retain the cap-clipped miss, wrong minimum depth, exact touch/raw neighbors,
algebraic depths, sign reversal and principal-direction degeneracy.

## Retained Changes

`BoxCylinderAnalyticFeatures.BuildAxis` cancels the positive squared axis length
before encoding an analytic support gap and moves the nonnegative radius outside
its radical. The candidate retains exactly the same signed gap, normal,
minimum-depth ordering and ties. Smaller coefficients reduce the cost of every
subsequent exact comparison. The existing forty-word representation remains
sufficient; construction scratch shrinks from eleven slots to ten. The algebra
and coefficient bounds are documented beside the implementation.

The shared `CompareRadicalPairs` first compares the two contributions to the
difference of squared radical sums. Zero or agreeing signs already decide the
answer exactly; only opposing signs need the larger squared products. This
improves existing contact consumers without another comparison owner or cache.

Positive admitted edge roots now reuse the analytic candidate's nearest-even
depth as a certified upper bound. An unclamped rounded raw depth `d` implies
that its exact gap is at most `d + 1/2` raw. The existing parameter-root endpoint
evaluator tests `4N - (2d+1)²D`; a nonnegative result proves the edge cannot win,
so value-polynomial construction, mapping and ranking are unnecessary. Negative
and zero-gap admission still run first. Clamped analytic depths bypass this
bound, and surviving roots retain the existing exact ranking. The unsigned
two-limb threshold supports `d = long.MaxValue` without narrowing overflow.

## Matched Measurements

Baseline: `8012137`; retained source: the refinement of that revision.
The fixtures were unchanged. Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29,
Release, BenchmarkDotNet 0.15.8, concurrent workstation GC, two-core affinity,
below-normal launcher. Two launches, three warmups and twelve measurements;
`UseLocalLsfStack=true`, `BuildInParallel=false`, `UseSharedCompilation=false`
and `DOTNET_PROCESSOR_COUNT=2`. Build, benchmark and test workloads were serialized
for the timing captures.

Means +/- 99.9% confidence half-width, in microseconds:

| Frozen fixture | Baseline | Retained | Mean reduction |
| --- | ---: | ---: | ---: |
| `CylinderManifold` | 1,803.548 +/- 40.604 | 1,118.005 +/- 12.686 | 38.0% |
| `CylinderObliquePrimary` | 2,576.448 +/- 40.756 | 1,643.204 +/- 9.199 | 36.2% |
| `CylinderParallelManifold` | 238.429 +/- 1.627 | 239.269 +/- 1.935 | Control |
| `CylinderZeroRadiusPrimary` | 86.941 +/- 0.945 | 84.272 +/- 0.620 | Control |
| `CylinderCapClippedSeparatedPrimary` | 255.146 +/- 4.118 | 241.117 +/- 2.934 | Control |
| `CylinderCapClippedSeparatedManifold` | 253.029 +/- 6.050 | 244.301 +/- 5.416 | Control |
| `CylinderStrict` | 0.308 +/- 0.003 | 0.304 +/- 0.003 | Control |

The two targets use different cylinder rotations; their difference does not
isolate manifold overhead. Baseline and retained captures each contain fourteen
successful children, all reporting zero allocated bytes and GC collections.
Raw logs and exported reports are in `artifacts/box-cylinder-cost1-baseline`
and `artifacts/box-cylinder-cost1-upper-final`, with sibling `.log` files.
The first two changes alone measured 1.525 / 1.860 ms in
`artifacts/box-cylinder-cost1-final`; a narrow upper-bound experiment measured
1.101 / 1.662 ms before the final seven-fixture capture above.

```powershell
$env:UseLocalLsfStack='true'
$env:BuildInParallel='false'
$env:UseSharedCompilation='false'
$env:DOTNET_PROCESSOR_COUNT='2'
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll oriented-box-anchor --filter '*Cylinder*' --launchCount 2 --warmupCount 3 --iterationCount 12 --affinity 3 --keepFiles --exporters json --artifacts artifacts/benchmarks/box-cylinder-throughput
```

## Experiments Not Retained

- Disposable counts showed at most one mapped root per chart and no retained
  edge-value incumbent at any mapping in these fixtures. Reusing a value-root
  batch or rejecting against an incumbent's upper endpoint would not help them.
- Removing common whole zero words in the shared three-term comparison had no
  incremental gain in a repeat capture, so it was discarded.
- Certifying a positive gap from the chart's coefficient signs was exact, but
  the measured means increased to 1.559 / 2.005 ms. That query can also retain
  useful parameter refinement; omitting its cost need not improve the subsequent
  mapping. The cause of this regression was not fully isolated, and the original
  query remains in place.

Temporary instrumentation was removed. No benchmark-test suite, new runtime
diagnostic contract, approximation, public API or dependency was introduced.

## Validation And Remaining Work

The new runtime tests check exact radical ordering, cancellation ties, reversed
signs and the largest coefficient stride. A separate `BigInteger` oracle
reconstructs the unreduced support identity from the selected direction and
checks its rational and radical contributions independently; it passes against
both the original and retained encodings. Existing geometry/resource tests cover
touch, separating gaps, minimum depth, nearest-even rounding, clamping,
repeatability, warmed zero allocation and a 1 MiB worker stack with 64 KiB live
caller headroom.
Parameterized root tests additionally check the upper-bound query on rational
and irrational parameters, exact half-raw equality, both parity cases, adjacent
squared values differing by `2^-65`, and the full unsigned maximum threshold.
The clamped fallback regression uses a full-width cube and the exact cylinder
axis `(2,2,1)/3`. Contained balls independently prove clamping; the signed chart
derivative changes from positive to negative to positive at `0`, `3/4`, `1`,
proving two admitted stationary roots. Both lose to an analytic face, so the
case exercises value-construction reuse and exact fallback ranking as well as
the clamp bypass.

Fresh solution builds and core coverage collection use `UseLocalLsfStack=true`
in both repositories. All tests pass and all reachable lines, branches and
methods are fully covered in both configurations, using the existing generated
code report filters without additional exclusions:

| Repository / configuration | Core tests | Covered lines | Covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 4,113 | 52,691 / 52,691 | 12,134 / 12,134 | 3,935 / 3,935 |
| FixedMathSharp ReleaseLean | 4,092 | 52,784 / 52,784 | 12,134 / 12,134 | 3,931 / 3,931 |
| Gravitas Release | 4,363 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |
| Gravitas ReleaseLean | 4,304 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |

All 49 Chronicler tests pass in each configuration. Both runtime target
frameworks build. FixedMathSharp reports are under
`artifacts/box-cylinder-cost3-Release-report` and its ReleaseLean counterpart;
Gravitas uses `artifacts/box-cylinder-cost2-Release-report` and its ReleaseLean
counterpart in that repository. Independent correctness and minimality reviews
found no runtime callouts.

The serialized post-validation EventPipe profile attributes the following
inclusive shares of sampled benchmark activity to existing owners:

| Owner / operation | Manifold | Oblique primary |
| --- | ---: | ---: |
| Edge contacts | 57.2% | 49.0% |
| Analytic candidate selection | 30.9% | 39.9% |
| Parameter-root sign evaluation | 29.0% | 18.4% |
| Edge parameter construction | 16.6% | 15.1% |
| Shared exact candidate comparison | 14.4% | 26.9% |

These inclusive shares overlap and must not be added. Profiles and summaries
are in `artifacts/box-cylinder-cost2-post-validation-profile` and its sibling
summary file. Value mapping is no longer among the twenty-four hottest methods
in either target. The next useful isolation is parameter-root proof cost and
the shared three-term analytic comparison, using fresh matched evidence before
retaining another change. This refinement does not establish a universal
contact-count or frame-time acceptance budget.

The rebuilt benchmark catalog succeeds. The serialized broad out-of-process
smoke (`all -j Short --iterationTime 10 --affinity 3 --keepFiles --exporters json`)
contains 459 successful child exits and 459 populated statistics in 25 reports,
under `artifacts/box-cylinder-cost2-smoke`. These short smoke timings are not
the throughput evidence above.

The smoke also emitted one raw-counter warning outside the contact targets:
`CenteredCapsule2DDistance`, `Scale=1`, process 28024 reported 336 bytes over
1,984 operations while its rounded summary reported zero. The warning stopped
the disposable PowerShell wrapper's post-run checks; child exits and exported
statistics were verified directly afterward. The capture and generated binaries
are preserved. A narrow repeat with 100 ms iterations and two launches at each
scale returned zero, with zero raw allocation and GC counters in all four
children (`artifacts/box-cylinder-cost2-memory-warning-repeat`). This does not
attribute the original bytes to either runtime or harness. All fourteen matched
contact-target/control launches and the warmed runtime guards report zero.

## Second Refinement: Shared Exact Ranking

The next experiment starts from committed revision `0c929f8`. Both callers of
`CompareRadicalPairs` pack at most three nonzero signed radical terms into its
four slots. At least one pair product is therefore zero. After the existing
zero/agreeing-sign cases, opposing signs reduce exactly to `B - 2*sqrt(P)`;
comparing `B²` with `4P` preserves its sign and exact equality. The general
four-term reduction and its larger nested scratch are removed. This invariant
is documented at the private comparison boundary.

`BoxCylinderAnalyticFeatures.BuildRankingAxis` now omits the denominator factor
`U*S²` during private same-geometry selection, where `U` is the positive squared
cylinder-axis length and `S` is the positive coordinate scale. All signed gaps
are multiplied by the same positive `S*sqrt(U)`, preserving ordering, negative
gaps, zero and ties. Both exits restore the selected denominator before returning
to edge pruning, ranking or public rounding. Restoration reuses the dead
three-slot direction buffer; construction scratch shrinks from ten slots to
nine. No cache, additional comparison owner or public representation is added.

New runtime regressions bracket an exact radical cancellation tie, independently
prove minimum depth and stable normal across ordinary and full-width coordinate
scales, and reconstruct the raw denominator on the vertex/rim separation exit.
The denominator oracle includes the rational world-basis denominator; its
corrected form and the exact-depth cases pass against the original and factored
builders.

A separate retained-root experiment tried the existing approximate sign
certificate before allocating expanded root-cell scratch when refinement would
already be a no-op. Only nonzero proofs returned early; uncertainty used the
canonical exact fallback. It measured 1.139 / 1.515 ms against 1.121 / 1.376 ms
for the retained ranking changes, so the prototype and its additional tests were
discarded. Uncertain queries can repeat the certificate evaluation; the capture
does not isolate how much of the regression that accounts for. Evidence is in
`artifacts/box-cylinder-cost4-probe` and its sibling log. The root-proof owner is
unchanged.

A bounded construction review also tested skipping zero-sign products in the
shared `CylinderEdgeContactPolynomial.Term`. The box metric's middle coefficient
is zero, so four of ten K/L products contribute nothing. The guard passed 186
focused contact regressions but produced no measured gain: 1,119.730 +/- 20.764 /
1,452.386 +/- 64.705 us before, versus 1,139.705 +/- 26.947 /
1,530.582 +/- 31.787 us afterward. The oblique baseline has wider uncertainty;
these captures do not establish a precise regression. The guard was discarded
and the original helper restored, leaving the previously validated source
unchanged. Captures are under `artifacts/box-cylinder-cost4-zero-before` and
`artifacts/box-cylinder-cost4-zero-after` with sibling logs. No additional small
algebraic reduction was identified in the bounded construction review.

The fresh seven-fixture baseline and retained capture use the same environment
and command as the first refinement above, with unchanged fixtures. Means +/-
99.9% confidence half-width, in microseconds:

| Frozen fixture | Baseline `0c929f8` | Retained | Mean reduction |
| --- | ---: | ---: | ---: |
| `CylinderManifold` | 1,193.969 +/- 21.017 | 1,137.135 +/- 22.528 | 4.8% |
| `CylinderObliquePrimary` | 1,758.801 +/- 30.715 | 1,394.405 +/- 18.172 | 20.7% |
| `CylinderParallelManifold` | 255.512 +/- 5.408 | 247.783 +/- 4.515 | Control |
| `CylinderZeroRadiusPrimary` | 91.541 +/- 1.516 | 90.934 +/- 1.405 | Control |
| `CylinderCapClippedSeparatedPrimary` | 261.662 +/- 4.297 | 259.621 +/- 3.644 | Control |
| `CylinderCapClippedSeparatedManifold` | 259.813 +/- 4.917 | 261.954 +/- 4.911 | Control |
| `CylinderStrict` | 0.323 +/- 0.006 | 0.317 +/- 0.005 | Control |

Both captures contain fourteen successful child exits with zero raw allocation
and GC counters, under `artifacts/box-cylinder-cost4-baseline-full` and
`artifacts/box-cylinder-cost4-final`, with sibling logs. An earlier narrow
isolation of the radical deletion alone measured 1.185 / 1.655 ms against
1.180 / 1.757 ms; the manifold change was neutral. Factoring combined with the
deletion measured 1.121 / 1.376 ms before the complete seven-fixture capture.
Use the matched table for this phase's gains, rather than comparing against the
first refinement's differently timed capture. Both targets remain above one
millisecond; this phase does not establish a universal frame budget.

Fresh final solution builds and coverage runs pass with `UseLocalLsfStack=true`
throughout, using the existing generated-code filters without additional
exclusions. Both target frameworks build. All 49 Chronicler tests pass in each
configuration:

| Repository / configuration | Core tests | Covered lines | Covered branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 4,119 | 52,666 / 52,666 | 12,132 / 12,132 | 3,935 / 3,935 |
| FixedMathSharp ReleaseLean | 4,098 | 52,759 / 52,759 | 12,132 / 12,132 | 3,931 / 3,931 |
| Gravitas Release | 4,363 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |
| Gravitas ReleaseLean | 4,304 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |

Reports are under `artifacts/box-cylinder-cost4-Release-report` and its
ReleaseLean counterpart in each repository. Relative to the first refinement,
the core report removes 25 reachable runtime lines and two branches, with no
increase in method count. Independent arithmetic, ownership, test and minimality
reviews found no actionable callouts. Existing full-domain geometry, exact
rounding/clamping, warmed zero-allocation and constrained-stack tests all pass.

The final serialized EventPipe profile reports these inclusive shares of sampled
benchmark activity; overlapping owners must not be added:

| Owner / operation | Manifold | Oblique primary |
| --- | ---: | ---: |
| Edge contacts | 60.0% | 59.0% |
| Analytic candidate selection | 24.6% | 28.9% |
| Parameter-root sign evaluation | 28.2% | 20.2% |
| Edge parameter construction | 19.5% | 21.6% |
| Shared exact candidate comparison | 6.0% | 16.2% |

Profiles and summaries are under
`artifacts/box-cylinder-cost4-post-validation-profile` and its sibling summary
file. Ranking consumes a smaller share than the first refinement's 14.4% / 26.9%,
while remaining edge cost includes polynomial construction, root isolation,
admission and certification. The two subsequent small experiments produced no
gain. Stop this bounded pass here; a new optimization should start from a
concrete shared root-proof reduction or representative simulation capture.
Reopen for that evidence or a reproducible throughput regression. This closure
preserves the measured cost rather than treating it as universally acceptable.

The rebuilt catalog and serialized broad out-of-process smoke
(`all -j Short --iterationTime 10 --affinity 3 --keepFiles --exporters json`)
complete successfully: 459 child exits zero and 459 populated statistics across
25 reports, under `artifacts/box-cylinder-cost4-smoke`. These short timings are
smoke evidence rather than throughput measurements.

The runner preserved two raw-counter warnings outside the contact targets:
`BoundsBenchmarks.Triangle2dArea`, process 17848, reported 336 bytes over 127,488
operations; `BoundsBenchmarks.SphereClampPoint`, process 6160, reported 336 bytes
over 7,936 operations. Their rounded summaries were zero. The raw logs and
generated binaries are retained without attributing these observations to
runtime or harness. A narrow repeat of both methods with 100 ms iterations,
three warmups, twelve measurements and two launches succeeds with zero raw
allocation and GC counters in all four children, under
`artifacts/box-cylinder-cost4-memory-warning-repeat`. All matched contact/control
children and warmed runtime allocation guards remain zero. No native failure
occurred in either capture.
