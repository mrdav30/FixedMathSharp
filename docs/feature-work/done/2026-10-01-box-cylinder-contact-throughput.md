# Box/Cylinder Contact Throughput Refinement

This records the first refinement of the general rotated manifold and oblique
primary contact [benchmark signal](../benchmark-signal-hardening-backlog.md).
The signal remains open: exact edge-root work still contributes materially to
both workloads, and no observed simulation workload establishes an acceptance
budget.

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
