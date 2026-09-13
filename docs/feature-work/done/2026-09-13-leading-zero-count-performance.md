# Leading-Zero Count Performance Follow-up

## Scope and compatibility

Completed locally on 2026-09-13 UTC during Trailblazer's `TRB-Benchmark-005`
investigation, against FixedMathSharp `c8d343c`.

`Fixed64.CountLeadingZeroes` now uses the exact integer
`System.Numerics.BitOperations.LeadingZeroCount(ulong)` when compiled for
`NET8_0_OR_GREATER`. The existing `netstandard2.1` implementation remains
unchanged under `#else`. Both targets remain supported; .NET 8 is not a new
minimum requirement. No public API, dependency, rounding, overflow, division
algorithm or serialization layout changes.

The helper is used by exact division and wide arithmetic. It counts the same
bits, including returning 64 for zero. This is not a floating-point shortcut
or a new prepared-divisor abstraction.

## Measured result

Environment: Windows 11 25H2, Intel Core i7-9700K, .NET SDK 10.0.302,
.NET 8.0.29 x64 RyuJIT, BenchmarkDotNet 0.15.8, Release. Captures ran serially
with normal runtime settings and no concurrent tests, builds or profiles.
Run from the FixedMathSharp repository root:

```powershell
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll fixed64-arithmetic --filter '*Fixed64ArithmeticBenchmarks.Divide' --launchCount 3 --warmupCount 3 --iterationCount 5 --exporters json --keepFiles --artifacts artifacts/leading-zero-baseline
```

Repeat for candidate, original control and candidate confirmation. Each
measurement below is the untrimmed median of all 15 `WorkloadActual` values
after dividing nanoseconds by the reported operation count. One `Divide`
benchmark invocation performs **256 divisions plus accumulation**, not a single
scalar division.

| Comparison | Original median | Candidate median | Reduction |
| --- | ---: | ---: | ---: |
| Initial pair | 4.857407 us | 4.447649 us | 8.44% |
| Repeat pair | 4.983671 us | 4.564143 us | 8.42% |

All four memory diagnostics report zero allocation and collections. BDN's
outlier-filtered result sets retain 12/14/13/14 observations; those smaller
sets are not used for the table. The scalar fixture favors denominators with
trailing zero bits, so its reduction is not a universal division or frame-time
claim. Trailblazer's separately repeated complete-frame measurements retain
exact replay; their combined change also includes Trailblazer ray refactoring
and does not isolate this helper's frame-time contribution.

## Verification and evidence

The bit test checks every highest-set-bit position with isolated and filled
lower bits, and the immediately smaller value, reaching zero: 192 assertions.
The focused Fixed64 suite passes 179 tests, including exact division oracles,
rounding, saturation and allocation behavior. Direct checks execute 192 bit
assertions against each of the four Release/ReleaseLean and netstandard2.1/net8.0
assemblies (768 total). Both portable helper IL bodies match the archived
original implementation byte-for-byte. This executes the portable fallback on
the local .NET host; a net8.0 test run alone would not establish that evidence.

Both library targets build in Release and ReleaseLean. FixedMathSharp passes
2,744 core + 8 Chronicler tests in Release and 2,723 + 8 in ReleaseLean, without
failures or skips. Coverage reports retain exact 100% line, branch and fully
covered method coverage with unchanged collection settings. Cross-stack
validation is retained with the consuming Trailblazer investigation.

Independent source/Ponytail and evidence reviews found no actionable issue.
All 204 archived microbenchmark file hashes validate. Authored portable-PDB
hashes match original/control and candidate/confirmation; only
`Fixed64.Statics.cs` changes, with unchanged benchmark source.

Ignored evidence is retained in the Trailblazer checkout under
`artifacts/benchmark005/leading-zero-*`. Successful captures are `baseline`,
`candidate`, `control-retry` and `confirm`; `control-retry.log` is archived in
`leading-zero-control-children`. The earlier `leading-zero-control.log` failed
before execution because the runner was invoked from the wrong repository;
its zero measurements are retained and excluded. This is local Windows/.NET 8
evidence, not Linux, Mono, Unity or other host-runtime acceptance.
