# Capsule interval process-wide allocation diagnostic

## Outcome

Closed 2026-10-14 with an explicit no-runtime-change decision. A fresh
reproduction of `FiniteAxisIntersectionBenchmarks.Capsule3DIntersectionInterval`
at scale 100,000 attributes a 336-byte process-wide delta to .NET's finalizer
thread trimming `SharedArrayPool<char>`. The measured capsule thread allocates
zero bytes. The counters report real background allocations; this is distinct
from
[Issue 019's per-thread accounting artifact](2026-09-21-allocation-counter-accounting.md).

No library, benchmark, dependency, GC setting or allocation tolerance changes
are warranted. Retain the launcher's raw-counter warning: a zero summary alone
is insufficient to decide allocation ownership. Diagnostic code stays in ignored
artifacts, reusing the earlier paired-counter/native investigation rather than
adding a benchmark test project or a permanent probe framework.

## Intake and environment

The [Issue 031 validation](2026-10-14-degenerate-convex-relations.md) retained
two unattributed observations in this unchanged 3D benchmark:

| Capture                                  | Child process | Raw bytes / operations |
| ---------------------------------------- | ------------: | ---------------------: |
| `artifacts/issue031-smoke.log`           |          9372 |              336 / 864 |
| `artifacts/issue031-counter-control.log` |         21336 |              336 / 784 |

The three-launch repeat covers eight related cases, with 24 successful children.
Its blocking-GC control has 24 zero raw rows. Both original positive rows round
to zero integer bytes per operation. These facts establish the signal, not its
allocator. No stacks were captured inside those original measurement intervals.

The fresh investigation uses Windows 11, i7-9700K, .NET SDK 10.0.302, runtime
8.0.29 and BenchmarkDotNet 0.15.8. Builds use `UseLocalLsfStack=true`,
`DOTNET_PROCESSOR_COUNT=2`, serial source builds and
`UseSharedCompilation=false`. The probe uses the default workstation GC with no
GC-mode override. Runtime source is committed Issue 031 (`f6c586f`); no runtime
source is edited during this investigation.

## Paired controls

The ignored probe constructs the actual benchmark fixture with
`Scale = 100_000`, calls its existing `Setup`, and measures its existing capsule
method. Setup, delegates, warmup and console output stay outside each interval.
The precise process interval encloses both calling-thread counter reads and the
workload. Each sample follows BenchmarkDotNet's memory pre-pass sequence:
`GC.Collect`, `GC.WaitForPendingFinalizers`, `GC.Collect`.

An uninstrumented run measures 32 samples of 128 calls per operation:

| Operation                    | Calling-thread bytes per sample | Process bytes per sample | Positive process samples |
| ---------------------------- | ------------------------------: | ------------------------ | -----------------------: |
| Capsule interval             |                               0 | 0 or 336                 |                  31 / 32 |
| Idle (`Thread.Sleep(1)`)     |                               0 | 336                      |                  32 / 32 |
| Allocate `byte[64]` per call |                          11,264 | 11,264 or 11,600         |                  32 / 32 |

The real-allocation control asserts its expected calling-thread total and a
process total at least that large. This both reproduces the signal without the
BDN runner and verifies that the nested counters detect actual workload
allocations. It does not by itself identify the background allocator.

## Fresh native attribution

CDB brackets precise process-counter returns on the measuring OS thread and
captures `GCInterface::GetMemoryInfo` stacks on the finalizer thread. Captured
object metadata is reset at each before-return; only a fresh event inside that
pair qualifies for an object match. SOS verifies the actual types and sizes of
both objects, rather than assuming that an adjacent address contains an object.

The native run alternates eight 784-call and eight 864-call capsule samples,
with idle and real-allocation controls. Every capsule calling-thread delta is
zero. Two cleanly matched windows in `native-output.log` are:

| Sample / calls | Before return / total | After return / total | Managed result: thread / process bytes |
| -------------- | --------------------- | -------------------- | -------------------------------------- |
| 10 / 784       | 69 / 952,976          | 70 / 953,312         | 0 / 336                                |
| 11 / 864       | 75 / 966,288          | 76 / 966,624         | 0 / 336                                |

Both windows measure on OS thread `6a2c`. Between each pair, OS thread `1280`
executes this finalizer stack:

```text
Gen2GcCallback.Finalize
  SharedArrayPool<char>.Trim
    Utilities.GetMemoryPressure
      GC.GetGCMemoryInfo
        GCInterface::GetMemoryInfo
```

SOS identifies a **288-byte `System.GCMemoryInfoData`** and the following
**48-byte `ConditionalWeakTable` enumerator** for the character pool's tracked
thread-local arrays. Their combined size exactly matches each fresh 336-byte
process delta. The matching objects are at `0000010a42000028` /
`0000010a42000148` for sample 10 and `0000010a42400028` / `0000010a42400148` for
sample 11.

Debugger scheduling changes the observed intervals. Other samples include
partial background deltas and an idle 6,528-byte observation; these are not
attributed by the two clean windows. Some debugger return labels also skip, so
attribution uses the explicit before/after values, fresh finalizer event,
verified objects and matching managed result together, rather than treating
every automatic parity-based label as a valid pair.

This proves a fresh same-fixture reproduction of the 336-byte mechanism. It does
**not** retroactively attribute either original Issue 031 process. It is enough
to close this bounded signal without modifying the math path or hiding raw
diagnostics. Reopen for a positive calling-thread delta, a different
representative allocation stack, or repeatable allocation growth with workload.

## Evidence and verification

Evidence root: `artifacts/capsule-interval-allocation/`.

- `original-child/` and `original-child-hashes.json` preserve the retained BDN
  child, generated source, binaries and PDBs before rebuilding. This is the
  retained blocking-control child, not a reconstructed original positive child.
- `probe.csv` retains the uninstrumented 32-by-128 controls.
- `probe/Program.cs`, `probe/CounterProbe.csproj`, `probe-hashes.json` and
  `native-build.log` retain the native probe source/build and binary identities.
- `counter-objects.cdb`, `counter-entry.cdb`, `counter-return.cdb`, `native.log`
  and `native-output.log` retain the fresh native capture.

Both probe builds succeed with zero warnings/errors, and both probe processes
exit zero with their allocation controls passing. The existing public capsule
entry point and its shared finite-axis owner are unchanged. The prior Issue 031
Release/ReleaseLean tests and 100% coverage evidence in both repositories remain
applicable to unchanged runtime source; the full matrix is not rerun for this
documentation-only closure. No Gravitas changes are needed.

To repeat the original benchmark signal after building the benchmark runner:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll finite-axis-intersection -j Short --filter '*Capsule3DIntersectionInterval*' --iterationTime 10 --launchCount 3 --affinity 3 --keepFiles --exporters json --artifacts artifacts/capsule-interval-repeat
```

For a native reproduction, build the retained ignored probe with the same local
source properties, then run CDB with `counter-objects.cdb` against its DLL.
Resolve tool and script paths locally; preserve fresh binaries and hashes, and
require matched managed/native intervals before assigning ownership.
