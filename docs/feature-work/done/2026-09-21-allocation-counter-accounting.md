# FMS-Issue-019: Background GC allocation-counter accounting

## Resolution

The core test host uses blocking GC through
`<ConcurrentGarbageCollection>false</ConcurrentGarbageCollection>` in
`tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj`. This prevents a verified
.NET 8 background-GC accounting transition from appearing as an allocation in
otherwise allocation-free operations. Test parallelism and exact zero-byte
assertions remain enabled. Production math, consumer GC settings, Chronicler's
separate test host and BenchmarkDotNet settings are unchanged.

This resolves the allocation-test reliability issue locally. It does not fix the
upstream runtime counter. Do not infer a managed allocator merely from a positive
per-thread byte delta captured with background GC enabled.

## Root cause and confirming evidence

Investigation used source base `0b540a1` on Windows 11 x64, .NET SDK 10.0.302 and
.NET runtime 8.0.29. The only pre-fix test edit strengthened the product guard's
result assertion; production source and the allocation helper were unchanged.

CoreCLR's per-thread counter computes:

```text
alloc_bytes + alloc_bytes_uoh - (alloc_limit - alloc_ptr)
```

During the end of background marking, `void_allocation` clears a thread's
allocation pointer and limit without deducting unused buffer space from
`alloc_bytes`. That unused space consequently appears in the returned count.
The foreground collection path deducts the space before clearing those fields.
See the version-pinned sources:

- [Per-thread counter](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/vm/comutilnative.cpp#L936)
- [Foreground adjustment and background pointer clearing](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/gc/gc.cpp#L7920)
- [Background marking calls the clearing operation](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/gc/gc.cpp#L38628)

A native hardware watchpoint confirmed this mechanism while the real
`GeneralizedComparison_DirtyStackAndWarmedExecutionRemainAllocationFree` guard
was running under controlled background-GC pressure:

| Observation | Value |
| --- | --- |
| Measured thread's initial allocation pointer | `0x0000014c1bc0ec88` |
| Initial allocation limit | `0x0000014c1bc10270` |
| Unused space: limit minus pointer | `0x15e8` = **5,608 bytes** |
| Observed byte-counter increase | **5,608 bytes** |
| Sampled gen-0 collection count | `22` before and after |
| Native writer | `WKS::void_allocation` |
| Native caller | `WKS::gc_heap::background_mark_phase` |

The watchpoint observed the background GC thread clearing the measured thread's
context. A separate counter-only program, containing no FixedMathSharp code,
produced 49 false positives in 50 measurements. Its measured operation only
slept. Native watchpoints in that program also matched unused space to the exact
counter delta, including 8,024 and 8,144 bytes.

The unchanged gen-0 count does not rule this out: a background collection can
already be in progress when measurement begins. Forced blocking gen-0
collections did not reproduce the accounting error in 1,000 control measurements.

## Observed failures and limits of attribution

The original reports were 2,208 bytes in a covered ReleaseLean product-comparison
guard on 2026-09-13 and 7,240 bytes in an uninstrumented Release point-anchor
guard on 2026-09-21. Their inconsistent warmup protocols were corrected earlier,
but those corrections did not establish the cause of their byte counts.

This investigation reproduced failures after those corrections:

- One Release solution run reported 5,512 bytes in the capsule parameter-enclosure
  guard and 8,048 bytes in the repeated planar-sweep guard; the other 2,778 core
  tests and all eight Chronicler tests passed.
- A subsequent exact-command batch passed nine times, then reported 7,568 bytes
  in the swept-sphere/oriented-box guard on run ten.
- Temporary all-counter instrumentation captured 3,512 bytes in the generalized
  wide-comparison guard without a collection-count change. A later hardware
  watchpoint plus controlled pressure established the 5,608-byte mechanism above.

The original failures have no allocation stacks, so they cannot each be
retroactively attributed with certainty. The confirmed runtime mechanism
reproduces the same class of failure in all six affected guards, including the
original product and anchor guards. No production allocator was identified.

## Correction and validation

The supported [background-GC project setting](https://learn.microsoft.com/en-us/dotnet/core/runtime-config/garbage-collector#background-gc)
generates `System.GC.Concurrent: false` in the core test runtime configuration.
Using that generated configuration eliminated the controlled failures without
changing the operations being measured:

| Controlled check | Background GC | Blocking GC |
| --- | --- | --- |
| Counter-only sleeping operation, 50 measurements | 49 nonzero deltas | 50 exact-zero deltas |
| Same operation with one `byte[64]`, 50 measurements | Not needed for attribution | 50 exact 88-byte deltas on x64 |
| Six affected guards under GC pressure, up to 500 calls each | All six fail an allocation assertion | All 3,000 calls pass |

The six guards cover product comparison, point-anchor transforms, capsule
parameter enclosure, planar sweep, swept-sphere/oriented-box intersection and
generalized wide comparison. The pressure driver retained one million small
arrays, requested nonblocking gen-2 collections on another thread and called the
existing test methods on separate threads. It did not modify the math or guard
bodies. With blocking GC selected, those collection requests execute without
the background-mark accounting transition.

Temporary real-allocation mutations still failed both original guards in
Release and covered ReleaseLean: 128 `byte[64]` allocations reported 11,264 bytes
in the product guard, and 256 reported 22,528 bytes in the anchor guard. All
mutations and diagnostic hooks were removed before final verification.

The product guard also received an independent assertion correction: summing
opposite comparison signs allowed two incorrect zero results to cancel. It now
checks each expected sign with non-short-circuiting `&=`. Returning zero from both
overloads passed the old guard and failed the corrected guard. This assertion
change is separate from the background-GC fix.

Final Windows verification after restoring all diagnostic edits:

| Configuration | Repeated complete solution runs | Tests per solution run | Covered core run | Core line / branch coverage |
| --- | ---: | ---: | ---: | ---: |
| Release | 20 passed | 2,780 core + 8 Chronicler | 2,780 passed | 100% / 100% |
| ReleaseLean | 10 passed | 2,759 core + 8 Chronicler | 2,759 passed | 100% / 100% |

All six guards also passed 500 pressure-driven calls each against the final
binaries in each configuration (6,000 total guard executions). Both generated
test runtime configurations contain `System.GC.Concurrent: false`. The complete
solution command was `dotnet test FixedMathSharp.slnx -c <configuration>`;
coverage used the core project with `--collect:"XPlat Code Coverage"` and
`tests/FixedMathSharp.Tests/coverlet.runsettings`. Existing CI applies the same
project setting to its Windows and Linux test hosts; this investigation's native
debugger and repetition evidence was collected on Windows.

## Portable counter-only reproducer

Create an ignored directory such as `artifacts/fms-issue019-counter` with this
`CounterProbe.csproj` and the following `Program.cs`. The live graph deliberately
keeps background marking active after its initial pause; the race can produce a
different number of nonzero observations on another machine. The x64 positive
control expects an 88-byte object for a 64-byte array including its header.

```xml
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <OutputType>Exe</OutputType>
    <TargetFramework>net8.0</TargetFramework>
  </PropertyGroup>
</Project>
```

```csharp
using System;
using System.Runtime.CompilerServices;
using System.Threading;

internal static class Program
{
    private static void Main(string[] args)
    {
        object[] retained = new object[1_000_000];
        for (int index = 0; index < retained.Length; index++)
            retained[index] = new byte[64];
        GC.Collect(2, GCCollectionMode.Forced, blocking: true, compacting: true);
        Thread.Sleep(1);
        bool allocate = args.Length > 0 && args[0] == "allocate";
        int unexpected = 0;
        for (int run = 0; run < 50; run++)
        {
            GC.Collect(2, GCCollectionMode.Forced, blocking: false);
            byte[] marker = new byte[1 + run * 13];
            long bytes = Measure(allocate);
            long expected = allocate ? 88 : 0;
            if (bytes != expected)
            {
                unexpected++;
                Console.WriteLine($"run={run}: expected={expected}, observed={bytes}");
            }
            GC.KeepAlive(marker);
        }
        Console.WriteLine($"Unexpected measurements: {unexpected}/50");
        Console.WriteLine($"Background GC index: {GC.GetGCMemoryInfo(GCKind.Background).Index}");
        GC.KeepAlive(retained);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static long Measure(bool allocate)
    {
        long before = GC.GetAllocatedBytesForCurrentThread();
        if (allocate) GC.KeepAlive(new byte[64]);
        Thread.Sleep(100);
        return GC.GetAllocatedBytesForCurrentThread() - before;
    }
}
```

Run the default background-GC case, the blocking-GC control, then the
real-allocation control:

```powershell
dotnet run --project artifacts/fms-issue019-counter -c Release
dotnet run --project artifacts/fms-issue019-counter -c Release -p:ConcurrentGarbageCollection=false
dotnet run --project artifacts/fms-issue019-counter -c Release -p:ConcurrentGarbageCollection=false -- allocate
```

## Reopening criteria

Reopen with a fresh nonzero allocation failure under the configured blocking-GC
test host, or evidence of a production allocator. Record the source revision,
test, observed bytes, SDK/runtime, generated runtime configuration, coverage
context and exact command before rerunning. Prefer an allocation stack or a
small reproducer. A runtime upgrade alone is not evidence that this accounting
behavior was fixed upstream.

Ignored logs, dumps and diagnostic builds under `artifacts/fms-issue019` are
disposable supporting evidence. The source links, mechanism, observed values and
standalone reproducer above preserve the investigation independently of them.
