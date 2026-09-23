# FixedMathSharp.Chronicler

`FixedMathSharp.Chronicler` connects FixedMathSharp to Chronicler: exact
simulation-time conversions and deterministic `ChronicleHashWriter` extensions.
Use it for long-running simulation clocks, replay checks, and state
fingerprints.

## Install

```bash
dotnet add package FixedMathSharp.Chronicler
```

Use
[`FixedMathSharp.Chronicler.Lean`](https://www.nuget.org/packages/FixedMathSharp.Chronicler.Lean)
when the rest of your application uses the FixedMathSharp Lean package graph.
Both variants target .NET Standard 2.1 and .NET 8.

## Record hashes

```csharp
using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Chronicler;

var writer = new ChronicleHashWriter();

writer.WriteVector3d(new Vector3d(10, 2, -4));
writer.WriteQuaternion(FixedQuaternion.Identity);

ChronicleHash hash = writer.ToHash();
```

Extensions cover `Fixed64`, vectors, quaternions, matrices, `FixedTransform`,
core bounds, rays, and planes. Each method writes fields in an explicit stable
order. Changing that order or the authoritative fields is a hash and replay
compatibility event.

`WriteTransform` hashes local position, local rotation, and local scale. Parent
identity and derived world values are intentionally excluded.

## Exact simulation-time conversions

`FixedChronicleTime` bridges Fixed64 seconds and
`Chronicler.Timing.ChronicleDuration`. Both have 32 fractional bits, so widening
preserves every raw bit, including negative fractions. Narrowing succeeds only
when the duration fits Fixed64; it never saturates or rounds.

Subtract timestamps **before** converting to Fixed64. This complete method
measures a quarter second after 100 Julian years (365.25 days per year):

```csharp
using Chronicler.Timing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;

public static class TimingExample
{
    public static Fixed64 MeasureInterval()
    {
        var start = new ChronicleTimestamp(3_155_760_000, 0);
        var end = start + FixedChronicleTime.FromFixed64(Fixed64.FromRaw(0x40000000));
        return FixedChronicleTime.ToFixed64(end - start); // Exactly 0.25 seconds.
    }
}
```

A 100-year interval itself does not fit Fixed64. `TryToFixed64` returns `false`
and a zero output for an out-of-range duration; `ToFixed64` throws
`OverflowException`. Keep wide intervals as `ChronicleDuration` when narrowing
is unnecessary. There is deliberately no direct timestamp conversion.

`GetFrameCountForDuration(duration, stepDuration)` counts complete steps at the
supplied step size and returns a `long`, including results above `int.MaxValue`.
Duration must be nonnegative and step duration positive; invalid arguments throw
`ArgumentOutOfRangeException`. It divides the raw integers and floors the
result: no reciprocal multiplication, saturation, historical frame lookup, or
wait scheduling is involved. With a rounded 1/30-second step (143165577 raw
units), one second contains 29 complete steps; 30 steps require 4294967310 raw
units. Use the actual represented step, not an assumed exact decimal rate.

Chronicler owns clock advancement and wide time values; FixedMathSharp core
remains independent of Chronicler's timing APIs. Hosts still own fixed-step
execution, lifetime boundaries, and coordinated restore.

## Learn more

- [FixedMathSharp repository](https://github.com/mrdav30/FixedMathSharp)
- [FixedMathSharp API reference](https://mrdav30.github.io/FixedMathSharp/)
- [Chronicler source](https://github.com/mrdav30/Chronicler)

FixedMathSharp.Chronicler is licensed under the
[MIT License](https://github.com/mrdav30/FixedMathSharp/blob/main/LICENSE).
