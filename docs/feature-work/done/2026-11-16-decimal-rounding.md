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
