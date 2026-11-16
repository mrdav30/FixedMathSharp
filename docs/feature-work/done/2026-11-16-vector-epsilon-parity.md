# FMS-Issue-044 Vector Epsilon Predicate Parity

`AllComponentsGreaterThanEpsilon()` now returns `true` in 2D, 3D and 4D when
every component's absolute value strictly exceeds `Fixed64.Epsilon`. A component
equal to either signed epsilon boundary makes the predicate return `false`.
`FixedMath.Abs` handles `Fixed64.MinValue` by saturating its magnitude to
`Fixed64.MaxValue`, so that component passes.

The API documentation review identified the inconsistency at
`1a9a270db3a6c79d4f501fc849883ef6ffd6ea12`. It was split from FMS-Issue-041
after reproduction at `2a65963`. At `d340981`, vectors filled with `-1` returned
`true` in 2D and 3D and `false` in 4D. The 4D implementation compared each
signed component directly with epsilon. The updated 4D code uses the same
absolute-value comparison as 2D and 3D.

This changes the published 4D behavior for vectors whose components all exceed
epsilon in magnitude and include a negative component. Callers that used the old
result to require every component to be positive and above epsilon can compare
each component directly with `Fixed64.Epsilon`. A search of FixedMathSharp and
the local LSF sibling repositories found no production callers of the predicate.

The
[parity tests](../../../tests/FixedMathSharp.Tests/Numerics/Vectors/Vector4d.Tests.cs)
exercise all-negative vectors, `Fixed64.MinValue`, and mixed-sign vectors with
one component one raw unit below, exactly at, and one raw unit above epsilon in
magnitude. Three cases failed before the change, all on the 4D assertion.
Fourteen controls passed.

```bash
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release \
  -p:UseLocalLsfStack=false -p:DisableTransitiveProjectReferences=false \
  --filter 'FullyQualifiedName~AllComponentsGreaterThanEpsilon'
```

All 17 focused tests passed in `Release` and `ReleaseLean`. They exercised every
instrumented branch outcome in the three predicates. The full solution suites
passed 7,487 and 7,466 tests, respectively. An integer oracle confirmed all
three dimensional results for 6,561 component combinations on each Lean target.
DocFX rendered the same contract for all three methods.
