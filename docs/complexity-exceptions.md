# Cyclomatic Complexity Exception Register

This document records methods that intentionally exceed the current cyclomatic complexity review threshold.

## Policy

- Review threshold: cyclomatic complexity greater than 10.
- Risk threshold: CRAP score greater than 30 requires immediate test hardening or refactoring.
- Current status: the fresh coverage/CRAP report generated on 2026-05-18 has no methods above CRAP 30.
- Source report: `tests/FixedMathSharp.Tests/TestResults/coverage-analysis/reports/Summary.txt`.

Complexity exceptions are acceptable when the method is a hot deterministic math path, a direct component-wise value comparison, a fixed-shape assertion helper, or an algorithm where extraction would add indirection without reducing real maintenance risk. These exceptions should be revisited when coverage drops, behavior changes, or the implementation becomes harder to reason about.

## Exception Register

| Module | Method | Complexity | Coverage | Rationale | Revisit if |
| --- | --- | ---: | --- | --- | --- |
| `FixedMathSharp.FluentAssertions` | `FixedAssertionHelpers.AreComponentApproximatelyEqual(Fixed4x4, Fixed4x4, Fixed64)` | 30 | 100% line / 100% branch | Fixed-shape assertion over all matrix components. The explicit checks keep failure location and assertion intent clear. | Assertion diagnostics degrade, matrix shape changes, or repeated assertion logic grows further. |
| `FixedMathSharp` | `Fixed4x4.Equals(Fixed4x4)` | 30 | 100% line / 100% branch | Direct 4x4 value comparison avoids loops, allocations, and indexer overhead on a hot value type. | Equality semantics change or a generated/source-shared component comparison becomes available without runtime cost. |
| `FixedMathSharp` | `BoundingSphere.CreateFromPointList(IReadOnlyList<Vector3d>)` | 26 | 98.2% line / 100% branch | Ritter-style bounding sphere construction has fixed selection and expansion branches; keeping it local preserves data flow and avoids extra passes. | More sphere construction modes are added, coverage drops, or the algorithm needs accuracy/performance tuning. |
| `FixedMathSharp` | `FixedMath.Sin(Fixed64)` | 24 | 100% line / 100% branch | Trigonometric range reduction and approximation are performance-sensitive and deterministic. Extraction would split a compact numeric routine. | Approximation strategy changes or benchmark evidence shows a helper split is neutral or faster. |
| `FixedMathSharp` | `Fixed64.op_Division(Fixed64, Fixed64)` | 24 | 100% line / 100% branch | Saturating fixed-point division uses low-level bit operations and guarded overflow paths that should remain explicit. | Division semantics change or additional edge cases make the method difficult to audit. |
| `FixedMathSharp` | `FixedMath.Sqrt(Fixed64)` | 18 | 100% line / 100% branch | Integer square-root logic is branchy by nature and sits on a core deterministic math path. | A simpler algorithm is adopted with equal determinism and performance. |
| `FixedMathSharp` | `Fixed3x3Extensions.FuzzyEqual(Fixed3x3, Fixed3x3, Fixed64?)` | 18 | 100% line / 100% branch | Component-wise fuzzy equality is intentionally explicit to avoid allocation and preserve inlining. | The matrix equality helpers are generated or centralized without adding runtime overhead. |
| `FixedMathSharp` | `BoundingSphere.ContainsBoxLike(Vector3d, Vector3d)` | 18 | 100% line / 100% branch | Checks all box-like corners against the sphere; explicit shape avoids temporary corner arrays. | Bounds internals move to a shared corner iterator that is allocation-free. |
| `FixedMathSharp` | `Fixed4x4.get_Item(int)` | 17 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection. | The matrix layout changes or generated indexer code becomes part of the build. |
| `FixedMathSharp` | `Fixed4x4.set_Item(int, Fixed64)` | 17 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection. | The matrix layout changes or generated indexer code becomes part of the build. |
| `FixedMathSharp` | `Fixed64.op_Multiply(Fixed64, Fixed64)` | 16 | 95.6% line / 81.2% branch | Full-width multiply with saturating and round-half-to-even behavior requires explicit guarded paths. | Additional uncovered reachable branches appear, arithmetic semantics change, or benchmarks support a simpler equivalent. |
| `FixedMathSharp` | `Fixed3x3Extensions.FuzzyEqualAbsolute(Fixed3x3, Fixed3x3, Fixed64)` | 16 | 100% line / 100% branch | Direct component-wise comparison avoids loops and keeps the extension inlinable. | The matrix equality helpers are generated or centralized without adding runtime overhead. |
| `FixedMathSharp` | `Fixed3x3.Equals(Fixed3x3)` | 16 | 100% line / 100% branch | Direct 3x3 value comparison avoids loops, allocations, and indexer overhead on a hot value type. | Equality semantics change or a generated/source-shared component comparison becomes available without runtime cost. |
| `FixedMathSharp` | `FixedCurve.Evaluate(Fixed64)` | 16 | 95.2% line / 87.5% branch | Curve evaluation combines clamping, segment search, and interpolation mode dispatch in one readable flow. | More interpolation modes are added or segment lookup becomes a performance bottleneck. |
| `FixedMathSharp` | `FixedMath.Pow2(Fixed64)` | 16 | 100% line / 100% branch | Fixed-point exponent approximation is compact, deterministic, and performance-sensitive. | Approximation strategy changes or benchmark evidence supports decomposition. |
| `FixedMathSharp` | `FixedPlane.IntersectsBoxLike(Vector3d, Vector3d)` | 16 | 100% line / 100% branch | Plane-vs-bounds classification is branch-heavy but fixed-shape and allocation-free. | Additional bound shapes are added and a shared allocation-free helper becomes clearer. |
| `FixedMathSharp` | `BoundingFrustum.Intersects(FixedRay)` | 16 | 100% line / 93.8% branch | Slab-style frustum clipping has unavoidable enter/exit branches and benefits from locality. | More ray/frustum edge cases are discovered or clipping logic is shared elsewhere. |
| `FixedMathSharp` | `FixedMath.Atan(Fixed64)` | 16 | 100% line / 100% branch | Trigonometric approximation and range handling are deterministic hot-path math. | Approximation strategy changes or benchmark evidence supports decomposition. |
| `FixedMathSharp` | `FixedMath.Asin(Fixed64)` | 16 | 100% line / 100% branch | Trigonometric domain handling and approximation are deterministic hot-path math. | Approximation strategy changes or benchmark evidence supports decomposition. |
| `FixedMathSharp` | `FixedMath.Tan(Fixed64)` | 16 | 95.6% line / 100% branch | Tangent delegates through fixed trigonometric identities and guarded edge handling. | New domain behavior is added or uncovered lines become reachable with valid input. |
| `FixedMathSharp.FluentAssertions` | `FixedAssertionHelpers.AreComponentApproximatelyEqual(Fixed3x3, Fixed3x3, Fixed64)` | 16 | 100% line / 100% branch | Fixed-shape assertion over all matrix components. The explicit checks keep failure location and assertion intent clear. | Assertion diagnostics degrade, matrix shape changes, or repeated assertion logic grows further. |
| `FixedMathSharp` | `BoundingFrustum.IntersectsFrustum(BoundingFrustum)` | 14 | 91.3% line / 85.7% branch | Separating-axis frustum checks are algorithmically branch-heavy and intentionally avoid allocations. | A robust shared SAT helper can improve clarity without extra allocations or worse benchmarks. |
| `FixedMathSharp` | `BoundingBox.ClosestPointOnSurface(Vector3d)` | 14 | 100% line / 100% branch | The nearest-face selection is fixed-shape and explicit; helper extraction would not reduce real complexity. | Box surface projection grows beyond nearest-face selection. |
| `FixedMathSharp` | `Fixed3x3.get_Item(int)` | 12 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection. | The matrix layout changes or generated indexer code becomes part of the build. |
| `FixedMathSharp` | `Fixed3x3.set_Item(int, Fixed64)` | 12 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection. | The matrix layout changes or generated indexer code becomes part of the build. |
| `FixedMathSharp` | `BoundingFrustum.Contains(BoundingFrustum)` | 12 | 100% line / 100% branch | Frustum containment needs null handling, equality fast-path, corner checks, and intersection fallback. | Frustum containment semantics change or the corner loop is shared with other bound types. |
| `FixedMathSharp` | `FixedMath.Acos(Fixed64)` | 12 | 100% line / 100% branch | Trigonometric domain handling and approximation are deterministic hot-path math. | Approximation strategy changes or benchmark evidence supports decomposition. |
| `FixedMathSharp` | `FixedQuaternion.FromEulerAngles(Fixed64, Fixed64, Fixed64)` | 12 | 100% line / 100% branch | Validates three angles and builds the quaternion in a compact deterministic path. | Angle validation policy changes or Euler conversion variants are added. |
| `FixedMathSharp` | `FixedRay.Intersects(BoundingSphere)` | 12 | 100% line / 91.7% branch | Ray-sphere intersection has standard geometric early exits and should remain local for clarity and speed. | New ray semantics are added or branch gaps reveal reachable untested behavior. |

## Review Notes

- Methods at 100% line and branch coverage with direct component-wise logic should usually remain explicit unless a zero-overhead generated approach is introduced.
- For fixed-point arithmetic and trigonometric routines, prefer benchmark-backed refactors over structural changes made only to reduce the reported complexity number.
- If a method remains above the complexity threshold and below full branch coverage, prefer adding focused tests for reachable branches before refactoring.
- Re-run coverage and CRAP analysis after any changes that touch the methods listed above, then update this register.
