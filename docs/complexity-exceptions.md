# Cyclomatic Complexity Exception Register

This document records methods that intentionally exceed the repository's
cyclomatic complexity review threshold.

## Policy

- Review threshold: cyclomatic complexity greater than 10.
- Source-size warning: roughly 1,200 lines is a hard review warning, not a
  target or automatic split point. Owner cohesion and an independent reason to
  change determine whether code stays together or moves; file-count reduction
  alone does not justify a partial, helper, or forwarding layer.
- Risk threshold: a coverage-amplified or unregistered CRAP score greater than
  30 requires immediate test hardening or refactoring. A fully covered method's
  score cannot fall below its cyclomatic complexity, so documented complexity
  floors are reviewed rather than mechanically refactored.
- Coverage requirement: registered methods remain at 100% reachable line and
  branch coverage. Refresh the per-method coverage column whenever a listed
  implementation changes. Aggregate test and coverage counts belong in generated
  reports rather than this long-lived register.
- Keep this register evergreen and self-contained: record current rationale,
  invariants and revisit criteria, without dated validation summaries, artifact
  references, feature-work links or references to other libraries' documentation.

Complexity exceptions are acceptable when the method is a hot deterministic math
path, a direct component-wise value comparison, a fixed-shape assertion helper,
or an algorithm where extraction would add indirection without reducing real
maintenance risk. These exceptions should be revisited when coverage drops,
behavior changes, or the implementation becomes harder to reason about.

## Exception Register

| Module                            | Method                                                                              | Complexity | Coverage                | Rationale                                                                                                                                                                                                                    | Revisit if                                                                                                                                     |
| --------------------------------- | ----------------------------------------------------------------------------------- | ---------: | ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `FixedMathSharp` | `WideConvexPrismRelations.BuildCylinderCapsuleAxisCandidate(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Exact authored axes, signed axial support and radial support form one shared squared-gap candidate without normalizing geometry or rounding classification inputs. | A simpler candidate representation preserves full-domain support and its width proof. |
| `FixedMathSharp` | `WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(...)` | 40 | 100% line / 100% branch (Release / ReleaseLean) | One canonical feature traversal owns cap, side, endpoint-rim and capsule-interior candidates, degenerate reductions and certified early returns for public contacts and admitted full slab lengths. | The complete feature proof changes or measured equivalent pruning removes work without weakening minimum-depth selection. |
| `FixedMathSharp` | `WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Shared quadratic bounds prepare one denominator enclosure for all components while retaining exact axis shortcuts, signs, scale, and uncertain-denominator fallback. | A simpler exact normal conversion preserves scaled anchors and cancellation without duplicating the sign owner. |
| `FixedMathSharp` | `WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | The shared cylinder/capsule, analytic cylinder-pair and large parallel-radius owner narrows exact depth search while preserving zero gaps, nearest-even ties and conceptual clamping. | A cheaper exact conversion preserves half-raw ties and maximum-range behavior for every consumer. |
| `FixedMathSharp` | `WideConvexPrismRelations.GetRoundedCylinderCapsuleEllipseDepth(...)` | 20 | 100% line / 100% branch (Release / ReleaseLean) | Exact magnitude-floor bounds narrow the signed depth search; paired positive pseudo-reduction is reused across thresholds while final half-raw comparisons and conceptual clamping remain authoritative. | A cheaper exact conversion preserves zero gap, half-raw ties and maximum-range behavior. |
| `FixedMathSharp` | `WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Zero terms and like signs resolve directly; opposing rational/radical terms use one exact squared comparison shared by the existing contact consumers. | The shared sign or input-width contract expands, or a cheaper exact comparator preserves cancellation. |
| `FixedMathSharp` | `WideConvexPrismRelations.GetConvexContactCandidateThreeTermSign(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Cross-candidate comparison retains both radicals, sign reductions and exact equality before deterministic winner selection. | A simpler comparison preserves distinct algebraic depths that round to the same scalar. |
| `FixedMathSharp` | `WideConvexPrismRelations.CompareCylinderCapsuleEllipseSquaredGap(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Sign admission precedes radical elimination, then an exact degree-at-most-eight query compares the retained ellipse root with the shared analytic winner. | The candidate representation or degree/width contract changes. |
| `FixedMathSharp` | `WideArithmetic.GetMagnitudeSquareRootBounds(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Even-bit truncation reuses the existing narrow root, preserves discarded-bit exactness and returns factored 97-bit endpoints without a general wide square-root solver. | The unsigned input domain or endpoint width changes, or a measured simpler enclosure preserves perfect-square and carry boundaries. |
| `FixedMathSharp` | `WideArithmetic.GetGreatestCommonDivisor(Signed192, Signed192)` | 20 | 100% line / 100% branch (Release / ReleaseLean) | Shared three-word binary Euclid removes positive integer content for ellipse preparation and cylinder-pair primitive axes without changing authored geometry. | The admitted signed domain changes or another exact content reducer has equal or lower measured cost. |
| `FixedMathSharp` | `WideArithmetic.DivideExactSigned192(...)` | 20 | 100% line / 100% branch (Release / ReleaseLean) | Shared bounded signed division applies a proven positive divisor without rounding; the consumed-prefix bound protects remainder shifts throughout the admitted signed domain. | Exact-divisibility, signed-range or consumer-width preconditions change. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(...)` | 28 | 100% line / 100% branch | Degree reduction, excluded zero roots, Sturm counts and dyadic isolation retain the largest positive root with explicit rational and repeated-root ownership. | The degree or root-selection contract changes, or a simpler exact isolation proof reduces work. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetFiniteRootVariations(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Nonconstant unit-interval rows reuse a certified integer point sign; uncertain, zero, out-of-domain and derivative right-limit cases retain exact Sturm counting. | The root domain, precision or sign acceptance proof changes, or a measured simpler evaluator removes the certificate. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(...)` | 16 | 100% line / 100% branch | Constant/rational queries and sign-preserving pseudo-reduction evaluate the same retained root exactly, including equality. | Query degree exceeds eight or another representation reduces exact sign work without losing root identity. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetReducedSignAtFiniteAxisRoot(...)` | 28 | 100% line / 100% branch | Certified interval signs and exact linear-boundary evaluation precede bounded crossing/Sturm refinement and the Hermite fallback; the fast-path budget never determines the answer. | A cheaper exact equality/sign certificate replaces refinement or the fallback without losing multiple-root behavior. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | The shared polynomial owner removes and returns a common positive power-of-two factor across nonzero coefficients, preserving roots, signs and jointly normalized ratios. | Coefficient storage or normalization policy changes. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetFiniteAxisRootRatioWords(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Joint low-factor and high-degree trimming preserves the ratio at a strictly positive root and supplies the bounded coefficient width for shared positive pseudo-steps. | Root positivity, padded ratio storage or the degree/height contract changes. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetFiniteRootPolynomialBounds(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Shared signed interval Horner bounds certify signs and exact ratio-floor enclosures over the retained positive dyadic cell; constants and rational cells retain their exact treatment. | A tighter exact interval evaluation measurably reduces fallback work. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetFiniteRootHermiteIntervalSign(...)` | 16 | 100% line / 100% branch | Two endpoint-weighted Hermite signatures cancel every other real root and recover the selected root's exact query sign, including zero. | Root-cell endpoint or degree invariants change. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetFiniteRootSymmetricSignature(...)` | 28 | 100% line / 100% branch | At-most-four-dimensional principal minors and exact real-eigenvalue sign variations handle singular and repeated-root trace forms without iterative numerical decisions. | A smaller exact signature algorithm preserves singular cases and the determinant-width proof. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.DoStrictCylinderConeLateralInteriorsMeet(...)` | 20 | 100% line / 100% branch | Exact cone-generator normals enumerate the complete smooth lateral candidate set, retaining finite axial parameters in one quadratic field. | A simpler complete feature proof reduces arithmetic without sampled directions or rounded parameters. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(...)` | 14 | 100% line / 100% branch | Disk center, complete rim crossing and target-section containment cover both crossing and enclosure without materializing contacts. | A shared exact planar-section representation lowers work without losing finite-cap boundaries. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.BuildStrictDiskCirclePolynomials(...)` | 20 | 100% line / 100% branch | Fixed quadratic circle coordinates produce the radial quartic and both axial inequalities with explicit cylinder/cone coefficients. | Another consumer justifies a narrower shared polynomial construction without allocation. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.IsStrictSolidSectionWitnessInDisk(...)` | 12 | 100% line / 100% branch | Parallel and oblique plane sections retain rational interior witnesses and exact disk containment. | A shared section witness removes duplicated setup while preserving enclosure completeness. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.TryGetStrictDiskQuadraticWitness(...)` | 18 | 100% line / 100% branch | Midpoint, interior minimum and inward endpoint witnesses cover every negative quadratic interval without root approximation. | The interval contract changes or a simpler exact witness construction has equivalent fixed-width bounds. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.HasStrictCirclePolynomialPoint(...)` | 18 | 100% line / 100% branch | Sorted algebraic strip boundaries and radial signs prove that all three strict inequalities hold on the same arc. | An equally complete common-sign classifier lowers workspace or measured cost. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetStrictCirclePolynomialLimitSign(...)` | 18 | 100% line / 100% branch | Infinity parity and the first nonzero inward derivative preserve excluded endpoint roots and strict tangency. | A shared endpoint representation expresses both policies without extra work or ambiguity. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.CountStrictCircleOddRoots(...)` | 28 | 100% line / 100% branch | Bounded Sturm variations and degree-at-most-four gcd multiplicities distinguish crossings from even-root tangency, including roots on excluded arc boundaries. | A simpler exact multiplicity authority supports the same algebraic endpoint contract. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.EvaluateStrictCirclePolynomial(...)` | 16 | 100% line / 100% branch | Homogeneous derivative evaluation at a quadratic endpoint retains rational/radical cancellation and an exact final sign comparison. | A shared quadratic-field evaluator reduces fixed workspace or cost without wider intermediates. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(...)` | 12 | 100% line / 100% branch | Exact core entry, the side cylinder and both expanded disks form the complete finite-cylinder Minkowski relation, including collapsed inputs. | Another owner can share the complete feature proof without rounded endpoints or sampled axes. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(...)` | 12 | 100% line / 100% branch | The base disk, apex sphere and offset-side frustum retain finite cone features and exact rigid chord authority. | A shared finite-solid distance representation reduces work without weakening boundary truth. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.DoesStrictChordEnterExpandedDisk(...)` | 14 | 100% line / 100% branch | The disk core and complete rim tube retain unsquared negative terms before the quartic sign test; coefficient construction stays fixed-width and allocation-free. | An equally complete lower-degree formulation removes the rim polynomial. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.HasNegativeFiniteAxisPolynomial(...)` | 24 | 100% line / 100% branch | Endpoint signs and bounded Sturm/gcd multiplicity cases distinguish negative intervals from repeated-root tangency without isolating or rounding roots. | The degree/input-width contract changes or a simpler exact multiplicity authority is available. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.BuildFiniteAxisPolynomialSturmSequence(...)` | 14 | 100% line / 100% branch | One degree-at-most-four chain shares factorized quartic construction and lower-degree pseudo-remainders at caller-proven fixed widths. | Another consumer requires a wider degree or different algebraic contract; do not silently generalize this bounded owner. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetStrictPolynomialEndpointSign(...)` | 14 | 100% line / 100% branch | The first nonzero inward derivative determines the sign at an excluded endpoint root, with the nonzero leading coefficient as the exact final case. | The interval policy changes or a shared derivative-sign primitive preserves these invariants. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.GetStrictChordBounds(...)` | 12 | 100% line / 100% branch | Stationary and reversed rigid chords clip against rational or single-quadratic-field axial bounds without rounding the interval. | Another relation can share this bound representation with neutral or lower measured cost. |
| `FixedMathSharp` | `WideOrientedBox.DoesCenteredCylinderPenetrateBox(...)` | 18 | 100% line / 100% branch | Exact center containment and six fixed box faces form a complete cylinder/polytope test, with stable early exits and no contact ranking. | A shared polytope owner removes the face setup without allocation or a measured regression. |
| `FixedMathSharp` | `WideOrientedBox.DoesCylinderTrianglePenetrate(...)` | 22 | 100% line / 100% branch | The complete cap-clipped triangle feature set includes projected-axis containment, original edges and both newly clipped cap sections. | A simpler complete radial minimum preserves open axial boundaries and degenerate standalone triangles. |
| `FixedMathSharp` | `WideOrientedBox.IsCylinderAxisInsideTriangleProjection(...)` | 18 | 100% line / 100% branch | Exact winding and barycentric axial comparison identify the zero-radial minimum without rounding the cylinder-axis intersection. | A shared rational triangle containment primitive preserves these dimensional and strict-boundary guarantees. |
| `FixedMathSharp` | `WideOrientedBox.IsCylinderRadialSegmentInside(...)` | 12 | 100% line / 100% branch | Rational cap-clipped endpoints, endpoint minima and the interior projection share a bounded squared-distance comparison. | A shared rational segment primitive reduces duplication without wider intermediates or allocation. |
| `FixedMathSharp` | `WideCenteredCapsule2dRelations.TryGetContactAxis(...)` | 22 | 100% line / 100% branch | One ordered SAT traversal serves closed contacts, strict classification and point normals. Validated nondegenerate points stop after face axes; general contacts retain capsule-side and vertex axes for degenerate boundaries. Strict queries skip depth ranking. | The SAT candidate set or boundary admission contract changes, or an equally exact lower-cost traversal becomes available. |
| `FixedMathSharp` | `WideCenteredCapsule2dRelations.TryKeepAxis(...)` | 18 | 100% line / 100% branch | Signed axial overlap and exact radial support preserve strict/closed boundary policy; only contact queries retain the minimum normalized depth. | Another capsule relation can share this invariant without widening products or obscuring signs. |
| `FixedMathSharp` | `WideOrientedBox.TryGetCenteredCapsulePenetration(...)` | 30 | 100% line / 100% branch | One complete box/capsule candidate traversal preserves ordinary contact ordering and exact strict rejection without constructing a second geometry kernel. | Candidate families change or an equally complete lower-cost distance authority is available. |
| `FixedMathSharp` | `WideOrientedBox.TryKeepCapsuleAxis(...)` | 24 | 100% line / 100% branch | Exact rational/radical signs distinguish separation, touch and penetration; strict queries avoid squared norms when possible and skip contact ranking. | Classification and contact policies can be separated without duplicated geometry or a measured regression. |
| `FixedMathSharp` | `WideOrientedBox.DoesSpherePenetrate(...)` | 12 | 100% line / 100% branch | Three exact rigid-frame gap clamps and zero-radius interior semantics avoid rounded witnesses and square roots. | A shared closest-feature primitive preserves strict signs and improves measured cost. |
| `FixedMathSharp` | `WideOrientedBox.TryFindHullCapsuleCandidate(...)` | 18 | 100% line / 100% branch | Shared face, edge-cross, vertex-core and endpoint-edge enumeration preserves complete feature coverage and authored contact ties. | Hull candidate ownership changes or duplication is introduced. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateSphere(...)` | 16 | 100% line / 100% branch | Exact finite base, rim, side, apex and interior classification selects meridian features before comparing radical distances. | Another finite-solid relation can share the feature proof without widening hot intermediates. |
| `FixedMathSharp` | `WideFiniteAxisIntersection.CompareStrictSphereRadical(...)` | 12 | 100% line / 100% branch | Sign-aware fixed-width comparison of one rational and one radical term handles cancellation and tangency without roots or allocation. | A shared comparator has the same proven widths and neutral or lower measured cost. |
| `FixedMathSharp` | `WideCenteredCapsule2dRelations.GetSupportFeature(...)` | 14 | 100% line / 100% branch | Repeated boundary vertices and exact neighboring projection ties distinguish an edge from an opposite supporting vertex. | Boundary validation guarantees unique vertices or a shared exact support-feature owner removes duplication. |
| `FixedMathSharp` | `WideCenteredCapsule2dRelations.GetDirectionFromCenteredAxis(...)` | 12 | 100% line / 100% branch | Exact endpoint/interior clamping uses the supplied direction's actual squared length and a common rational distance denominator. | Closest-feature queries gain a simpler shared representation with the same full-domain guarantees. |
| `FixedMathSharp` | `WideCenteredCapsule2dRelations.TryGetContacts(...)` | 16 | 100% line / 100% branch | Side-pair clipping and exact side-axis ties select paired witnesses while retaining separate axial and radial terms after classification. | Contact-witness construction changes or shared feature clipping reduces the branch set. |
| `FixedMathSharp`                  | `Fixed64.GetSignedRatio(Signed320, Signed320)`                                      |         52 | 100% line / 100% branch | Full-domain Gram ratios require explicit sign, fixed five-word alignment/division, guard/sticky rounding, signed limits, and final saturation.                                                                               | A shared fixed-limb divider preserves every signed boundary with lower complexity and neutral or faster measured cost.                         |
| `FixedMathSharp`                  | `Fixed64.GetSignedRatio(Signed192, Signed192)`                                      |         48 | 100% line / 100% branch | General signed wide ratios require explicit sign, exact unit-interval dispatch, signed-limit, fixed-limb quotient, guard/sticky, and saturation.                                                                             | A shared fixed-limb division primitive preserves the complete signed contract with lower complexity and neutral cost.                          |
| `FixedMathSharp` | `WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(Signed832, ...)` | 70 | 100% line / 100% branch | One bounded conic classifier owns compact and full-width clipping, derivative/discriminant decisions, and nearest-even entry/exit selection; both paths share the interval state machine. | A simpler shared state representation reduces real branching without duplicating classification or slowing ordinary queries. |
| `FixedMathSharp` | `WideFiniteConeIntersection.TryGetCompactPolynomial(...)` | 14 | 100% line / 100% branch | Explicit bit bounds admit compact arithmetic only when coefficients, clip products, discriminant and rounded numerators all fit. | The shared carriers or coefficient-width contract changes. |
| `FixedMathSharp` | `WideFiniteConeIntersection.EvaluateWidePolynomialSign(...)` | 26 | 100% line / 100% branch | One fixed-storage signed-product loop evaluates the polynomial and its derivative without allocating or truncating rigid clip ratios. | An existing magnitude owner can express both evaluations with fewer decisions and the same width proof. |
| `FixedMathSharp` | `WideFiniteConeIntersection.RoundRoot(...)` | 14 | 100% line / 100% branch | At most 63 exact midpoint comparisons distinguish lower/upper roots and nearest-even ties without a wider square-root implementation. | A measured alternative preserves root ownership, full-domain bounds and exact ties. |
| `FixedMathSharp` | `WideTriangleConeIntersection.TryGetMinimumAxialPoint(...)` | 16 | 100% line / 100% branch | A single authored-edge traversal and face reduction preserve deterministic candidate order for scalar and rigid queries. | Candidate ownership can be simplified without changing tie behavior. |
| `FixedMathSharp` | `WideTriangleConeIntersection.QueryFrame.TryGetEdgePoint(...)` | 24 | 100% line / 100% branch | Exact axial clipping admits the scalar translation path or preserves rational rigid geometry through the shared bounded conic owner. | Another existing interval owner can preserve these frame widths without duplicate arithmetic or allocations. |
| `FixedMathSharp` | `WideTriangleConeIntersection.TryGetFacePoint(...)` | 24 | 100% line / 100% branch | Exact cone-plane reduction, no-edge topology, edge/face lattice ownership and rational-frame witness construction stay in one allocation-free face path. | Another exact reducer can share plane state without narrowing authored frames or weakening feature ownership. |
| `FixedMathSharp`                  | `FixedSegment.SolveClosestParameters(...)`                                          |         44 | 100% line / 100% branch | Exact determinant classification, coupled finite-segment clamps, and endpoint-candidate state form one allocation-free solver decision path.                                                                                 | A lower-complexity state representation preserves exact policy and measures neutral or faster on the closest-pair row.                         |
| `FixedMathSharp`                  | `WideArithmetic.CompareNonNegative(Signed704, Signed704)`                           |         44 | 100% line / 100% branch | Eleven-word lexicographic comparison keeps full-domain radical, ratio, and polynomial ordering allocation-free and independent of target-specific wide-integer support.                                                      | A portable fixed-width value type or intrinsic provides identical limb ordering with lower complexity and neutral measured cost.               |
| `FixedMathSharp`                  | `WideOrientedBox.TryGetRationalSegmentSweepDistance(...)`                           |         38 | 100% line / 100% branch | Exact rational segment sweeps retain degenerate features, bounded closest parameters, radical distance ordering, and nearest-even hit conversion in one allocation-free query path.                                          | A shared rational closest-feature result reduces decision state without weakening full-domain ordering or regressing the sweep benchmark.      |
| `FixedMathSharp`                  | `FixedTriangle.ClosestPoint(Vector3d)`                                              |         32 | 100% line / 100% branch | Exact Gram degeneracy, six stable Voronoi regions, full-domain interpolation, and deterministic collapsed-edge fallback form one solver decision tree.                                                                       | Another exact primitive can share the region state or reduce branches without allocations or ordinary-input regression.                        |
| `FixedMathSharp`                  | `WideTriangleRelations.GetClosestTriangleLocalPoint(...)`                           |         32 | 100% line / 100% branch | Exact triangle Voronoi selection in a rigid triangle frame retains rational vertices, stable degenerate-edge ownership, and one final local-point narrowing without allocations.                                             | A shared wide triangle-region state reduces branches while preserving local feature ordering and measured triangle-query cost.                 |
| `FixedMathSharp.FluentAssertions` | `FixedAssertionHelpers.AreComponentApproximatelyEqual(Fixed4x4, Fixed4x4, Fixed64)` |         30 | 100% line / 100% branch | Fixed-shape assertion over all matrix components. The explicit checks keep assertion intent clear and avoid allocations in test helpers.                                                                                     | Assertion diagnostics degrade, matrix shape changes, or repeated assertion logic grows further.                                                |
| `FixedMathSharp`                  | `Fixed4x4.Equals(Fixed4x4)`                                                         |         30 | 100% line / 100% branch | Direct 4x4 value comparison avoids loops, allocations, and indexer overhead on a hot value type.                                                                                                                             | Equality semantics change or a generated/source-shared component comparison becomes available without runtime cost.                            |
| `FixedMathSharp`                  | `FixedSegment.GetClosestPoints(FixedSegment)`                                       |         26 | 100% line / 100% branch | Full-domain setup, exact point-degeneracy handling, and bit-exact endpoint preservation remain at the public query boundary.                                                                                                 | Endpoint identity can move into a simpler shared primitive without extra wide products or an ordinary-input regression.                        |
| `FixedMathSharp`                  | `WideTriangleRelations.TryGetContact(...)`                                          |         20 | 100% line / 100% branch | Triangle-pair SAT retains two face normals, six in-plane edge normals, nine edge crosses, stable minimum-depth selection, and explicit separated exits in one allocation-free reducer.                                       | A shared fixed-shape SAT iterator reduces branches while preserving axis order, tie ownership, and the ordinary triangle-pair benchmark.       |
| `FixedMathSharp`                  | `WideArithmetic.GetRoundedNonNegativeNormalizedDepth(...)`                          |         18 | 100% line / 100% branch | Exact clamping, tiny-axis bisection, constant-time ordinary-axis approximation, and nearest-even midpoint correction share one allocation-free final-depth conversion.                                                       | One constant-time exact divider covers tiny axes without weakening half-even results or regressing the ordinary normalized-depth path.         |
| `FixedMathSharp`                  | `FixedBoundSphere.CreateFromPointSpan(ReadOnlySpan<Vector3d>)`                      |         18 | 100% line / 100% branch | Ritter-style bounding sphere construction has fixed selection and expansion branches; keeping it span-based preserves the allocation-free path.                                                                              | More sphere construction modes are added, coverage drops, or the algorithm needs accuracy/performance tuning.                                  |
| `FixedMathSharp`                  | `FixedBoundSphere.CreateFromPointList(IReadOnlyList<Vector3d>)`                     |         18 | 100% line / 100% branch | Ritter-style bounding sphere construction has fixed selection and expansion branches; keeping it local preserves data flow and avoids extra passes.                                                                          | More sphere construction modes are added, coverage drops, or the algorithm needs accuracy/performance tuning.                                  |
| `FixedMathSharp`                  | `FixedMath.Sin(Fixed64)`                                                            |         24 | 100% line / 100% branch | Trigonometric range reduction and complementary reduced-range polynomial selection are performance-sensitive and deterministic. Extraction would split a compact numeric routine.                                            | Approximation guarantees change or benchmark evidence supports a simpler equally accurate path.                                                |
| `FixedMathSharp`                  | `Fixed64.RoundSignedToFixed(Signed192, int)`                                        |         24 | 100% line / 100% branch | Scaled signed conversion keeps high-word overflow, both signed limits, nearest-even ties, carry, and saturation explicit without allocation.                                                                                 | A shared conversion primitive can preserve all 32/33-bit shift boundaries with lower complexity and neutral cost.                              |
| `FixedMathSharp`                  | `WideArithmetic.MultiplySigned192(Signed192, Signed192)`                            | 22 | 100% line / 100% branch (Release / ReleaseLean) | Five-word signed products require explicit fixed-limb carry and two's-complement propagation without allocation or target-specific semantics.                                                                                | A portable runtime intrinsic replaces the manual multiword product with equal netstandard behavior and lower cost.                             |
| `FixedMathSharp`                  | `WideArithmetic.GetMagnitude(Signed320, out ...)`                                   | 20 | 100% line / 100% branch (Release / ReleaseLean) | Five-word two's-complement magnitude conversion keeps carry propagation explicit and shared by comparisons, thresholds, and ratios.                                                                                          | A focused fixed-width value type can centralize magnitude conversion without obscuring word order or adding overhead.                          |
| `FixedMathSharp`                  | `Fixed4x4.Decompose(Fixed4x4, out Vector3d, out FixedQuaternion, out Vector3d)`     |         22 | 100% line / 100% branch | Strict affine decomposition keeps magnitude validation, orthogonality, reflection canonicalization, quaternion reconstruction, and exact failure outputs in one atomic public contract.                                      | A shared decomposition result can reduce decision state without allocations or weakening rejection semantics.                                  |
| `FixedMathSharp`                  | `Fixed64.DivideMagnitude(ulong, ulong, bool)`                                       |         32 | 100% line / 100% branch | The shared saturating division core uses exact shifts for binary-power divisors and guarded quotient construction otherwise, preserving signed limits and round-half-to-even behavior without allocations.                                                                                | Division semantics change or benchmarks support a simpler guarded implementation.                                                              |
| `FixedMathSharp`                  | `FixedSegment2d.TryGetUniqueIntersection(FixedSegment2d, out Fixed64, out Fixed64)` |         20 | 100% line / 100% branch | Exact point, determinant, collinearity, and closed-endpoint classification stay together so the public result has one deterministic decision path.                                                                           | Intersection semantics grow beyond closed segments or another exact predicate can remove branches without allocations.                         |
| `FixedMathSharp`                  | `Fixed64.TryGetUnitIntervalRatio(Signed320, Signed320, out Fixed64)`                |         16 | 100% line / 100% branch | Exact five-word sign/range classification and a 33-bit guard/sticky quotient preserve one deterministic conversion for extreme determinants.                                                                                 | A simpler fixed-limb division primitive retains identical sign, range, guard, sticky, and nearest-even behavior.                               |
| `FixedMathSharp`                  | `WideArithmetic.CompareUnsigned(5-word, 5-word)`                                    |         20 | 100% line / 100% branch | Lexicographic comparison over five fixed words is direct, allocation-free, and shared by determinant thresholds, clamps, and ratios.                                                                                         | A fixed-width value type provides an equally inlinable comparison without hiding word significance.                                            |
| `FixedMathSharp`                  | `WideGeometry.AccumulateDifferenceProduct(...)`                                     |         18 | 100% line / 100% branch | Signed 65-by-65-bit products and 192-bit two's-complement accumulation share one allocation-free carry path.                                                                                                                 | A reusable fixed-width integer primitive replaces the local word arithmetic without changing hot-path cost.                                    |
| `FixedMathSharp`                  | `WideGeometry.AccumulateRawProduct(...)`                                            |         18 | 100% line / 100% branch | The proven narrow-difference path accumulates signed raw products with explicit two's-complement carry propagation and no intermediate conversion.                                                                           | A shared signed product accumulator expresses the same word behavior more simply and measures neutral or faster.                               |
| `FixedMathSharp`                  | `FixedBoundBox.HasStrictAxisOverlap(Vector3d, Vector3d)`                            |         18 | 100% line / 100% branch | Six direct closed-bound comparisons express strict overlap on three fixed axes without temporary ranges or iteration overhead.                                                                                               | More dimensional bound types share the exact policy through an equally inlinable helper.                                                       |
| `FixedMathSharp`                  | `FixedBoundSphere.CreateFromFrustumCorners(FixedBoundFrustum)`                      |         16 | 100% line / 100% branch | Frustum-corner sphere construction mirrors the point-list algorithm while avoiding a temporary public collection.                                                                                                            | Frustum corner ordering changes or the sphere construction algorithm is replaced.                                                              |
| `FixedMathSharp`                  | `WideRayIntersection.TrySolveInterval(...)`                                         |         22 | 100% line / 100% branch | Closed radial intervals retain exact overlap, zero-motion, discriminant, clipping, and entry/exit ownership while narrowing only the two public parameters.                                                                  | A shared bounded-root result reduces state without charging first-hit callers for exit refinement or regressing benchmarks.                    |
| `FixedMathSharp`                  | `WideRayIntersection.SolveEntry(...)`                                               |         22 | 100% line / 100% branch | The allocation-free first-root path keeps analytic seeding, bounded existence, exact correction, and nearest-even conversion together after shared discriminant setup.                                                       | A simpler fixed-width root primitive preserves every boundary and ordinary first-hit benchmark cost.                                           |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TryGetFiniteAxisInterval(...)`                          |         32 | 100% line / 100% branch | Exact ordinary-axis clipping, radial polynomial classification, narrow fast-path dispatch, wide discriminant solving, and endpoint rounding form one allocation-free solver path.                                            | A shared bounded-root state reduces branches while preserving full-domain results and measured ordinary-case throughput.                       |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TrySolveBoundedQuadratic(..., RationalBound320, ...)`   | 30 | 100% line / 100% branch (Release / ReleaseLean) | Centered and affine axial bounds require exact 320-bit rational evaluation before the same narrow/wide radial solve and nearest-even result conversion.                                                                      | A shared rational-bound solver reduces state without widening the hot path or weakening full-domain products.                                  |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TryGetSphericallyExpandedBoxFirstDistance(...)`         |         14 | 100% line / 100% branch | At most six exact box-plane transitions partition one authored chord into fixed closest-feature intervals whose wide squared-distance quadratics are solved in deterministic order.                                          | Another exact box-distance primitive can share the transition state with lower complexity and neutral or faster measured cost.                 |
| `FixedMathSharp`                  | `Fixed64.TryGetSignedRawRatio(Signed576, Signed576, out Fixed64)`                   |         26 | 100% line / 100% branch | Direct signed 576-bit ratio conversion keeps magnitude alignment, fixed-limb quotient, guard/sticky rounding, signed limits, and explicit failure allocation-free.                                                           | A shared fixed-limb divider preserves every raw-ratio boundary with lower complexity and neutral or faster measured cost.                      |
| `FixedMathSharp`                  | `WideArithmetic.GetBitLength(Signed704)`                                            |         20 | 100% line / 100% branch | Eleven-word lexicographic bit selection keeps the scaled discriminant square-root seed allocation-free and explicit across every limb.                                                                                       | A portable fixed-width bit-scan primitive provides identical word order and lower measured cost.                                               |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TrySolveBoundedQuadraticAtDistance(...)`                | 24 | 100% line / 100% branch (Release / ReleaseLean) | Exact physical-distance roots combine unit-interval endpoint sums or general rational clipping, a scaled 704-bit discriminant, midpoint correction, and one final Q32.32 conversion without normalizing the authored chord. | A shared root result reduces state without adding rounding boundaries or regressing the zero-allocation benchmark.                             |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.SubtractRoundedCylinderSigned(...)`                     | 18 | 100% line / 100% branch (Release / ReleaseLean) | Fixed-width sign-magnitude subtraction keeps zero, equal-sign, opposite-sign, and magnitude-order handling allocation-free inside the exact rim solver.                                                                      | A reusable signed fixed-limb value preserves the same carry order and improves the rounded-rim benchmark.                                      |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.FindRoundedCylinderRoot(...)`                           |         22 | 100% line / 100% branch | Exact Sturm isolation retains repeated roots, endpoint roots, first/last ownership, sign-changing fast paths, and half-even distance conversion in one bounded loop.                                                         | A narrower root-isolation state reduces decisions without weakening repeated-root handling or slowing exact sweeps.                            |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TryGetRoundedCylinderCapCoreDistanceInterval(...)`      |         20 | 100% line / 100% branch | Exact cap-core ownership intersects radial and axial quadratics, resolves rounded public-distance ties algebraically, and preserves strict endpoint containment.                                                             | A shared exact intersection-of-quadratics primitive reduces state without reintroducing rounded false positives.                               |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TryGetRoundedCylinderRimDistanceInterval(...)`          |         18 | 100% line / 100% branch | The allocation-free rim interval driver handles stationary geometry, repeated roots, start/end containment, first-only callers, and exact entry/exit isolation.                                                              | Root state can be shared with another exact quartic consumer at neutral or faster measured cost.                                               |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.BuildRoundedCylinderNegativeRemainder(...)`             |         18 | 100% line / 100% branch | Degenerate quartics require an exact pseudo-remainder fallback with explicit coefficient alignment, sign propagation, degree trimming, and power-of-two normalization.                                                       | A specialized linear-remainder formula covers every reachable degeneration with simpler proven bounds.                                         |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.BuildRoundedCylinderQuarticSturmSequence(...)`          |         16 | 100% line / 100% branch | Factorized quartic subresultants preserve repeated-root tangency without materializing coefficients beyond the proven 44-limb workspace.                                                                                     | A smaller factorization or reusable exact quartic primitive preserves all degeneracies and improves benchmarks.                                |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.IsPointInRoundedCylinderRimTube(...)`                   |         14 | 100% line / 100% branch | Closed and strict endpoint classification share exact inner-region and quartic predicates for both segment endpoints.                                                                                                        | Endpoint classification can reuse a simpler exact torus predicate without duplicate wide evaluation.                                           |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.IsRoundedCylinderEntryBeforeOrEqualToExit(...)`         |         12 | 100% line / 100% branch | Exact radical-bound comparison prevents two distinct cap intervals that round to the same public distance from becoming a false hit.                                                                                         | A shared exact quadratic-bound comparator expresses the same inequality with fewer wide products.                                              |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TryGetAxialInterval(...)`                               |         14 | 100% line / 100% branch | Closed finite-axis admission keeps stationary, reversed, disjoint, and clipped rational bounds in one fixed-width decision path.                                                                                             | Another exact segment primitive can reuse the bound result without extra products or branches.                                                 |
| `FixedMathSharp`                  | `WideFiniteAxisIntersection.TryGetCenteredAxialInterval(...)`                       |         14 | 100% line / 100% branch | Centered finite axes use exact signed 320-bit symmetric bounds before deterministic clipping, including stationary-query classification.                                                                                     | Centered and endpoint axes can share a simpler bound primitive without widening products or adding branches.                                   |
| `FixedMathSharp`                  | `WideArithmetic.GetBitLength(Signed576)`                                            |         16 | 100% line / 100% branch | Nine-word lexicographic bit selection keeps the scaled integer-square-root seed allocation-free and explicit across every limb.                                                                                              | A portable fixed-width bit-scan primitive provides identical word order and lower measured cost.                                               |
| `FixedMathSharp`                  | `WideArithmetic.CompareNonNegative(Signed576, Signed576)`                           |         36 | 100% line / 100% branch | Nine-word lexicographic comparison keeps fixed-width square-root and midpoint ordering allocation-free and independent of target-specific big-integer support.                                                               | A fixed-width value type or portable intrinsic provides the same limb ordering with lower complexity and neutral measured cost.                |
| `FixedMathSharp`                  | `WideArithmetic.GetBitPair(Signed704, int)`                                         |         11 | 100% line / 100% branch | Restoring square root consumes the fixed eleven-limb input two bits at a time through one allocation-free word-selection helper.                                                                                             | A fixed-width shift primitive produces the same bit pairs with simpler equally portable code.                                                  |
| `FixedMathSharp`                  | `FixedQuaternion.TryFormat(...)`                                                    |         18 | 100% line / 100% branch | Span formatting has fixed delimiter and component-write branches that avoid intermediate strings.                                                                                                                            | Formatting shape changes or a shared zero-allocation formatter becomes available.                                                              |
| `FixedMathSharp`                  | `Fixed4x4.TryFormat(...)`                                                           |         18 | 100% line / 100% branch | Matrix span formatting writes a fixed 4x4 shape without temporary strings.                                                                                                                                                   | Formatting shape changes or a shared zero-allocation formatter becomes available.                                                              |
| `FixedMathSharp`                  | `Vector4d.TryFormat(...)`                                                           |         18 | 100% line / 100% branch | Vector span formatting writes a fixed component shape without temporary strings.                                                                                                                                             | Formatting shape changes or a shared zero-allocation formatter becomes available.                                                              |
| `FixedMathSharp`                  | `FixedMath.Sqrt(Fixed64)`                                                           |         18 | 100% line / 100% branch | Integer square-root logic is branchy by nature and sits on a core deterministic math path.                                                                                                                                   | A simpler algorithm is adopted with equal determinism and performance.                                                                         |
| `FixedMathSharp`                  | `Fixed3x3Extensions.FuzzyEqual(Fixed3x3, Fixed3x3, Fixed64?)`                       |         18 | 100% line / 100% branch | Component-wise fuzzy equality is intentionally explicit to avoid allocation and preserve inlining.                                                                                                                           | The matrix equality helpers are generated or centralized without adding runtime overhead.                                                      |
| `FixedMathSharp`                  | `FixedBoundSphere.ContainsBoxLike(Vector3d, Vector3d)`                              |         18 | 100% line / 100% branch | Checks all box-like corners against the sphere; explicit shape avoids temporary corner arrays.                                                                                                                               | Bounds internals move to a shared corner iterator that is allocation-free.                                                                     |
| `FixedMathSharp`                  | `FixedMath.Atan(Fixed64)`                                                           |         18 | 100% line / 100% branch | Trigonometric approximation and range handling are deterministic hot-path math.                                                                                                                                              | Approximation strategy changes or benchmark evidence supports decomposition.                                                                   |
| `FixedMathSharp`                  | `Fixed4x4.get_Item(int)`                                                            |         17 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection.                                                                                                                                      | The matrix layout changes or generated indexer code becomes part of the build.                                                                 |
| `FixedMathSharp`                  | `Fixed4x4.set_Item(int, Fixed64)`                                                   |         17 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection.                                                                                                                                      | The matrix layout changes or generated indexer code becomes part of the build.                                                                 |
| `FixedMathSharp.FluentAssertions` | `FixedAssertionHelpers.AreComponentApproximatelyEqual(Fixed3x3, Fixed3x3, Fixed64)` |         16 | 100% line / 100% branch | Fixed-shape assertion over all matrix components keeps helper behavior direct and allocation-free.                                                                                                                           | Assertion diagnostics degrade, matrix shape changes, or repeated assertion logic grows further.                                                |
| `FixedMathSharp`                  | `Fixed3x3Extensions.FuzzyEqualAbsolute(Fixed3x3, Fixed3x3, Fixed64)`                |         16 | 100% line / 100% branch | Direct component-wise comparison avoids loops and keeps the extension inlinable.                                                                                                                                             | The matrix equality helpers are generated or centralized without adding runtime overhead.                                                      |
| `FixedMathSharp`                  | `Fixed3x3.Equals(Fixed3x3)`                                                         |         16 | 100% line / 100% branch | Direct 3x3 value comparison avoids loops, allocations, and indexer overhead on a hot value type.                                                                                                                             | Equality semantics change or a generated/source-shared component comparison becomes available without runtime cost.                            |
| `FixedMathSharp`                  | `FixedMath.Asin(Fixed64)`                                                           |         16 | 100% line / 100% branch | Trigonometric domain handling and approximation are deterministic hot-path math.                                                                                                                                             | Approximation strategy changes or benchmark evidence supports decomposition.                                                                   |
| `FixedMathSharp`                  | `Fixed64.LerpFullDomain(Fixed64, Fixed64, Fixed64)`                                 |         16 | 100% line / 100% branch | Full-domain interpolation must preserve a 65-bit difference and final-result nearest-even parity without a saturating intermediate.                                                                                          | A simpler full-width primitive provides identical endpoint, sign, and tie behavior at equal or lower measured cost.                            |
| `FixedMathSharp`                  | `Fixed64.RoundSquareRootToFixed(...)`                                               |         16 | 100% line / 100% branch | Exact triangle area conversion keeps root remainder, half-even midpoint, final carry, and positive saturation at one Q32.32 boundary.                                                                                        | Another exact square-root result type can centralize the conversion without obscuring remainder-based rounding.                                |
| `FixedMathSharp`                  | `Fixed4x4.IsNormalizedOrthogonalBasis(Vector3d, Vector3d, Vector3d)`                |         16 | 100% line / 100% branch | Fixed-shape basis validation keeps three representable magnitudes, unit tolerances, and pairwise orthogonality checks together at the decomposition gate.                                                                    | Another decomposition path can reuse the exact predicate without duplicating its fixed sequence.                                               |
| `FixedMathSharp`                  | `WideArithmetic.GetFloorSquareRoot(Signed320, out Signed192)`                       |         15 | 100% line / 100% branch | Fixed five-word restoring square root preserves exact root/remainder and dispatches to a smaller ordinary-range path without allocation.                                                                                     | A faster portable integer-root algorithm preserves exact remainder and both fixed-width paths across targets.                                  |
| `FixedMathSharp`                  | `Fixed64.TryGetUnitIntervalRatio(Signed192, Signed192, out Fixed64)`                |         12 | 100% line / 100% branch | Exact sign/range classification dispatches to the minimum fixed-width 33-bit guard/sticky quotient while preserving explicit default failure.                                                                                | A shared fixed-width division core can express both representation widths more simply and without regression.                                  |
| `FixedMathSharp`                  | `FixedPlane.IntersectsBoxLike(Vector3d, Vector3d)`                                  |         16 | 100% line / 100% branch | Plane-vs-bounds classification is branch-heavy but fixed-shape and allocation-free.                                                                                                                                          | Additional bound shapes are added and a shared allocation-free helper becomes clearer.                                                         |
| `FixedMathSharp`                  | `FixedBoundFrustum.Contains(FixedBoundBox)`                                         |         14 | 100% line / 100% branch | Frustum containment combines plane checks and box corners without allocating an intermediate shape.                                                                                                                          | Containment semantics change or a shared allocation-free corner helper is introduced.                                                          |
| `FixedMathSharp`                  | `FixedBoundFrustum.Contains(FixedBoundSphere)`                                      |         14 | 100% line / 100% branch | Frustum containment must distinguish disjoint, intersecting, and fully contained sphere cases.                                                                                                                               | Sphere/frustum containment semantics change or the plane classification helper is redesigned.                                                  |
| `FixedMathSharp`                  | `FixedBoundFrustum.IntersectsFrustum(FixedBoundFrustum)`                            |         14 | 100% line / 100% branch | Separating-axis frustum checks are algorithmically branch-heavy and intentionally avoid allocations.                                                                                                                         | A robust shared SAT helper can improve clarity without extra allocations or worse benchmarks.                                                  |
| `FixedMathSharp`                  | `FixedBoundBox.ClosestPointOnSurface(Vector3d)`                                     |         14 | 100% line / 100% branch | The nearest-face selection is fixed-shape and explicit; helper extraction would not reduce real complexity.                                                                                                                  | Box surface projection grows beyond nearest-face selection.                                                                                    |
| `FixedMathSharp`                  | `FixedCurve.Evaluate(Fixed64)`                                                      |         14 | 100% line / 100% branch | Curve evaluation combines clamping, segment search, and interpolation mode dispatch in one readable flow.                                                                                                                    | More interpolation modes are added or segment lookup becomes a performance bottleneck.                                                         |
| `FixedMathSharp`                  | `Fixed3x3.TryFormat(...)`                                                           |         14 | 100% line / 100% branch | Matrix span formatting writes a fixed 3x3 shape without temporary strings.                                                                                                                                                   | Formatting shape changes or a shared zero-allocation formatter becomes available.                                                              |
| `FixedMathSharp`                  | `FixedMath.Pow2(Fixed64)`                                                           |         14 | 100% line / 100% branch | Fixed-point exponent approximation is compact, deterministic, and performance-sensitive.                                                                                                                                     | Approximation strategy changes or benchmark evidence supports decomposition.                                                                   |
| `FixedMathSharp`                  | `FixedMath.Tan(Fixed64)`                                                            |         14 | 100% line / 100% branch | Tangent delegates through fixed trigonometric identities and guarded edge handling.                                                                                                                                          | New domain behavior is added or uncovered lines become reachable with valid input.                                                             |
| `FixedMathSharp`                  | `Vector3d.TryFormat(...)`                                                           |         14 | 100% line / 100% branch | Vector span formatting writes a fixed component shape without temporary strings.                                                                                                                                             | Formatting shape changes or a shared zero-allocation formatter becomes available.                                                              |
| `FixedMathSharp`                  | `Fixed64.RoundSquaredDistance(Signed192)`                                           |         14 | 100% line / 100% branch | Exact Q64.64 squared sums need explicit nearest-even tie handling and positive saturation at the single public conversion boundary.                                                                                          | A shared exact-distance conversion expresses the same degeneracy, rounding, and saturation contract more simply.                               |
| `FixedMathSharp`                  | `WideArithmetic.GetFloorSquareRoot192(...)`                                         | 12 | 100% line / 100% branch (Release / ReleaseLean) | The common three-word root path uses two-word root/remainder state to avoid full five-word work while returning the identical exact result.                                                                                  | A faster portable root primitive preserves every perfect-square, remainder, and word-boundary result.                                          |
| `FixedMathSharp`                  | `Fixed64.NormalizeWideComponent(...)`                                               |         14 | 100% line / 100% branch | Exact component normalization combines common scaling, lower-candidate division, and squared midpoint correction into one allocation-free path.                                                                              | A shared three-component normalization root removes repeated work while preserving independent nearest-even results.                           |
| `FixedMathSharp`                  | `WideArithmetic.AddSigned320(Signed320, Signed320)`                                 | 12 | 100% line / 100% branch (Release / ReleaseLean) | Five-word signed addition keeps carry propagation explicit, allocation-free, and independent of target-specific wide integer support.                                                                                        | A portable fixed-width value type provides equally visible carry order and neutral or faster measured cost.                                    |
| `FixedMathSharp`                  | `Fixed64.GetFloorSquareRoot(ulong, ulong, out ulong, out ulong)`                    | 12 | 100% line / 100% branch (Release / ReleaseLean) | The private two-word restoring square root keeps pair selection, remainder comparison, subtraction, and exact remainder output in one hot loop.                                                                              | A portable intrinsic or simpler root primitive preserves every 128-bit root and remainder boundary.                                            |
| `FixedMathSharp`                  | `Fixed64.op_Multiply(Fixed64, Fixed64)`                                             |         12 | 100% line / 100% branch | Full-width multiply with saturating and round-half-to-even behavior requires explicit guarded paths.                                                                                                                         | Additional uncovered reachable branches appear, arithmetic semantics change, or benchmarks support a simpler equivalent.                       |
| `FixedMathSharp`                  | `FixedSegment.PointOnSegment(...)`                                                  |         12 | 100% line / 100% branch | Exact axis bounds plus a Gram determinant preserve an existing endpoint only for a true finite-segment zero-separation contact.                                                                                              | A cheaper exact collinearity predicate preserves endpoint identity across the complete raw domain.                                             |
| `FixedMathSharp`                  | `Fixed3x3.get_Item(int)`                                                            |         12 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection.                                                                                                                                      | The matrix layout changes or generated indexer code becomes part of the build.                                                                 |
| `FixedMathSharp`                  | `Fixed3x3.set_Item(int, Fixed64)`                                                   |         12 | 100% line / 100% branch | Switch-based fixed matrix indexing is direct and avoids table allocation or reflection.                                                                                                                                      | The matrix layout changes or generated indexer code becomes part of the build.                                                                 |
| `FixedMathSharp`                  | `FixedBoundFrustum.Contains(Vector3d)`                                              |         12 | 100% line / 100% branch | Point containment checks all frustum planes directly and avoids temporary arrays.                                                                                                                                            | Plane ordering or containment semantics change.                                                                                                |
| `FixedMathSharp`                  | `FixedBoundFrustum.Intersects(FixedRay)`                                            |         12 | 100% line / 100% branch | Slab-style frustum clipping has unavoidable enter/exit branches and benefits from locality.                                                                                                                                  | More ray/frustum edge cases are discovered or clipping logic is shared elsewhere.                                                              |
| `FixedMathSharp`                  | `FixedMath.FloorLog2(ulong)`                                                        |         12 | 100% line / 100% branch | Bit-scan fallback is branch-heavy by nature and must remain deterministic across targets.                                                                                                                                    | A target-safe intrinsic or simpler implementation is introduced.                                                                               |
| `FixedMathSharp`                  | `WideArithmetic.CompareUnsigned(3-word, 3-word)`                                    |         12 | 100% line / 100% branch | Lexicographic comparison over three fixed words is explicit, allocation-free, and shared by exact geometry ratio and distance ordering.                                                                                      | A fixed-width value type centralizes comparison with equal inlining and benchmark behavior.                                                    |
| `FixedMathSharp`                  | `WideGeometry.GetDifferenceCrossProduct3D(...)`                                     |         12 | 100% line / 100% branch | The exact cross root selects a proven signed-raw fast path only when all six differences fit, otherwise retaining full 65-bit endpoint arithmetic.                                                                           | Difference ownership can be shared without adding state or slowing ordinary and extreme triangle rows.                                         |
| `FixedMathSharp`                  | `WideGeometry.GetDifferenceDotProduct3D(...)`                                       |         12 | 100% line / 100% branch | The exact dot root selects a signed-raw fast path only when all six differences fit and falls back to full endpoint-difference accumulation.                                                                                 | Cached difference state simplifies repeated triangle dots without allocations or a benchmark regression.                                       |
| `FixedMathSharp`                  | `FixedTriangle2d.Contains(Vector2d)`                                                |         12 | 100% line / 100% branch | Exact winding, three epsilon-inclusive edge classifications, and collapsed-edge fallback stay together at the public containment boundary.                                                                                   | Containment policy expands or a shared exact-orientation classifier removes branches without changing hot-path cost.                           |
| `FixedMathSharp`                  | `Fixed4x4.AppendRow(...)`                                                           |         12 | 100% line / 100% branch | Private span-format helper appends a fixed row shape with delimiter handling and no temporary strings.                                                                                                                       | Formatting shape changes or a shared zero-allocation formatter becomes available.                                                              |
| `FixedMathSharp`                  | `Fixed64.CompareMagnitudeSquared(...)`                                              |         12 | 100% line / 100% branch | Exact squared-magnitude ordering compares overflow, high, and low words directly without projecting either sum back into Q32.32.                                                                                             | A reusable fixed-width magnitude value provides the same lexicographic order without extra construction or cost.                               |
| `FixedMathSharp`                  | `CoordinateConvention3d.ctor(Axis3d, Axis3d, Axis3d)`                               |         12 | 100% line / 100% branch | The public constructor validates three defined signed axes and rejects every duplicate absolute-axis pairing before storing an immutable basis.                                                                              | Axis validation moves to a shared zero-overhead basis type with equally specific argument errors.                                              |
| `FixedMathSharp`                  | `FixedTransform.TrySetParentKeepingWorld(FixedTransform?)`                          |         12 | 100% line / 100% branch | Atomic reparenting validates ancestry, inverse, decomposition, and world recomposition before committing any local or parent state.                                                                                          | A shared atomic transform mutation result reduces branches without exposing partial state or weakening verification.                           |
| `FixedMathSharp`                  | `Fixed64.TryGetSignedRawRatioCore(...)`                                             | 20 | 100% line / 100% branch (Release / ReleaseLean) | Shared zero-denominator rejection, quotient-width validation, the measured single-limb specialization, general fixed-limb division, rounding, and signed materialization stay in one allocation-free deterministic boundary. | A simpler shared divider preserves zero, width, rounding, and signed-range policies with lower complexity and neutral or faster measured cost. |

### Cylinder-pair contacts

Exact feature selection retains rank-one recovery because it can determine the
unique minimum. Geometry, degree, sign, rounding and live-workspace invariants
are documented beside the owning implementation; the registered methods keep
these decisions within their bounded arithmetic owners.

| Method | Complexity | Coverage | Rationale | Revisit if |
| --- | ---: | --- | --- | --- |
| `CylinderPairAnalyticFeatures.Build(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Exact cap axes and their cross product populate the shared rational/radical candidate with separate authored half-axes and primitive directions. | The analytic feature set, raw scale or shared candidate-width proof changes. |
| `CylinderPairRepeatedRim.TryGetZeroNormal(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Exact rim intersection and tangent-cross admission preserve closed zero contact; cap/side boundaries retain their earlier ownership. | Zero-gap cone ownership or finite-cap admission changes. |
| `CylinderPairRepeatedRim.GetValueAndLine(...)` | 60 | 100% line / 100% branch (Release / ReleaseLean) | Independent, dependent and orthogonal projections require distinct exact eigenvalue compatibility cases before one certified rank-one line is retained. | A narrower exact parameterization preserves the unique-minimum rank-one family and all projection degeneracies. |
| `CylinderPairRepeatedRim.GetRegularNormal(...)` | 24 | 100% line / 100% branch (Release / ReleaseLean) | Line-circle recovery retains merged tangent branches, unsquared radial signs and finite cap inequalities in the same quadratic extension; dominated H=0 values are excluded. | A shared recovery representation reduces work without admitting a squared conjugate or losing a true rank-one minimum. |
| `CylinderPairRepeatedRim.RoundNormal(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Three exact world-normal components retain both quadratic extensions through signed nearest-even midpoint comparisons. | Another exact rounding owner supports these nested radicals within the same scratch bound. |
| `ConvexContactValueRoot.GetRoundedDepth(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Shared signed root-to-dyadic comparisons classify radius-plus-gap before rounding, distinguish conceptual overflow from an exact maximum, and preserve nearest-even half-raw depth ties for existing contact owners and capsule slabs. | The physical-value scale or shared algebraic conversion contract changes. |
| `ConvexContactValueRoot.GetRoundedNormalComponent(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Conservative common-normalization bounds for component square / squared length narrow the existing integer search; exact component and half-raw signs retain orientation, zero components and nearest-even rounding, including radius-scaled witnesses. | A cheaper certified enclosure preserves the exact rounding and bounded-resource contracts. |
| `CylinderPairSideValuePolynomial.Build(in CylinderPairGeometry, ...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Side/cap orientation selects exact geometry invariants for the shared unshifted quartic and full-radius octic construction. | Side ownership or exact first/second-frame scaling changes. |
| `CylinderPairValuePolynomial.Build(...)` | 22 | 100% line / 100% branch (Release / ReleaseLean) | One fixed cubic-pencil construction produces the squared-distance discriminant and optional analytic derivatives with shared polynomial arithmetic; actual value/direction heights narrow scratch within both the generic and caller capacity proofs, while retained output keeps its original stride. | The feature/width proof or derivative consumers change, or a simpler exact construction lowers degree or live workspace. |
| `WideConvexPrismRelations.TryImproveCylinderCapsuleEllipse(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | The existing quartic ellipse winner competes against the shared analytic candidate before exact full-gap classification and final rounding. | Winner ordering, signed radius offsets or the ellipse's sole-interior-minimum proof changes. |
| `WideConvexPrismRelations.TrySelectCylinderPairFeature(...)` | 30 | 100% line / 100% branch (Release / ReleaseLean) | Ordered cap/cross, side and rim traversal owns separation, exact zero and stable ties; no duplicate sampled-direction solver is introduced. | The complete feature proof or canonical tie order changes. |
| `WideConvexPrismRelations.TryKeepCylinderPairCapValues(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Compact all-root enumeration shares one repeated-factor classification and lazily constructs admission derivatives within one cap-pair lifetime. | Root storage, repeated-family ownership or derivative lifetime bounds change. |
| `WideConvexPrismRelations.TryHandleRepeatedCylinderPairValue(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Exact factor-cell matching identifies repeated values before positive-gap-only rank-one recovery; non-improving values and later equal branches stop immediately. | The nonseparating repeated-family proof or first-admitted-branch tie policy changes. |
| `WideConvexPrismRelations.TryGetCylinderPairSideFeature(...)` | 52 | 100% line / 100% branch (Release / ReleaseLean) | Existing cylinder/capsule principal, perpendicular-corner and ellipse owners select one signed side minimum, then map its complete radius-offset value exactly. | A simpler complete side representation preserves authored geometry, signed offsets and common-perpendicular tie ownership. |
| `WideFiniteAxisIntersection.AddFiniteAxisPolynomial(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Signed integer multipliers and degree offsets accumulate into independently strided caller storage using the shared shifted-magnitude owner. | Construction callers no longer need signed scaling/offsets or coefficient-width preconditions change. |
| `WideFiniteAxisIntersection.TryIsolateFiniteValueRootCells(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Ascending output slots own pending dyadic intervals, sharing subdivision counts without another numerator tree; canonical singleton roots and compact failure preserve authoritative count and repetition. | Root ordering, interval inclusion or compact layout changes. |
| `WideFiniteAxisIntersection.IsolateFiniteValueCell(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | One tail serves independent and shared isolation, preserving positive nonroot endpoints, exact singleton roots and explicit compact capacity. | Root interval policy or the proven separation/storage bounds change. |
| `WideFiniteAxisIntersection.GetFiniteValueVariations(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Borrowed evaluation scratch certifies nonzero row signs; uncertain, constant and endpoint cases retain exact derivative right limits. | The arena, precision cap or sign acceptance proof changes. |
| `WideFiniteAxisIntersection.GetFiniteValueReciprocalRoots(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Eligible reciprocal polynomials share one Sturm construction with independent counts, cells and compact outcomes; endpoint-degenerate inputs preserve existing zero-factor metadata through complete owners. | Nominal degree, endpoint policy or borrowed-view ownership changes. |
| `WideFiniteAxisIntersection.ReverseFiniteValueSturm(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Actual-degree reversal and alternating row signs preserve generalized Sturm orientation on positive parameters, including repeated roots and abnormal degree drops. | Reciprocity is extended across zero/infinity factors or ordering changes. |
| `WideFiniteAxisIntersection.RefineFiniteValueRoot(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Singleton/finer cells are unchanged; the shared crossing classifier/refiner accepts certified opposite signs while noncrossing roots retain exact Sturm identity. Simple-root callers may reuse proven nonzero endpoint signs. | A new refinement certificate preserves repeated roots and caller-owned cell capacity at lower cost. |
| `WideFiniteAxisIntersection.BuildFiniteValueSturm(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Brown subresultant recurrence tracks positive power-of-two content and separate Sturm row orientation in graded storage, including abnormal degree drops. | Degree, coefficient-height or exact scalar-divisibility proofs change. |
| `WideFiniteAxisIntersection.BuildFiniteValueSubresultant(...)` | 28 | 100% line / 100% branch (Release / ReleaseLean) | Signed pseudo-division, exact Brown scalar division and degree trimming remain one bounded arithmetic operation with proven intermediate widths. | A simpler exact PRS preserves signs and abnormal drops while reducing measured work or live scratch. |
| `WideFiniteAxisIntersection.NormalizeFiniteValueContent(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Shared magnitude GCD and exact division remove only positive polynomial content, preserving every root and Sturm sign. | Content policy changes or a cheaper exact reducer preserves canonical coefficients. |
| `WideFiniteAxisIntersection.TryGetFiniteValueRootsBernstein(...)` | 40 | 100% line / 100% branch (Release / ReleaseLean) | Fixed-depth/node integer subdivision certifies zero/one-root cells; midpoint roots or exhausted budgets discard partial output and defer to Sturm. | The measured certificate benefit disappears or a different bounded exact certificate lowers workspace without weakening fallback. |
| `WideFiniteAxisIntersection.HasFiniteValueSquareFreeCertificate(...)` | 28 | 100% line / 100% branch (Release / ReleaseLean) | Degree-preserving arithmetic modulo 65537 certifies a nonzero resultant; an inconclusive modular GCD never asserts repeated roots over the integers. | The degree bound or prime changes, or modular preprocessing no longer benefits the isolation workload. |
| `WideFiniteAxisIntersection.MapKnownFiniteValueRootRadicalOffset(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Exact mapped cells identify a proven target member; source/target precision bounds certify unresolved overlap without another polynomial-composition solver. | The known-membership, signed-radius branch or root-separation contract changes. |
| `WideFiniteAxisIntersection.ClassifyRadicalOffsetCells(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Ordered known-member search combines negative-branch clipping, exact singleton handling and strict mapped containment without redundant maximum rejection. | Candidate order or the admitted negative-offset domain changes. |
| `WideFiniteAxisIntersection.CompareRadicalOffsetEndpoints(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Exponent-aligned rational/radical signs compare dyadic endpoints exactly, retaining zero terms and cancellation before squaring. | Endpoint domain, radius scaling or the five-exponent-width scratch proof changes. |
| `WideFiniteAxisIntersection.TryRefineFiniteValueCrossingRoot(...)` | 46 | 100% line / 100% branch (Release / ReleaseLean) | Shared endpoint certificates classify an odd crossing and supply secant hints; uncertain points use exact evaluation. Equal/zero parent signs retain Sturm fallback. Hints predict byte cells only; certified opposite signs admit the same root, with singleton detection and restoring bisection after an inconclusive prediction. | Prediction ceases to improve measured refinement or sign/error bounds change. |
| `WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(...)` | 22 | 100% line / 100% branch (Release / ReleaseLean) | Positive-domain boundaries, singleton equality, cell order and exact interior evaluation preserve root identity without rounding the comparison point. | The root domain or excluded-endpoint policy changes. |
| `WideFiniteAxisIntersection.CompareFiniteValueRoots(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Shared-factor equality and exact dyadic ordering distinguish equal roots from different ordinals before bounded refinement. | Factor identity, cell ownership or the joint separation proof changes. |
| `WideFiniteAxisIntersection.GetSignAtFiniteValueRootCore(...)` | 38 | 100% line / 100% branch (Release / ReleaseLean) | Constant/singleton cases, a certified retained-cell trial, caller-capacity-aware retained refinement and root-local normalized signs share one exact equality/sign authority. Up to 59 retained work bits preserve the production two-word budget. A decisive trial leaves the cell unchanged and skips defining-height scanning; uncertainty retains the resultant bound and equality fallback. Virtual dyadic scaling preserves the scratch/refinement bound. | Query degree/height, normalization proof or retained-cell mutation policy changes. |
| `WideFiniteAxisIntersection.GetFiniteValueApproximateSignCore(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | One caller-buffer Horner core retains the proved truncation/error bound; counting avoids unused hints while the prediction wrapper computes the same magnitude/exponent. Uncertain signs retain exact fallback. | Precision, small-value attenuation or coefficient-quantization bounds change. |
| `WideArithmetic.AddShiftedSignedMagnitude(...)` | 22 | 100% line / 100% branch (Release / ReleaseLean) | One shared in-place owner handles shifted carry, borrow, cancellation and result sign without a shifted copy or subtraction buffer. Same-sign addition visits active source words and reuses bounded carry propagation beyond them. | Aliasing, canonical sign or destination-capacity preconditions change. |
| `WideArithmetic.MultiplyMagnitudes(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | One shared truncated convolution retains sparse-row carry and dirty-output clearing. Input clipping and bounded zero-prefix factoring remove terms that cannot affect retained product limbs, with no additional scratch. | Product/input overlap, retained width, carry bounds or the exact whole-word factoring contract changes. |
| `WideArithmetic.DivideMagnitudes(...)` | 32 | 100% line / 100% branch (Release / ReleaseLean) | Normalized multiword division bounds quotient correction, retains exact remainder and supports documented numerator/output overlap with caller-owned scratch. | A portable exact divider lowers cost while preserving correction bounds and every overlap contract. |
| `Fixed64.Divide128By64(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Shared quotient-fit division uses two base-2^32 digits with bounded trial correction and an exact remainder, replacing bit-at-a-time wide division. | The quotient-fit precondition changes or a portable intrinsic matches both target frameworks and remainder semantics. |

### Box/cylinder contacts

The complete box/cylinder owner reuses the convex candidate comparator,
projected-disk quartic and finite-value root kernel. Shared normal/depth
materialization belongs to `ConvexContactValueRoot`. The box path does not build
the cylinder-pair radius-offset octic merely to recover its unshifted quartic.

| Method | Complexity | Coverage | Rationale | Revisit if |
| --- | ---: | --- | --- | --- |
| `BoxCylinderAnalyticFeatures.TryGetBest(...)` | 34 | 100% line / 100% branch (Release / ReleaseLean) | Canonical poles, face/side boundaries, radial vertices and exceptional edge directions exhaust the analytic feature set; smooth vertex/rim certificates still detect separation. | The complete support-feature proof changes or measured pruning removes repeated directions. |
| `BoxCylinderAnalyticFeatures.KeepAxis(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Zero directions, separation and exact stable ties are handled before copying the shared candidate. | Candidate ownership or canonical tie order changes. |
| `BoxCylinderAnalyticFeatures.BuildRankingAxis(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | The exact support square cancels the positive axis-length factor, then omits the common positive gap-denominator factor during private same-geometry ranking. Both selection exits restore raw units using dead direction scratch. Signed gaps, ties, zero-radius reductions and canonical coefficient signs retain one bounded owner. | A shared support representation preserves the same cancellation and exit restoration without larger intermediates or extra ownership. |
| `BoxCylinderAnalyticFeatures.HasVertexRimSeparation(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | A smooth rim residual is a separator only after finite-cap, radial and exact box-cone admission; positive penetration minima belong to boundary features. | The zero-curvature proof or cap/cone ownership changes. |
| `BoxCylinderEdgeContacts.TryGetContact(...)` | 32 | 100% line / 100% branch (Release / ReleaseLean) | Bounded signed edge/cap charts retain exact zero and stable winners, with proven circle/rectangle reductions and cone-wide dominance pruning. | A smaller complete parameterization removes charts without approximating geometry. |
| `BoxCylinderEdgeContacts.TryChart(...)` | 30 | 100% line / 100% branch (Release / ReleaseLean) | Quartic roots retain cap and unsquared stationary signs before separation and exact zero; an unclamped analytic rounding bound rejects proven nonwinners before squared-value mapping and exact ranking. | A cheaper exact parameter-root admission or comparison preserves separation, zero gaps and stable minimum-depth ties. |
| `WideOrientedBox.TryGetCenteredCylinderContact(..., out CenteredCylinderContactFeature)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | One boundary owns conservative broad rejection, analytic/edge selection, final rounding, exact box support signs and manifold feature metadata. | Contact or manifold ownership changes. |
| `WideOrientedBox.GetCylinderCapFace(...)` | 30 | 100% line / 100% branch (Release / ReleaseLean) | Three explicit component tests admit only exact parallel box-face/cylinder-cap pairs; rounded world normals cannot fabricate a manifold. | Another consumer shares this exact three-axis classification. |

### Triangle/cylinder and capsule-slab contacts

Triangle/cylinder contact shares support algebra, stationary quartics and exact
root signs with the box/cylinder owner. It keeps triangle feature selection and
paired witness construction local. The box's analytic support builder cancels
its positive axis-length factor before encoding the shared candidate; its
signed-gap and coefficient decisions remain registered above.
Positive-core slabs reuse that owner in two admitted endpoint regions and a
bounded straight-section seam, rather than introducing another contact solver.

| Method | Complexity | Coverage | Rationale | Revisit if |
| --- | ---: | --- | --- | --- |
| `TriangleCylinderAnalyticFeatures.TryGetBest(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | One exact selection spans the seam and admitted endpoint regions, with proven face certificates. | A new exact certificate removes feature work while preserving canonical ties. |
| `TriangleCylinderAnalyticFeatures.KeepRegion(...)` | 28 | 100% line / 100% branch (Release / ReleaseLean) | Ordered face, pole, side and edge boundaries exhaust one analytic region; smooth vertex/rim residuals still certify separation. | The support partition admits fewer necessary boundaries. |
| `TriangleCylinderAnalyticFeatures.KeepAxis(...)` | 26 | 100% line / 100% branch (Release / ReleaseLean) | Region admission precedes exact support signs and stable comparison. Opposite directions reuse squared support terms only after an admitted build; the retained local winner is transformed once after selection. | Support ownership or retained candidate layout changes. |
| `TriangleCylinderAnalyticFeatures.HasVertexRimSeparation(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | A closest-rim residual proves separation only inside both the triangle vertex cone and the selected core hemisphere. | A shared residual authority preserves both admission conditions. |
| `TriangleCylinderContact.TryGetContact(...)` | 52 | 100% line / 100% branch (Release / ReleaseLean) | One owner selects analytic/root winners and materializes matched cap, side, rim or seam witnesses with retained exact endpoint metadata. | Another shape can share materialization without weakening authored-frame or feature contracts. |
| `TriangleCircularGeometry.GetAnalyticDepth(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Principal directions cancel their common scale before exact rational rounding; other directions retain the shared radical depth comparison. | A cheaper full-domain reducer preserves odd raw dimensions and conceptual overflow flags. |
| `TriangleCylinderEdgeContacts.TryGetContact(...)` | 32 | 100% line / 100% branch (Release / ReleaseLean) | Both endpoint regions, finite caps and nondegenerate triangle edges retain one complete bounded chart traversal and exact winner. | The stationary-feature proof permits fewer charts. |
| `TriangleCylinderEdgeContacts.TryCharts(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Reciprocal views independently pass admission, share one exact coefficient construction when both are admitted, and finish the first root traversal before reversing the shared algebra and rewriting caller-owned admission coefficients. | A complete shared root proof reduces the two traversals without changing canonical ties or bounded scratch. |
| `TriangleCylinderEdgeContacts.IsChartAdmitted(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Exact affine cap, triangle-cone and core-region endpoint signs reject only wholly inadmissible charts. The basis has one nonzero vertical component, so the cap sign admits the whole retained interval. | The chart basis or support-region contract changes; preserve open parameter endpoints and seam ownership. |
| `TriangleCylinderEdgeContacts.TryChart(...)` | 30 | 100% line / 100% branch (Release / ReleaseLean) | Each stationary root retains triangle-cone and core-region admission. Negative/zero gaps preserve classification, and positive roots reuse the analytic half-raw upper bound before exact value ranking; strict comparisons retain the earlier canonical feature. | A cheaper shared root proof preserves completeness, clamping and canonical boundary ownership. |
| `TriangleCircularGeometry.HasFaceMinimumCertificate(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Exact triangle projection and an inscribed cylinder ball prove the selected face already attains the global minimum. | A broader exact certificate reduces measured feature traversal without replacing the complete fallback. |
| `TriangleCylinderRimWitnesses.GetAnalyticPoint(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Vertex, edge and face support retain rational-plus-radical coordinates until a single nearest-even conversion. | An existing exact reducer can share this representation without rounded intermediate witnesses. |
| `TriangleCylinderRimWitnesses.RoundAnalyticCoordinate(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Vanished or cancelled radicals use exact rational division; other coordinates retain the established signed nearest-even search. Supplied polygon bounds admit a face projection outside its seed without restricting final rounding to that seed. | A shared exact reducer removes further witness search without changing ties, scalar extrema or whole-polygon witness bounds. |
| `TriangleCylinderWitnesses.GetSidePoint(...)` | 26 | 100% line / 100% branch (Release / ReleaseLean) | Tangent-plane intersection and finite axial clipping select a paired witness on the winning triangle support feature. | Feature representation changes or the same exact clipping is needed by another contact owner. |
| `TriangleCylinderWitnesses.GetFaceWeights(...)` | 24 | 100% line / 100% branch (Release / ReleaseLean) | Exact Voronoi regions retain barycentric weights for independently rounded triangle/cylinder anchors. | A shared projection owner preserves full-width weights and first-feature boundary ownership. |
| `TriangleCapsuleSlabWitnesses.GetSideWeights(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Admitted vertices and finite rectangle boundaries recover a paired witness on the selected straight-side feature. | The same exact clipping contract gains another consumer. |
| `TriangleCapsuleSlabWitnesses.GetCapWeights(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Core intersections, triangle vertices and nearest endpoint features exhaust the cap's planar closest-feature cases. | Another owner can share this finite feature reduction without rounding admission. |
| `TriangleQuadraticSlice.TryGetWeights(...)` | 26 | 100% line / 100% branch (Release / ReleaseLean) | Shared slab/cone support-plane crossings and finite interval clipping retain rational or quadratic weights, including closed boundaries. | A simpler bounded slice preserves feature ordering with lower measured cost. |
| `TriangleCapsuleSlabWitnesses.GetMaterials(...)` | 28 | 100% line / 100% branch (Release / ReleaseLean) | Paired triangle/core/radial expressions stay exact until combined-coordinate rounding; integral radial coordinates alone retain exact endpoint residuals. | Anchor storage gains a simpler representation of the same exact feature identity. |
| `ContactQuadratic.RoundRatio(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Shared slab/cone nearest-even conversion handles signed quadratic coordinates and total-coordinate parity without rounded intermediate admission. | Another ratio owner can consume this representation with the same proven width and rounding contract. |

The cone-specific owners retain the same collector-reported complexity
convention as the rows above. The collector counts instrumented branch outcomes
and can omit branchless conditional expressions, so its values differ from
source-decision complexity. With full coverage, each reported CRAP score equals
its listed complexity: no uncovered
coverage multiplier remains. Keep the complete feature proofs together rather
than extracting forwarding methods to lower a metric.

| Method | Complexity | Coverage | Rationale | Revisit if |
| --- | ---: | --- | --- | --- |
| `TriangleConeContact.TryGetContact(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | One winner owns complete classification and exact final materialization; radius zero shares the cylinder segment and analytic witnesses share the same retained-feature owner as polygon contacts. | Another shape shares the same feature/witness contract without a generic mode flag. |
| `TriangleConeContact.GetAnalyticWitnesses(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | The retained apex, base pole, base rim or generator selects matched witnesses, preserving the unrounded local normal until one world-direction conversion and final coordinate materialization. | Another owner can share the same feature and frame contract without duplicating support interpretation. |
| `TriangleConeContact.TryGetPatchFaceContact(...)` | 22 | 100% line / 100% branch (Release / ReleaseLean) | Exact opposite face exits precede sufficient chord-tube or whole-projection clearance proofs over the complete perimeter; generator equality and whole-patch bounds preserve paired final witnesses. | A broader complete certificate removes work without admitting uncovered holes, notches or artificial seams. |
| `TriangleConeContact.TryCertifyPatchChord(...)` | 26 | 100% line / 100% branch (Release / ReleaseLean) | Axial chords, cardinal base diameters and tilted apex supports have distinct exact plane-extremum proofs. Seed membership and opposite-end clearance precede the shared complete-perimeter tube certificate. | A shared contained-chord representation preserves every region and width invariant with lower measured cost. |
| `TriangleConeContact.TryGetConvexPatchContact(...)` | 26 | 100% line / 100% branch (Release / ReleaseLean) | Strict convex corner charts admit only global supports before separation. One unreduced frame and value scale rank analytic and stationary-rim candidates exactly across the perimeter; the retained feature determines matched corner, edge or face witnesses. | A simpler complete polygon normal-fan traversal preserves exact cross-chart ranking and paired face-generator admission. |
| `TriangleConeContact.KeepAnalytic(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Stable poles, nonduplicated face directions, generator fan and analytic rim boundaries precede stationary roots. | A proven boundary reduction removes measured redundant work. |
| `TriangleConeContact.HasFaceMinimumCertificate(...)` | 12 | 100% line / 100% branch (Release / ReleaseLean) | Cardinal faces reuse an exact contained-segment disk certificate without mutating the winner or witness frame. | A broader certificate preserves full-domain bounds and improves measured general-face cost. |
| `TriangleConeContact.KeepAxis(...)` | 14 | 100% line / 100% branch (Release / ReleaseLean) | Both orientations of a nonzero direction use the same exact admission and stop after proven separation. | Axis representation or feature ownership changes. |
| `TriangleConeContact.KeepDirection(...)` | 22 | 100% line / 100% branch (Release / ReleaseLean) | Triangle support masks and cone apex/rim/generator regions precede exact gap ranking; projected seams defer equality before normalized denominator construction, while compact seams share generator evaluation. | A shared support partition avoids repeated work without weakening admission. |
| `TriangleConeGeneratorFeatures.KeepAll(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Constant azimuth, vertex extrema and both edge-circle roots include double tangencies and zero-radial degeneracies. | The complete normal-circle enumeration can be reduced with a proof. |
| `TriangleConeRimContacts.TryKeepContacts(...)` | 24 | 100% line / 100% branch (Release / ReleaseLean) | Boundary masks exclude artificial edges before chart or negative-root admission. Bounded edge charts preserve one exact analytic/algebraic winner across a shared frame and defer paired materialization until traversal finishes. | A measured nonwinning-root certificate removes chart work without changing complete perimeter ownership. |
| `TriangleConeRimContacts.TryChart(...)` | 30 | 100% line / 100% branch (Release / ReleaseLean) | Unsquared stationary signs and triangle/cone region admission precede separation, zero-depth handling, analytic-bound rejection and exact root ranking. | A simpler exact admission or value-comparison path preserves every boundary. |
| `TriangleConeWitnesses.TryGetGenerator(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | One supported-feature slice admits matched generator parameters before materializing exact endpoint terms and once-rounded interior coordinates. A complete convex fan can decline an earlier triangle without fabricating a witness. | Another contact needs the same slice/materialization contract. |

### Capsule/stadium-slab contacts

The complete capsule/stadium-slab relation retains these focused feature owners:

| Method | Complexity | Coverage | Rationale | Revisit if |
| --- | ---: | --- | --- | --- |
| `WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(...)` | 52 | 100% line / 100% branch (Release / ReleaseLean) | One ordered traversal owns analytic normal planes, exact whole-shape early certificates, degenerate reductions and curved-feature dispatch. Splitting its candidate state into separate solvers would obscure completeness and tie order. | A proved complete reduction removes features or a measured ownership change lowers cost without duplicate state. |
| `WideConvexPrismRelations.BuildCapsuleSlabEndpointRim(...)` | 16 | 100% line / 100% branch (Release / ReleaseLean) | Strict cap, end and capsule support signs certify a global closest-point residual before retaining its exact radical normal and signed gap. | Another exact closest-feature consumer can reuse the same support proof. |
| `WideConvexPrismRelations.TryImproveCapsuleSlabRim(...)` | 24 | 100% line / 100% branch (Release / ReleaseLean) | Stable signed/reciprocal charts retain parameter/value roots, with one exit to existing materialization after a proved negative global minimum. Positive and zero gaps retain complete traversal. | Another whole-shape certificate preserves correctness and earlier ties at lower cost. |
| `WideConvexPrismRelations.KeepCapsuleSlabRimChart(...)` | 34 | 100% line / 100% branch (Release / ReleaseLean) | Unsquared stationary admission, nonwinning-root rejection and exact ranking precede a finite-core closest-point certificate; copied winners retain further parameter refinement. | Measured certificates or root reuse lower cost without weakening admission or stable ties. |
| `CircularRimContactAlgebra.HasSegmentSupport(...)` | 18 | 100% line / 100% branch (Release / ReleaseLean) | Two exact unsquared radical inequalities test finite-segment projection, including equality; opposite signs use one quadratic query in borrowed scratch. Stationarity and negative-gap selection remain caller policy. | Another geometry owner can reuse this neutral support test or width/positivity contracts change. |

## Fixed-Width Workspace Bounds

The internal capsule/stadium-slab relation shares `CylinderContactAlgebra`,
`CircularRimContactAlgebra` and the existing exact value-root owners. Its
normal partition is the three support-sign planes (cap, slab core, capsule
core), their intersections, and strictly admitted endpoint/rim residuals.
Horizontal capsule axes project the disk to a rational segment; only genuinely
oblique capsule-interior normals require quartic charts. Negative core gaps
remain candidates until capsule radius is included in the exact depth test.

Reduced slab-local extents and offsets are below 200 bits, retained rotation
columns below 132 bits, and squared-value shift at most 410. Parameter/value-map
coefficients stay below 1,900 bits in forty-word fields; value polynomials stay
below 2,800 bits in fifty-six-word fields. Projected residual directions remain
in wide spans because their numerators can exceed 320 bits. Root admission,
mapping, comparison and materialization own separate returning scratch frames.
The winner retains independent parameter and value roots, including active
polynomial spans and refined cell metadata, rather than reconstructing its
parameter for normal rounding. Parameter storage reserves 636 words from the
shared stationary coefficient bound below 2,520 bits; stadium parameters remain
below 1,900 bits. The additional retained buffers consume 6,693 bytes, raising
the reviewed outer-buffer bound to 44,601 bytes before structs/control, within
the existing 48 KiB reserve. Stadium value height at most 2,800 bits gives a
319,136-byte Sturm arena; the generic field-capacity arena is not the stadium
bound. Including copied comparison cells, the 48 KiB outer reserve, a live
64 KiB caller and a 16 KiB control/spill margin keeps the reviewed peak below
512 KiB. The 1 MiB worker test supplements this bound with a dirty live caller
and an oblique winner.

Retained-cell sign trials use the same normalized Horner certificate at
`precision<=DenominatorShift-variableShift-2*ceilLog2(degree+1)`. Only a
certified nonzero sign returns early; uncertainty still reaches the original
resultant/equality bound. Crossing classification reuses its endpoint signs
and byte-prediction hints, with exact point evaluation when uncertain and
Sturm fallback for equal signs or zero parent endpoints. Neither adds a
retained cell, root field, arena or overlapping scratch lifetime.

The 128-bit point guard is shared by endpoint, byte-step and midpoint
certificates. The guard preserves exact sign acceptance and root target shifts;
uncertainty falls back to exact evaluation. Relative to a 64-bit guard, this
requires one extra word in each approximate result/product buffer: 16 bytes
per active call. Returning frames do not accumulate this allowance. Including
it gives conservative stadium comparison and mapped-sign budgets of 462,240
and 418,685 bytes,
including caller/control reserves, still below 512 KiB. Exact integer fallback
remains available at every uncertain point, including repeated and dyadic roots.

Shared subdivision adds 64 bytes of pending counts in existing output slots.
Reciprocal pairing adds 2,149 bytes for one reversed polynomial and second
batch, bringing explicit outer buffers to 46,750 bytes inside the 48 KiB
reserve. Each oriented row reverses as `(-1)^i*s^degree(F_i)*F_i(1/s)`;
new right limits preserve independent counts and global repetition when endpoint
coefficients are nonzero. Endpoint-degenerate inputs use existing owners.
Conservatively charging both additions gives **464,453 / 420,898 bytes**.

The negative stationary-rim certificate reuses the dead mapped cell only after
copying its winning value. Seven forty-word slots fit its 706 words; the
parameter copy retains subsequent support-query refinement. With shared chart
components below 198 bits and coordinate/radius components below 232 bits,
the finite-core inequalities have quadratic coefficients below 1,723 bits.
The stadium root/query bounds give `nonzeroBits<=8981`, `targetShift<=10714`
and retained/evaluation capacity at most 168,526 bits, below the existing
1,728,320-bit allowance. The returning arena stays 220,136 bytes; no new wide
buffers raise either peak. Five sign bytes and scalar locals use the control
reserve. Only an admitted, strictly winning negative gap with feasible finite
support terminates traversal, by the convex closest-point proof; equal,
positive and zero cases preserve prior ordering.

Triangle/cone contact shares the triangle circular frame, rim chart/value
algebra and exact supported-feature slice rather than copying a root or wide
arithmetic engine. Relative coordinates and chart directions remain below
198 bits. Rim parameter coefficients fit the existing forty-word slots;
the cone-region constraint is below 528 bits and value polynomials retain the
existing fifty-six-word storage. Generator effective normal heights are below
458 bits, support coefficients below 658 and squared-gap coefficients below
1,320. Endpoint witness numerators stay below 2,040 bits; straddling slices use
their exact midpoint parameter directly, with triangle-coordinate expressions
below 2,176 bits. These bounds fit forty-word quadratic fields.

The cone's largest explicit simultaneous scratch remains below 512 KiB,
including the existing 405,376-byte value Sturm arena and retained cells.
Its separate mapped-value sign path stays below 424 KiB. Analytic generator
and witness frames are not live concurrently with the polynomial arena. A 1 MiB
worker with 64 KiB of live dirty caller storage exercises both an ordinary
interior stationary root and large extents in distinct rational frames;
these checks supplement the full-domain width proof, not replace it.
Horizontal and local X/Z face certificates reuse the existing contained-segment
disk proof and avoid the curved search only when its global lower bound is met.

Triangle/cylinder contact shares the cylinder support algebra, edge stationary
polynomial and squared-value root mapping with box/cylinder contact. Its normal
partition consists of triangle face directions, edge normal-cone arcs and vertex
cone interiors, intersected with cylinder cap poles, side directions and rim
hemispheres. Analytic candidates own the face/pole and side boundaries. Smooth
vertex/rim interiors cannot own a strict positive minimum because meridional
support curvature is zero; their exact residual still detects separation. Edge
arcs retain every admitted stationary quartic root, with unsquared sign, finite
cap and triangle-cone checks. Analytic chart endpoints also cover the root
owner's excluded zero parameter.

After nonnegative pole/face admission, horizontal and vertical faces can also
certify the global minimum without enumerating the remaining features. The
origin's exact projection must lie inside the triangle, and a ball centered at
that projection with radius equal to the face gap must fit inside the cylinder.
That ball supplies the lower bound attained by the selected face. A failed
certificate continues through the complete feature set; a successful one keeps
the already-selected candidate and its canonical tie ownership.

For a positive core parallel to the triangle's face normal, a second exact
certificate uses the stadium's contained normal-axis segment. After both
endpoint regions are tested, the retained depth is no greater than the nearest
whole-stadium face gap. If the triangle contains a face-plane disk of that
radius centered at the origin's projection, subtracting the contained segment
produces a set containing the corresponding origin-centered ball. The retained
support attains this lower bound, so the edge quartics cannot improve it.
Three inward edge clearances prove disk containment with the existing exact
candidate comparator. The parallelism cross product stays below 295 bits;
using the raw core axis crossed with each retained edge keeps clearance
tangents below 231 bits, projection numerators below 465 bits, and squared
denominators below 792 bits. No new polynomial or rounded admission is needed.

Positive-core capsule slabs reuse that partition in each open core-normal
hemisphere. Every candidate and separation certificate is admitted to its
hemisphere before ranking. On the shared seam, the support function is that of
a rectangle in the plane perpendicular to the core. Its cap poles, planar
side normals and triangle-edge crossings complete the partition. One exact
winner spans both endpoint regions and the seam; independent cylinder-query
winners would not describe the stadium. The face certificate also applies on
the seam: the centered cylinder is a subset of the stadium, and an exactly
core-perpendicular face attains the same lower bound.

The positive core adds an exact common `2*Q32` scale to coordinates and
extents, retaining odd half-raw endpoints. Original edges remain unscaled;
`EdgeScale` records their relation to the shifted coordinates. Exact quaternion
norm bounds and orthogonality keep those edges below 196 bits. Shifted
coordinates remain below 232 bits, radius below 227 and raw scale below 164.
Parameter polynomials and value-map numerators remain below 2,534 bits, inside
forty-word fields. Both endpoint regions share a squared-value scale chosen
from their actual coordinate, extent and core-offset bounds; the value quartic
stays below 3,500 bits inside fifty-six words. Cancelling the common edge scale
before forming barycentric minors keeps them below 827 bits in `Signed832`.

Side-feature weighted-coordinate products stay below 2,200 bits. The seam's
quadratic interval blends stay below 1,740 bits, rational cap-radius checks
below 2,300, and combined curved-coordinate queries below 2,520, all inside
forty words. Barycentric weights remain exact until final coordinate rounding,
and radial witnesses scale by the full authored radius before rounding.
Rim-edge witness coordinates retain the selected parameter root as a linear
ratio; their midpoint queries remain below 500 bits in `Signed576`. Analytic
rim witnesses compare the complete rational-plus-radical coordinate through
the existing quadratic sign owner. Neither path projects an already-rounded
radial support point.
At an endpoint, integral radial coordinates retain the exact half-core term.
Other coordinates use exact comparisons against neighboring rounding cells;
the correction is at most one raw unit and tie parity belongs to the combined
coordinate. Thresholds and scaled-radius factors stay below 98 bits. Analytic
products stay below 730 bits, while root comparison coefficients require four
additional words beyond the squared-gradient width. No extra root isolation or
binary rounding search is introduced.
The positive-core contact uses `FiniteAxisValueRoot` isolation and bounded
Horner signs, not the ellipse owner's Hermite path described below. Using full
storage widths (2,560 parameter bits and 3,584 value bits), its largest value
Sturm arena uses 405,376 bytes. Outer contact/chart buffers and bounded
root-comparison cells bring the explicit simultaneous scratch below 512 KiB,
including sign/index arrays. The separate value-mapping endpoint-sign path
stays below 420 KiB: even its conservative 61,248-bit query fits the parameter
arena's precision bound without expansion. Endpoint joint rounding's extra
query words do not raise either peak. These are source-derived scratch bounds,
not total JIT stack measurements; structs, spills, alignment and host frames
require additional headroom.

Resource tests exercise ordinary interior-root and large-extent, distinct
rigid-frame contacts on a 1 MiB thread stack with 64 KiB of live, dirty caller
storage, for both cylinders and positive-core slabs. The large cases use a
shared extreme origin, so they supplement rather than replace the full-domain
translated-coordinate width proof.

Box/cylinder geometry uses the authored rational box frame, an independently
primitive cylinder axis and one reduced coordinate scale. Coordinates remain
below 237 bits under conservative bounds. Analytic candidates and edge-parameter
quartics fit forty-word fields; the scaled squared-value quartic fits fifty-six
words. Polynomial construction returns before root isolation/refinement.
The cone lower bound fits the existing candidate fields and returns before
either chart allocates its root workspace. `FixedOrientedBoxCylinderResourceTests`
checks ordinary algebraic and full-width contacts on a 1 MiB thread stack with
64 KiB of live, dirty caller storage, plus repeatable zero-allocation output.
Exact arithmetic and live-frame proofs remain beside their owning operations;
these stack checks supplement those proofs rather than replacing them.

Cylinder/capsule contact retains authored rational geometry through complete
feature selection and rounds only the winning normal and depth. Analytic
candidates use forty-word scalar slots. Their generated rational directions
remain below `2^440`, dot products below `2^542`, common support below `2^674`
and squared-gap coefficients below `2^1780`. Cross-denominated comparison
coefficients need at most 81/80 words; radical radicands need at most 200 words
plus a carry word. Normal-square coefficients need at most 122 words, with
integer midpoint factors adding at most two words. Arithmetic scratch sizes
follow active operand lengths plus the documented carries, not truncation.

The oblique capsule-interior family retains at most one interior minimum;
principal-axis and degenerate boundaries remain analytic candidates. Ellipse
stationary and signed-gap coefficients stay below `2^1818` and `2^1244`;
squared-gap numerator/denominator coefficients stay below `2^2490` and
`2^2354`, within forty-word slots. Eliminating the analytic candidate's radical
uses 202-word comparison slots. Original scalar depth and normal thresholds
use 43-word slots; paired depth reduction below uses up to 72-word slots.
Preparation's positive common-divisor reduction and exact division
stay in three words with inputs below `2^100`; they reduce integer content
without changing authored cylinder half-lengths or the capsule-normal plane.

The existing ellipse retained-root storage is sized from actual coefficient
bit lengths within the degree-at-most-four root and degree-at-most-eight query
contracts. For root
coefficient height of `B` bits, the dyadic cell reserves at least `9B+192` bits
and Sturm coefficient slots reserve `7B+32` bits. Isolation and later
refinement use the source's separation and Cauchy bounds; the later shift
ceiling is `8B+128`. The 64-refinement fast-path budget is not a tolerance or
termination fallback: unresolved signs and equality proceed to the exact
Hermite query. Positive pseudo-division adds at most `B+1` bits per eliminated
coefficient, with at most eight eliminations.

Sturm variation counting reuses the value-root normalized point certificate
for nonconstant rows strictly inside `(0,1)`. A nonzero certificate proves
the point sign; uncertainty retains exact derivative/right-limit evaluation.
For numerator bits `b` and dyadic shift `q`, work precision is
`p=q+128+d*(q-b+1)`, with `1<=b<=q` and `d<=4`. Acquisition and later
noncrossing refinement together give `q<=max(9B+64,8B+129)`. At `B<=1818`,
the returning certificate uses at most **22,648 bytes** beside the existing
40,000-byte Sturm arena; it returns before exact fallback evaluation. Root
cells, separation bounds and equality decisions remain unchanged.

Analytic normal rounding retains its two threshold-independent products
`(2*scale)^2` times each component-square coefficient once per returning
component frame. Rational/radical norm slots use
`W=max(2R,2L+K,R+L+1)+2<=122`, with each candidate field at most forty words.
Products reserve `P=W+3<=125` words. Two retained products replace one of
the comparator's four scratch products, for a net maximum explicit live
increase of **1,000 bytes**. The doubled nonnegative Fixed64 raw scale fits
ulong even at MaxValue; its square fits two words. Threshold and quadratic
sign decisions retain the original full-width arithmetic and rounding rules.

Analytic depth and scaled-normal rounding also narrow searches using
policy-neutral factored square-root bounds. With radicand bit length `b`,
`k=max(0,ceil((b-192)/2))` and `T=floor(C/2^(2k))`, the existing narrow
integer root `q=floor(sqrt(T))` gives endpoints `q*2^k` and `(q+1)*2^k`.
Only a zero root remainder and entirely zero discarded bits permit collapse.
The endpoints remain at most 97 bits plus `k`; the upper `2^96` carry is
retained. Existing shifted signed-magnitude accumulation bounds each
quadratic expression, reversing endpoints for negative radical coefficients.

For forty-word fields, `k<=1184`. Depth bounds reserve at most **62 words**;
normal bounds reserve `W+6+ceil(k/64)<=147` words, including the invariant
scale products and denominator multiplication by four. The exact clipped
ratio-floor owner requires equal padded magnitudes of at least two words
and a positive denominator; it retains full-width division before its proved
low-128-bit packing. Nonpositive denominator minima retain full normal
searches, while nonpositive numerator minima bound the true square by zero.
Negative-gap depth bounds include the additional fractional raw unit from
radius subtraction. Exact midpoint and MaxValue comparisons still determine
rounding and conceptual clamping.

Shared normal denominator bounds and factored endpoints retain at most
`2*147+4=298` additional words. Component numerator bounds return before the
exact sign comparisons. The maximum analytic-normal exact-sign explicit
buffer peak is **23,176 bytes**, excluding caller frames, alignment and JIT
spills. Prefix/root interval bounds never truncate a full signed result to
fit scratch storage; the upper carry of a forty-word radicand is preserved
without allocating a twenty-one-word shifted root.

The same signed interval Horner owner supplies exact square-root ratio floor
bounds for output materialization. Numerator and denominator retain an equal
padded degree so their homogeneous scales cancel. An uncertain denominator
keeps the complete search range. For interval width `W`, four live bounds plus
the returning division frame consume at most `9W+7` words. Existing ellipse
coefficient and retained-cell bounds give `W<=1071`, or **77,168 bytes**. The
non-inlined bounds owner returns before exact sign queries and paired depth
storage are allocated. Full-width division precedes the proved two-word
quotient packing; the existing integer square-root owner is reused.

Depth rounding reduces the numerator/denominator pair once at the same root,
after obtaining bounds from the original structured pair. Common positive
parameter powers cancel jointly, then every pseudo-step and power-of-two
normalization applies one shared positive scale. The genuinely quartic case
needs one step; a zero ellipse `Q` cancels common `t^2` before one quadratic
step. Coefficients stay below 4,309 bits, requiring 69-word slots under the
sizing rule. The retained pair adds at most **5,530 bytes**, including signs;
each pseudo-step's three product buffers return before sign queries. Its
72-word threshold queries have degree below the defining polynomial, so no
further pseudo-division is needed. Exact binary and half-raw comparisons keep
rounding and conceptual clamping authoritative.

An unresolved linear query has its rational zero inside the retained positive
cell. Homogeneous evaluation at that exact fraction needs
`Bp + degree*max(Bnumerator,Bdenominator) + 3` bits for at most five terms,
using five magnitude buffers. Equality or a crossing-root sign decides the
query directly; noncrossing nonequality retains the general fallback. For
the contact caller, even conservatively allowing eight pseudo-eliminations
before this linear query, those five buffers consume less than 70 KiB and
do not overlap the later Sturm/Hermite scratch lifetime.

Hermite traces and matrix entries also use coefficient-sized bounded spans.
For root/query heights `Bp`/`Bq` and maximum trace order `K`, entries require
at most `Bq+K(Bp+1)+7` bits; determinants of order at most four fit
`4*entryBits+8` bits. A conservative simultaneous-buffer calculation for the
ellipse contact caller (`B <= 1818`, query slots at most 202 words) includes
at most five quartic pseudo-eliminations, retained Sturm storage, weighted
query, trace powers, Hermite matrix/determinants, and outer feature/comparison
buffers: **413,185 bytes** of explicit scratch, including sign/index arrays,
below the existing 448 KiB allowance. The zero-`Q` quadratic case is smaller.
The reduced depth materialization path, including its retained pair, uses
at most **277,719 bytes**; its returning 77,168-byte bounds frame does not
overlap that pair or the sign workspace. This is a
source-derived buffer bound, not a measured total thread-stack maximum; value
structs, alignment, JIT spills and host frames need additional headroom. Use
at least a 1 MiB worker-thread stack with adequate caller headroom, not a
deliberately reduced stack. Ordinary analytic contacts do not construct this
workspace. The geometry, root-selection and width derivations remain beside
their owning source; changing their
degree or authored-input contracts requires re-review, not a generic symbolic
algebra layer or runtime-sized integer fallback.

Cylinder-pair and cylinder/cone strict classification tests all finite cap
disks plus the complete smooth lateral feature set. Circle radial coefficients
remain below 540 bits and axial-strip coefficients below 204 bits. Sturm
transients and homogeneous evaluation at quadratic endpoints fit the fixed
96-word workspace; the final radical sign comparison uses twice that width
plus nine words. Cone-generator expressions remain below 804 bits before
entering the existing 4,096-bit quadratic-field sign workspace. Exact disk
section witnesses need fewer than 1,280 coordinate bits and 2,563 squared
comparison bits, fitting their fixed 48-word workspace. No feature candidate,
algebraic arc boundary or strict overlap decision is rounded to Fixed64.

Capsule/finite-solid strict classification keeps the dual-rigid core chord
rational, including odd raw lengths and 65-bit origin differences. Chord
coordinates remain below 232 bits and rim quartic coefficients below 960 bits.
The degree-at-most-four sign owner admits 1,024-bit coefficients: factorized
Sturm terms need at most `6B+32` bits, exceptional pseudo-remainders at most
`7B+32`, all within its fixed 128-word workspace. Existing rounded-cylinder
callers retain their original 44-word workspace. Cone-side comparisons use one
quadratic field, with exact endpoint/vertex expressions below 1,820 bits and
squared sign comparisons within 4,096 bits. No algebraic root is converted to
Fixed64 to make an overlap decision.

Cylinder/polytope strict classification transforms authored vertices through
the exact relative rigid basis and clips against the two cylinder cap planes.
With normalized quaternion bases below 68 denominator bits each, the relative
denominator is below 136 bits and doubled vertex numerators below 204 bits.
Clipped vertices need fewer than 410 numerator bits over 341 denominator bits;
radial segment differences need fewer than 752 bits and their cross products
fewer than 821 bits. The final squared comparison stays below 1,642 bits,
within the existing fixed 36-word stack workspace. Open axial admission plus a
strict radial margin distinguishes cap-only tangency from intrusion without
rounding an intersection point.

The projected rigid-triangle finite-slab sweep keeps every triangle vertex and
Y-plane clipping intersection rational until the final public witness and
distance conversions. A rigid vertex numerator is below 192 bits over a 126-bit
quaternion-basis denominator. Exact finite-Y clipping raises a point to at most
386 numerator bits over 319 denominator bits. Cross-multiplying two clipped
endpoints therefore needs at most 706 bits, the signed line violation needs at
most 1,094 bits, and the squared half-step comparison needs fewer than 2,252
bits. The implementation consequently uses a fixed 36-word stack workspace
(2,304 bits), with no heap allocation or target-specific arbitrary precision
dependency.

## Review Notes

The native planar capsule relations retain two focused branch-heavy methods:

| Method | Complexity | Coverage | Rationale | Revisit if |
| --- | ---: | --- | --- | --- |
| `WideConvex2dRelations.IntersectsSweptUprightCapsule(...)` | 20 | 100% line / 100% branch | Exact doubled-raw swept core, zero-radius policy and finite edge-distance reduction share one allocation-free O(n) path for closed pose and strict sweep queries. | Another exact relation can share the core without a generic shape framework or rounded endpoints. |
| `WideConvex2dRelations.HasPlanarSweepCoreOverlap(...)` | 30 | 100% line / 100% branch | Polygon halfspaces and the two core edge normals form the complete linear-time SAT proof, including point/segment degeneracy and strict versus closed boundaries. | Supported shape families or input validation policy change. |

- Methods at 100% line and branch coverage with direct component-wise logic
  should usually remain explicit unless a zero-overhead generated approach is
  introduced.
- For fixed-point arithmetic and trigonometric routines, prefer benchmark-backed
  refactors over structural changes made only to reduce the reported complexity
  number.
- If a method remains above the complexity threshold and below full branch
  coverage, prefer adding focused tests for reachable branches before
  refactoring.
- Re-run coverage and CRAP analysis after any changes that touch the methods
  listed above, then update this register.
