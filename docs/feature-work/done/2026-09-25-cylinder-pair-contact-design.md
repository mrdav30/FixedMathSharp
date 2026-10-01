# FMS-Issue-024: Complete finite-cylinder pair contacts

## Status and completion boundary

Design approved for execution on 2026-09-25; geometry repair and coordinated
Gravitas GRV-Issue-084 migration completed on 2026-09-26. Standard/Lean coverage,
consumer validation, matched timings and independent reviews are recorded below.
This closes the incorrect-contact issue, not release validation or a claim that
general nonparallel contacts are cheap enough for high-volume physics. Their
historical 11.57–25.14ms positive-contact cost prompted the
[throughput refinement closed on 2026-10-01](#throughput-refinement-closure-2026-10-01).
The [benchmark backlog archive](../benchmark-signal-hardening-backlog.md#archived-signals)
records the completed diagnostic investigation and recurrence criteria; the
original native-fault and historical allocation-counter causes remain unknown.
The previously accepted zero-radius
trade-off is not blanket acceptance of that much larger general-case cost.

Repair `FixedSegment.TryGetCenteredFiniteCylindersContact` without changing its
public signature. Closed separation/tangency, minimum translation depth,
normal selection, rounding and clamping must describe the exact authored finite
solids. Preserve the separate inexpensive strict-overlap authority.

This is a focused geometry repair, not a general collision framework or public
circle-distance API. Do not stage, commit or publish without explicit
authorization. Validate released dependency packages separately from source mode.

## Evidence, scope and ownership

At FixedMathSharp `6368582`, five enabled regressions produced three failures
and two passes; another 75 strict-classification, rigid-frame and rounding
controls passed. The old reducer evaluated the two axes, their cross and a
closest-centerline direction. Exact arithmetic did not make this set complete:

- Separated rims: height 2/radius 1, first center zero/axis +Y, second center
  `(7/4,7/4,11/8)`/axis +X incorrectly returned contact.
- Rim touch: radius `5/4`, second Z=2 has unique common point `(3/4,1,1)`;
  old depth was `0.09497214644216001`. One raw outward Z step was also admitted.
- Penetrating rims: radius 1, second Z=`5/4` gave raw depth `597275436`, about
  `0.139064`. Direction `(1,1,1)` instead has support gap
  `(2*sqrt(2)-11/4)/sqrt(3)`, about `0.045280`. Independently, the old answer
  exceeds `1/8`, whereas `sqrt(2)<99/70` and `sqrt(3)>5/3` bound this witness
  below `33/700<1/20`. Thus the old positive depth is provably not minimum.

The wrong selected-axis depth was removed from public-contact expectations;
its benchmark geometry remains `PenetratingRimsCylinderCylinder`. Preserve
meaningful arithmetic tests, not dead helpers or incorrect answers.
[FMS-Issue-026](2026-09-24-cylinder-pair-depth-rounding.md) fixed bounded
rounding work, not geometric completeness.

FixedMathSharp owns exact geometry, arithmetic and contact materialization.
Gravitas owns pair ordering, manifolds, materials, anchors and response. Its
ordinary cylinder consumer already used this query; the independently duplicated
cylinder/circle-slab path required the `GRV-Issue-084` migration. Full doubled
raw slab length is carried internally, including `Fixed64.MaxValue` half-width,
rather than overflowing a `Fixed64` length or moving physics policy upstream.

Out of scope: FMS-Issue-025 box/cylinder, GRV-Issue-082 triangle/cylinder,
GRV-Issue-083 saturated circle/circle arithmetic and GRV-Issue-085 mixed
capsule/circle-slab migration. Filled `FixedBoundCircle` disks are not free 3D
rim curves. This audit did not establish a public circle-distance API or certify
every neighboring finite-shape solver. Record further defects in their owners.

## Exact support and feature selection

For exact half-axis displacements `A,B`, center difference `d`, and perpendicular
axis projections `Pa,Pb`, minimize over unit directions:

```text
g(n)=|A.n|+|B.n|+ra*sqrt(n^T Pa n)+rb*sqrt(n^T Pb n)-d.n.
```

A negative minimum separates; zero admits closed contact; a positive minimum
is the minimum separating translation magnitude. Do not add a center-projection
restriction. Preserve first-to-second normals and deterministic representatives
for nonunique minima. Candidate values remain exact until selection.

| Feature | Authority and stable tie order |
| --- | --- |
| Cap normals | First axis, then second axis |
| Common perpendicular | Axis cross, with an explicit parallel reduction |
| Side/rim | First and second side-normal planes, including boundaries |
| Rim/rim | Ascending cap signs, then increasing value-root/branch order |

Zero-radius cylinders dispatch as exact finite segments to the existing complete
cylinder/capsule authority, including both-zero pairs. Reversal swaps anchors
and negates normals while preserving depth, clamping and default false output.
Parallel positive-radius pairs share exact cap/perpendicular comparisons and
rounding; their radial support is zero or the unsigned radius sum. The missing
coincident-core radial candidate is restored, including antiparallel frames.

Side planes reuse the projected ellipse and analytic principal/corner candidates
from [FMS-Issue-023](2026-09-24-cylinder-capsule-contact-design.md), not a
second minimizer. Include the opposite cylinder's constant radius before global
comparison. Consider all four nonparallel cap-pair rim polynomials; the greatest
positive root is not generally the answer. Radial signs and cap inequalities
reject extraneous squared solutions. Repeated values require rank-one recovery.

Round only the selected depth/normal to nearest-even. `DepthIsClamped`
distinguishes conceptual overflow from exact maximum depth. Anchors remain
independent support points for the rounded normal, with origin-relative storage;
they are not promised to be paired algebraic closest-point witnesses. False
returns initialize contact to default; invalid inputs keep their exception contract.

## Shared owners and exact frame

Reuse `WideArithmetic`, caller-owned magnitude spans, exact rational frames,
signed polynomial operations, radical signs and output conversion. No runtime
`BigInteger`, float math, dependency, allocation, shape registry, callback solver
or compatibility facade is introduced. Remove superseded code after migrating
real callers. Keep the optimized degree-four largest-positive-root API separate
from the all-root degree-eight value API; widening its arrays is not a proof.

The inverse first-cylinder frame preserves authored rotations and full-domain
differences. Quaternion denominators `D1,D2<2^65` give common raw scale
`Hscale=2*2^32*D1*D2<2^163`. Reduce common integer content jointly from this
scale, half-axes, center difference and radii; reduce axis/basis directions
independently. Half-raw endpoints survive. Never substitute a rounded transform.

Conservative bounds are `|a|,|u|<2^33`, `|b|<2^163`, `|w|<2^66`, scaled
`|c|<2^229`, radii `<2^226`. Physical squared values are below `2^460`.
Use `S=2^ValueShift*t`, `t in(0,1]`, `ValueShift<=460`, with a tighter
per-query component bound when available. Physical raw squared depth is
`S/Hscale^2`; a generic coefficient-height Cauchy bound is unnecessary here.

## Rim value polynomial and admission

For one cap pair, `c=sA*A+sB*B-d`. Choose rational u perpendicular to axis a
and `w=a cross u` (positive primitive rescaling is harmless):

```text
alpha=a.a, beta=u.u, delta=b.b, p=x*u+y*w, z=(x,y,1)
C=diag(beta,alpha*beta,-ra^2), z^T C z=0
Z=c.c+ra^2+2*c.p, J=b.(c+p), K=delta*Z-J^2
T(p,S)=delta*(Z+rb^2-S)^2-4*rb^2*K
f(lambda,S)=det(T(S)-lambda*C).
```

T is a conic: its spatial block is independent of S, linear entries are affine,
and its constant entry is quadratic. The cubic pencil has constant nonzero
leading coefficient. Tangency repeats a lambda root, so its discriminant
contains every stationary squared distance and has degree eight. In this basis
the leading coefficient is
`16*alpha^2*beta^8*delta^4*rb^4*(alpha*delta-(a.b)^2)^2>0`.
Parallel circles are excluded: persistent complex intersections at infinity
can make their discriminant identically zero.

Construct scalar invariants, not a general determinant. For independently
reduced u,w, write `g=u.u`, `h=w.w`, `rho=ra²`, `tau=rb²`,
`ci=c.(u,w)`, `bi=b.(u,w)`, `j=b.c`, `k=c.c+rho-tau-S`:

```text
e=(c0*b1-c1*b0)^2
K=delta*(g*c1^2+h*c0^2)+tau*(g*b1^2+h*b0^2)
L=g*(2*j*c1-k*b1)^2+h*(2*j*c0-k*b0)^2
d=delta*(k^2-4*tau*S)+4*tau*j^2
A=g*h*rho; B=g*h*d-4*rho*K
C1=4*delta*tau*(4*rho*e+4*S*K-L)
D=-64*delta^2*tau^2*e*S
P=B^2-3*A*C1; R=C1^2-3*B*D; N=9*A*D-B*C1
W=4*P*R-N^2 = 3*discriminant.
```

D is linear, not quadratic. One product schedule optionally differentiates in
a geometry direction: value construction uses 73 coefficient slots; one
derivative adds 72. Independent integer determinants/discriminants certify
values and derivatives. Keep raw W_S and geometry derivatives on the same
scale; input-specific content normalization must not change envelope ratios.

Matrix row weights `(33,66,229)` bound `S^j` coefficients of Tij by
`2^(790+weight_i+weight_j-458*j)`. Tighter schedule heights, including W=3V,
are `(650,1445,2227,3004)` for A,B,C1,D and `(2893,4457,3675)` for P,R,N.
W is below 7356 bits; after physical scaling, below 7372. Cap-direction
derivatives are below 7376; world derivatives replace a 229-bit direction by a
65-bit rotation numerator and are below 7212. All fit 116 words.

For a nontriple repeated pencil root,
`lambda=(9*A*D-B*C1)/(2*(B²-3*A*C1))`. A rank-two kernel supplies p;
determinant differentiation certifies its circle constraint. If `H=Z+rb²-S`
is nonzero, recover `q=-2*rb²*Pb(c+p)/H`, `v=c+p+q`. The conic proves
`q.q=rb²`, `v.v=S`. For `n=sigma*v/sqrt(S)`, require positive
`sigma*(p.v),sigma*(q.v)` and nonnegative
`sA*sigma*(A.v),sB*sigma*(B.v)`. The signed gap is `sigma*sqrt(S)`;
negative admitted gaps separate. Radial equality belongs to cap boundaries.

At a simple value root use `v_j=-V_cj/(2*V_S)`,
`p.v=-ra*V_ra/(2*V_S)` and the second-radius analogue. The known crossing
sign supplies V_S's sign without a derivative-zero proof. Retain each cap
pair's four admission polynomials across its roots, then release them and the
selection frame before final normal materialization.

## Side values and known-root mapping

For projected cylinder axis a, side axis b, cap offset c and squared radii
rho,tau, let `U=a²`, `B=b²`, `C=a.b`, `q=c²`, `x=a.c`, `y=b.c`, `H=UB`.
Cancel principal-frame denominators before constructing the squared signed
unshifted-gap quartic:

```text
l=H*z+rho*(H+C²)-H*q+U*y²
m=-(H+C²)*z-rho*C²+C²*q+B*x²-2*C*x*y
v=C²*z
F=m²*l²+4*H*rho*m³-4*v*l³-18*H*rho*v*m*l-27*H²*rho²*v².
```

Its leading coefficient is `H²(H-C²)²>0`. Eliminate z against
`z²-2*(S+tau)*z+(S-tau)²` for the full-radius octic. Quartic weighted height
is below 3417; scaling/conjugate expansion gives octic height below 6871.
Construction uses 56 scratch coefficient slots and shared polynomial arithmetic.
Independent conic determinants and Sylvester resultants certify it.

Retain the existing ellipse parameter for candidate identity; do not minimize
again. Smooth stationary values satisfy F(h²)=0. A winning principal minor has
zero major-center projection and is stationary. For perpendicular axes,
`F=m²(l²+4H*rho*m)` contains the line value and both endpoint distances.
The major winner belongs to the earlier analytic family. Handle h=0 by the
known positive linear radius-square root.

Selected nonzero h² cannot be F's greatest physical root. At a winning smooth
minimum, ellipse distance satisfies `D'=2*rho_curvature*h'` with positive
curvature radius; a strictly farther stationary maximum exists. A principal
minor with semiaxes A>B and offset q has `B+q<A`, hence
`h''=A²/B-B-q>0`. A perpendicular line value has a farther endpoint; an
admitted corner requires `|c_major|>A`, making its opposite endpoint strictly
farther. These values also obey the physical scaling bound. Thus Count>=2,
but h² need not be the smallest root: an inadmissible line root can precede it.

Known membership permits ascending identification among ordinals 0..Count−2:
reject earlier candidates exactly, then choose the final eligible root by
exclusion. Current-root<=selected-value makes the open lower-endpoint check
redundant; only its upper endpoint can reject. Rational cells require equality.

Map alpha=h²/2^ValueShift to
`beta=(sqrt(alpha)+sign(h)*sqrt(tau/2^ValueShift))²`. The full gap is positive;
negative h implies alpha<tau/2^ValueShift. The conjugate identity proves beta
is a positive target root: this is known-member mapping, not an absence test.
Strict mapped-cell containment certifies ordinary cases; otherwise refine exact
endpoints with one-radical comparisons. Clip negative-branch intervals crossing
the minimum at zero. `alpha>2^(-Bf-2)` and derivative bound `<2^(Bf+4)` make
source shift `q+Bf+8`, target shift `q=16*(Bg+11)+100` sufficient for certified
overlap. Reject Count−1 candidates and select the final target by exclusion.

## Exceptional families and completeness

A rank-one pencil supplies a kernel line; intersect it with the first circle
and retain zero, one or two real points. Its finite value degree is at most two:
the spatial block fixes quadratic lambda, and compatibility is linear in S.
If that coefficient vanishes, lambda is rational and the remaining rank
condition is quadratic in S. The circle adds one quadratic extension. Keep
denominators explicit: rationalizing H can divide by zero when only its conjugate
vanishes. Dependent projections use the rational-eigenvalue branch, including
its zero eigenvalue, not division by a projected dot product.

Final spatial compatibility is a consequence, not extra admission. For rank-one
`[[a,b],[b,c]]`, `ac=b²`, solving `b*l0-a*l1=0` with a!=0 implies the other
equation (symmetrically for c). If the compatibility slope vanishes, both
coefficients already vanish. The mixed-independent branch's eigenvector
relation and explicit S give the same identity. These checks are debug-only;
the remaining diagonal minor is a genuine rank condition and still rejects.

The discriminant cannot vanish identically in the nonparallel positive-radius
domain: as S grows, finite generalized eigenvalues tend to zero and
`4*tau*(b0²/g+b1²/h)>0`, while the third diverges. A spatial rank-zero block
with nonzero linear entries has only infinite kernels. Finite constant-distance
families need no arbitrary-point solver: admitted arcs reach an earlier cap-axis
normal without crossing a cap-sign boundary, or lie on a side plane. Example:
`a=(-3,0,4),b=(0,0,1),ra=rb=5,c=(0,3,0)` has distance 3. In nonorthogonal
families radii agree, c is perpendicular to both axes and `|c|=r*sin(angle)`;
orthogonal continua belong to a side plane.

At v=0, nonparallel rim tangents supply their cross normal. Parallel tangents
give a planar cone of radial/cap inequalities; any nontrivial cone has an active
cap/side boundary already enumerated. Never normalize the zero residual.

### Stationary families that cannot improve the minimum

Independent proofs exclude H=0 and finite rank-two triple pencils from improving
earlier cap/side minima; their recovery implementations/helper-only tests were
removed. Public witnesses remain. Rank-one tangent-line cases, including
discriminant-zero cases, remain necessary.

For H=0, use unit axes a,b with actual half-lengths. Write `c+p=T*b`,
`v=T*b+q`, `g=|v|>0`, `n=v/g`; then `g²=T²+rb²`. The spherical support
Hessian determinant is `-g*(ra/|Pa*n|)*(ta.tb)²`. An interior minimum requires
`a.b=(a.n)*(b.n)`. First-cap direction `u=sign(a.n)*a` selects the same
second cap and has gap `T*(u.b)+rb*sqrt(1-(u.b)²)<=g`, strictly smaller under
nonzero radial admission. If T=0 or a cap boundary is active, a side minimum
is no greater. Hence H=0 cannot improve the selected minimum.

For a rank-two triple, translate orthonormal first-circle coordinates to its
finite kernel: `C=2*r*u+u²+v²`, `Q=a*u²+2*b*u*v+c*v²`.
`det(Q+mu*C)=-r²*mu²*(c+mu)` forces c=0; rank two forces b!=0. On the
circle `Q=-(b/r)*v³+O(v⁴)` changes sign. For H!=0, T factors into the actual
unsquared branch's squared-distance difference and a locally nonzero factor,
so this is not a squaring artifact. At a positive support minimum,
`Hess(g)=g*(J*D^-1*J^T-I)` with tangent columns J and positive radial products
D forces independent tangents. Nearby rim pairs share continuous normals with
positive radial signs and are genuine support points; their distances cannot
cross below the minimum. This contradicts the cubic crossing. Nonzero cap
boundaries are side planes; zero half-lengths impose no restriction. Invertible
coordinates cover nonunit axes. Genuine rank-two triples occur at H=0 too:
that exception uses the separate H=0 proof, not this factorization.

### Independent repeated-root minimum certificate

Rank-one recovery cannot be removed. Use radii 1, lengths 10, centers zero and
`(0,1,8)`, axes `a=(3,0,4)/5`, `b=(-3,0,4)/5`. Author quaternions
`(0,-s,0,2s)`/`(0,s,0,2s)`, s raw `1920767767`, local axes +X/−X.
The unique normal is `(0,15,sqrt(399))/(4*sqrt(39))`, depth
`3*sqrt(39)/20`, squared depth `351/400`; the old reducer gave about 0.96.

For caps++, `c=(0,-1,0)` and smooth support is
`F(n)=sqrt(1-(a.n)²)+sqrt(1-(b.n)²)-n_y`. Full support adds nonnegative
`|A.n|-A.n+|B.n|-B.n`. For `y=n_y>=0`, minimizing radial terms gives:

```text
L(y)=(24/25)*(1+y)              for 0<=y<=9/16
     (2/5)*sqrt(9+16*y²)       for 9/16<=y<=1.
```

Initially `n_x²=(1+y)*(9-16*y)/25`, `n_z²=(1-y)*(16+9*y)/25`; above that
interval the minimum has n_x=0. The other squared stationary solution has a
negative radial-length product. L(y)−y decreases initially and reaches its
unique second-interval minimum at y²=75/208. The two smooth normals differ
in Z sign; only positive Z admits both caps. This certifies the full-solid minimum.

In basis `(4,0,-3)/5,(0,1,0)`, `C=diag(1,1,-1)`, `Txx=2304/625`,
`Tyy=4`, `Tyz=2(S-1)`, `Tzz=S²-6S+1`. At S=351/400, lambda=2304/625,
the pencil is `(196/625)*(0,1,-25/32)*(0,1,-25/32)^T`. With k=2304/625:

```text
discriminant=[(3+6*S-S²)²-64*S]*[-k*(S-351/400)*(S-176/225)]².
```

Public tests independently certify nearest-even raw depth/normal using integer
midpoint inequalities, unique-minimum swapping, unclamped depth and origins.

## Exact root protocols and arithmetic

All-root value isolation uses signed subresultant Sturm chains on `(0,1]`;
right-limit variations count distinct roots, including repetition and 1.
Nonrational cells have positive lower numerators and strictly nonroot endpoints.
The separate largest-positive quartic retains its contract. Both share compact
two-accumulator dyadic evaluation and existing limb arithmetic.

Raw Brown subresultants replace per-remainder content GCDs. Initial rows and
exported GCDs use positive primitive normalization. Construction retains raw
signs; Sturm flips obey
`t_next=-t_previous*sign(beta)*sign(lc_current)^(degreeGap+1)`.
Abnormal drops retain every pseudo-leading multiplication. Identities were
cross-checked against the [primary Brown implementation](https://github.com/sympy/sympy/blob/master/sympy/polys/euclidtools.py),
not copied into a second polynomial framework.

Store raw rows as `2^a*T`. For `beta=2^b*betaOdd` and t pseudo-leading powers,
compute `U=prem(Tleft,Tcurrent)/betaOdd` exactly. Strip U's common `2^u`
before packing; next valuation is `a_left+t*a_current-b+u`. The intermediate
exponent before u may be negative; only the stripped row is guaranteed no wider
than its raw counterpart. Propagate scalar valuations similarly. Positive
powers of two preserve variations; metadata is nine row/two scalar integers.

Construction is bounded by `660*(Bf+64)` bits, retained rows by `221*(Bf+64)`.
The arena remains `880*(Bf+64)` bits plus 4096 bytes, with deeper refinement
separately budgeted. Compact padded coefficients before retention. Portable
128/64 division uses two base-2^32 digits (normalized
[Algorithm D](https://github.com/llvm/llvm-project/blob/main/compiler-rt/lib/builtins/udivmodti4.c));
magnitude multiplication uses one row carry and skips zero right limbs.
Independent BigInteger tests characterize dense/sparse/truncated arithmetic.

Optional Bernstein isolation seeks degree-preserving square-freeness modulo
65537. GCD1 certifies global square-freeness; inconclusive images fall back.
Integer `b_j=n!*sum(i<=j,a_i*C(j,i)/C(n,i))` have height <=B+20. Exact
midpoint de Casteljau adds at most n bits/level. Depth 32/node 256 limits,
zero endpoints/midpoints or unresolved cells use unchanged Sturm isolation.
Variation0 proves no roots; variation1 with positive lower endpoint proves one.
The isolated helper uses at most 296,208 bytes at B=7424 plus <1KiB metadata,
released before Sturm. These budgets affect cost, never geometric answers.

Batch isolation shares one chain. `FiniteAxisValueRoots` retains eight compact
cells/shifts (544 bytes), total distinct Count and global repeated-root metadata.
Count stays authoritative on capacity failure, but all partial cells are
discarded. Its scoped materializer copies into cleared caller storage or uses
the same valid-ordinal core with full storage. Empty sets are not materialized.
Returned roots borrow explicit polynomial/cell spans, never the compact cache.
A 600-bit cluster and partial-prefix failure characterize exact fallback.

Repeated roots use `G/gcd(G,G')`, `G=gcd(F,F')`, of degree <=4 and height
<=`ceil(Bf/2)+5`; unreduced G can have degree7. GCD phases are sequential.
Last Sturm-row degree detects global repetition, even outside `(0,1]`, without
unnecessary extraction for square-free polynomials.

### Finite sign/equality certificates and refinement reuse

For alpha in `(0,1]`, defining degree/height n,Bf, query degree/height m,Bq:

```text
L=(n-1)*(Bq+ceil(log2(m+1)))+m*(Bf+ceil(log2(n+1)))
Q(alpha)!=0 => |Q(alpha)|>2^-L.
```

The integer resultant with alpha's primitive minimal factor and its Mahler
measure prove this without constructing that factor. Normalize Q by `2^Bq`:
Horner coefficient/product truncations and cell error total <`2*(m+1)` units.
Refine to shift `P+2*ceil(log2(m+1))`; at
`P=L+Bq+ceil(log2(m+1))+4`, unresolved sign certifies exact zero. Earlier
small precisions accelerate separated signs without changing the final bound.
Rational roots evaluate exactly; no tolerance or endless refinement is used.

Distinct roots of degree-eight integer polynomials, including factors, have
separation >`2^(-16*(Bf+11)-89)`. Unequal roots of two defining polynomials
have resultant/Mahler bound `2^(-8*Bf-8*Bg-89)`; implementation adds slack.
Sufficiently refined overlapping cells certify equality despite different
polynomials/multiplicities. No query-GCD or degree16 Sturm chain is added.

The preserving sign API leaves caller cells unchanged. Mutable queries retain
numerator, shift and rational flag together only when both endpoints fit.
An all-ones nonrational numerator needs room for its upper carry; otherwise
refinement stays local. Tiny roots may legitimately fit despite large shifts.

Odd crossings reuse exact bisection; even multiplicities retain Sturm. Normalized
Horner certificates precede exact point evaluation; uncertainty always falls
back. For shift q and positive numerator N,
`k=q-bitLength(N)+1<=min(q,B+3)` bounds the small-value budget. Degree>=2
scratch is comparable to exact Horner; degree1 has a separate conservative
57,760-byte bound, including degree18 query controls.

Byte refinement uses leading32-bit values only to predict a child; both opposite
endpoint signs must certify it. Exact dyadics strip trailing zeros to preserve
minimal denominators. Failure restores the whole parent numerator and disables
prediction for that call. Each iteration returns or advances 1/8 bits; hints
add only scalars. Normal thresholds reuse component squares with exact signed
differences/canonical zeros. Depth rounding refines once to the common grid
`ValueShift+2<=462`; subsequent thresholds use retained-cell comparisons.

## Peak storage and public resource gates

Construction returns before evaluation; batching returns before admission;
selection returns before normal rounding. Repeated-factor phases, side F
isolation, old-quartic membership and mapped-endpoint comparisons have separate
lifetimes. Retain only the winning full cell across selection/materialization.

At Bf=7416, the conservative degree16/Bq=14932 numerical phase including its
inputs is 911,744 bytes, leaving 136,832 bytes of a 1MiB stack for containing
frames. This is a phase bound, not permission to ignore callers. W=3V and
derivative heights must remain included in each actual schedule.

Rank-one recovery uses weighted pairs `x+y*sqrt(E)`, not repeated monic-field
denominators. Bounds: E<=4234 bits, line3008, D6208, Delta6720, U6528,
H numerator8448, world direction15424, cap dots15584<244*64. Its 64-pair
arena is 249,856 bytes. Degree<=4 matching uses quadratic queries below4400
bits, keeping that phase below720KiB. Separate rounding uses133,280 bytes
plus <128KiB radical-sign scratch. Side F shifts are at most55065; old ellipse
comparison queries stay below58500 bits and their quartic/Hermite phase below700KiB.

Validate public contacts, not only helpers, on 1MiB workers with 64KiB of live
dirty caller storage read after return: ordinary, skew/full-domain and repeated
cases. A primitive 7200-bit polynomial separately stresses numerical height,
not merely common coefficient content. Tests complement the proof; they do not
authorize higher stack requirements or allocation. Final standard and Lean
public-stack and allocation tests pass.

## Acceptance ledger and reproduction

Completed evidence, with revision boundaries:

- Frozen fixtures/baselines precede runtime changes; independent minimum and
  raw-boundary tests replaced incorrect selected-axis expectations.
- Zero-radius dispatch passed focused controls and both framework builds;
  its ordinary-case cost trade-off below was explicitly accepted.
- Final FixedMathSharp source validation uses the working tree based on
  `ad9c88b`; Gravitas uses the coordinated working tree based on `97fee61`.
  These are local-source results, not released-package validation.
- Full core Release: 3638 passed, no failures/skips; 50775/50775 lines,
  10828/10828 branches and 3723/3723 method records covered. Full ReleaseLean:
  3617 passed, no failures/skips; 50868/50868 lines, 10828/10828 branches and
  3719/3719 method records covered. Reports: `artifacts/fms024-final-release`
  and `artifacts/fms024-final-lean`. No new exclusions or skipped tests.
- The subsequently added frozen penetrating-rim 64-call allocation regression
  passes separately in both configurations with exactly zero calling-thread
  bytes (`artifacts/fms024-allocation-Release` and `-ReleaseLean`). Runtime
  source is unchanged from the complete coverage runs.
- Both solution configurations build netstandard2.1/net8.0 with zero warnings
  or errors; both Chronicler extension suites pass all 49 tests. DocFX with
  warnings as errors passes. The full suites include the public 1MiB worker /
  live 64KiB caller-buffer gates and the 7200-bit primitive-polynomial stress.
- Gravitas Release: 4287 passed, one existing GRV-Issue-082 failure, no skips;
  all 14 cylinder/circle-slab cases pass. Coverage is 44639/44639 lines,
  13262/13262 branches and 4569/4569 method records. ReleaseLean: 4228 passed,
  the same one existing failure, no skips; all 14 mixed cases pass. Coverage is
  44637/44637 lines, 13262/13262 branches and 4568/4568 method records.
  Reports: Gravitas `artifacts/grv084-final-Release` and `-ReleaseLean`.
  Both configurations build both target frameworks with zero warnings/errors.
  The unrelated triangle/cylinder regression stays enabled; neither complete
  Gravitas suite is represented as green.
- Independent correctness/resource and Ponytail reviews found no remaining
  actionable findings. The final borrowed-span rounding extraction was reviewed
  separately; exact midpoint/neighbour and input-immutability tests pass.

Completion gates:

- [x] Final Release/ReleaseLean netstandard2.1/net8.0 builds, core/Chronicler
  suites and DocFX with warnings as errors.
- [x] Final core/Gravitas matrices: 100% reachable line, branch and method
  coverage, no new exclusions/skips. Report unrelated GRV-Issue-082 failures
  explicitly rather than claiming a green complete consumer suite.
- [x] Final warmed allocation and containing public stack gates.
- [x] Matched out-of-process benchmarks for repaired and unchanged workloads,
  including shared cylinder/capsule controls.
- [x] Final independent correctness/Ponytail review and resolved findings;
  preserve expensive general-case performance as a measured limitation, with
  subsequent routine refinement recorded below.

Cover exact touch/raw neighbors, positive sub-raw depth, nearest-even ties,
clamping, coincident/near-parallel axes, zero-radius reductions, exact frames,
full-domain translations and unique-minimum swapping. Rounded rotations are
not exact test isometries; production helpers are not independent oracles.

Serialize heavy work: one tree, at most two cores, below-normal launcher.
Read-only agents start no workloads. Set `UseLocalLsfStack=true`,
`BuildInParallel=false`, `DOTNET_PROCESSOR_COUNT=2` in the process environment:

```text
dotnet build FixedMathSharp.slnx -c Release -p:UseSharedCompilation=false -m:1
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --no-build --collect:"XPlat Code Coverage" --settings tests/FixedMathSharp.Tests/coverlet.runsettings --results-directory artifacts/fms024-coverage -- RunConfiguration.MaxCpuCount=1 xUnit.MaxParallelThreads=1
dotnet test tests/FixedMathSharp.Chronicler.Tests/FixedMathSharp.Chronicler.Tests.csproj -c Release --no-build -- RunConfiguration.MaxCpuCount=1 xUnit.MaxParallelThreads=1
```

Repeat ReleaseLean with distinct artifacts; any nonzero test exit remains red.
Existing projects suffice: no new diagnostic framework or CI performance gate.
Ignored artifacts support, not replace, this portable acceptance record.

## Frozen baselines and performance acceptance

Frozen zero-radius captures: Windows11/i7-9700K, SDK10.0.302, runtime8.0.29,
BenchmarkDotNet0.15.8, two-core affinity, below-normal launcher, two launches,
three warmups/twelve measurements. Means are us with 99.9% confidence
half-width; all rows allocated zero and launchers exited zero.

| Unchanged geometry | Before dispatch | After dispatch |
| --- | ---: | ---: |
| Penetrating rims | 357.780 +/- 5.341 | 358.924 +/- 5.253 |
| Separated rims | 363.220 +/- 5.715 | 360.420 +/- 7.346 |
| Tangent rims | 360.632 +/- 5.478 | 360.201 +/- 6.600 |
| Zero-radius parallel segment | 29.439 +/- 0.387 | 31.170 +/- 0.444 |
| Ordinary strict predicate | 0.899 +/- 0.012 | 0.905 +/- 0.013 |
| Ordinary contact | 25.825 +/- 0.487 | 25.940 +/- 0.380 |
| Multi-radical contact | 498.860 +/- 5.877 | 501.238 +/- 8.918 |
| Full-domain cancellation | 353.676 +/- 3.270 | 357.171 +/- 4.975 |

The first three rows still returned incorrect answers: these are workload-cost
baselines, not equivalent-result comparisons. Bimodal distributions make
sub-percent differences inconclusive. A repeated matched capture confirmed
about 31.3us for zero-radius pairs; removing the unused minor-axis calculation
did not measurably alter it or the shared controls:

| Shared control | Before cleanup | After cleanup |
| --- | ---: | ---: |
| Ordinary cylinder/capsule | 21.31 +/- 0.265 | 21.30 +/- 0.274 |
| Zero-radius cylinder pair | 31.31 +/- 0.437 | 31.26 +/- 0.398 |
| Oblique interior rim | 600.88 +/- 10.344 | 597.95 +/- 9.237 |
| Intersecting capsule core | 461.19 +/- 6.871 | 463.19 +/- 7.055 |

Retain segment correctness with the accepted ordinary cost of about 1.8us/6%
above the old path. Do not call this a speed improvement or add a second solver
to recover that small absolute cost.

Complete-solver profiles identified redundant normal refinement, per-row GCDs,
per-ordinal chains and unnecessary worst-case mapping refinement. The bounded
reuse/certificates above address those measured costs without relaxing geometry.
The final matched capture uses the same machine/runtime, affinity, priority,
two launches, three warmups and twelve measurements as the frozen baseline.
All 22 child processes exited zero; the launcher exited zero. Results are us
with 99.9% confidence half-width:

| Workload | Previous path / shared control | Complete solver |
| --- | ---: | ---: |
| Penetrating rims | 358.924 +/- 5.253 | 11566.091 +/- 109.827 |
| Separated rims | 360.420 +/- 7.346 | 5409.430 +/- 25.290 |
| Tangent rims | 360.201 +/- 6.600 | 5141.238 +/- 125.680 |
| Zero-radius parallel segment | 31.170 +/- 0.444 | 27.624 +/- 0.639 |
| Ordinary strict predicate | 0.905 +/- 0.013 | 0.856 +/- 0.008 |
| Ordinary parallel contact | 25.940 +/- 0.380 | 19.448 +/- 0.098 |
| Multi-radical contact | 501.238 +/- 8.918 | 25137.112 +/- 164.294 |
| Full-domain cancellation | 357.171 +/- 4.975 | 49.338 +/- 0.494 |
| Ordinary cylinder/capsule | 21.30 +/- 0.274 | 19.645 +/- 0.103 |
| Oblique interior capsule rim | 597.95 +/- 9.237 | 516.137 +/- 6.623 |
| Intersecting capsule core | 463.19 +/- 7.055 | 414.585 +/- 3.346 |

Artifacts: `artifacts/fms024-matched-final`, log suffix `20260926-055725`.
The first three previous answers were geometrically wrong; speed ratios are
workload costs, not equivalent-correctness comparisons. Tangent and zero-radius
distributions were bimodal. Parallel/shared paths improved, but the complete
nonparallel solver is substantially more expensive. Retain the correctness
repair and measured limitation together; do not declare a high-volume workload
budget met. The subsequent throughput closure below preserves the completed
refinement and the profiling evidence required to reopen it.

The report summary shows zero bytes for every row, but penetrating-rim launch 1
reported 12,336 process-wide bytes across 64 calls while launch 2 reported zero.
Every other launch reported zero. The summary alone therefore does not establish
zero process-wide allocation for every launch. Direct warmed calling-thread
guards pass; the bounded counter investigation below preserves the discrepancy.

Investigate repeatable regressions above 5%, reporting absolute cost and error
as well as percentages. Correctness is not traded for the old fast wrong answer.
Build Release benchmarks under the same process settings and run:

```text
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll rigid-finite-shape-relation --filter '*CylinderCylinder*' '*OrdinaryCylinderCapsule' '*InteriorObliqueRimCylinderCapsule*' '*CoreInsideCylinderCapsule*' --warmupCount 3 --iterationCount 12 --launchCount 2 --keepFiles --exporters json --artifacts artifacts/fms024-matched-final
```

Use distinct artifacts per revision. Shared filters:
`'*ZeroRadiusCylinderCylinder*' '*OrdinaryCylinderCapsule'
'*InteriorObliqueRimCylinderCapsule*' '*CoreInsideCylinderCapsule*'`.
Baseline was `038d2b8` plus fixture corrections, before runtime changes. Closure
must identify the tested revision, complete matrix, matched captures, resource
results and resolved independent review findings.

### Bounded benchmark crash and counter investigation (2026-09-26)

The original `artifacts/fms024-final-performance` capture is excluded from final
performance evidence: MultiRadical launch 2 (PID23480) exited `-1073741819`
(`0xC0000005`) after its environment header, before workload results. No fault
stack was captured. Penetrating launch 1 reported 0 allocated bytes; launch 2
reported 12,336 bytes across 64 operations (193 B/op rounded), with no collections.

The process-scoped CDB rerun (`artifacts/fms024-diagnostics/cdb-benchmark.log`,
BDN output `artifacts/fms024-diagnostic-rerun`) completed all four children with
exit 0 and no unhandled-AV dump. Penetrating again reported 0 then 12,336 bytes;
both MultiRadical launches reported 0. This did not reproduce or explain the crash.

BDN 0.15.8 measures an extra workload pass using the process-wide precise total,
not the current-thread counter ([GcStats](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Engines/GcStats.cs),
[Engine](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Engines/Engine.cs)); background managed allocations can contribute.
In .NET 8.0.29, both [current-thread](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/vm/comutilnative.cpp#L936-L946)
and [precise-total](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/vm/comutilnative.cpp#L1012-L1034) counters subtract unused allocation-context space.
Background GC [clears those pointers without the matching counter adjustment](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/gc/gc.cpp#L7946-L7956)
through [allocation-context repair](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/gc/gc.cpp#L38628-L38630), unlike the [foreground adjustment](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/gc/gc.cpp#L7920-L7927).
Thus both counters are structurally susceptible to an accounting discontinuity;
this is not evidence that it caused these 12,336 bytes. Threading module loads
also occurred in zero-byte children and do not identify an allocator or cause.

The four-child `--envVars DOTNET_gcConcurrent:0` rerun
(`artifacts/fms024-blocking-gc-verified`) reported zero bytes, but was **not** a
blocking-GC control: BDN generated `System.GC.Concurrent=true`, overriding that
child environment variable. Its child headers still reported concurrent GC.
[BDN derives the default from the launcher](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Environments/GcResolver.cs#L10-L18);
[the runtime honors the explicit true setting first](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/vm/eeconfig.cpp#L353-L369).

The corrected control sets `DOTNET_gcConcurrent=0` in the **fresh launcher's**
process environment before running the same two workloads with two launches,
two warmups and three measurements, retaining generated files. All four child
headers report `GC=Non-concurrent Workstation`; the retained runtimeconfig has
`System.GC.Concurrent=false`. All four children and the launcher exited zero,
and each child reported zero allocated bytes (`artifacts/fms024-blocking-gc-control`,
log suffix `20260926-060942`). No committed GC setting or global machine setting
was changed. This is compatible with a GC-sensitive counter artifact, but the
concurrent zero-byte rerun also shows intermittency: it does **not** prove the
12,336-byte attribution. Keep the direct warmed zero-allocation regression and
raw per-launch records; no tolerance was added to hide the discrepancy.

An initial attempt used unsupported `--gcConcurrent false`; it produced no
benchmark results and is not counted as a control. It exposed the runner
treating an empty parse-error result as successful informational output.
The runner now validates through BDN's own parser, permits empty results only
for validated informational requests, and retains every child-exit check.
Temporary launcher probes passed all 25 cases, including malformed
alias/all/direct arguments, help/version/list combinations, no-match filters,
valid informational commands and runtime/validation/partial/late-exit failures.
Artifacts: `artifacts/benchmark-exit-codes/LauncherProbe_2fc0441ace0648a599f84b1189b591b0`.
Both real benchmark configurations build without warnings/errors; independent
correctness/Ponytail review found no actionable runner findings. No additional
benchmark project, CI gate or custom argument grammar was introduced.

For a recurrence, create the ignored capture directory and run installed x64 CDB
from the repo root with `-o -G -hd -logo <log> -cf <commands>` before the usual
dotnet benchmark command, adding `--keepFiles` to retain generated binaries/PDBs.
The command file below captures only an unhandled AV, without global registry changes
([CDB options](https://learn.microsoft.com/en-us/windows-hardware/drivers/debugger/cdb-command-line-options), [exception commands](https://learn.microsoft.com/en-us/windows-hardware/drivers/debuggercmds/sx--sxd--sxe--sxi--sxn--sxr--sx---set-exceptions-)):

```text
sxi ibp
sxd -c "" -c2 ".echo FMS024_UNHANDLED_AV; .dump /ma /u artifacts/fms024-diagnostics/unhandled-av.dmp; .exr -1; r; k; gn" av
g
```

Keep the serialized two-core/below-normal launcher; debugger timings are not
performance acceptance. Check child exits and complete results even without a dump.

### Benchmark diagnostic hardening (2026-10-01)

Investigation used `678818a` plus the benchmark reporting change below, Windows
11 x64, SDK 10.0.302, .NET 8.0.29 and BDN 0.15.8. Every build and child build
used `UseLocalLsfStack=true`; workloads remained serialized, below normal
priority and on two cores. Runtime math, JIT/GC settings and exact allocation
guards were unchanged. This record resolves the reporting gap and identifies a
fresh counter mechanism; it does not establish a native fix or retroactively
attribute the historical 12,336-byte observation.

**Why the zero summary concealed positive launches.** BDN's
[report constructor](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Reports/BenchmarkReport.cs)
selects the final execution's GC statistics. Its
[bytes/op conversion](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Engines/GcStats.cs)
also rounds to an integer. Consequently, first-launch counts of 12,336/64 and
336/128 disappear when the final launch is zero; a 336/1,048,576 count rounds
to zero even when selected. This is neither averaging nor an empty-workload
allocation subtraction.

The existing launcher now warns when a zero memory summary conceals any positive
raw child count, preserving the benchmark, PID, bytes and operations. It reuses
BDN's public GC-line parser; it changes neither the summary nor the exit policy.
Temporary launcher probes passed 29 checks. The allocation
reproducer failed before the correction: launch one allocated 2,816 bytes
across 32 calls, launch two reported zero, and the summary was zero. The corrected
launcher reports that first count, as well as a 24-byte/64-call rounding case.
Zero-counter and positive-summary controls emit no zero-summary warning.
Artifacts: `artifacts/finite-shape-diagnostics/launcher-{red,green}.log` and
`artifacts/benchmark-exit-codes/LauncherProbe_dc21aea09f4a44d18054eefc5dac0f21`.

**Paired process and calling-thread controls.** A bounded ignored console probe
reused the actual penetrating benchmark fixture and measured the calling-thread
interval inside the precise process-counter interval. It matched BDN's pre-pass
collection sequence and warmed the counter readers and operations first.
Each of 32 samples executed 128 operations:

| Operation | Calling-thread bytes | Process bytes | Positive process observations |
| --- | ---: | ---: | ---: |
| Penetrating contact | 0 | 0 or 336 | 17/32 |
| Idle (`Thread.Sleep(1)`) | 0 | 0, 336 or 6,192 | 23/32 |
| One `byte[64]` per call | 11,264 | 11,264 or 11,600 | 32/32 |

The idle control reproduces the discrepancy without executing a solver. The
positive control measures every real allocation. These results alone do not
identify the allocator; the following native capture supplies that evidence.
Artifact: `artifacts/finite-shape-diagnostics/counters.csv`.

**Matched native attribution of a fresh 336-byte interval.** Process-scoped CDB
breakpoints at `GCInterface::GetTotalAllocatedBytes` captured the returned precise
totals; a breakpoint at `GCInterface::GetMemoryInfo` captured the finalizer stack
and object address. Between main-thread returns 9 and 10, totals changed from
816,960 to 817,296. The intervening finalizer-thread stack was
`Gen2GcCallback.Finalize -> SharedArrayPool<char>.Trim -> GetMemoryPressure ->
GC.GetGCMemoryInfo`. SOS identified its `GCMemoryInfoData` as 288 bytes and the
immediately following pool `ConditionalWeakTable` enumerator as 48 bytes,
accounting for the exact 336-byte increase. Returns 15/16 and 23/24 supply the
same complete bracket. The calling-thread counter stayed zero. Artifact:
`artifacts/finite-shape-diagnostics/counter-objects-matched.log`.

This matches the pinned runtime sources: the
[GC-info allocation](https://github.com/dotnet/runtime/blob/v8.0.29/src/coreclr/System.Private.CoreLib/src/System/GC.CoreCLR.cs#L76-L89),
[pool enumeration](https://github.com/dotnet/runtime/blob/v8.0.29/src/libraries/System.Private.CoreLib/src/System/Buffers/SharedArrayPool.cs#L244),
and [enumerator allocation/layout](https://github.com/dotnet/runtime/blob/v8.0.29/src/libraries/System.Private.CoreLib/src/System/Runtime/CompilerServices/ConditionalWeakTable.cs#L255-L290)
explain the two real objects. BDN's
[forced-collection sequence](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Engines/Engine.cs#L271-L276)
is `Collect -> WaitForPendingFinalizers -> Collect`. The callback
[re-registers itself](https://github.com/dotnet/runtime/blob/v8.0.29/src/libraries/System.Private.CoreLib/src/System/Gen2GcCallback.cs#L54-L110),
so the second collection can queue another trim during the separate memory pass.
Blocking GC does not prevent this asynchronous finalizer work. This is a real
background allocation mechanism, distinct from the counter-accounting transition
in [FMS-Issue-019](2026-09-21-allocation-counter-accounting.md).

The historical 12,336-byte event and fresh 6,192-byte idle event lack matching
allocation evidence and remain unattributed. Earlier BDN 336-byte events are
consistent with the confirmed mechanism, but their original intervals were not
captured. No runtime setting, tolerance or measured-byte subtraction was added.

**Native failure preservation.** A fresh 12-launch capture of each affected row
(`TangentRimsCylinderCylinder`, `ZeroCoreLengthCylinderCapsule`) passed all 24
children with zero raw bytes and no AV. The selected zero-core row's original
failure actually occurred in the earlier separated-rim check in shared setup.
The original failing smoke omitted `--keepFiles`; its reused generated directory
now contains newer binaries. It cannot supply the original faulting instructions.
The fresh capture archives the generated directory and hashes its binaries/PDBs
before any later build. Artifacts: `artifacts/finite-shape-diagnostics/native`,
`debugger.log`, `binaries` and `binary-hashes.json`.

For future recurrence, use a fresh ignored capture directory, `--keepFiles` and
the process-scoped CDB recipe above. A first-chance alternative additionally
captures the instruction and thread stack bounds:

```text
sxi ibp
sxe -c ".dump /ma /u artifacts/<fresh-capture>/fault.dmp; .exr -1; r; kv; !teb; dq @rsp L40; u @rip-20 @rip+30; lmvm coreclr; gn" av
g
```

Correlate first-chance exceptions with each child's final exit. Immediately copy
the generated `FixedMathSharp.Benchmarks-*` directory to the capture directory,
including source, runtime configuration, DLLs and PDBs; record SHA-256 hashes,
revision, `dotnet --info`, command and loaded module paths. A later build can
overwrite the same generated name even with `--keepFiles`. On an actual fault,
inspect the native read address and the source span's pointer/length in that
retained dump. Successful repetitions establish nonreproduction, not a fix.

Final benchmark builds passed in Release and ReleaseLean with zero warnings or
errors. The complete Release out-of-process Short smoke under CDB passed all
459 children with 459 populated statistics across 25 reports and no AV. Its
generated binaries/PDBs are separately archived and hashed in
`artifacts/finite-shape-diagnostics/broad-binaries`; logs and exports are under
`broad-debugger.log`, `broad-output.log` and `broad`. Together with the narrow
capture, this adds 483 successful children, without establishing a native fix.
Two independent reviews found no actionable reporting or minimality findings.

The math runtime and its tests are identical to `c9485ef`, and Gravitas is
unchanged. Their sixth-pass Release/ReleaseLean coverage matrix below remains
applicable: 4,080/4,059 core tests, 49 Chronicler tests per configuration and
4,363/4,304 Gravitas tests, with 100% reachable line, branch and full-method
coverage. These are retained results, not newly rerun suites for the reporting
change. Fresh validation exercises the changed benchmark launcher through the
temporary diagnostic probes, both benchmark builds and the complete smoke above.

The separate launcher verification script and its runnable documentation were
removed on 2026-10-01 by maintainer direction. The historical probe captures
above remain evidence; they are not a maintained test suite for the benchmark
project. Future verification uses the real benchmark runner and disposable
diagnostic captures.

**Final disposition.** A further four-launch penetrating-contact capture used
512 invocations per iteration, two warmups and three measurements under CDB,
extending each memory pass to roughly two seconds. All four children passed,
each reported zero raw bytes across 512 calls, and no AV recurred. Its generated
directory and binary hashes are retained under
`artifacts/finite-shape-diagnostics/final-long-pass*`. Together with the prior
narrow and broad captures, this investigation completed 487 successful children.

The benchmark signal is archived as complete with an explicit no-change decision
for runtime math. The reporting gap is hardened and fresh 336-byte intervals are
attributed; no actionable solver allocator or native defect was identified.
The original AV, historical 12,336-byte count and fresh 6,192-byte idle count
remain unattributed observations. Further blind repetitions or speculative
runtime changes would not resolve missing original fault evidence. Reopen on a
fresh fault dump with its exact binaries/PDBs or a reproducible allocator/counter
failure attributable to the library; retain the procedure above for that event.

## Throughput refinement closure (2026-10-01)

Routine optimization of the complete nonparallel cylinder-contact signal is
closed after six retained improvements and a final bounded shared-code audit.
This accepts an explicit no-change decision for further speculative work; it
neither establishes a universal physics budget nor makes difficult contacts
cheap. Preserve the measured residual cost when budgeting callers.

The expensive fixtures exercise simple positive nonparallel contacts with small
coordinates and ordinary axis orientations, rather than a near-parallel limit,
extreme arithmetic or repeated-root recovery. Their prevalence in a real
simulation is unknown. Cheap parallel/strict controls measure different paths
and cannot establish an average nonparallel-contact cost.

The final audit found that generic triangular square convolution can change
signed cancellation order or need another bit for a doubled term. Replacing
binary scaled additions with a materialized product can also exceed existing
capacity: one-word x=2^63-1, initial destination -x and multiplier3 produces zero
then 2x within the current ordered steps, while materializing 3x exceeds a word.
Stronger contracts, new scratch or additional paths are not justified by the
current workload evidence. The existing shared owners remain unchanged.

One last shared experiment cleared a zero-sign destination once and copied only
clipped active shifted support. Its integer-oracle checks passed, but neither
contact showed a gain beyond timing variation; the runtime/test prototype was
removed. Reopen algorithmic work only when representative positive nonparallel
fixtures or a consuming simulation identify a consequential residual bottleneck.
Native-fault and allocation-counter diagnostics are archived with recurrence
criteria in the [benchmark backlog](../benchmark-signal-hardening-backlog.md#archived-signals).

The final bounded captures use committed `c9485ef`, the same two frozen inputs
and the serialized source-mode job documented below. Values are microseconds,
mean +/- 99.9% confidence half-width.

| Workload | Fresh baseline | Shifted-copy prototype | Restored committed source |
| --- | ---: | ---: | ---: |
| Penetrating rims | 4,387.843 +/- 52.950 | 4,344.511 +/- 34.016 | 4,317.534 +/- 44.017 |
| Multi-radical contact | 13,970.247 +/- 101.207 | 13,885.530 +/- 145.776 | 13,866.975 +/- 95.760 |

The apparent initial gain is under 1%, intervals overlap, and the restored
implementation is slightly faster than the prototype in both rows. Retaining
extra code is not justified. Artifacts: `artifacts/cylinder-cost5-investigation-baseline`,
`-copy` and `-close-final`. The unchanged implementation passes 210 fresh focused
arithmetic/root/construction tests (`cylinder-cost7-close-tests.log`). Runtime
and test files match the committed source exactly; the full Release/ReleaseLean
coverage matrix below is the unchanged sixth-pass validation, not a rerun for
this documentation-only closure.
All twelve final-audit benchmark children exit zero, with zero raw process
bytes and collections; all six exported rows report zero bytes per operation.

The preserved measurements, proofs, rejected experiments and validation follow.
### Complete nonparallel cylinder contacts are expensive

- **Status:** Routine refinement closed by explicit no-change decision on
  2026-10-01; six retained throughput
  passes on 2026-09-30 / 2026-10-01 reduce penetrating-rim cost to about 4.5 ms
  and multi-radical cost to about 14.4 ms, with interlaunch variation.
  Originally measured on 2026-09-26 while closing
  FMS-Issue-024. The complete exact relation is correct, but its general positive
  contacts still have substantial measured cost. This is distinct from
  the explicitly accepted zero-radius dispatch trade-off.
- **Source:** `RigidFiniteShapeRelationBenchmarks` on the working tree based on
  `ad9c88b`. Earlier sections of this completed design
  retain exact geometry, arithmetic/resource proofs, prior profiles, matched
  controls and validation evidence.
- **Environment:** Windows 11, i7-9700K, SDK 10.0.302, .NET 8.0.29, Release,
  BenchmarkDotNet 0.15.8, concurrent workstation GC, two-core affinity and
  below-normal launcher; two launches, three warmups, twelve measurements.
  Source mode used `UseLocalLsfStack=true`, `BuildInParallel=false`,
  `UseSharedCompilation=false` and `DOTNET_PROCESSOR_COUNT=2`.
- **Historical measurement (2026-09-26):** Penetrating rims **11.566 +/- 0.110ms**; multi-radical contact
  **25.137 +/- 0.164ms**, mean and 99.9% confidence half-width. Separated/tangent
  rim workloads cost **5.409 / 5.141ms**. By contrast, ordinary parallel contact
  costs **19.448 +/- 0.098us** and its strict predicate **0.856 +/- 0.008us**.
  All 22 matched child launches exited zero. Artifacts:
  `artifacts/fms024-matched-final`. The old limited-direction rim answers were
  wrong and cannot serve as equivalent-correctness throughput targets.
- **Allocation boundary:** Direct warmed calling-thread guards pass at zero.
  One matched penetrating-rim child reported 12,336 process-wide bytes across
  64 calls, although the second child and summary reported zero. The completed
  plan records the bounded counter investigation; do not silently round this
  discrepancy away or attribute it to the solver without evidence.
- **Retained change (2026-09-30):** The shared exact normal owner sizes squared
  coefficients from active gradient limbs, keeping the previous worst-case
  capacity. One additional limb covers convolution/axis-summation carry; the
  existing generated-gradient proof supplies the other capacity bound. This
  removes padding from repeated nearest-even threshold queries without changing
  coefficient values or increasing stack use. A zero first radial derivative
  now rejects a simple rim before constructing the other admission queries.
  No public API, root-selection policy, dependency or allocation was added.
- **Fresh matched evidence:** Baseline is `f6d9a2a`; geometry and commands are
  unchanged between captures. Same environment/job as above, with `--affinity 3`.
  Values below are microseconds, mean +/- 99.9% confidence half-width.

  | Frozen workload | Before | Retained change |
  | --- | ---: | ---: |
  | Penetrating rims | 11,393.033 +/- 287.947 | 7,970.354 +/- 110.001 |
  | Multi-radical contact | 26,363.528 +/- 368.507 | 25,415.384 +/- 129.939 |
  | Ordinary parallel contact | 19.611 +/- 0.227 | 19.066 +/- 0.079 |
  | Ordinary cylinder strict | 0.895 +/- 0.010 | 0.893 +/- 0.010 |
  | Ordinary cylinder/capsule | 20.268 +/- 0.360 | 19.402 +/- 0.345 |
  | Ordinary cylinder/capsule strict | 0.421 +/- 0.005 | 0.412 +/- 0.005 |
  | Oblique interior capsule rim | 558.685 +/- 12.912 | 564.926 +/- 7.455 |
  | Intersecting capsule core | 436.615 +/- 6.318 | 422.260 +/- 3.808 |

  Artifacts: `artifacts/cylinder-cost-baseline`, `cylinder-cost-profile`,
  `cylinder-cost-pruning`, `cylinder-cost-final` (combined experiment), and
  `cylinder-cost-retained` (final source). Both matched captures completed all
  sixteen child launches with zero exits and collections. Allocation summaries
  report zero; one retained strict cylinder/capsule child reported 336
  process-wide bytes over 1,048,576 operations. Preserve that counter record;
  it does not identify an allocation in the contact solver. Small changes in
  unchanged controls limit interpretation of the modest multi-radical delta.
- **Second retained change (2026-09-30):** Exact sign queries use a root-local
  dyadic change of variable inside the existing normalized Horner evaluator.
  A certified cell-wide power-of-two bound keeps the virtual variable in
  `[0,1]`; coefficient shifts are evaluated directly without a polynomial copy.
  The original resultant bound still proves zero/nonzero termination. A fallback
  keeps the previous worst-case stack/refinement bound whenever local scaling
  would increase it. Negative normalization exponents and completely discarded
  coefficients are handled explicitly. Point-refinement hints retain their
  original scale, and borrowed numerator/denominator metadata remain unscaled.
  No new solver, cache, dependency or public API was introduced.
- **Second matched evidence:** Fresh baseline is committed `43de9db`; the same
  eight frozen workloads and environment/job above are used before and after.
  Values are microseconds, mean +/- 99.9% confidence half-width.

  | Frozen workload | Before local scaling | With local scaling |
  | --- | ---: | ---: |
  | Penetrating rims | 7,808.041 +/- 47.540 | 7,066.599 +/- 138.217 |
  | Multi-radical contact | 26,219.694 +/- 299.818 | 21,470.704 +/- 224.632 |
  | Ordinary parallel contact | 19.764 +/- 0.333 | 18.628 +/- 0.086 |
  | Ordinary cylinder strict | 0.906 +/- 0.013 | 0.889 +/- 0.006 |
  | Ordinary cylinder/capsule | 20.373 +/- 0.130 | 19.210 +/- 0.200 |
  | Ordinary cylinder/capsule strict | 0.426 +/- 0.001 | 0.418 +/- 0.002 |
  | Oblique interior capsule rim | 553.774 +/- 14.819 | 552.398 +/- 7.443 |
  | Intersecting capsule core | 435.926 +/- 5.307 | 422.410 +/- 6.086 |

  Artifacts: `artifacts/cylinder-cost2-baseline`, `cylinder-cost2-profile` and
  `cylinder-cost2-scaled`. Both captures complete all sixteen child launches
  with zero exits, collections and process-wide bytes. The multi-radical mean
  improves by about 18%; penetrating rims by about 9.5%. Unchanged controls also
  move modestly, so these are matched workload deltas rather than a universal
  speedup claim. Tiny-root regression fixtures independently reproduce the
  previous excessive refinement and verify exact signs/cell containment.
- **Third retained change (2026-09-30):** The existing cubic-pencil builder
  narrows its construction workspace using the smaller of the caller's proven
  capacity and the documented generic `32*b+128`-bit bound. The actual input
  height includes optional direction invariants. Inputs and retained outputs
  keep their original strides; shared polynomial addition already supports
  independent source/output strides. This removes padded arithmetic and clears
  for small invariants without another representation or arithmetic owner.
  Caller stack allocations and worst-case capacity remain unchanged. Independent
  BigInteger fixtures cover six-word scratch with sixteen-word outputs, exact
  derivatives, a direction wider than the values, dirty padding and zero input.
- **Third matched evidence:** Fresh baseline is committed `f5d4f43`, with the
  same eight frozen workloads and environment/job above. Values are microseconds,
  mean +/- 99.9% confidence half-width.

  | Frozen workload | Before compact workspace | With compact workspace |
  | --- | ---: | ---: |
  | Penetrating rims | 6,894.589 +/- 87.459 | 5,251.505 +/- 53.626 |
  | Multi-radical contact | 22,497.088 +/- 374.413 | 22,187.533 +/- 237.640 |
  | Ordinary parallel contact | 19.922 +/- 0.218 | 20.270 +/- 0.343 |
  | Ordinary cylinder strict | 0.919 +/- 0.012 | 0.893 +/- 0.006 |
  | Ordinary cylinder/capsule | 19.702 +/- 0.200 | 19.428 +/- 0.173 |
  | Ordinary cylinder/capsule strict | 0.420 +/- 0.004 | 0.415 +/- 0.003 |
  | Oblique interior capsule rim | 565.759 +/- 8.205 | 546.317 +/- 4.842 |
  | Intersecting capsule core | 445.729 +/- 5.331 | 439.319 +/- 5.463 |

  Artifacts: `artifacts/cylinder-cost3-baseline` and `cylinder-cost3-compact`.
  Both captures complete sixteen child launches with zero exits, collections
  and process-wide bytes. Penetrating rims improve by about 24%; the
  multi-radical confidence intervals overlap, so no gain is established there.
  Unchanged controls drift modestly; these are matched workload measurements.
- **Fourth retained change (2026-09-30):** The shared shifted-magnitude owner
  limits same-sign addition to the shifted source's active support, then reuses
  its existing carry helper. Lower limbs are unchanged; higher limbs need work
  only while carry remains. The clipped support calculation uses wide integer
  arithmetic and preserves destination-edge truncation and existing signs.
  Zero/opposite-sign paths, disjoint storage, scratch bounds and allocations are
  unchanged. Independent integer tests cover padded inputs, both signs,
  cross-word shifts, carry beyond the source, input preservation and edge shifts.
- **Fourth matched evidence:** Baseline is the third retained source, with the
  same eight workloads and job. Values are microseconds, mean +/- 99.9%
  confidence half-width.

  | Frozen workload | Before bounded addition | With bounded addition |
  | --- | ---: | ---: |
  | Penetrating rims | 5,307.459 +/- 99.581 | 5,217.296 +/- 103.923 |
  | Multi-radical contact | 22,446.355 +/- 459.394 | 21,817.759 +/- 374.135 |
  | Ordinary parallel contact | 20.104 +/- 0.225 | 20.077 +/- 0.267 |
  | Ordinary cylinder strict | 0.905 +/- 0.012 | 0.908 +/- 0.012 |
  | Ordinary cylinder/capsule | 19.882 +/- 0.348 | 19.809 +/- 0.345 |
  | Ordinary cylinder/capsule strict | 0.424 +/- 0.004 | 0.421 +/- 0.006 |
  | Oblique interior capsule rim | 578.789 +/- 10.426 | 562.700 +/- 14.469 |
  | Intersecting capsule core | 446.533 +/- 7.999 | 440.965 +/- 7.010 |

  Artifacts: `artifacts/cylinder-cost4-baseline` and `cylinder-cost4-carry`.
  The baseline has split timing modes and more interlaunch variation, so the
  pooled 2.80% multi-radical gain was checked in reverse capture order. That
  confirmation measures `22,512.525 +/- 440.588us` before bounded addition
  versus `21,899.239 +/- 431.526us` with it, a 2.72% gain. All four prototype
  launch means are below all four baseline launch means. Penetrating rims are
  `5,385.256 +/- 80.650us` versus `5,271.773 +/- 101.187us` in the reversal;
  no strong additional penetrating-rim gain is established. Report a modest
  reproducible multi-radical workload improvement, not a universal speedup.
  Artifacts: `artifacts/cylinder-cost4-baseline-confirm` and `-carry-confirm`.
  The two broad captures complete sixteen children each and the two narrow
  confirmations four each, all with zero exits, collections and process bytes.
- **Fifth retained change (2026-10-01):** Simple rim admission already proves
  the defining root's slope sign. Pass its negation through the existing
  retained-sign query as the lower endpoint sign, avoiding two exact defining
  polynomial evaluations whenever refinement is needed. A nonrational cell
  isolating that simple root has opposite endpoint signs; its descendants
  preserve that crossing. Rational discovery still exits through the existing
  path, and repeated roots and generic callers retain exact endpoint evaluation.
  The optional internal certificate adds no scratch or cache and leaves query
  coefficients, metadata, ranking, refinement bounds and fallbacks unchanged.
  Integer-oracle tests compare hinted and ordinary signs/cells for both slope
  orientations, tiny roots, cancellation, undersized caller storage and dyadic
  discovery followed by another query.
- **Fifth matched evidence:** Fresh baseline is committed `e548376`. The same
  eight frozen workloads and job above were captured before and after; a narrow
  reverse-order baseline checks the modest gain. Values are microseconds, mean
  +/- 99.9% confidence half-width.

  | Frozen workload | Fresh baseline | Crossing reuse | Reverse baseline |
  | --- | ---: | ---: | ---: |
  | Penetrating rims | 5,195.724 +/- 85.085 | 5,166.120 +/- 73.863 | 5,321.807 +/- 95.625 |
  | Multi-radical contact | 22,202.157 +/- 376.602 | 21,327.191 +/- 293.130 | 21,985.681 +/- 253.988 |

  Artifacts: `artifacts/cylinder-cost5-baseline`, `-crossing` and
  `-baseline-crossing-confirm`. Multi-radical cost improves by about 3% against
  the reverse baseline; both prototype launch means are below both reversal
  baseline means, and their pooled confidence intervals are disjoint.
  Penetrating rims show no consistent additional improvement. Across the
  captures, unchanged controls drift modestly, including an oblique capsule
  rim increase; this is a modest measured workload gain, not a universal
  speedup. All 36 child launches exit zero, with zero raw process bytes and
  collections. The reverse baseline also agrees with an earlier two-case
  baseline (`cylinder-cost5-baseline-confirm`, 21,833.825 +/- 342.645us).
- **Sixth retained change (2026-10-01):** The existing shared magnitude
  multiplier clips inputs to the retained product width and factors out their
  whole zero low limbs before convolution. Limbs at or above the output width
  cannot affect its low product; factoring `a+b` zero limbs leaves that cleared
  output prefix zero and computes the same truncated product in the suffix.
  The subtraction-based width guard avoids overflow when adding offsets. Both
  prefix scans are bounded by output width, and the original convolution,
  sparse carry and disjoint-output contracts remain intact. No scratch,
  representation, dependency or helper signature is added. Shared integer
  oracles cover full/truncated products, dirty output, sparse interior carry,
  zero/empty inputs in either position, same-input squares and source preservation.
- **Sixth matched evidence:** Baseline is the fifth retained source. Same eight
  workloads, environment and job; values are microseconds, mean +/- 99.9%
  confidence half-width.

  | Frozen workload | Before factoring | With factoring |
  | --- | ---: | ---: |
  | Penetrating rims | 5,166.120 +/- 73.863 | 4,613.120 +/- 98.457 |
  | Multi-radical contact | 21,327.191 +/- 293.130 | 15,402.844 +/- 613.027 |
  | Ordinary parallel contact | 20.123 +/- 0.269 | 19.443 +/- 0.253 |
  | Ordinary cylinder strict | 0.910 +/- 0.014 | 0.902 +/- 0.010 |
  | Ordinary cylinder/capsule | 20.083 +/- 0.368 | 17.850 +/- 0.268 |
  | Ordinary cylinder/capsule strict | 0.420 +/- 0.007 | 0.415 +/- 0.005 |
  | Oblique interior capsule rim | 579.012 +/- 12.377 | 555.532 +/- 7.609 |
  | Intersecting capsule core | 443.227 +/- 8.604 | 430.512 +/- 6.093 |

  Artifacts: `artifacts/cylinder-cost5-crossing` and `-prefix`. The pooled
  multi-radical gain is about 28%, with launch means of 14.718 / 16.149ms and
  several detected timing modes; report that variation. Even the slower launch
  remains substantially faster than either baseline launch. Penetrating rims
  improve by about 11%. All sixteen prototype children exit zero with zero raw
  process bytes and collections.
  Four existing nonzero-low-limb controls check the shared helper's overhead:

  | Existing workload | Before factoring | With factoring |
  | --- | ---: | ---: |
  | `RadialProjectionWorstCaseComparison` | 3.802 +/- 0.053 | 3.792 +/- 0.064 |
  | `TrianglePairTinyAxisFallback` | 10.699 +/- 0.127 | 10.580 +/- 0.177 |
  | `ArbitraryRawCylinderIntersectionInterval`, Scale=1 | 2.944 +/- 0.038 | 2.983 +/- 0.038 |
  | Same interval, Scale=100000 | 7.373 +/- 0.096 | 7.375 +/- 0.096 |

  No meaningful overhead regression is established in these controls. Artifacts:
  `artifacts/cylinder-cost6-controls-before-box`, `-before-axis`, `-prefix-box`
  and `-prefix-axis`; reuse these fixtures instead of adding benchmark scaffolding.
  A reverse-order two-case baseline without factoring measures penetrating rims
  at 5,101.971 +/- 76.136us and multi-radical contact at 20,956.847 +/- 339.380us
  (`cylinder-cost5-crossing-prefix-confirm`). The final factoring-source
  confirmation measures 4,497.877 +/- 56.579us and 14,388.334 +/- 117.786us
  (`cylinder-cost5-prefix-confirm`). Both captures support substantial gains;
  the multi-radical reduction ranges about 26.5--31.3% against that reverse
  baseline. The original broad prototype has greater interlaunch variation,
  so retain both reports rather than treating the final mean as a guarantee.
  All eight confirmation children exit zero with zero collections. One final
  penetrating-rim child reports 336 process-wide bytes across 128 operations,
  while its paired child, both multi-radical children and JSON summaries report
  zero. Preserve that discrepancy; it does not establish a solver allocation.
  Direct warmed calling-thread guards remain the solver's allocation gate.
- **Rejected experiments:** Ranking values before full radial/cap admission
  skipped only potential positive nonwinners, preserving negative separation
  candidates. It added comparisons for inadmissible roots and did not resolve
  the multi-radical cost; that selection-policy change was removed. The retained
  patch keeps the original exact ranking order and borrowed-root contracts.
  A third-pass experiment reused local dyadic scaling for byte-refinement
  endpoint certificates while preserving the absolute error quantum and exact
  fallback. It passed 153 focused exact tests but produced no measurable gain:
  penetrating rims were `5,229.851 +/- 61.943us` and multi-radical contact
  `22,448.588 +/- 268.844us`, compared with the compact-workspace column above.
  All sixteen child launches passed with zero allocations/collections. That
  runtime/test experiment was removed; do not repeat it as a new optimization.
  Artifact: `artifacts/cylinder-cost3-refinement`.
  A fifth-pass square-free root-order shortcut obtained slope signs from parity
  instead of exact derivative queries. Its full capture measured multi-radical
  cost at 22,124.165 +/- 413.197us versus 22,202.157 +/- 376.602us: no established
  gain, so the additional API/tests were removed (`cylinder-cost5-slope`).
  A dual-polynomial square shortcut replaced the second equal cross-product
  with whole-span doubling. An initial apparent gain did not survive reversal:
  21,801.680 +/- 389.358us with it versus 21,833.825 +/- 342.645us without it;
  penetrating rims became slower. That runtime/test experiment was removed
  (`cylinder-cost5-dual`, `-dual-confirm` and `-baseline-confirm`). Do not
  repeat these as newly demonstrated optimizations.
  Skipping the normalized Horner evaluator's initial zero multiplication and
  zero-coefficient assembly passed 212 focused tests, but its 14,932.604 +/-
  642.645us multi-radical result overlaps the factoring capture's range.
  The extra branches were removed (`cylinder-cost5-horner`); no further gain
  is established from that experiment.
- **Validation:** Fresh source-backed solution builds pass in both repositories
  for `Release` and `ReleaseLean`, targeting `netstandard2.1` and `net8.0`, with
  zero warnings/errors. All tests pass with no skips or new coverage exclusions;
  both Chronicler suites also pass all 49 tests. Exact covered/total counts from
  ReportGenerator are below. The core suites include the warmed zero-allocation,
  1 MiB worker stack / live 64 KiB caller-buffer, full-domain, deterministic tie
  and nearest-even rounding gates. New tests independently check full-limb
  summation carry, padded normal coefficients, input preservation, zero-radial
  lazy admission, relative precision for tiny/zero-lower cells, low-bit
  cancellation, negative normalization heights and upper-endpoint capacity.
  Compact construction fixtures also check padded outputs, exact derivatives,
  direction-dominant input height and canonical zero with dirty storage.
  Shared arithmetic fixtures check carry beyond active support and retained
  destination-edge behavior against independent integer results.
  Crossing fixtures additionally verify exported nonzero endpoints beside an
  excluded zero and another dyadic root, while shared multiplication fixtures
  verify full/truncated zero-prefix products and same-input squares.
  Independent correctness/resource and Ponytail reviews found no actionable
  issues in the retained changes.

  | Repository / configuration | Tests passed | Lines | Branches | Methods |
  | --- | ---: | ---: | ---: | ---: |
  | FixedMathSharp Release | 4,080 | 52,673 / 52,673 | 12,118 / 12,118 | 3,934 / 3,934 |
  | FixedMathSharp ReleaseLean | 4,059 | 52,766 / 52,766 | 12,118 / 12,118 | 3,930 / 3,930 |
  | Gravitas Release | 4,363 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |
  | Gravitas ReleaseLean | 4,304 | 56,091 / 56,091 | 16,224 / 16,224 | 5,370 / 5,370 |

  Every reported method is fully covered. Each repository's
  `artifacts/cylinder-cost6-Release*-tests.log`,
  `-coverage` and `-report/Summary.json` retain the raw validation evidence.
  The benchmark catalog lists successfully. The first-pass broad smoke passed
  all 459 cases (`artifacts/cylinder-cost-smoke`). The fresh second-pass native
  capture and final ordinary confirmation both pass all 459 cases, with complete
  results and zero child/launcher exit codes (`artifacts/cylinder-cost2-native/broad`
  and `cylinder-cost2-smoke-confirm`). The intervening failure is recorded below.
  The third-pass ordinary smoke also completes all 459 child launches with
  complete statistics, zero child/launcher exits and no native fault
  (`artifacts/cylinder-cost3-smoke`).
  The final fourth-pass source-backed smoke also passes all 459 child launches,
  with 459 complete statistics across 25 reports, zero child/launcher exits and
  no native fault (`artifacts/cylinder-cost4-smoke`).
  The final sixth-pass source-backed smoke likewise passes all 459 child
  launches, with 459 complete statistics across 25 reports, zero child/launcher
  exits and no native fault (`artifacts/cylinder-cost6-smoke`). The fresh catalog
  and both retained-source EventPipe profile children also pass.
  These `all -j Short --iterationTime 10 --affinity 3` runs verify execution
  only; performance claims use the separate matched captures above.
- **Native smoke follow-up:** The initial second-pass broad run failed two
  children with `0xC0000005` in the shared finite-shape setup, at
  `GetShiftedMagnitudeWord` during unchanged value/derivative construction.
  Tangency and zero-core capsule selections were affected; the latter failed
  while setup checked a separated-rim fixture. There were no workload results
  for those children, and the launcher correctly returned failure. Preserve
  `artifacts/cylinder-cost2-smoke`; it is not a successful smoke capture.
  A twelve-child ordinary rerun, forty-eight-child process-scoped CDB rerun,
  full 459-child CDB capture and final ordinary 459-child confirmation all pass,
  with no AV dump. Artifacts:
  `artifacts/cylinder-cost2-smoke-repro` and `cylinder-cost2-native`.
  Safe-span/resource review found no corruption path, and the completed design
  records an unexplained pre-optimization AV on 2026-09-26. These reruns do not
  establish a JIT/harness cause or explain the new failures. Keep this native
  follow-up open; capture the faulting instruction, registers and span backing
  storage on recurrence. No runtime source or JIT/GC settings were changed
  during this investigation.
- **Impact:** A few difficult oblique contacts can dominate a fixed frame.
  At 32 FPS, the entire simulation has 31.25 ms per step: one latest penetrating
  fixture consumes about 14% of that time and one multi-radical fixture about
  46%, before other collision queries, solver and simulation work. These are
  illustrative single-thread costs, not a universal acceptance threshold.
  No observed game workload or contact-count budget establishes acceptability.
  Exact determinism, minimum-depth selection and rounding remain mandatory.
- **Evidence required to reopen:** The sixth-pass retained-source profile places about
  58% of multi-radical cost in cap-root selection, 37% in derivative construction,
  34% in side features, 15% in exact root refinement and 15% in sign queries.
  Inclusive costs overlap and must not be summed. The fourth-pass profile had
  approximately 62 / 25 / 28 / 37 / 31% in those respective owners; shared
  magnitude multiplication falls from about 53% to 40% of sampled time.
  These are sampled shares, not separately measured phase-speedup claims.
  Penetrating rims retain about 54% cap selection, 31% sign queries, 29%
  derivative construction and 20% side features. Focus the next pass on exact
  derivative construction and side-family polynomial work, preserving width,
  ranking, certification and live stack bounds. Avoid retaining another large
  polynomial cache across sign queries or reviving the rejected whole-span
  square shortcut without new evidence. Artifacts:
  `artifacts/cylinder-cost6-after-profile` and `-after-profile-summary.json`;
  the earlier comparison is `cylinder-cost4-after-profile`.
  Existing profiles already removed repeated
  GCDs/chains, redundant normals and worst-case mapping refinement; do not
  repeat those experiments or add another approximate fallback.

In coordinated source mode, reproduce the two expensive positive contacts:

```powershell
$env:UseLocalLsfStack = 'true'
$env:BuildInParallel = 'false'
$env:UseSharedCompilation = 'false'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -p:UseSharedCompilation=false -m:1
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll rigid-finite-shape-relation --filter '*PenetratingRimsCylinderCylinder*' '*MultiRadicalCylinderCylinder*' --warmupCount 3 --iterationCount 12 --launchCount 2 --affinity 3 --keepFiles --exporters json --artifacts artifacts/benchmarks/cylinder-pair-cost
```

Serialize runs with other build/test/profile work. Inspect each child exit and
GC record as well as the aggregate report. Recheck shared cylinder/capsule and
parallel controls for any retained optimization.
