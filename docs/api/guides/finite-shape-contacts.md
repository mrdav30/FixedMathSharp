---
title: Finite-shape contacts
---

# Finite-shape contacts

These advanced contracts cover cylinder, cone, and vertical convex-prism
contacts. Start with [Bounds and Geometry](bounds-and-geometry.md) for shape
selection and shared query semantics.

## Cylinder contacts

`FixedOrientedBox.TryGetCenteredCylinderContact` includes the box's faces, edges
and vertices against the finite cylinder's caps, side and rims. It finds the
minimum translation without replacing the cylinder with a capsule. A zero radius
is a finite axis segment. The manifold overload additionally clips up to four
matched contacts when a parallel box face and cylinder cap are selected; other
features use the primary contact with a zero manifold count.

`FixedTriangle.TryGetCenteredFiniteCylinderContact` includes the triangle's
face, edges and vertices against the finite cylinder's caps, side and rims. It
returns a triangle-to-cylinder normal, depth and shape-local contact anchors. A
triangle's closest point to the cylinder center is not sufficient to decide
contact: another part of the triangle can cross a flat cap or a curved rim.
Degenerate triangles return false, matching the triangle/slab contact APIs.
`TryGetCircleSlabContact` uses the same geometry for an upright finite cylinder,
while retaining the circle's authored yaw and half-thickness. Half-thickness is
not doubled in `Fixed64`, so a representable slab is not shortened when its full
height exceeds the scalar range. `TryGetCenteredCapsuleSlabContact` describes a
planar capsule (a rectangle with semicircular ends) extruded through a flat Y
slab, not a rounded 3D capsule. Its contact query covers both rounded ends, the
straight sides, and the flat caps, including oblique rim contacts. Contact
points are paired on the selected features; a straight-side or cap contact can
lie between the rounded ends. The query retains the authored local axis and yaw,
including half-raw endpoints from odd raw core lengths. A zero core uses the
cylinder path above, with the same contact anchors.

### Hull contacts for coordinated consumers

The internal closed-hull/cylinder contact path used by coordinated consumers
ranks these finite feature families across the complete hull. A shared rational
cylinder chart preserves exact ordering between triangles; the triangle that
first supplies a supporting direction need not contain its matched witness. An
exact inscribed-ball bound can certify a minimum before the remaining features
are enumerated. Its contained-center entry includes hull boundaries; declining
center admission does not imply separation for an exterior center. The same
internal reducer admits finite stadium slabs through a shared planar core chart,
retaining endpoint regions and straight-section seam witnesses. An expanded-hull
ball certificate includes every original face and supported edge/core extrusion
facet; an uncertified seed is discarded before the complete feature search.
Affine seam products retain wider fields without widening the native rigid path
or rounding a conceptual core endpoint.

### Cylinder pairs

`FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact` includes the finite
cylinder's caps, side, and rims when finding the minimum translation. It also
handles a capsule core that enters the cylinder: depth is not necessarily just
the capsule radius. A zero-length capsule core is a sphere; a zero-radius
cylinder is its finite axis segment.

`FixedSegment.TryGetCenteredFiniteCylindersContact` similarly finds the minimum
translation between two finite cylinders, including side/rim and rim/rim
constraints. Testing only the axes and their cross product is insufficient for
general finite cylinders. Parallel pairs use a simpler exact path; general
interacting rims require substantially more work. Measure the contact workloads
that occur in your simulation, not only the ordinary parallel case.

### Depth, witnesses, and stack usage

Classification and depth ordering use the exact authored geometry. Exact
tangency is contact with zero depth, and a sub-raw separation remains a miss.
Only the selected normal and complete depth are rounded; `DepthIsClamped`
distinguishes conceptual overflow from an exactly representable maximum. Equal
minima follow stable feature order. Continuous radial ties use a representative
in the authored cylinder frame.

The returned anchors retain independently selected shape-local support features.
They are not a promise that two algebraic closest points were rounded together
or that the materialized anchors coincide at every tangency. Box anchors retain
the exact selected direction's support signs even if a tiny normal component
rounds to zero.

Curved oblique contacts require more work than ordinary cap/side contacts. The
query allocates no managed memory, but its exact fallback can use hundreds of
KiB of bounded stack scratch. Avoid deliberately small-stack worker threads;
allow at least a 1 MiB thread stack with adequate headroom for the caller.

Cylinder pairs use the same finite-segment contact path when either radius is
zero. The stack guidance above applies to both positive-radius and zero-radius
oblique pairs. A zero-radius shape's anchor can lie at the segment center when
its whole core supports the selected normal; callers should not require a
particular endpoint in a tie.

## Triangle/cone contacts

`FixedTriangle.TryGetCenteredFiniteConeContact` finds an inclusive minimum-depth
contact with a finite cone whose local axis is +Y: the base is at `-height/2`
and the apex at `+height/2`. It includes triangle face, edge and vertex features
against the base disk, curved rim, apex and lateral generators. The normal
points from triangle toward cone; depth and both anchors come from that same
selected feature.

Equal minima retain the earlier feature. When both axial exits share the minimum
for a horizontal face, the earlier +Up candidate retains the base witness.
Witness coordinates whose radical terms vanish or cancel use exact rational
nearest-even rounding, including signed half-raw ties and scalar extrema.

Both frame rotations must be normalized, height must be positive, and radius
must be nonnegative. A zero radius is the identical axial-segment relation used
by the cylinder contact query. A segment piercing a triangle can require a
positive exit distance even though the segment has no volume. Degenerate
triangles follow `FixedTriangle.IsDegenerate` and return false.

Classification precedes final rounding: exact touching is contact, a genuine
positive gap is not, and a tiny overlap can round to zero depth. Anchors retain
the authored frames even when the relative center or absolute world witness
cannot be materialized as a `Vector3d`. Curved contacts have the same bounded
stack-scratch considerations described above.

### Coplanar patches and convex polygons

The internal cone contact owner can also certify a minimum face exit for a
borrower-owned connected coplanar patch. The borrower supplies trusted seam
topology and every exposed perimeter edge, including holes. Exact projection
membership in a seed triangle establishes membership in the actual patch;
unrounded boundary clearances then certify either a radius-depth tube around an
actual cone chord or the whole cone projection enlarged by that depth. The
center's signed side chooses each perimeter half-plane; exact cone support
clearance preserves concave patches without filling their holes. Rational
quaternion frames and complete cone support avoid rounded intermediate extrema.
A failed certificate leaves complete single-triangle feature selection
available.

For a trusted filled convex polygon, the internal owner also solves its complete
face, perimeter-edge and corner normal fan. An ordered strict-corner loop
excludes collinear subdivisions. Corner charts admit only globally supporting
features before classifying separation; their artificial diagonals never become
contacts. One unreduced rational frame and squared-value scale retain exact
ranking across analytic candidates and stationary rim roots. A complete polygon
fan admits supporting generator slices before materializing paired witnesses.
The borrower establishes convex topology and supplies an intersecting seed for
base-pole witnesses. Supplied local witness bounds may contain the whole patch,
allowing face coordinates outside the seed triangle to round without restricting
them to that triangle. This internal contract does not validate topology or
determine a whole noncoplanar surface union's minimum exit.

`TryGetCenteredFiniteConeSupportContact` is a different operation: it projects
one caller-selected cone support onto the triangle. A failed support projection
does not prove that the complete shapes are separated. Use the complete contact
query when deciding whether the shapes intersect.

## Vertical convex-prism contacts

`FixedConvexPrismRelations` provides contacts against a convex footprint
extruded through a positive Y half-thickness. Footprint offsets are
boundary-ordered in either winding; repeated or collinear offsets retain their
finite closed extent. Touching counts as contact, including zero-radius finite
axes. The shape's local axis and rotation remain separately authored.

`TryGetSphereContact` and `TryGetCenteredCapsuleContact` start with canonical
projection axes and certify finite opposing supports. Clip an exposed capsule
core to the selected native-yaw prism plane; keep its midpoint when admitted,
otherwise its nearest finite parameter. An empty projection is completed by the
exact closest core/real-edge pair, including horizontal rims and vertices. Each
exit retains the same signed depth, normal and opposing feature, including
containment. Translate the shape opposite that normal to follow the exit;
equal-depth axes and equivalent finite core parameters retain stable order.
Apply the radius before radial-direction rounding, so full-domain radii do not
amplify an already rounded unit normal's error.

`TryGetCenteredCylinderContact` and `TryGetCenteredConeContact` find the minimum
exit through the complete support fan. Prism faces, real edges and vertices meet
the cylinder's caps, side and rims, or the cone's base disk and rim, apex and
lateral generators. Complete closed admission precedes depth selection;
projection overlap alone does not establish intersection. The authored local
axis retains its actual squared norm, and the prism retains its scalar yaw.

Classification, feature admission and minimum-depth ordering are exact. Touching
returns zero depth; a tiny positive overlap can also round to zero. Equal minima
retain stable feature order. The normal points from the curved primitive toward
the prism, so the exact minimum translation moves the primitive opposite that
direction to the touching boundary. `DepthIsClamped` distinguishes an exact
minimum above `Fixed64.MaxValue`.

### Rounding and witness frames

The selected depth, unit-normal components and witness coordinates round
nearest-even only at the public boundary. An unclamped depth has at most half a
raw unit of error. Normal rounding also matters when reconstructing that
unclamped exit: with `e = 2^-33`, interpreting the returned scalars exactly
gives a displacement error of at most `e + Depth * sqrt(3) * e` from the exact
minimum translation. Each subsequent `Fixed64` component product can add another
half raw unit of rounding error. A strict separating move needs margin for those
final rounding terms; touching remains contact.

Both witnesses are matched on the selected finite support features before
coordinate rounding. Their origins remain the primitive center and prism origin,
respectively, while both anchors use the primitive's quaternion frame. This
preserves the native scalar-yaw footprint without introducing a rounded prism
quaternion. Materialize or compare points through the anchor operations; the
second anchor's local coordinates are expressed in that retained frame. Separate
additive terms preserve coordinates outside a single scalar's range. Curved
prism contacts use the bounded stack scratch described above.
