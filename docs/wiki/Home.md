# FixedMathSharp Wiki

FixedMathSharp is an engine-agnostic, deterministic Q32.32 mathematics library
for simulations, games, procedural generation, replays, and lockstep systems.

Use this wiki for design context and behavioral contracts. Use the generated API
documentation for individual types and members.

## Quick Links

| Destination                                                                       | Use it for                                          |
| --------------------------------------------------------------------------------- | --------------------------------------------------- |
| [README](https://github.com/mrdav30/FixedMathSharp/blob/main/README.md)           | Installation, package selection, and first examples |
| [Documentation Site](https://mrdav30.github.io/FixedMathSharp/)                   | Generated documentation landing page                |
| [API Reference](https://mrdav30.github.io/FixedMathSharp/api/FixedMathSharp.html) | Public namespaces, types, and members               |
| [Coverage Report](https://mrdav30.github.io/FixedMathSharp/coverage/)             | Current test coverage details                       |
| [GitHub Repository](https://github.com/mrdav30/FixedMathSharp)                    | Source, issues, releases, and contribution history  |

## Wiki Navigation

| Page                                                          | Focus                                                                     |
| ------------------------------------------------------------- | ------------------------------------------------------------------------- |
| Home                                                          | Concise navigation and project links                                      |
| [Technical Overview](Overview.md)                             | System shape, API ownership, package variants, and validation strategy    |
| [Fixed64 Representation](fixed64-representation.md)           | Q32.32 raw layout, range, precision, conversion, and overflow behavior    |
| [Full-Domain Wide Arithmetic](full-domain-wide-arithmetic.md) | Exact fixed-width intermediates, rounding, and public narrowing contracts |
| [Coordinate Conventions](coordinate-conventions.md)           | Core `+Z`-forward convention and adapter-boundary mappings                |
| [Bounds and Geometry](bounds-and-geometry.md)                 | Dimension-explicit shapes, relations, anchors, and boundary semantics     |
| [Diagnostics Formatting](diagnostics-formatting.md)           | Diagnostic formatting, raw payload text, and the serialization boundary   |

Start with the Technical Overview, then Fixed64 Representation; read Full-Domain
Wide Arithmetic before fused or `Try*` APIs, then the domain page you need.
