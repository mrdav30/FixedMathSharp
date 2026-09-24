---
uid: FixedMathSharp.Chronicler
summary: *content
---

The optional Chronicler companion owns exact simulation-time conversions and
deterministic record-hash extensions; FixedMathSharp core has no timing dependency.

Use `FixedChronicleTime` to widen Fixed64 seconds or explicitly narrow a bounded
Chronicler duration. Subtract wide timestamps first. Out-of-range conversions
reject rather than saturating, and complete-step counts use raw integer division
rounded down, not a historical timestamp-to-frame lookup.

See the [companion guide](https://github.com/mrdav30/FixedMathSharp/blob/main/src/FixedMathSharp.Chronicler/README.md)
for exact conversion and record-hash examples, and use matching Standard or Lean
package families throughout the dependency graph.
