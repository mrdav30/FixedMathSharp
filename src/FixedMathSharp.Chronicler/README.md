# FixedMathSharp.Chronicler

`FixedMathSharp.Chronicler` adds deterministic `ChronicleHashWriter` extensions
for FixedMathSharp values. Use it to build replay checks, conformance signals,
and state fingerprints without duplicating component-order rules.

## Install

```bash
dotnet add package FixedMathSharp.Chronicler
```

Use
[`FixedMathSharp.Chronicler.Lean`](https://www.nuget.org/packages/FixedMathSharp.Chronicler.Lean)
when the rest of your application uses the FixedMathSharp Lean package graph.
Both variants target .NET Standard 2.1 and .NET 8.

## Example

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

## Learn more

- [FixedMathSharp repository](https://github.com/mrdav30/FixedMathSharp)
- [FixedMathSharp API reference](https://mrdav30.github.io/FixedMathSharp/)
- [Chronicler source](https://github.com/mrdav30/Chronicler)

FixedMathSharp.Chronicler is licensed under the
[MIT License](https://github.com/mrdav30/FixedMathSharp/blob/main/LICENSE).
