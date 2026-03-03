# FixedMathSharp agent instructions

## Project intent and architecture
- This repository is a deterministic fixed-point math library centered on `Fixed64` (`src/FixedMathSharp/Numerics/Fixed64.cs`) with Q32.32 representation (`SHIFT_AMOUNT_I = 32` in `src/FixedMathSharp/Core/FixedMath.cs`).
- Most API surface lives in value-type numerics (`Fixed64`, `Vector2d`, `Vector3d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`) plus bounds types (`BoundingBox`, `BoundingSphere`, `BoundingArea`) implementing `IBound`.
- Keep operations deterministic and allocation-light; many methods use `m_rawValue` and `[MethodImpl(MethodImplOptions.AggressiveInlining)]` for hot paths.
- `FixedMath` and `FixedTrigonometry` are the shared algorithm backbones; extension classes in `src/FixedMathSharp/Numerics/Extensions/` are thin forwarding wrappers, not alternate implementations.

## Build and test workflows
- Solution: `FixedMathSharp.sln` with library project and test project.
- Target frameworks are multi-targeted in both projects: `net48;net8`.
- Typical local workflow:
  - `dotnet restore`
  - `dotnet build --configuration Debug --no-restore`
  - `dotnet test --configuration Debug`
- CI detail from `.github/workflows/dotnet.yml`:
  - Linux runs `net48` tests via Mono + `xunit.console.exe` and `net8` via `dotnet test -f net8`.
  - Windows runs `dotnet test` for both TFMs.
- Packaging/versioning comes from `src/FixedMathSharp/FixedMathSharp.csproj`: GitVersion variables are consumed when present, otherwise version falls back to `0.0.0`.

## Code conventions specific to this repo
- Prefer `Fixed64` constants (`Fixed64.Zero`, `Fixed64.One`, `FixedMath.PI`) over primitive literals in math-heavy code.
- Preserve saturating/guarded semantics in operators and math helpers (for example `Fixed64` add/sub overflow behavior).
- When touching bounds logic, maintain cross-type dispatch shape in `Intersects(IBound)` and shared clamping projection via `IBoundExtensions.ProjectPointWithinBounds`.
- Serialization compatibility is intentional:
  - MessagePack attributes on serializable structs (`[MessagePackObject]`, `[Key]`) across TFMs.
  - Conditional serializers in tests (`BinaryFormatter` for `NET48`, `System.Text.Json` for `NET8`).
- `ThreadLocalRandom` is marked `[Obsolete]`; new deterministic RNG work should prefer `DeterministicRandom` and `DeterministicRandom.FromWorldFeature(...)` in `src/FixedMathSharp/Utility/DeterministicRandom.cs`.

## Testing patterns to mirror
- Tests are xUnit (`tests/FixedMathSharp.Tests`). Keep one feature area per test file (e.g., `Vector3d.Tests.cs`, `Bounds/BoundingBox.Tests.cs`).
- Use helper assertions from `tests/FixedMathSharp.Tests/Support/FixedMathTestHelper.cs` for tolerance/range checks rather than ad-hoc epsilon logic.
- For deterministic RNG changes, validate same-seed reproducibility and bounds/argument exceptions like in `DeterministicRandom.Tests.cs`.

## Agent editing guidance
- Keep public API shape stable unless the task explicitly requests API changes.
- Match existing style (regions, XML docs, explicit namespaces, no implicit usings).
- Make focused edits in the relevant numeric/bounds module and update corresponding tests in the parallel test file.