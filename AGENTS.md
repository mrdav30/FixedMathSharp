# FixedMathSharp agent instructions

## Project intent and architecture

- This repository is a deterministic fixed-point math library centered on `Fixed64` (`src/FixedMathSharp/Numerics/Fixed64.cs`) with Q32.32 representation (`SHIFT_AMOUNT_I = 32` in `src/FixedMathSharp/Core/FixedMath.cs`).
- Most API surface lives in value-type numerics (`Fixed64`, `Vector2d`, `Vector3d`, `FixedQuaternion`, `Fixed3x3`, `Fixed4x4`) plus bounds types (`BoundingBox`, `BoundingSphere`, `BoundingArea`) implementing `IBound`.
- Keep operations deterministic and allocation-light; many methods use `m_rawValue` and `[MethodImpl(MethodImplOptions.AggressiveInlining)]` for hot paths.
- `FixedMath` and `FixedTrigonometry` are the shared algorithm backbones; extension classes in `src/FixedMathSharp/Numerics/Extensions/` are thin forwarding wrappers, not alternate implementations.

## Build and test workflows

- Solution: `FixedMathSharp.slnx` with library project and test project.
- Target frameworks are configured in the respective `.csproj` files; `net8.0` is the primary TFM.
- There are two release build configurations:
  - `Release` — standard release build with MemoryPack support.
  - `ReleaseNoMemoryPack` — release build with MemoryPack excluded.
- Typical local workflow:
  - `dotnet restore`
  - `dotnet build --configuration Debug --no-restore`
  - `dotnet test --configuration Debug`
- CI detail from `.github/workflows/dotnet.yml`:
  - Linux and Windows both run `dotnet test` against the supported TFMs (with `net8.0` as the primary test target).
  - Refer to the workflow file for the exact matrix of OS/TFM combinations.
- Packaging/versioning comes from `src/FixedMathSharp/FixedMathSharp.csproj`: GitVersion variables are consumed when present, otherwise version falls back to `0.0.0`.

## Code conventions specific to this repo

- Prefer `Fixed64` constants (`Fixed64.Zero`, `Fixed64.One`, `FixedMath.PI`) over primitive literals in math-heavy code.
- Preserve saturating/guarded semantics in operators and math helpers (for example `Fixed64` add/sub overflow behavior).
- When touching bounds logic, maintain cross-type dispatch shape in `Intersects(IBound)` and shared clamping projection via `IBoundExtensions.ProjectPointWithinBounds`.
- Serialization compatibility is intentional and now uses MemoryPack:
  - MemoryPack attributes on serializable structs (for example `[MemoryPackable]`, `[MemoryPackInclude]`) are the source of truth for serialized layouts.
  - Tests use MemoryPack-based roundtrips (and `System.Text.Json` where appropriate) instead of legacy `MessagePack`/`BinaryFormatter` serializers.

## Testing patterns to mirror

- Tests are xUnit (`tests/FixedMathSharp.Tests`). Keep one feature area per test file (e.g., `Vector3d.Tests.cs`, `Bounds/BoundingBox.Tests.cs`).
- Use helper assertions from `tests/FixedMathSharp.Tests/Support/FixedMathTestHelper.cs` for tolerance/range checks rather than ad-hoc epsilon logic.
- For deterministic RNG changes, validate same-seed reproducibility and bounds/argument exceptions like in `DeterministicRandom.Tests.cs`.

## Agent editing guidance

- Keep public API shape stable unless the task explicitly requests API changes.
- Match existing style (regions, XML docs, explicit namespaces, no implicit usings).
- Make focused edits in the relevant numeric/bounds module and update corresponding tests in the parallel test file.
- For serialization changes, ensure MemoryPack attributes are correctly applied and roundtrip tests are updated.