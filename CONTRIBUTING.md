# Contributing

For substantial API, architecture, or serialization changes, open an issue or
discussion before implementation so the intended contract is clear. Focused
fixes can proceed directly through a pull request.

Please follow the [Code of Conduct](CODE_OF_CONDUCT.md). See
[Support](.github/SUPPORT.md) for help and [Security](SECURITY.md) for private
vulnerability reports.

## Pull Request Process

1. Keep the change focused and preserve deterministic behavior, stable ordering,
   and engine-agnostic runtime boundaries.
2. Add or update tests for behavior changes and keep the README, guides, XML
   documentation, and benchmarks aligned with the affected public contract.
3. Run the `Release` build and tests. Also run `ReleaseLean` when changing
   packages, serialization, MemoryPack conditionals, or shared runtime APIs.
4. Do not hand-edit package versions for normal releases; repository workflows
   derive versions through GitVersion.
5. Exclude generated build, test, coverage, package, and benchmark artifacts
   from the pull request.

## Commit messages and versions

[`GitVersion.yml`](GitVersion.yml) recognizes Conventional Commits, including
optional scopes:

- `feat: ...` or `feat(geometry): ...` requests a minor increment.
- `fix: ...` and `perf: ...` request a patch increment.
- A breaking header such as `refactor!: ...` or `feat(api)!: ...`, or a
  `BREAKING CHANGE:` / `BREAKING-CHANGE:` footer, requests a major increment.
- Existing `+semver:` directives remain supported. Other commit types retain the
  branch's default increment; `main` defaults to Patch.

GitVersion evaluates the history since its version source, so an earlier
breaking change still determines the next major release when later commits are
fixes or documentation. Do not set a permanent minor increment on `main` or
hand-edit project package versions to select one release.

Preview the calculated version from the repository root:

```powershell
dotnet-gitversion -output json -nofetch
```

The local build/archive script exports `MajorMinorPatch` as its package version;
the preview's `FullSemVer` can also contain a prerelease suffix before tagging.
Commit the release changes before creating the matching release tag. Publishing
continues to validate that the tag and calculated package version agree.

The core and Chronicler test projects use blocking GC so their allocation guards
are not affected by background-GC allocation-buffer accounting. This applies to
the test host only; it does not configure consumers or benchmarks. Keep this
setting when changing test runners. See the
[allocation-counter investigation](docs/feature-work/done/2026-09-21-allocation-counter-accounting.md)
for the reproducer and evidence.

## Coordinated source development

Place the Chronicler checkout beside FixedMathSharp and run (repeat with
`ReleaseLean`):

```powershell
dotnet test FixedMathSharp.slnx -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1
```

This selects source Chronicler for the companion and its tests, including the
Lean shim. Package mode remains the default and is the release-validation path.
Before release, align package references with the published Chronicler versions
and validate both profiles in package mode. Source-mode identities are not proof
that the released dependency graph supplies the same APIs. Alternatively, set
`$env:UseLocalLsfStack='true'` for the PowerShell session. Keep
`-p:DisableTransitiveProjectReferences=true` on restore/build/test commands even
when selecting source mode through the environment. Source mode also selects the
local shim for the core math library's Lean build. Local development identities
only coordinate assembly resolution; generated source-mode packages are not
release artifacts.

## Branch-Specific CI Dependencies

Develop pushes and PRs targeting `develop` use `UseLocalLsfStack=true` and
immutable sibling source revisions in `build-and-test.yml`. Main and other
branches use released packages with no sibling checkouts; a `develop` -> `main`
PR validates packages. Release upstream dependencies before promotion and update
source pins deliberately as the coordinated stack changes. Source/package NuGet
caches are separated, and source graphs build serially to avoid shared-output
races. Restore, build and test also pass
`-p:DisableTransitiveProjectReferences=${{ env.UseLocalLsfStack }}` globally, so
inferred references in sibling projects cannot discard explicit assembly
identities. Serialization alone does not preserve those identities. Package mode
sets the flag to false. Both Release/Lean Windows/Linux lanes remain required.

## Documentation and cleanup

Guides live in `docs/api/guides` and publish with the API reference and coverage
through the existing Pages workflow. The GitHub Wiki is disabled; DocFX is the
published documentation home, with no wiki copies or synchronization workflow.
Keep current guides evergreen and put version-specific upgrade steps in
[Migration](docs/MIGRATION.md). Generated files under `docs/api/obj` are
ignored.

After a Release build, generate the site with:

```bash
dotnet tool restore
dotnet tool run docfx docs/api/docfx.json --warningsAsErrors
```

Run cleanup explicitly before review, using the SDK from `global.json` and
Node.js/npm for Prettier:

```bash
dotnet format FixedMathSharp.slnx --severity error
npx --yes prettier@3.9.5 --write .
```

In coordinated-source mode, set `UseLocalLsfStack=true` and
`DisableTransitiveProjectReferences=true` in the environment for the formatter,
just as for source builds. Keep formatting separate from commits; no hook
rewrites or stages files. Prettier's version is pinned in the command and its
first use can download the tool. Generated output is excluded by
`.prettierignore`. The .NET formatter follows `.editorconfig`, including
whitespace and error-level style/analyzer fixes; it does not reproduce a Visual
Studio cleanup profile.

Check without rewriting files:

```bash
dotnet format FixedMathSharp.slnx --severity error --verify-no-changes
npx --yes prettier@3.9.5 --check .
```

Each namespace-level type or delegate has its own file. Public/internal nested
types use separate files while preserving their containing types and CLR
identities; private nested helpers may stay with their owner. Authored C# files
under `src` carry the filename and MIT license header; test and benchmark files
under `tests` do not.
