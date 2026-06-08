# Benchmark Regression Follow-Up Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:systematic-debugging before changing runtime code for any
> benchmark-discovered issue, use superpowers:test-driven-development for
> correctness fixes, and use superpowers:verification-before-completion before
> claiming a phase is complete. Steps use checkbox (`- [ ]`) syntax for
> tracking.

**Status:** Active

**Goal:** Convert the completed benchmark scaffold into repeatable performance
regression practice without adding noisy CI gates or unsupported optimization
claims.

**Architecture:** Keep `tests/FixedMathSharp.Benchmarks` as the evidence lab,
keep `docs/feature-work/issue-tracker.md` as the tooling/correctness issue
queue, and use full `Release` BenchmarkDotNet artifacts for performance claims.
Short in-process runs remain smoke checks only.

**Tech Stack:** `netstandard2.1` and `net8.0` runtime targets, .NET 8
BenchmarkDotNet runner, xUnit, MemoryPack, System.Text.Json, `Release`, and
`ReleaseLean`.

---

## Phase 1: Capture A Fresh Full Baseline

**Files:**

- Review: `tests/FixedMathSharp.Benchmarks/README.md`
- Review: `BenchmarkDotNet.Artifacts/results/`

- [x] Run a full `Release` benchmark export after the Phase 1-6 scaffold
  changes are committed.
- [x] Preserve the JSON artifacts under a dated local archive or agreed docs
  location.
- [x] Record the SDK/runtime, OS, CPU, power mode, and command used.
- [x] Do not compare against short in-process smoke numbers.

Phase 1 result on 2026-06-04: completed. The full `Release` run executed 136
benchmarks in 00:52:05 and exited successfully.

Verification:

```bash
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0
dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --exporters json
```

## Phase 2: Compare And Prioritize Real Hot Spots

**Files:**

- Review: `BenchmarkDotNet.Artifacts/results/*-report-github.md`
- Review: `docs/feature-work/issue-tracker.md`

- [x] Compare speed, allocation, GC counts, branch/data movement, and
  algorithmic complexity by subsystem.
- [x] Separate intentional serialization payload allocation from accidental
  hot-path allocation.
- [x] Promote confirmed defects or tooling failures to
  `docs/feature-work/issue-tracker.md`.
- [x] Create focused optimization plans only for measured bottlenecks with a
  clear correctness test strategy.

Phase 2 result on 2026-06-04: completed. The full baseline confirmed that most
runtime hot paths are allocation-free; the only non-serialization allocation was
`BoundsBenchmarks.FrustumCreateFromMatrix` at 165,888 B per benchmark operation,
which comes from `FixedBoundFrustum` being a sealed reference type with internal
arrays. Serialization allocation was classified separately as payload/serializer
cost. Measured optimization follow-up work was extracted to
`docs/feature-work/2026-06-04-benchmark-hotspot-optimization-plan.md`.
`FMS-Issue-008` was updated for the BenchmarkDotNet generated-project restore
case discovered while capturing the baseline.

## Phase 3: Publish A Bencher Release Snapshot

**Files:**

- Review: `.github/workflows/build-and-test.yml`
- Review: `.github/workflows/coverage.yml`
- Create: `.github/workflows/bencher-snapshot.yml`
- Create: `.bencher/Dockerfile`
- Create: `.bencher/run-release-snapshot.sh`
- Modify: `tests/FixedMathSharp.Benchmarks/README.md`
- Modify: `README.md`

Adopt a release-snapshot model, not continuous benchmarking. The benchmark
workflow should mirror `.github/workflows/coverage.yml`: it runs only after
`build-and-test` completes successfully on `main`, so benchmark publishing
happens from merged trusted code. GitHub Actions is orchestration only; the
timed benchmark run should execute through Bencher's bare-metal image path
instead of on the shared Actions runner.

- [ ] Keep `.github/workflows/build-and-test.yml` as the required PR gate and
  continue relying on the solution build to catch benchmark compilation
  failures. Do not run or publish benchmark measurements from PR workflows.
- [ ] Configure the Bencher project and secrets before wiring the workflow:
  - Use the existing tested Bencher project slug (`mrdav30s-project`) unless it
    is renamed before implementation.
  - Add the project API key as the repository secret `BENCHER_API_KEY`.
  - Do not commit, paste, echo, or log the literal key value.
  - Add a repository variable such as `BENCHER_PROJECT` if the same workflow
    pattern will be copied into other LSF libraries.
- [ ] Create `.github/workflows/bencher-snapshot.yml` with the same trusted
  trigger shape as coverage:

```yaml
name: Bencher Snapshot

on:
  workflow_run:
    workflows:
      - build-and-test
    types:
      - completed
    branches:
      - main

permissions:
  contents: read

concurrency:
  group: bencher-snapshot
  cancel-in-progress: false

jobs:
  snapshot:
    if: ${{ github.event.workflow_run.conclusion == 'success' }}
    runs-on: ubuntu-latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v6
        with:
          fetch-depth: 0
          persist-credentials: false
          ref: ${{ github.event.workflow_run.head_sha }}
```

- [ ] Install the Bencher CLI in the workflow via `bencherdev/bencher@main`.
  Local agents can add `$HOME/.cargo/bin` to `PATH` when using a
  cargo-installed CLI; the workflow should use the action so this does not
  depend on a developer shell profile.
- [ ] Build a self-contained benchmark image in `.bencher/Dockerfile`. The
  image must restore and build
  `tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj` in
  `Release` for `net8.0` at image-build time, then use
  `.bencher/run-release-snapshot.sh` as the command/entrypoint. Bencher shared
  bare-metal runners cannot fetch missing dependencies while the benchmark job
  is executing, so the image must contain everything needed to run the selected
  benchmark suite.
- [ ] Push the benchmark image to the Bencher OCI registry with a tag tied to
  `${{ github.event.workflow_run.head_sha }}`. Prefer
  `registry.bencher.dev/<project-slug>:<sha>` unless implementation verifies
  this Bencher account supports external registry image references. Confirm the
  non-interactive `docker login registry.bencher.dev` credential shape before
  landing the workflow, and keep credentials in `BENCHER_API_KEY`.
- [ ] In the workflow shell, set `BENCHER_PROJECT` from the repository variable
  or the agreed project slug, and set `BENCHER_HASH` from
  `${{ github.event.workflow_run.head_sha }}`. Do not use the workflow-run
  job's default `GITHUB_SHA` for the benchmark report hash.
- [ ] Run Bencher in publish-only mode for the main branch snapshot. Do not add
  alert thresholds, `--error-on-alert`, PR comments, or build-failing gates in
  this phase:

```bash
bencher run \
  --project "$BENCHER_PROJECT" \
  --key "$BENCHER_API_KEY" \
  --branch main \
  --hash "$BENCHER_HASH" \
  --testbed fixedmathsharp-bare-metal \
  --adapter c_sharp_dot_net \
  --average median \
  --file BenchmarkDotNet.Artifacts/results/FixedMathSharp.Benchmarks.ReleaseSnapshotBenchmarks-report-full-compressed.json \
  --image "$BENCHER_PROJECT:$BENCHER_HASH" \
  --job-timeout 300 \
  ".bencher/run-release-snapshot.sh"
```

- [ ] Verify the exact Bencher image reference syntax during implementation.
  The CLI accepts `--image`, `--file`, `--adapter c_sharp_dot_net`, `--key`,
  `--branch`, `--hash`, `--testbed`, and `--job-timeout`; keep the workflow
  command aligned with the installed Bencher CLI help and the current bare-metal
  docs.
- [ ] Craft a dedicated `release-snapshot` benchmark selection instead of
  publishing the full suite. The 2026-06-04 full run took 52 minutes, and the
  current alias classes are broad enough that choosing whole aliases blindly can
  exceed Bencher's 5-minute free-tier job timeout. Start by evaluating the
  existing aliases:
  - `fixed64-arithmetic`
  - `vector3d`
  - `matrix4x4`
  - `quaternion`
  - `bounds`
  - `deterministic-random`
  - `diagnostics-formatting`
- [ ] If whole-class aliases exceed the time budget, add a
  `ReleaseSnapshotBenchmarks` class or BenchmarkDotNet category that batches a
  small number of representative operations per benchmark method. Prefer a
  handful of high-signal scenarios over a miniature copy of every subsystem:
  scalar arithmetic/trigonometry, vector normalization/transform work,
  quaternion rotation/interpolation, matrix transform/inversion, bounds
  containment/intersection, and deterministic RNG. Exclude serialization from
  the first snapshot unless the release story needs payload/allocation data,
  because serialization allocation is intentional and can dominate the report.
- [ ] Keep the release snapshot under 240 seconds in the bare-metal job, leaving
  headroom for image startup, BenchmarkDotNet process overhead, JSON export, and
  Bencher upload within a 300-second timeout.
- [ ] Have `.bencher/run-release-snapshot.sh` run the compiled benchmark DLL,
  emit BenchmarkDotNet JSON, and fail clearly if the expected
  `ReleaseSnapshotBenchmarks` compressed JSON report is missing. Avoid
  `dotnet run` in the benchmark command; build the runner in the image and
  execute the compiled DLL.
- [ ] Document the final selected scenarios and command in
  `tests/FixedMathSharp.Benchmarks/README.md`, including why excluded aliases
  were left out of the 5-minute snapshot.
- [ ] Add a README badge once the first Bencher project/report URL is stable.
  If Bencher does not expose a dynamic badge for the chosen public project, add
  a simple static badge beside the existing badges:

```markdown
[![Benchmarks](https://img.shields.io/badge/benchmarks-Bencher-orange)](https://bencher.dev/perf/mrdav30s-project)
```

- [ ] Make the workflow copyable across the rest of the stack by parameterizing
  only the Bencher project slug, benchmark project path, image tag, and release
  snapshot alias. Keep the event model post-merge-only unless a future library
  explicitly needs continuous PR benchmarking.
- [ ] Account for the host-isolated intermediate output paths and
  configuration-scoped project extension/intermediate paths added for
  `Release`/`ReleaseLean` conditional package graphs, including the
  `BenchmarkDotNet.Autogenerated` exception needed for generated restore/build
  agreement.

This phase is intentionally informational. It publishes focused release
performance evidence for merged code without introducing PR noise, GitHub-hosted
timing claims, or release-blocking performance gates.

## Related Issue Tracker Items

- Resolved on 2026-06-04: `FMS-Issue-005`, `FMS-Issue-006`, and
  `FMS-Issue-008` were closed by making coverage opt-in for local test runs and
  isolating intermediate/NuGet assets by host platform and configuration, with a
  generated-project exception for BenchmarkDotNet boilerplate projects.
