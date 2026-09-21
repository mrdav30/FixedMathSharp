# Exercise the real launcher with isolated synthetic benchmarks, without adding
# deliberately failing cases to the benchmark catalog or core coverage graph.
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$repository = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$probeName = 'LauncherProbe_' + [Guid]::NewGuid().ToString('N')
$scratch = Join-Path $repository ('artifacts/benchmark-exit-codes/' + $probeName)
New-Item -ItemType Directory -Path $scratch -Force | Out-Null
[xml]$benchmarkProject = Get-Content (Join-Path $PSScriptRoot 'FixedMathSharp.Benchmarks.csproj')
$version = ($benchmarkProject.Project.ItemGroup.PackageReference |
    Where-Object Include -EQ 'BenchmarkDotNet').Version
$program = [System.Security.SecurityElement]::Escape((Join-Path $PSScriptRoot 'Program.cs'))
$catalog = [System.Security.SecurityElement]::Escape((Join-Path $PSScriptRoot 'Support/BenchmarkCatalog.cs'))
@"
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <OutputType>Exe</OutputType>
    <TargetFramework>net8.0</TargetFramework>
    <Optimize>true</Optimize>
  </PropertyGroup>
  <ItemGroup>
    <Compile Include="$program" Link="Program.cs" />
    <Compile Include="$catalog" Link="BenchmarkCatalog.cs" />
    <PackageReference Include="BenchmarkDotNet" Version="$version" />
  </ItemGroup>
</Project>
"@ | Set-Content (Join-Path $scratch "$probeName.csproj")
@'
using System;
using System.IO;
using BenchmarkDotNet.Attributes;

public class ExitProbeBenchmarks
{
    [Benchmark] public int Success() => 42;
    [Benchmark] public int Failure() => throw new InvalidOperationException("EXPECTED_CHILD_FAILURE");
}

public class PartialProbeBenchmarks
{
    [GlobalSetup]
    public void Setup()
    {
        string marker = Environment.GetEnvironmentVariable("FMS_LAUNCHER_PROBE_MARKER");
        if (File.Exists(marker))
            throw new InvalidOperationException("EXPECTED_SECOND_LAUNCH_FAILURE");
        File.WriteAllText(marker, "First launch completed setup.");
    }
    [Benchmark] public int Partial() => 42;
}

public class LateExitProbeBenchmarks
{
    [GlobalSetup]
    public void Setup() => AppDomain.CurrentDomain.ProcessExit += (_, _) => Environment.ExitCode = 23;
    [Benchmark] public int LateExit() => 42;
}
'@ | Set-Content (Join-Path $scratch 'Probes.cs')

function Assert-LauncherExit([string]$name, [int]$expected, [string[]]$arguments, [string]$evidence) {
    $log = Join-Path $scratch "$name.log"
    & dotnet $script:runner @arguments *> $log
    $actual = $LASTEXITCODE
    if ($evidence -and -not (Select-String -LiteralPath $log -SimpleMatch $evidence -Quiet)) {
        throw "$name did not exercise the expected path; see $log"
    }
    if ($actual -ne $expected) {
        throw "$name returned $actual; expected $expected. See $log"
    }
    Write-Host "PASS $name (exit $actual)"
}

$previousMarker = $env:FMS_LAUNCHER_PROBE_MARKER
Push-Location $scratch
try {
    & dotnet build "$probeName.csproj" -c Release *> build.log
    if ($LASTEXITCODE -ne 0) { throw "Probe build failed; see $scratch/build.log" }
    $script:runner = Join-Path $scratch "bin/Release/net8.0/$probeName.dll"
    $dry = @('-j', 'Dry', '--artifacts', (Join-Path $scratch 'bdn'))
    Assert-LauncherExit 'alias-failure' 1 (@('exit-probe', '--filter', '*Failure*') + $dry) 'EXPECTED_CHILD_FAILURE'
    Assert-LauncherExit 'all-failure' 1 (@('all', '--filter', '*Failure*') + $dry) 'EXPECTED_CHILD_FAILURE'
    Assert-LauncherExit 'direct-failure' 1 (@('--filter', '*Failure*') + $dry) 'EXPECTED_CHILD_FAILURE'
    Assert-LauncherExit 'success' 0 (@('exit-probe', '--filter', '*Success*') + $dry) 'WorkloadResult'
    $env:FMS_LAUNCHER_PROBE_MARKER = Join-Path $scratch 'first-launch.txt'
    Assert-LauncherExit 'partial-failure' 1 (@('partial-probe', '--launchCount', '3') + $dry) 'EXPECTED_SECOND_LAUNCH_FAILURE'
    if (-not (Select-String -LiteralPath partial-failure.log -SimpleMatch 'WorkloadResult' -Quiet)) {
        throw 'Partial failure did not retain results from a successful first launch.'
    }
    Assert-LauncherExit 'late-exit-failure' 1 (@('late-exit-probe') + $dry) 'WorkloadResult'
    if (-not (Select-String -LiteralPath late-exit-failure.log -SimpleMatch 'has exited with code 23' -Quiet)) {
        throw 'Late failure did not produce the expected nonzero child exit.'
    }
    Assert-LauncherExit 'validation-failure' 1 (@('exit-probe', '--filter', '*Success*', '--invocationCount', '1', '--unrollFactor', '16') + $dry) 'InvocationCount'
    Assert-LauncherExit 'list' 0 @('list') 'exit-probe'
    Assert-LauncherExit 'bdn-list' 0 @('all', '--list', 'flat') 'ExitProbeBenchmarks.Success'
    Assert-LauncherExit 'help' 0 @('help') 'Usage:'
    Assert-LauncherExit 'unknown-alias' 1 @('missing-alias') 'Unknown benchmark selection'
    Write-Host "Launcher checks passed. Logs: $scratch"
}
finally {
    $env:FMS_LAUNCHER_PROBE_MARKER = $previousMarker
    Pop-Location
}
