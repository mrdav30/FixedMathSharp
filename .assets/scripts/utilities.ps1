function Get-SolutionDirectory {
    param ([string]$StartPath = $(Get-Location))

    $currentPath = $StartPath
    while ($true) {
        if (Test-Path (Join-Path $currentPath "FixedMathSharp.sln")) {
            return $currentPath
        }
        $parent = [System.IO.Directory]::GetParent($currentPath)
        if ($parent -eq $null) { break }
        $currentPath = $parent.FullName
    }
    throw "Solution directory not found."
}

function Ensure-GitVersion-Environment {
	# Ensure GitVersion is installed and available
	if (-not (Get-Command "dotnet-gitversion" -ErrorAction SilentlyContinue)) {
		Write-Host "GitVersion is not installed. Install it with:"
		Write-Host "dotnet tool install -g GitVersion.Tool"
		exit 1
	}

    Write-Host "Fetching version information using GitVersion..."

    # Capture GitVersion output as JSON and convert it to PowerShell objects
    $gitVersionOutput = dotnet-gitversion -output json | ConvertFrom-Json

    if ($null -eq $gitVersionOutput) {
        Write-Host "ERROR: Failed to get version information from GitVersion." -ForegroundColor Red
        exit 1
    }

    # Extract key version properties
    $semVer = $gitVersionOutput.MajorMinorPatch
    $assemblySemVer = $gitVersionOutput.AssemblySemVer
    $assemblySemFileVer = $gitVersionOutput.AssemblySemFileVer
    $infoVersion = $gitVersionOutput.InformationalVersion

    # Set environment variables for the build process
    [System.Environment]::SetEnvironmentVariable('GitVersion_FullSemVer', $semVer, 'Process')
    [System.Environment]::SetEnvironmentVariable('GitVersion_AssemblySemVer', $assemblySemVer, 'Process')
    [System.Environment]::SetEnvironmentVariable('GitVersion_AssemblySemFileVer', $assemblySemFileVer, 'Process')
    [System.Environment]::SetEnvironmentVariable('GitVersion_InformationalVersion', $infoVersion, 'Process')

    Write-Host "Environment variables set:"
    Write-Host "  GitVersion_FullSemVer = $semVer"
    Write-Host "  GitVersion_AssemblySemVer = $assemblySemVer"
    Write-Host "  GitVersion_AssemblySemFileVer = $assemblySemFileVer"
    Write-Host "  GitVersion_InformationalVersion = $infoVersion"
}

function Build-Project {
    param (
        [string]$SolutionPath = "FixedMathSharp.sln",
        [string]$Configuration = "Release"
    )

    Write-Host "Building $SolutionPath in $Configuration mode..."
	# Clean and build the project with the selected configuration
	dotnet clean
    dotnet build $SolutionPath -c $Configuration

    if ($LASTEXITCODE -ne 0) {
        Write-Host "Build failed." -ForegroundColor Red
        exit 1
    }

    Write-Host "Build succeeded!" -ForegroundColor Green
}
