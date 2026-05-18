# Import shared functions
Set-Location (Split-Path $MyInvocation.MyCommand.Path)
. .\utilities.ps1

# Locate solution directory and switch to it
$solutionDir = Get-SolutionDirectory
Set-Location $solutionDir

# Ensure GitVersion environment variables are set
Ensure-GitVersion-Environment

$solutionName = Split-Path $solutionDir -Leaf
$archiveRoot = Join-Path $solutionDir "artifacts"
$archiveOutputDir = Join-Path $archiveRoot "releases"

if (-not (Test-Path $archiveOutputDir)) {
    New-Item -ItemType Directory -Path $archiveOutputDir -Force | Out-Null
}

Write-Host "Archives will be written to: $archiveOutputDir"

# Build and package for each release configuration
$configurations = @("Release", "ReleaseLean")

foreach ($config in $configurations) {
    # Build the project with the current configuration
    Build-Project -Configuration $config

    # Output directory for this configuration
    $releaseDir = Join-Path $solutionDir "src\$solutionName\bin\$config"

    # Determine archive label suffix (lowercase, hyphen-separated)
    $configLabel = $config.ToLower() -replace "release", "release" # keeps "release" / "releaselean"

	if (-not (Test-Path $releaseDir)) {
		Write-Warning "Release directory not found for configuration '$config': $releaseDir"
		continue
	}

	Get-ChildItem -Path $releaseDir -Directory | ForEach-Object {
		$targetDir = $_.FullName
		$frameworkName = $_.Name

		# Construct final archive name
		$zipFileName = "${solutionName}-v$($Env:GitVersion_FullSemVer)-${frameworkName}-${configLabel}.zip"
		$zipPath = Join-Path $archiveOutputDir $zipFileName

		Write-Host "Creating archive: $zipPath"

		if (Test-Path $zipPath) {
			Remove-Item $zipPath -Force
		}

		Compress-Archive -Path "$targetDir\*" -DestinationPath $zipPath -Force

		if (Test-Path $zipPath) {
			Write-Host "Archive created for $frameworkName ($config)"
		} else {
			Write-Warning "Failed to create archive for $frameworkName ($config)"
		}
	}
}
