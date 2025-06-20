param (
    [string]$BuildType = "Release"
)

# Import shared functions
Set-Location (Split-Path $MyInvocation.MyCommand.Path)
. .\utilities.ps1

# Locate solution directory and switch to it
$solutionDir = Get-SolutionDirectory
Set-Location $solutionDir

# Ensure GitVersion environment variables are set
Ensure-GitVersion-Environment

# Build the project with the version information applied
Build-Project -Configuration $BuildType

# Output directory
$releaseDir = Join-Path $solutionDir "src\$solutionDir\bin\Release"

# Ensure release directory exists
if (Test-Path $releaseDir) {
    Get-ChildItem -Path $releaseDir -Directory | ForEach-Object {
        $targetDir = $_.FullName
        $frameworkName = $_.Name

        # Construct final archive name
        $zipFileName = "${solutionDir}-v${GitVersion_FullSemVer}-${frameworkName}-release.zip"
        $zipPath = Join-Path $releaseDir $zipFileName

        Write-Host "Creating archive: $zipPath"

        if (Test-Path $zipPath) {
            Remove-Item $zipPath -Force
        }

        Compress-Archive -Path "$targetDir\*" -DestinationPath $zipPath -Force

        if (Test-Path $zipPath) {
            Write-Host "✔ Archive created for $frameworkName"
        } else {
            Write-Warning "⚠ Failed to create archive for $frameworkName"
        }
    }
} else {
    Write-Warning "⚠ Release directory not found: $releaseDir"
}
