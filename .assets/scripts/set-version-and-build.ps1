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