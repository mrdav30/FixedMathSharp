param (
    [string]$OutputPath = "UnityPackages",
	[string]$UnityVersion = "2022.3.20f1"
)

# Import shared functions
Set-Location (Split-Path $MyInvocation.MyCommand.Path)
. .\utilities.ps1

# Locate solution directory and switch to it
$solutionDir = Get-SolutionDirectory
Set-Location $solutionDir

$packageName = "FixedMathSharp.$env:GitVersion_FullSemVer.unitypackage"
$packagePath = "$solutionDir\$OutputPath\$packageName"

$fixedMathSharpPluginsPath = "$solutionDir\src\FixedMathSharp.Editor\bin\Release\net48"
$unityProjectPath = "$solutionDir\FMS_UnityProject"
$unityAssetsPath = "$unityProjectPath\Assets\FixedMathSharp"
$unityPluginsPath = "$unityAssetsPath\Plugins"

# Ensure a fresh Unity project by deleting the directory if it exists
if (Test-Path $unityProjectPath) {
    Write-Host "Deleting existing Unity project directory at $unityProjectPath..."
    Remove-Item -Recurse -Force $unityProjectPath
}

# Recreate necessary folders
@($unityAssetsPath, $unityPluginsPath, $packagePath) | ForEach-Object {
    if (-Not (Test-Path $_)) { New-Item -ItemType Directory -Path $_ }
}

# Ensure GitVersion environment variables are set
Ensure-GitVersion-Environment $UnityVersion

# Build the project with the version information applied
Build-Project -Configuration "Release"

# Copy DLLs (including PDB and XML) to Unity Plugins folder
Copy-Item "$fixedMathSharpPluginsPath\*" $unityPluginsPath -Recurse -ErrorAction SilentlyContinue

# Copy Unity editor-specific scripts to the Assets folder
Copy-Item "$solutionDir\src\FixedMathSharp.Editor\Editor" $unityAssetsPath -Recurse -ErrorAction SilentlyContinue

$unityExePath = "C:\Program Files\Unity\Hub\Editor\$env:UnityVersion\Editor\Unity.exe"
$unityArgs = @(
    "-quit",               # Quit after the operation completes
    "-batchmode",           # Run in batch mode (no UI)
    "-projectPath", "$unityProjectPath",  # Path to the Unity project
    "-exportPackage", "Assets", "$packagePath"
)

Write-Host "Packing to $packagePath..."

# Run Unity to create the package
Start-Process $unityExePath -ArgumentList $unityArgs -Wait -NoNewWindow

if ($LASTEXITCODE -ne 0) {
    Write-Host "Package creation failed." -ForegroundColor Red
    exit 1
}

Write-Host "Package $packageName created successfully!" -ForegroundColor Green


