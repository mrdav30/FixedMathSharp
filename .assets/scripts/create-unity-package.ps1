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

$fixedMathSharpPluginsPath = "$solutionDir\src\FixedMathSharp\bin\ReleaseUnity\net48"
$unityProjectPath = "$solutionDir\FMS_UnityProject"
$unityAssetsPath = "$unityProjectPath\Assets\Integration\FixedMathSharp"
$unityPluginsPath = "$unityAssetsPath\Plugins"
$packagePath = "$solutionDir\$OutputPath"

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
Build-Project -Configuration "ReleaseUnity"

# Copy DLLs (including PDB and XML) to Unity Plugins folder
Copy-Item "$fixedMathSharpPluginsPath\*" $unityPluginsPath -Recurse -ErrorAction SilentlyContinue

# Copy Unity editor-specific scripts to the Assets folder
Copy-Item "$solutionDir\src\FixedMathSharp\Integration\Unity\*" $unityAssetsPath -Recurse -ErrorAction SilentlyContinue

$packageName = "FixedMathSharp.$env:GitVersion_FullSemVer.unitypackage"
$unityExePath = "C:\Program Files\Unity\Hub\Editor\$env:UnityVersion\Editor\Unity.exe"
$unityArgs = @(
    "-quit",               # Quit after the operation completes
    "-batchmode",           # Run in batch mode (no UI)
    "-projectPath", "$unityProjectPath",  # Path to the Unity project
    "-exportPackage", "Assets", "$packagePath\$packageName"
)

Write-Host "Packing to $packagePath\$packageName..."

# Run Unity to create the package
Start-Process $unityExePath -ArgumentList $unityArgs -Wait -NoNewWindow

if ($LASTEXITCODE -ne 0) {
    Write-Host "Package creation failed." -ForegroundColor Red
    exit 1
}

Write-Host "Package $packageName created successfully!" -ForegroundColor Green


