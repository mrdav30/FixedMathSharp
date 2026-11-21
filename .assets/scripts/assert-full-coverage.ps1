<#
.SYNOPSIS
Requires full reported line, branch and executable-method coverage for the CI coverage assemblies.
.PARAMETER CoverageDirectory
Fresh Coverlet results directory from the core test project, for one configuration.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CoverageDirectory
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Read-Count([System.Xml.XmlElement]$Element, [string]$Name) {
    [long]$value = 0
    if (-not [long]::TryParse($Element.GetAttribute($Name), [ref]$value) -or $value -lt 0) {
        throw "Missing or invalid coverage count '$Name'."
    }
    return $value
}

$expectedAssemblies = @('FixedMathSharp', 'FixedMathSharp.FluentAssertions')
$reports = @(Get-ChildItem -LiteralPath $CoverageDirectory -Filter coverage.cobertura.xml -File -Recurse | Sort-Object FullName)
if ($reports.Count -eq 0) { throw 'No Cobertura coverage reports found.' }

foreach ($report in $reports) {
    [xml]$document = Get-Content -LiteralPath $report.FullName -Raw
    $coverage = $document.DocumentElement
    if ($coverage.Name -ne 'coverage') { throw "Invalid Cobertura report: $($report.FullName)" }

    $lines = Read-Count $coverage 'lines-valid'
    $coveredLines = Read-Count $coverage 'lines-covered'
    $branches = Read-Count $coverage 'branches-valid'
    $coveredBranches = Read-Count $coverage 'branches-covered'
    if ($lines -le 0 -or $branches -le 0 -or $coveredLines -ne $lines -or $coveredBranches -ne $branches) {
        throw "Coverage gate failed in $($report.FullName): lines $coveredLines/$lines; branches $coveredBranches/$branches."
    }

    foreach ($name in $expectedAssemblies) {
        $packages = @($coverage.SelectNodes('packages/package') | Where-Object { $_.GetAttribute('name') -ceq $name })
        if ($packages.Count -ne 1) { throw "Expected one coverage package for $name in $($report.FullName)." }
        $package = $packages[0]
        $methods = @($package.SelectNodes('classes/class/methods/method[lines/line]'))
        if ($methods.Count -eq 0) { throw "No executable methods reported for $name." }

        foreach ($line in $package.SelectNodes('classes/class/lines/line | classes/class/methods/method/lines/line')) {
            if ((Read-Count $line 'hits') -eq 0) {
                throw "Uncovered line in ${name}: $($line.GetAttribute('number'))."
            }
            if ($line.GetAttribute('branch') -eq 'true' -and
                $line.GetAttribute('condition-coverage') -notmatch '^100% \(([1-9][0-9]*)/\1\)$') {
                throw "Uncovered or invalid branch in ${name}: $($line.GetAttribute('number'))."
            }
        }
        Write-Output "${name}: full reported line/branch coverage; $($methods.Count) executable methods covered."
    }
}
