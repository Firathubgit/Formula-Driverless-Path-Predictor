<#
.SYNOPSIS
Configure, build, and test the Windows C++ core or native desktop application.
.EXAMPLE
.\scripts\build.ps1
.EXAMPLE
.\scripts\build.ps1 -Desktop
#>
[CmdletBinding()]
param([switch] $Desktop)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$cmake = Find-FdCMake
$ctest = Join-Path (Split-Path -Parent $cmake) 'ctest.exe'
if (-not (Test-Path -LiteralPath $ctest -PathType Leaf)) {
    throw "CTest was not found beside CMake: $ctest. Install the complete CMake distribution."
}
$preset = 'windows-core'
if ($Desktop) {
    $preset = 'windows-desktop'
    $qt = Get-FdQtLayout -RepositoryRoot $repositoryRoot
    if (-not (Test-FdQtInstallation -Layout $qt)) {
        throw 'The project Qt 6.8.3 kit is missing or incomplete. Run .\scripts\bootstrap-qt.ps1 first.'
    }
}

Push-Location -LiteralPath $repositoryRoot
try {
    Write-Host "Configuring $preset with $cmake"
    Invoke-FdCommand -Program $cmake -Arguments @('--preset', $preset)
    Invoke-FdCommand -Program $cmake -Arguments @('--build', '--preset', $preset, '--parallel')
    Invoke-FdCommand -Program $ctest -Arguments @('--preset', $preset, '--output-on-failure', '--no-tests=error')
    Write-Host "Build and tests passed: $preset"
}
finally { Pop-Location }
