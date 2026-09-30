<#
.SYNOPSIS
Rebuild the four-car Blender project, render every animation, and verify/package the clips.
.EXAMPLE
./scripts/build-showroom.ps1 -Python 'C:/path/to/python-with-pillow.exe' -FFmpeg 'C:/path/to/ffmpeg.exe'
#>
[CmdletBinding()]
param(
    [string] $Blender = 'C:/Program Files/Blender Foundation/Blender 4.5/blender.exe',
    [string] $Python = '',
    [string] $FFmpeg = '',
    [ValidateSet('black-vault', 'gt-studio')][string] $Look = 'gt-studio',
    [string] $OutputRoot = 'artifacts/showroom',
    [ValidateRange(640, 3840)][int] $Width = 1280,
    [ValidateRange(16, 512)][int] $Samples = 32,
    [switch] $SkipPrepare,
    [switch] $SkipRender,
    [switch] $PreviewFirst
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$showroomRoot = Split-Path -Parent $PSScriptRoot
$generatedRoot = [IO.Path]::GetFullPath((Join-Path $showroomRoot $OutputRoot))
$carsDir = Join-Path $generatedRoot 'cars'
$studioDir = Join-Path $generatedRoot 'studio'
$mediaDir = Join-Path $generatedRoot 'media'
if (-not (Test-Path -LiteralPath $Blender -PathType Leaf)) { throw 'Pass -Blender with the installed Blender binary.' }
if ($Width % 32 -ne 0) { throw 'Width must be a multiple of 32 for even 16:9 H.264 dimensions.' }
if (-not $Python) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
    if (Test-Path -LiteralPath $bundledPython) { $Python = $bundledPython }
    else { $Python = (Get-Command python -ErrorAction Stop).Source }
}
Invoke-FdCommand -Program $Python -Arguments @('-c', 'from PIL import Image')
Push-Location -LiteralPath $showroomRoot
try {
    foreach ($car in @('amr23', 'jesko', 'urus', 'rb19')) {
        if (-not $SkipPrepare) {
            Invoke-FdCommand -Program $Blender -Arguments @('-b', '--factory-startup', '--python-exit-code', '1',
                '--python', (Join-Path $showroomRoot 'tools/showroom/prepare_cars.py'), '--', '--car', $car,
                '--out', $carsDir)
        }
        $renderArgs = @('-b', '--factory-startup', '--python-exit-code', '1', '--python',
            (Join-Path $showroomRoot 'tools/showroom/build_showroom.py'), '--', '--car', $car,
            '--source', (Join-Path $carsDir "$car.blend"), '--out',
            $studioDir, '--look', $Look, '--width', "$Width", '--samples', "$Samples")
        if (-not $SkipRender) {
            $renderArgs += '--render'
            if ($PreviewFirst) { $renderArgs += '--preview-first' }
        }
        Invoke-FdCommand -Program $Blender -Arguments $renderArgs
    }
    Invoke-FdCommand -Program $Blender -Arguments @('-b', '--factory-startup', '--python-exit-code', '1',
        '--python', (Join-Path $showroomRoot 'tools/showroom/assemble_project.py'), '--',
        '--studio', $studioDir, '--out', (Join-Path $generatedRoot 'Black_Showroom.blend'))
    if (-not $SkipRender) {
        $packageArgs = @((Join-Path $showroomRoot 'tools/showroom/package_media.py'), '--overview',
            '--frames', (Join-Path $studioDir 'frames'), '--out', $mediaDir)
        if ($FFmpeg) { $packageArgs += @('--ffmpeg', $FFmpeg) }
        Invoke-FdCommand -Program $Python -Arguments $packageArgs
    }
}
finally { Pop-Location }
