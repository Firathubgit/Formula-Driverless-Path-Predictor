<#
.SYNOPSIS
Install this project's pinned Qt 6.8.3 MSVC 2022 kit locally using aqtinstall 3.3.0.
.DESCRIPTION
Uses an existing .venv, uv, py, or python. Downloads stay inside the repository.
Requires an existing Python interpreter; it does not install Python or Visual Studio.
.EXAMPLE
.\scripts\bootstrap-qt.ps1
.EXAMPLE
.\scripts\bootstrap-qt.ps1 -PythonPath 'C:\Python312\python.exe'
#>
[CmdletBinding()]
param([string] $PythonPath)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$qt = Get-FdQtLayout -RepositoryRoot $repositoryRoot
if (Test-FdQtInstallation -Layout $qt) {
    Write-Host "Qt $($qt.Version), including required modules, is already installed at $($qt.Prefix)."
    return
}

$venv = Join-Path $repositoryRoot '.venv'
$venvPython = Join-Path $venv 'Scripts\python.exe'
$uvCache = Join-Path $repositoryRoot '.uv-cache'
$downloadCache = Join-Path $repositoryRoot '.aqt-cache'
$uv = Get-Command uv -CommandType Application -ErrorAction SilentlyContinue
Push-Location -LiteralPath $repositoryRoot
try {
    if (-not (Test-Path -LiteralPath $venvPython -PathType Leaf)) {
        if (Test-Path -LiteralPath $venv) {
            throw 'The existing .venv has no Windows Python executable. Repair or rename that environment before retrying.'
        }
        if ($uv) {
            $arguments = @('venv', '--cache-dir', $uvCache, '--no-python-downloads')
            if ($PythonPath) { $arguments += @('--python', $PythonPath) }
            $arguments += $venv
            Invoke-FdCommand -Program $uv.Source -Arguments $arguments
        }
        else {
            $pythonArguments = @()
            if ($PythonPath) {
                $python = Get-Command $PythonPath -CommandType Application -ErrorAction SilentlyContinue
                if (-not $python) { throw "Python executable not found: $PythonPath" }
            }
            else {
                $python = Get-Command py -CommandType Application -ErrorAction SilentlyContinue
                if ($python) { $pythonArguments = @('-3') }
                else { $python = Get-Command python -CommandType Application -ErrorAction SilentlyContinue }
            }
            if (-not $python) {
                throw 'Python was not found. Install Python 3.9 or newer, or pass -PythonPath to an existing interpreter.'
            }
            Invoke-FdCommand -Program $python.Source -Arguments ($pythonArguments + @('-m', 'venv', $venv))
        }
    }

    if ($uv) {
        Invoke-FdCommand -Program $uv.Source -Arguments @('pip', 'install', '--python', $venvPython,
            '--cache-dir', $uvCache, 'aqtinstall==3.3.0')
    }
    else {
        Invoke-FdCommand -Program $venvPython -Arguments @('-m', 'pip', 'install', '--disable-pip-version-check',
            '--cache-dir', (Join-Path $downloadCache 'pip'), 'aqtinstall==3.3.0')
    }
    New-Item -ItemType Directory -Force -Path $downloadCache | Out-Null
    # aqt writes a log in its working directory; keep that generated output with the cache.
    Push-Location -LiteralPath $downloadCache
    try {
        $arguments = @('-m', 'aqt', 'install-qt', 'windows', 'desktop', $qt.Version, $qt.Architecture,
            '--outputdir', $qt.Root, '--archive-dest', $downloadCache, '--keep', '--archives') +
            $qt.Archives + @('--modules') + $qt.Modules
        Invoke-FdCommand -Program $venvPython -Arguments $arguments
    }
    finally { Pop-Location }
    if (-not (Test-FdQtInstallation -Layout $qt)) {
        throw 'aqt finished, but the required Qt version/module files are incomplete. Inspect .aqt-cache/aqtinstall.log and retry.'
    }
    Write-Host "Qt $($qt.Version) is ready. Next: .\scripts\build.ps1 -Desktop"
}
finally { Pop-Location }
