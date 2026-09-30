<#
.SYNOPSIS
Run the built desktop application with the project-local Qt kit.
.EXAMPLE
.\scripts\run.ps1
#>
[CmdletBinding()]
param([Parameter(ValueFromRemainingArguments = $true)][string[]] $ApplicationArguments = @())

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$executable = Join-Path $repositoryRoot 'build\desktop\Release\fd_desktop.exe'
if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
    throw 'The desktop executable is missing. Run .\scripts\bootstrap-qt.ps1, then .\scripts\build.ps1 -Desktop.'
}
$qt = Get-FdQtLayout -RepositoryRoot $repositoryRoot
if (-not (Test-FdQtInstallation -Layout $qt)) {
    throw 'The project Qt 6.8.3 kit is missing or incomplete. Run .\scripts\bootstrap-qt.ps1.'
}

# Keep the invoking shell unchanged, including when the application exits with an error.
$environmentNames = @('PATH', 'QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH', 'QML_IMPORT_PATH', 'QML2_IMPORT_PATH')
$savedEnvironment = @{}
foreach ($name in $environmentNames) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
Push-Location -LiteralPath $repositoryRoot
try {
    $env:PATH = (Join-Path $qt.Prefix 'bin') + [IO.Path]::PathSeparator + $savedEnvironment['PATH']
    $env:QT_PLUGIN_PATH = Join-Path $qt.Prefix 'plugins'
    $env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path $qt.Prefix 'plugins\platforms'
    $env:QML_IMPORT_PATH = Join-Path $qt.Prefix 'qml'
    $env:QML2_IMPORT_PATH = $env:QML_IMPORT_PATH
    & $executable @ApplicationArguments | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "Desktop application exited with code $LASTEXITCODE." }
}
finally {
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process')
    }
    Pop-Location
}


