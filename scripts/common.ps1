Set-StrictMode -Version Latest

function Invoke-FdCommand {
    param(
        [Parameter(Mandatory = $true)][string] $Program,
        [Parameter(Mandatory = $true)][string[]] $Arguments
    )
    # Windows PowerShell turns a native program's stderr line into a terminating error under 'Stop' when output is
    # redirected; the exit code is the failure signal, so warnings (such as CMake's for vendored OSQP) stay output.
    $preference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    # A stderr line arrives as an error record; its text is the exception's message, which is empty for a blank line.
    try {
        & $Program @Arguments 2>&1 | ForEach-Object {
            if ($_ -is [System.Management.Automation.ErrorRecord]) { "$($_.Exception.Message)" } else { "$_" }
        }
    }
    finally { $ErrorActionPreference = $preference }
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $Program $($Arguments -join ' ')"
    }
}

function Find-FdCMake {
    $command = Get-Command cmake -CommandType Application -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }

    $programFilesX86 = [Environment]::GetFolderPath('ProgramFilesX86')
    $vswhere = Join-Path $programFilesX86 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path -LiteralPath $vswhere -PathType Leaf) {
        $installations = & $vswhere -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($LASTEXITCODE -ne 0) { throw 'Visual Studio discovery failed.' }
        foreach ($installation in $installations) {
            $candidate = Join-Path $installation 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate }
        }
    }
    throw 'CMake was not found. Add CMake to PATH, or install Visual Studio 2022 Desktop development with C++ including CMake tools and a Windows SDK.'
}

function Get-FdQtLayout {
    param([Parameter(Mandatory = $true)][string] $RepositoryRoot)
    $version = '6.8.3'
    $root = Join-Path $RepositoryRoot '.tools\Qt'
    [PSCustomObject]@{
        Version = $version
        Architecture = 'win64_msvc2022_64'
        Root = $root
        Prefix = Join-Path $root "$version\msvc2022_64"
        Archives = @('qtbase', 'qtdeclarative', 'qtsvg', 'qttools', 'qttranslations')
        Modules = @('qtquick3d', 'qtshadertools', 'qtquicktimeline', 'qtmultimedia')
        CMakeModules = @('Core', 'Gui', 'Qml', 'Quick', 'QuickControls2', 'Quick3D', 'ShaderTools', 'QuickTimeline', 'Multimedia', 'Svg')
    }
}

function Test-FdQtInstallation {
    param([Parameter(Mandatory = $true)] $Layout)
    foreach ($module in $Layout.CMakeModules) {
        $directory = Join-Path $Layout.Prefix "lib\cmake\Qt6$module"
        $config = Join-Path $directory "Qt6${module}Config.cmake"
        $versionFile = Join-Path $directory "Qt6${module}ConfigVersionImpl.cmake"
        if (-not (Test-Path -LiteralPath $config -PathType Leaf) -or
            -not (Test-Path -LiteralPath $versionFile -PathType Leaf)) { return $false }
        $pattern = 'set\(PACKAGE_VERSION "' + [regex]::Escape($Layout.Version) + '"\)'
        if (-not (Select-String -LiteralPath $versionFile -Pattern $pattern -Quiet)) { return $false }
    }
    foreach ($relative in @('bin\Qt6Core.dll', 'bin\Qt6Quick.dll', 'bin\Qt6Quick3D.dll',
        'bin\Qt6Multimedia.dll', 'plugins\platforms\qwindows.dll', 'qml\QtQuick\qmldir',
        'qml\QtQuick3D\qmldir', 'qml\QtQuick\Timeline\qmldir', 'qml\QtMultimedia\qmldir')) {
        if (-not (Test-Path -LiteralPath (Join-Path $Layout.Prefix $relative) -PathType Leaf)) { return $false }
    }
    return $true
}
