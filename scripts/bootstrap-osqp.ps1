<#
.SYNOPSIS
Download this project's pinned OSQP v1.0.0 source and its qdldl v0.1.8 dependency into .tools\osqp.
.DESCRIPTION
OSQP (Apache-2.0) solves the minimum-curvature racing line's quadratic program in fd_raceline (decision 0020).
Both archives come from the osqp organisation's GitHub releases and tags and are checked against pinned SHA-256
hashes before anything is extracted. qdldl is fetched here, not by OSQP's own CMake at configure time, so builds stay
offline and reproducible. Without these sources the core, desktop and headless applications still build; fd_raceline
and its tests are skipped.
.EXAMPLE
.\scripts\bootstrap-osqp.ps1
.EXAMPLE
.\scripts\bootstrap-osqp.ps1 -ToolsRoot C:\temp\fd-tools
#>
[CmdletBinding()]
param([string] $ToolsRoot)

$ErrorActionPreference = 'Stop'
$repositoryRoot = Split-Path -Parent $PSScriptRoot
if (-not $ToolsRoot) { $ToolsRoot = Join-Path $repositoryRoot '.tools' }
$root = Join-Path $ToolsRoot 'osqp'

$packages = @(
    @{
        Name = 'OSQP v1.0.0'
        Url = 'https://github.com/osqp/osqp/releases/download/v1.0.0/osqp-v1.0.0-src.tar.gz'
        Archive = 'osqp-v1.0.0-src.tar.gz'
        Sha256 = 'ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646'
        Directory = 'v1.0.0'
        # The release archive has no top-level directory.
        Inner = $null
    },
    @{
        Name = 'qdldl v0.1.8'
        Url = 'https://github.com/osqp/qdldl/archive/refs/tags/v0.1.8.tar.gz'
        Archive = 'qdldl-v0.1.8.tar.gz'
        Sha256 = 'ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa'
        Directory = 'qdldl-v0.1.8'
        Inner = 'qdldl-0.1.8'
    }
)

New-Item -ItemType Directory -Force -Path $root | Out-Null
foreach ($package in $packages) {
    $destination = Join-Path $root $package.Directory
    if (Test-Path -LiteralPath (Join-Path $destination 'CMakeLists.txt') -PathType Leaf) {
        Write-Host "$($package.Name) is already present at $destination."
        continue
    }
    $archive = Join-Path $root $package.Archive
    if (-not (Test-Path -LiteralPath $archive -PathType Leaf)) {
        Write-Host "Downloading $($package.Name) from $($package.Url)"
        Invoke-WebRequest -Uri $package.Url -OutFile $archive -UseBasicParsing
    }
    $actual = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $package.Sha256) {
        Remove-Item -LiteralPath $archive -Force
        throw "$($package.Name) archive SHA-256 $actual does not match the pinned $($package.Sha256); the download was removed."
    }
    $staging = Join-Path $root ($package.Directory + '.staging')
    if (Test-Path -LiteralPath $staging) { Remove-Item -LiteralPath $staging -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $staging | Out-Null
    & tar.exe -xzf $archive -C $staging
    if ($LASTEXITCODE -ne 0) { throw "Extracting $archive failed with exit code $LASTEXITCODE" }
    $extracted = if ($package.Inner) { Join-Path $staging $package.Inner } else { $staging }
    Move-Item -LiteralPath $extracted -Destination $destination
    if (Test-Path -LiteralPath $staging) { Remove-Item -LiteralPath $staging -Recurse -Force }
    Write-Host "$($package.Name) verified and extracted to $destination."
}
