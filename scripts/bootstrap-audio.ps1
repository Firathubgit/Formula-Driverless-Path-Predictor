<#
.SYNOPSIS
Fetch the pinned, licensed Speed Dreams engine and tire recordings for the desktop.
.DESCRIPTION
Verifies SHA-256 before replacing each file. Media stays outside Git in artifacts/audio.
No packages or system software are installed. CMake embeds these WAVs so the app works offline.
#>
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$manifest = Get-Content -LiteralPath (Join-Path $repo 'assets/audio-licenses/sources.json') -Raw | ConvertFrom-Json
$destination = Join-Path $repo 'artifacts/audio'
New-Item -ItemType Directory -Path $destination -Force | Out-Null
foreach ($asset in $manifest.files) {
    $file = Join-Path $destination $asset.file
    if ((Test-Path -LiteralPath $file) -and (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -eq $asset.sha256) {
        Write-Host "Verified $($asset.file)"
        continue
    }
    $download = "$file.download"
    Invoke-WebRequest -Uri ($manifest.base + $asset.path) -OutFile $download
    if ((Get-FileHash -LiteralPath $download -Algorithm SHA256).Hash -ne $asset.sha256) {
        Remove-Item -LiteralPath $download
        throw "Checksum mismatch for $($asset.file); existing file preserved."
    }
    Move-Item -LiteralPath $download -Destination $file -Force
    Write-Host "Downloaded and verified $($asset.file)"
}
