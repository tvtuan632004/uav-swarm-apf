<#
.SYNOPSIS
    Set up the uav-swarm-apf environment on Windows and run every check.

.DESCRIPTION
    1. Installs uv if missing (winget first, then pip as a fallback).
    2. Installs Python 3.12 through uv (does not touch conda or system Python).
    3. Creates .venv and installs the exact versions pinned in uv.lock.
    4. Health check: the .venv interpreter must start. If it does not (its base Python
       was moved or deleted), reinstalls Python 3.12 and rebuilds .venv once.
    5. Runs ruff (lint + format check), pytest and the demo.
    Everything printed is also written to setup-report.txt in the repo root.

.PARAMETER SkipChecks
    Only install; do not run ruff and pytest.

.PARAMETER Clean
    Delete .venv and rebuild it from scratch (it only holds installed packages).

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File scripts\setup.ps1
#>
[CmdletBinding()]
param([switch]$SkipChecks, [switch]$Clean)

$ErrorActionPreference = 'Stop'
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
$Report = Join-Path $RepoRoot 'setup-report.txt'
Set-Content -Path $Report -Value ("uav-swarm-apf setup report - " + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss')) -Encoding UTF8

function Write-Report([string]$Text) {
    $Text | Out-File -FilePath $Report -Append -Encoding UTF8
    Write-Host $Text
}

function Test-Command([string]$Name) {
    return [bool](Get-Command $Name -ErrorAction SilentlyContinue)
}

function Invoke-Step([string]$Title, [string]$Exe, [string[]]$Arguments) {
    Write-Report ""
    Write-Report "=== $Title ==="
    Write-Report "> $Exe $($Arguments -join ' ')"
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'  # native tools write progress to stderr
    & $Exe @Arguments 2>&1 | ForEach-Object { "$_" } | Tee-Object -Variable lines | Out-Host
    $code = $LASTEXITCODE
    $ErrorActionPreference = $prev
    $lines | Out-File -FilePath $Report -Append -Encoding UTF8
    if ($code -ne 0) {
        Write-Report "FAILED: $Title (exit code $code)"
        throw "$Title failed (exit code $code). See setup-report.txt"
    }
}

function Update-SessionPath {
    $machine = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $user = [Environment]::GetEnvironmentVariable('Path', 'User')
    $local = Join-Path $env:USERPROFILE '.local\bin'
    $env:Path = "$machine;$user;$local"
}

# --- 1. uv ------------------------------------------------------------------
if (-not (Test-Command 'uv')) {
    Write-Report 'uv not found - installing.'
    if (Test-Command 'winget') {
        Invoke-Step 'Install uv (winget)' 'winget' @('install', '--id', 'astral-sh.uv', '-e')
    }
    elseif (Test-Command 'python') {
        Invoke-Step 'Install uv (pip --user)' 'python' @('-m', 'pip', 'install', '--user', 'uv')
    }
    else {
        throw 'Neither winget nor python found. Install uv by hand: https://docs.astral.sh/uv/getting-started/installation/'
    }
    Update-SessionPath
    if (-not (Test-Command 'uv')) {
        throw 'uv was installed but is not on PATH yet. Close this window, open a new one and run the script again.'
    }
}
Invoke-Step 'uv version' 'uv' @('--version')

# --- 2-3. Python 3.12 + locked environment -----------------------------------
Write-Report ''
Write-Report '=== Diagnostics ==='
Write-Report "UV_PYTHON_INSTALL_DIR = '$env:UV_PYTHON_INSTALL_DIR'"
Write-Report "UV_PYTHON             = '$env:UV_PYTHON'"
Write-Report "VIRTUAL_ENV           = '$env:VIRTUAL_ENV'"
Write-Report "CONDA_DEFAULT_ENV     = '$env:CONDA_DEFAULT_ENV'"
$cfg = Join-Path $RepoRoot '.venv\pyvenv.cfg'
if (Test-Path $cfg) { Get-Content $cfg | ForEach-Object { Write-Report ".venv: $_" } }

$VenvDir = Join-Path $RepoRoot '.venv'
if ($Clean -and (Test-Path $VenvDir)) {
    Write-Report 'Clean: removing .venv'
    Remove-Item -Recurse -Force $VenvDir
}

Invoke-Step 'Python 3.12 (managed by uv)' 'uv' @('python', 'install', '3.12')
Invoke-Step 'Sync .venv from uv.lock' 'uv' @('sync', '--locked')

function Test-Venv {
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $out = & uv run --no-sync python -c "import sys, uavapf; print(sys.executable, sys.version.split()[0])" 2>&1 | ForEach-Object { "$_" }
    $ok = ($LASTEXITCODE -eq 0)
    $ErrorActionPreference = $prev
    $out | ForEach-Object { Write-Report "  $_" }
    return $ok
}

Write-Report ''
Write-Report '=== Health check: .venv interpreter ==='
if (-not (Test-Venv)) {
    Write-Report 'Broken .venv (its base Python is missing). Reinstalling Python 3.12 and rebuilding .venv.'
    if (Test-Path $VenvDir) { Remove-Item -Recurse -Force $VenvDir }
    Invoke-Step 'Reinstall Python 3.12' 'uv' @('python', 'install', '3.12', '--reinstall')
    Invoke-Step 'Rebuild .venv from uv.lock' 'uv' @('sync', '--locked')
    Write-Report '=== Health check (after rebuild) ==='
    if (-not (Test-Venv)) {
        Write-Report 'FAILED: .venv still broken. Send setup-report.txt to Claude.'
        throw '.venv interpreter still does not start. See setup-report.txt'
    }
}
Write-Report 'OK'

# --- pytest temp dir guard ---------------------------------------------------
# A sandboxed agent (e.g. Codex) can leave %TEMP%\pytest-of-<user> with an ACL the
# normal account cannot read. pytest then fails every test that uses tmp_path.
$PytestTemp = Join-Path ([IO.Path]::GetTempPath()) ('pytest-of-' + $env:USERNAME)
$blocked = $false
if (Test-Path -LiteralPath $PytestTemp) {
    try { Get-ChildItem -LiteralPath $PytestTemp -ErrorAction Stop | Out-Null } catch { $blocked = $true }
}
if ($blocked) {
    $LocalTemp = Join-Path $RepoRoot '.pytest_tmp'
    New-Item -ItemType Directory -Force -Path $LocalTemp | Out-Null
    $env:PYTEST_DEBUG_TEMPROOT = $LocalTemp
    Write-Report ''
    Write-Report "WARNING: '$PytestTemp' is not readable by $env:USERNAME (left by another agent's sandbox)."
    Write-Report "Using PYTEST_DEBUG_TEMPROOT=$LocalTemp for this run. Permanent fix: README section 12."
}

# --- 4. Checks --------------------------------------------------------------
if (-not $SkipChecks) {
    Invoke-Step 'Lint (ruff check)' 'uv' @('run', 'ruff', 'check', '.')
    Invoke-Step 'Format (ruff format --check)' 'uv' @('run', 'ruff', 'format', '--check', '.')
    Invoke-Step 'Tests (pytest)' 'uv' @('run', 'pytest', '-q')
    Invoke-Step 'Demo (scripts/demo.py)' 'uv' @('run', 'python', 'scripts/demo.py')
}

Write-Report ''
Write-Report 'ALL DONE - environment ready.'
