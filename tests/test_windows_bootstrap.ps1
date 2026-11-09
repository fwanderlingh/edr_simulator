<# Run directly in Windows PowerShell; no Python or test framework required. #>
param([string]$ScratchDirectory = [IO.Path]::GetTempPath())
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$simulatorRoot = Split-Path -Parent $PSScriptRoot
$parseErrors = $null
$tokens = $null
$ast = [Management.Automation.Language.Parser]::ParseFile(
    (Join-Path $simulatorRoot 'start-simulator.ps1'), [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count) { throw 'Launcher failed to parse' }
foreach ($definition in $ast.FindAll({ param($node)
    $node -is [Management.Automation.Language.FunctionDefinitionAst]
}, $true)) {
    . ([scriptblock]::Create($definition.Extent.Text))
}
# Keep this test useful on machines that happen to provide the cmdlet.
function Get-FileHash { throw 'The bootstrap must work without Get-FileHash' }
$releaseAssignment = $ast.Find({ param($node)
    $node -is [Management.Automation.Language.AssignmentStatementAst] -and
    $node.Left.Extent.Text -eq '$mambaRelease'
}, $true)
. ([scriptblock]::Create($releaseAssignment.Extent.Text))
$scratchRoot = [IO.Path]::GetFullPath($ScratchDirectory)
$testRoot = Join-Path $scratchRoot ('edr-bootstrap-' + [guid]::NewGuid().ToString('N'))
$mambaExecutable = Join-Path $testRoot "Library/bin/micromamba-$mambaRelease.exe"
$process = $null
try {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $mambaExecutable) | Out-Null
    $cachedManager = Join-Path $simulatorRoot ".tools/Library/bin/micromamba-$mambaRelease.exe"
    if (Test-Path -LiteralPath $cachedManager) {
        Copy-Item -LiteralPath $cachedManager -Destination $mambaExecutable
    }
    Initialize-EnvironmentManager
    & $mambaExecutable --version
    if ($LASTEXITCODE -ne 0) { throw 'Private micromamba did not start' }

    # Keep micromamba alive while inspecting the libraries loaded by Windows.
    $env:MAMBA_ROOT_PREFIX = Join-Path $testRoot 'root'
    $env:CONDA_PKGS_DIRS = Join-Path $testRoot 'pkgs'
    $prefix = Join-Path $testRoot 'empty-environment'
    & $mambaExecutable create --yes --no-rc --offline --prefix $prefix
    if ($LASTEXITCODE -ne 0) { throw 'Could not create an empty test environment' }
    $startInfo = New-Object Diagnostics.ProcessStartInfo
    $startInfo.FileName = $mambaExecutable
    $startInfo.Arguments = "--no-rc run --prefix `"$prefix`" powershell.exe -NoProfile -Command `"Start-Sleep -Seconds 5`""
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.WindowStyle = [Diagnostics.ProcessWindowStyle]::Hidden
    $startInfo.RedirectStandardError = $true
    $process = [Diagnostics.Process]::Start($startInfo)
    Start-Sleep -Milliseconds 500
    if ($process.HasExited) {
        throw "micromamba exited before module inspection: $($process.StandardError.ReadToEnd())"
    }
    $runningManager = Get-Process -Id $process.Id
    $managerDirectory = [IO.Path]::GetFullPath((Split-Path -Parent $mambaExecutable))
    foreach ($name in @('msvcp140.dll', 'vcruntime140.dll', 'vcruntime140_1.dll')) {
        $module = @($runningManager.Modules | Where-Object { $_.ModuleName -ieq $name })
        if ($module.Count -ne 1 -or $module[0].FileName -ine (Join-Path $managerDirectory $name)) {
            throw "micromamba did not load its private copy of $name"
        }
        Write-Host "PASS: loaded private $name"
    }
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) { throw "Private micromamba run failed: $($process.StandardError.ReadToEnd())" }

    $library = Join-Path $managerDirectory 'msvcp140.dll'
    [IO.File]::WriteAllText($library, 'damaged private library')
    Initialize-EnvironmentManager
    if ((Get-LocalSha256 $library) -ne (Get-LocalSha256 (Join-Path $simulatorRoot 'bootstrap/win-64/msvcp140.dll'))) {
        throw 'Damaged private library was not replaced'
    }
    Write-Host 'PASS: repaired private library beside existing manager'

    $realSimulatorRoot = $simulatorRoot
    $simulatorRoot = Join-Path $testRoot 'incomplete-distribution'
    New-Item -ItemType Directory -Force -Path $simulatorRoot | Out-Null
    $rejected = $false
    try { Initialize-WindowsLibraries }
    catch {
        if ($_.Exception.Message -notlike 'Missing bootstrap library:*') { throw }
        $rejected = $true
    }
    if (-not $rejected) { throw 'Missing bootstrap library was accepted' }
    Write-Host 'PASS: missing source library rejected with recovery guidance'

    Copy-Item -LiteralPath (Join-Path $realSimulatorRoot 'bootstrap') -Destination $simulatorRoot -Recurse
    [IO.File]::WriteAllText((Join-Path $simulatorRoot 'bootstrap/win-64/vcruntime140.dll'), 'corrupt source')
    $rejected = $false
    try { Initialize-WindowsLibraries }
    catch {
        if ($_.Exception.Message -notlike 'Bootstrap library failed its integrity check:*') { throw }
        $rejected = $true
    }
    if (-not $rejected) { throw 'Corrupt bootstrap source was accepted' }
    Write-Host 'PASS: corrupt source library rejected'
} finally {
    if ($null -ne $process -and -not $process.HasExited) { $process.WaitForExit() }
    # Only clean up the unique test directory created directly inside scratch.
    if ([IO.Path]::GetDirectoryName([IO.Path]::GetFullPath($testRoot)) -eq $scratchRoot.TrimEnd('\') -and
        [IO.Path]::GetFileName($testRoot).StartsWith('edr-bootstrap-')) {
        Remove-Item -LiteralPath $testRoot -Recurse -Force
    }
}
