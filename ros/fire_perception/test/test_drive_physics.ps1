param([string]$UnityEditor = 'C:/Program Files/Unity/Hub/Editor/6000.0.75f1/Editor/Unity.exe')
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$taskProject = Join-Path ([IO.Path]::GetTempPath()) ('fire-drive-physics-' + [guid]::NewGuid().ToString('N'))
foreach ($taskDirectory in @('Assets/Editor','Packages','ProjectSettings')) {
    New-Item -ItemType Directory -Path (Join-Path $taskProject $taskDirectory) -Force | Out-Null
}
Copy-Item -LiteralPath "$taskRoot/Assets/Scripts/PlanarDriveExecution.cs" -Destination "$taskProject/Assets/PlanarDriveExecution.cs"
Copy-Item -LiteralPath "$PSScriptRoot/unity_drive/Editor/DrivePhysicsChecks.cs" -Destination "$taskProject/Assets/Editor/DrivePhysicsChecks.cs"
Copy-Item -LiteralPath "$PSScriptRoot/unity_drive/manifest.json" -Destination "$taskProject/Packages/manifest.json"
Copy-Item -LiteralPath "$taskRoot/ProjectSettings/ProjectVersion.txt" -Destination "$taskProject/ProjectSettings/ProjectVersion.txt"
Write-Output "Isolated Unity physics test: $taskProject"
$taskArguments = @('-batchmode','-nographics','-projectPath', ('"' + $taskProject + '"'), '-executeMethod', 'DrivePhysicsChecks.Run', '-logFile', ('"' + $taskProject + '/physics.log"'))
$taskProcess = Start-Process -FilePath $UnityEditor -ArgumentList $taskArguments -WindowStyle Hidden -PassThru
# Bound a stalled editor/license launch; never touch the user's open editor.
if (-not $taskProcess.WaitForExit(180000)) {
    Stop-Process -Id $taskProcess.Id
    throw "Isolated test timed out; see $taskProject/physics.log"
}
if ($taskProcess.ExitCode -ne 0) {
    Get-Content -LiteralPath "$taskProject/physics.log" -Tail 35
    throw "Unity physics check failed: $($taskProcess.ExitCode)"
}
if (-not (Select-String -LiteralPath "$taskProject/physics.log" -Pattern 'DRIVE_PHYSICS_PASS' -Quiet)) {
    throw 'Unity exited without completing the physics checks'
}
Select-String -LiteralPath "$taskProject/physics.log" -Pattern 'DRIVE_PHYSICS_PASS:|DRIVE_LEGACY_COMPARISON:' | ForEach-Object { $_.Line }
