param(
    [string]$UnityData = 'C:/Program Files/Unity/Hub/Editor/6000.0.75f1/Editor/Data'
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$taskOutput = Join-Path ([IO.Path]::GetTempPath()) ('fire-odom-check-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskOutput | Out-Null
$taskReferences = @(
    "$UnityData/UnityReferenceAssemblies/unity-4.8-api/mscorlib.dll",
    "$UnityData/UnityReferenceAssemblies/unity-4.8-api/System.dll",
    "$UnityData/UnityReferenceAssemblies/unity-4.8-api/Facades/netstandard.dll",
    "$UnityData/Managed/UnityEngine/UnityEngine.CoreModule.dll",
    "$UnityData/Managed/UnityEngine/UnityEngine.SharedInternalsModule.dll",
    "$taskRoot/Library/ScriptAssemblies/Unity.Robotics.ROSTCPConnector.dll",
    "$taskRoot/Library/ScriptAssemblies/Unity.Robotics.ROSTCPConnector.Messages.dll",
    "$taskRoot/Library/ScriptAssemblies/Unity.Robotics.ROSTCPConnector.MessageGeneration.dll"
)
$taskArgs = @('-nologo', '-nostdlib', '-target:exe', "-out:$taskOutput/OdomChecks.exe")
foreach ($taskReference in $taskReferences) { $taskArgs += "-r:$taskReference" }
$taskArgs += @("$taskRoot/Assets/Scripts/OdomPublisher.cs", "$PSScriptRoot/OdomCoordinateContractTests.cs")
& "$UnityData/NetCoreRuntime/dotnet.exe" "$UnityData/DotNetSdkRoslyn/csc.dll" @taskArgs
if ($LASTEXITCODE -ne 0) { throw 'OdomPublisher compile check failed' }
foreach ($taskReference in $taskReferences | Select-Object -Skip 3) {
    Copy-Item -LiteralPath $taskReference -Destination $taskOutput
}
& "$UnityData/MonoBleedingEdge/bin/mono.exe" "$taskOutput/OdomChecks.exe" "$taskRoot/Assets/Scripts/OdomPublisher.cs"
if ($LASTEXITCODE -ne 0) { throw 'Odom coordinate contract check failed' }
Write-Output "Offline build artifacts: $taskOutput"
