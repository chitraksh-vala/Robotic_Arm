$ErrorActionPreference = 'Stop'

$g = 9.81
$H = 0.180
$Lt = 0.100
$utilization = 0.67
$alphaLower = 60.0 * [Math]::PI / 180.0

$limits = @{
    J2Min = -90.0
    J2Max = 110.0
    J3Min = -140.0
    J3Max = 140.0
    J4Min = -130.0
    J4Max = 130.0
}

$poses = @(
    [pscustomobject]@{ Name = 'CP1'; R = 0.500; Z = 0.000; Phi = -90.0 },
    [pscustomobject]@{ Name = 'CP2'; R = 0.200; Z = 0.000; Phi = -90.0 },
    [pscustomobject]@{ Name = 'CP3'; R = 0.300; Z = 0.650; Phi = 0.0 },
    [pscustomobject]@{ Name = 'R600'; R = 0.600; Z = 0.180; Phi = 0.0 }
)

$baseline = [ordered]@{
    L1 = 0.280
    L2 = 0.260
    UpperLink = 0.210
    UpperCable = 0.075
    J3Assembly = 1.160
    ForearmLink = 0.200
    ForearmCable = 0.075
    WristAssembly = 0.300
    ToolAssembly = 0.500
    Payload = 0.750
    Counterbalance_Nm = 0.000
}

$scenarios = @(
    [pscustomobject]@{ Name = 'Baseline'; Description = 'Current provisional model.'; Changes = @{} },
    [pscustomobject]@{ Name = 'Link split B'; Description = 'Use L1 = 250 mm and L2 = 290 mm while keeping total reach constant.'; Changes = @{ L1 = 0.250; L2 = 0.290; UpperLink = 0.210 * (0.250 / 0.280); UpperCable = 0.075 * (0.250 / 0.280); ForearmLink = 0.200 * (0.290 / 0.260); ForearmCable = 0.075 * (0.290 / 0.260) } },
    [pscustomobject]@{ Name = 'Light tool'; Description = 'Reduce gripper/interface mass from 500 g to 350 g.'; Changes = @{ ToolAssembly = 0.350 } },
    [pscustomobject]@{ Name = 'Very light tool'; Description = 'Reduce gripper/interface mass from 500 g to 250 g.'; Changes = @{ ToolAssembly = 0.250 } },
    [pscustomobject]@{ Name = 'Light J3 assembly'; Description = 'Further reduce moving J3 motor/gearbox/bracket assembly from 1.16 kg to 1.00 kg.'; Changes = @{ J3Assembly = 1.000 } },
    [pscustomobject]@{ Name = 'Light wrist'; Description = 'Reduce wrist assembly from 300 g to 220 g.'; Changes = @{ WristAssembly = 0.220 } },
    [pscustomobject]@{ Name = 'Mass optimization combo'; Description = 'L1/L2 = 250/290, J3 assembly = 1.00 kg, wrist = 220 g, tool = 350 g.'; Changes = @{ L1 = 0.250; L2 = 0.290; UpperLink = 0.210 * (0.250 / 0.280); UpperCable = 0.075 * (0.250 / 0.280); ForearmLink = 0.200 * (0.290 / 0.260); ForearmCable = 0.075 * (0.290 / 0.260); J3Assembly = 1.000; WristAssembly = 0.220; ToolAssembly = 0.350 } },
    [pscustomobject]@{ Name = 'Counterbalance 5 Nm'; Description = 'Add shoulder counterbalance giving about 5 N.m near horizontal.'; Changes = @{ Counterbalance_Nm = 5.000 } },
    [pscustomobject]@{ Name = 'Counterbalance 8 Nm'; Description = 'Add shoulder counterbalance giving about 8 N.m near horizontal.'; Changes = @{ Counterbalance_Nm = 8.000 } },
    [pscustomobject]@{ Name = 'Mass combo + 5 Nm counterbalance'; Description = 'Mass optimization combo plus a 5 N.m shoulder counterbalance.'; Changes = @{ L1 = 0.250; L2 = 0.290; UpperLink = 0.210 * (0.250 / 0.280); UpperCable = 0.075 * (0.250 / 0.280); ForearmLink = 0.200 * (0.290 / 0.260); ForearmCable = 0.075 * (0.290 / 0.260); J3Assembly = 1.000; WristAssembly = 0.220; ToolAssembly = 0.350; Counterbalance_Nm = 5.000 } }
)

function Copy-ScenarioModel {
    param([hashtable]$Changes)

    $model = [ordered]@{}
    foreach ($key in $baseline.Keys) {
        if ($Changes.ContainsKey($key)) {
            $model[$key] = $Changes[$key]
        } else {
            $model[$key] = $baseline[$key]
        }
    }
    return $model
}

function Get-J2GravityTorque {
    param([hashtable]$M, [double]$Q2, [double]$Q3, [double]$Q4)

    $q23 = $Q2 + $Q3
    $phi = $q23 + $Q4
    $c2 = [Math]::Cos($Q2)
    $c23 = [Math]::Cos($q23)
    $cp = [Math]::Cos($phi)

    $xUpper = 0.5 * $M.L1 * $c2
    $xJ3 = $M.L1 * $c2
    $xForearm = $M.L1 * $c2 + 0.5 * $M.L2 * $c23
    $xWrist = $M.L1 * $c2 + $M.L2 * $c23
    $xTool = $xWrist + 0.050 * $cp
    $xPayload = $xWrist + $Lt * $cp

    $j2 = $g * (
        $M.UpperLink * $xUpper +
        $M.UpperCable * $xUpper +
        $M.J3Assembly * $xJ3 +
        $M.ForearmLink * $xForearm +
        $M.ForearmCable * $xForearm +
        $M.WristAssembly * $xWrist +
        $M.ToolAssembly * $xTool +
        $M.Payload * $xPayload
    )

    return [Math]::Abs($j2)
}

function Get-J2Inertia {
    param([hashtable]$M)

    return (
        (1.0 / 3.0) * $M.UpperLink * $M.L1 * $M.L1 +
        $M.UpperCable * [Math]::Pow(0.5 * $M.L1, 2) +
        $M.J3Assembly * $M.L1 * $M.L1 +
        $M.ForearmLink * ($M.L1 * $M.L1 + $M.L1 * $M.L2 + $M.L2 * $M.L2 / 3.0) +
        $M.ForearmCable * [Math]::Pow($M.L1 + 0.5 * $M.L2, 2) +
        $M.WristAssembly * [Math]::Pow($M.L1 + $M.L2, 2) +
        $M.ToolAssembly * [Math]::Pow($M.L1 + $M.L2 + 0.050, 2) +
        $M.Payload * [Math]::Pow($M.L1 + $M.L2 + $Lt, 2)
    )
}

function Get-IKPass {
    param([hashtable]$M)

    $degree = 180.0 / [Math]::PI
    $poseFlags = foreach ($pose in $poses) {
        $phi = $pose.Phi / $degree
        $wristR = $pose.R - $Lt * [Math]::Cos($phi)
        $wristZ = $pose.Z - $Lt * [Math]::Sin($phi)
        $dz = $wristZ - $H
        $cosQ3Raw = ($wristR * $wristR + $dz * $dz - $M.L1 * $M.L1 - $M.L2 * $M.L2) / (2.0 * $M.L1 * $M.L2)

        if ($cosQ3Raw -lt -1.0 -or $cosQ3Raw -gt 1.0) {
            $false
            continue
        }

        $cosQ3 = [Math]::Max(-1.0, [Math]::Min(1.0, $cosQ3Raw))
        $branchValid = @()
        foreach ($branch in @(-1.0, 1.0)) {
            $q3 = $branch * [Math]::Acos($cosQ3)
            $q2 = [Math]::Atan2($dz, $wristR) - [Math]::Atan2(
                $M.L2 * [Math]::Sin($q3),
                $M.L1 + $M.L2 * [Math]::Cos($q3)
            )
            $q4 = $phi - $q2 - $q3
            $q2d = $q2 * $degree
            $q3d = $q3 * $degree
            $q4d = $q4 * $degree
            $branchValid += (
                $q2d -ge $limits.J2Min -and $q2d -le $limits.J2Max -and
                $q3d -ge $limits.J3Min -and $q3d -le $limits.J3Max -and
                $q4d -ge $limits.J4Min -and $q4d -le $limits.J4Max
            )
        }
        $branchValid -contains $true
    }

    return (@($poseFlags) -notcontains $false)
}

$baselineModel = Copy-ScenarioModel -Changes @{}
$baselineStatic = Get-J2GravityTorque -M $baselineModel -Q2 0.0 -Q3 0.0 -Q4 0.0
$baselineAccel = (Get-J2Inertia -M $baselineModel) * $alphaLower
$baselineDemand = $baselineStatic + $baselineAccel + 0.10 * $baselineStatic
$baselineCapacity = $baselineDemand / $utilization

$results = foreach ($scenario in $scenarios) {
    $model = Copy-ScenarioModel -Changes $scenario.Changes
    $staticRaw = Get-J2GravityTorque -M $model -Q2 0.0 -Q3 0.0 -Q4 0.0
    $staticAfterCounter = [Math]::Max(0.0, $staticRaw - $model.Counterbalance_Nm)
    $accel = (Get-J2Inertia -M $model) * $alphaLower
    $friction = 0.10 * $staticAfterCounter
    $demand = $staticAfterCounter + $accel + $friction
    $capacity = $demand / $utilization
    $movingMass = $model.UpperLink + $model.UpperCable + $model.J3Assembly + $model.ForearmLink + $model.ForearmCable + $model.WristAssembly + $model.ToolAssembly + $model.Payload

    [pscustomobject]@{
        Scenario = $scenario.Name
        Description = $scenario.Description
        L1_mm = [Math]::Round($model.L1 * 1000, 0)
        L2_mm = [Math]::Round($model.L2 * 1000, 0)
        Distal_moving_mass_kg = [Math]::Round($movingMass, 3)
        J2_static_before_counterbalance_Nm = [Math]::Round($staticRaw, 3)
        Counterbalance_Nm = [Math]::Round($model.Counterbalance_Nm, 3)
        J2_static_after_counterbalance_Nm = [Math]::Round($staticAfterCounter, 3)
        J2_acceleration_allowance_Nm = [Math]::Round($accel, 3)
        J2_preliminary_demand_Nm = [Math]::Round($demand, 3)
        J2_min_output_capacity_Nm = [Math]::Round($capacity, 3)
        Reduction_vs_baseline_capacity_percent = [Math]::Round(100.0 * ($baselineCapacity - $capacity) / $baselineCapacity, 1)
        Required_poses_pass = Get-IKPass -M $model
    }
}

$report = [pscustomobject]@{
    Model_status = 'Preliminary J2 torque reduction study'
    Baseline_J2_capacity_Nm = [Math]::Round($baselineCapacity, 3)
    Notes = @(
        'Payload remains fixed at 0.75 kg.',
        'Counterbalance is treated as equivalent near-horizontal shoulder torque only.',
        'Final counterbalance design needs torque-angle curve matching, not a constant torque assumption.'
    )
    Results = @($results)
}

$jsonPath = Join-Path $PSScriptRoot 'j2_torque_reduction_study_summary.json'
$report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $jsonPath -Encoding utf8
$report | ConvertTo-Json -Depth 8
