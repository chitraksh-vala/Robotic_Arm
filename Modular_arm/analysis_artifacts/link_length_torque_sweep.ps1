$ErrorActionPreference = 'Stop'

$g = 9.81
$H = 0.180
$Lt = 0.100
$utilization = 0.67
$frictionFraction = 0.10
$alphaLower = 60.0 * [Math]::PI / 180.0
$alphaWrist = 120.0 * [Math]::PI / 180.0

$baseL1 = 0.280
$baseL2 = 0.260
$baseUpperLinkMass = 0.300
$baseForearmLinkMass = 0.280
$baseUpperCableMass = 0.075
$baseForearmCableMass = 0.075

$fixedMass = [ordered]@{
    J3Assembly = 1.260
    WristAssembly = 0.300
    ToolAssembly = 0.500
    Payload = 0.750
}

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

$candidates = @(
    [pscustomobject]@{ Name = 'Current'; L1 = 0.280; L2 = 0.260 },
    [pscustomobject]@{ Name = 'A'; L1 = 0.260; L2 = 0.280 },
    [pscustomobject]@{ Name = 'B'; L1 = 0.250; L2 = 0.290 },
    [pscustomobject]@{ Name = 'C'; L1 = 0.240; L2 = 0.300 },
    [pscustomobject]@{ Name = 'D'; L1 = 0.270; L2 = 0.270 }
)

function Get-MassModel {
    param([double]$L1, [double]$L2)

    return [ordered]@{
        UpperLink = $baseUpperLinkMass * ($L1 / $baseL1)
        UpperCable = $baseUpperCableMass * ($L1 / $baseL1)
        J3Assembly = $fixedMass.J3Assembly
        ForearmLink = $baseForearmLinkMass * ($L2 / $baseL2)
        ForearmCable = $baseForearmCableMass * ($L2 / $baseL2)
        WristAssembly = $fixedMass.WristAssembly
        ToolAssembly = $fixedMass.ToolAssembly
        Payload = $fixedMass.Payload
    }
}

function Get-GravityTorque {
    param(
        [double]$L1,
        [double]$L2,
        [hashtable]$Mass,
        [double]$Q2,
        [double]$Q3,
        [double]$Q4
    )

    $q23 = $Q2 + $Q3
    $phi = $q23 + $Q4
    $c2 = [Math]::Cos($Q2)
    $c23 = [Math]::Cos($q23)
    $cp = [Math]::Cos($phi)

    $xUpper = 0.5 * $L1 * $c2
    $xJ3 = $L1 * $c2
    $xForearm = $L1 * $c2 + 0.5 * $L2 * $c23
    $xWrist = $L1 * $c2 + $L2 * $c23
    $xTool = $xWrist + 0.050 * $cp
    $xPayload = $xWrist + $Lt * $cp

    $j2 = $g * (
        $Mass.UpperLink * $xUpper +
        $Mass.UpperCable * $xUpper +
        $Mass.J3Assembly * $xJ3 +
        $Mass.ForearmLink * $xForearm +
        $Mass.ForearmCable * $xForearm +
        $Mass.WristAssembly * $xWrist +
        $Mass.ToolAssembly * $xTool +
        $Mass.Payload * $xPayload
    )

    $j3 = $g * (
        $Mass.ForearmLink * (0.5 * $L2 * $c23) +
        $Mass.ForearmCable * (0.5 * $L2 * $c23) +
        $Mass.WristAssembly * ($L2 * $c23) +
        $Mass.ToolAssembly * ($L2 * $c23 + 0.050 * $cp) +
        $Mass.Payload * ($L2 * $c23 + $Lt * $cp)
    )

    $j4 = $g * (
        $Mass.ToolAssembly * 0.050 * $cp +
        $Mass.Payload * $Lt * $cp
    )

    return [pscustomobject]@{
        J2_Nm = [Math]::Round([Math]::Abs($j2), 3)
        J3_Nm = [Math]::Round([Math]::Abs($j3), 3)
        J4_Nm = [Math]::Round([Math]::Abs($j4), 3)
    }
}

function Get-IKSolutions {
    param(
        [double]$L1,
        [double]$L2,
        [hashtable]$Mass,
        [pscustomobject]$Pose
    )

    $degree = 180.0 / [Math]::PI
    $phi = $Pose.Phi / $degree
    $wristR = $Pose.R - $Lt * [Math]::Cos($phi)
    $wristZ = $Pose.Z - $Lt * [Math]::Sin($phi)
    $dz = $wristZ - $H
    $cosQ3Raw = ($wristR * $wristR + $dz * $dz - $L1 * $L1 - $L2 * $L2) / (2.0 * $L1 * $L2)

    if ($cosQ3Raw -lt -1.0 -or $cosQ3Raw -gt 1.0) {
        return @(
            [pscustomobject]@{ Branch = 'negative-elbow'; Valid = $false; Reason = 'geometrically unreachable'; J2_deg = $null; J3_deg = $null; J4_deg = $null; Gravity_torque = $null },
            [pscustomobject]@{ Branch = 'positive-elbow'; Valid = $false; Reason = 'geometrically unreachable'; J2_deg = $null; J3_deg = $null; J4_deg = $null; Gravity_torque = $null }
        )
    }

    $cosQ3 = [Math]::Max(-1.0, [Math]::Min(1.0, $cosQ3Raw))
    $solutions = foreach ($branch in @(-1.0, 1.0)) {
        $q3 = $branch * [Math]::Acos($cosQ3)
        $q2 = [Math]::Atan2($dz, $wristR) - [Math]::Atan2(
            $L2 * [Math]::Sin($q3),
            $L1 + $L2 * [Math]::Cos($q3)
        )
        $q4 = $phi - $q2 - $q3
        $q2d = $q2 * $degree
        $q3d = $q3 * $degree
        $q4d = $q4 * $degree
        $valid = (
            $q2d -ge $limits.J2Min -and $q2d -le $limits.J2Max -and
            $q3d -ge $limits.J3Min -and $q3d -le $limits.J3Max -and
            $q4d -ge $limits.J4Min -and $q4d -le $limits.J4Max
        )

        $reason = if ($valid) { 'pass' } else { 'joint-limit fail' }
        [pscustomobject]@{
            Branch = if ($branch -lt 0) { 'negative-elbow' } else { 'positive-elbow' }
            Valid = $valid
            Reason = $reason
            J2_deg = [Math]::Round($q2d, 2)
            J3_deg = [Math]::Round($q3d, 2)
            J4_deg = [Math]::Round($q4d, 2)
            Gravity_torque = Get-GravityTorque -L1 $L1 -L2 $L2 -Mass $Mass -Q2 $q2 -Q3 $q3 -Q4 $q4
        }
    }
    return $solutions
}

function Get-InertiaAndSizing {
    param([double]$L1, [double]$L2, [hashtable]$Mass)

    $straightStatic = Get-GravityTorque -L1 $L1 -L2 $L2 -Mass $Mass -Q2 0.0 -Q3 0.0 -Q4 0.0
    $iJ2 = (
        (1.0 / 3.0) * $Mass.UpperLink * $L1 * $L1 +
        $Mass.UpperCable * [Math]::Pow(0.5 * $L1, 2) +
        $Mass.J3Assembly * $L1 * $L1 +
        $Mass.ForearmLink * ($L1 * $L1 + $L1 * $L2 + $L2 * $L2 / 3.0) +
        $Mass.ForearmCable * [Math]::Pow($L1 + 0.5 * $L2, 2) +
        $Mass.WristAssembly * [Math]::Pow($L1 + $L2, 2) +
        $Mass.ToolAssembly * [Math]::Pow($L1 + $L2 + 0.050, 2) +
        $Mass.Payload * [Math]::Pow($L1 + $L2 + $Lt, 2)
    )

    $iJ3 = (
        (1.0 / 3.0) * $Mass.ForearmLink * $L2 * $L2 +
        $Mass.ForearmCable * [Math]::Pow(0.5 * $L2, 2) +
        $Mass.WristAssembly * $L2 * $L2 +
        $Mass.ToolAssembly * [Math]::Pow($L2 + 0.050, 2) +
        $Mass.Payload * [Math]::Pow($L2 + $Lt, 2)
    )

    $iJ4 = $Mass.ToolAssembly * 0.050 * 0.050 + $Mass.Payload * $Lt * $Lt
    $rows = @(
        [pscustomobject]@{ Joint = 'J1'; Static_Nm = 0.0; Acceleration_Nm = $iJ2 * $alphaLower; Friction_Nm = 0.20 },
        [pscustomobject]@{ Joint = 'J2'; Static_Nm = $straightStatic.J2_Nm; Acceleration_Nm = $iJ2 * $alphaLower; Friction_Nm = $frictionFraction * $straightStatic.J2_Nm },
        [pscustomobject]@{ Joint = 'J3'; Static_Nm = $straightStatic.J3_Nm; Acceleration_Nm = $iJ3 * $alphaLower; Friction_Nm = $frictionFraction * $straightStatic.J3_Nm },
        [pscustomobject]@{ Joint = 'J4'; Static_Nm = $straightStatic.J4_Nm; Acceleration_Nm = $iJ4 * $alphaWrist; Friction_Nm = $frictionFraction * $straightStatic.J4_Nm },
        [pscustomobject]@{ Joint = 'J5'; Static_Nm = 1.25 * $g * 0.025; Acceleration_Nm = (1.25 * 0.025 * 0.025) * $alphaWrist; Friction_Nm = $frictionFraction * (1.25 * $g * 0.025) }
    )

    $sizing = foreach ($row in $rows) {
        $demand = $row.Static_Nm + $row.Acceleration_Nm + $row.Friction_Nm
        [pscustomobject]@{
            Joint = $row.Joint
            Static_Nm = [Math]::Round($row.Static_Nm, 3)
            Acceleration_Nm = [Math]::Round($row.Acceleration_Nm, 3)
            Friction_allowance_Nm = [Math]::Round($row.Friction_Nm, 3)
            Preliminary_demand_Nm = [Math]::Round($demand, 3)
            Minimum_output_capacity_Nm = [Math]::Round($demand / $utilization, 3)
        }
    }
    return $sizing
}

$results = foreach ($candidate in $candidates) {
    $mass = Get-MassModel -L1 $candidate.L1 -L2 $candidate.L2
    $sizing = Get-InertiaAndSizing -L1 $candidate.L1 -L2 $candidate.L2 -Mass $mass
    $poseResults = foreach ($pose in $poses) {
        [pscustomobject]@{
            Name = $pose.Name
            R_mm = [Math]::Round($pose.R * 1000.0, 0)
            Z_mm = [Math]::Round($pose.Z * 1000.0, 0)
            Phi_deg = $pose.Phi
            Solutions = @(Get-IKSolutions -L1 $candidate.L1 -L2 $candidate.L2 -Mass $mass -Pose $pose)
        }
    }
    $posePassFlags = @($poseResults | ForEach-Object { @($_.Solutions | Where-Object Valid).Count -gt 0 })
    $allRequiredPass = $posePassFlags -notcontains $false

    [pscustomobject]@{
        Candidate = $candidate.Name
        L1_mm = [Math]::Round($candidate.L1 * 1000.0, 0)
        L2_mm = [Math]::Round($candidate.L2 * 1000.0, 0)
        L1_plus_L2_plus_Lt_mm = [Math]::Round(($candidate.L1 + $candidate.L2 + $Lt) * 1000.0, 0)
        Upper_link_mass_kg = [Math]::Round($mass.UpperLink, 3)
        Forearm_link_mass_kg = [Math]::Round($mass.ForearmLink, 3)
        Upper_cable_mass_kg = [Math]::Round($mass.UpperCable, 3)
        Forearm_cable_mass_kg = [Math]::Round($mass.ForearmCable, 3)
        Distal_moving_mass_kg = [Math]::Round(($mass.Values | Measure-Object -Sum).Sum, 3)
        Sizing = @($sizing)
        Required_poses = @($poseResults)
        All_required_poses_pass = $allRequiredPass
    }
}

$summaryRows = foreach ($result in $results) {
    $j1 = $result.Sizing | Where-Object Joint -eq 'J1'
    $j2 = $result.Sizing | Where-Object Joint -eq 'J2'
    $j3 = $result.Sizing | Where-Object Joint -eq 'J3'
    $j4 = $result.Sizing | Where-Object Joint -eq 'J4'
    $j5 = $result.Sizing | Where-Object Joint -eq 'J5'
    [pscustomobject]@{
        Candidate = $result.Candidate
        L1_mm = $result.L1_mm
        L2_mm = $result.L2_mm
        Distal_moving_mass_kg = $result.Distal_moving_mass_kg
        J1_capacity_Nm = $j1.Minimum_output_capacity_Nm
        J2_capacity_Nm = $j2.Minimum_output_capacity_Nm
        J3_capacity_Nm = $j3.Minimum_output_capacity_Nm
        J4_capacity_Nm = $j4.Minimum_output_capacity_Nm
        J5_capacity_Nm = $j5.Minimum_output_capacity_Nm
        J2_to_J3_ratio = [Math]::Round($j2.Minimum_output_capacity_Nm / $j3.Minimum_output_capacity_Nm, 2)
        All_required_poses_pass = $result.All_required_poses_pass
    }
}

$report = [pscustomobject]@{
    Model_status = 'Preliminary link-length torque redistribution sweep'
    Assumptions = [pscustomobject]@{
        Shoulder_height_mm = 180
        Tool_offset_mm = 100
        Total_planar_reach_held_mm = 640
        Link_and_cable_masses_scaled_with_length = $true
        Fixed_payload_kg = 0.750
        Fixed_tool_assembly_kg = 0.500
        Fixed_J3_assembly_kg = 1.260
        Fixed_wrist_assembly_kg = 0.300
    }
    Summary = @($summaryRows)
    Results = @($results)
}

$jsonPath = Join-Path $PSScriptRoot 'link_length_torque_sweep_summary.json'
$report | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $jsonPath -Encoding utf8
$report | ConvertTo-Json -Depth 12
