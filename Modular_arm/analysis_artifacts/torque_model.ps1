$ErrorActionPreference = 'Stop'

$g = 9.81
$L1 = 0.280
$L2 = 0.260
$H = 0.180
$Lt = 0.100

# J2 uses StepperOnline 23HS45-3504S NEMA 23.  Its rotor/body is centred
# around the J2 axis for pitch, but its centre is offset from J1 yaw by half
# the motor body length.  It therefore affects J1 yaw acceleration only.
$j2Nema23Mass_kg = 1.600
$j2Nema23YawRadius_m = 0.0565

$mass = [ordered]@{
    UpperLink = 0.210
    UpperCable = 0.075
    J3Assembly = 1.160
    ForearmLink = 0.200
    ForearmCable = 0.075
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

function Get-GravityTorque {
    param([double]$Q2, [double]$Q3, [double]$Q4)

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
        $mass.UpperLink * $xUpper +
        $mass.UpperCable * $xUpper +
        $mass.J3Assembly * $xJ3 +
        $mass.ForearmLink * $xForearm +
        $mass.ForearmCable * $xForearm +
        $mass.WristAssembly * $xWrist +
        $mass.ToolAssembly * $xTool +
        $mass.Payload * $xPayload
    )

    $j3 = $g * (
        $mass.ForearmLink * (0.5 * $L2 * $c23) +
        $mass.ForearmCable * (0.5 * $L2 * $c23) +
        $mass.WristAssembly * ($L2 * $c23) +
        $mass.ToolAssembly * ($L2 * $c23 + 0.050 * $cp) +
        $mass.Payload * ($L2 * $c23 + $Lt * $cp)
    )

    $j4 = $g * (
        $mass.ToolAssembly * 0.050 * $cp +
        $mass.Payload * $Lt * $cp
    )

    return [pscustomobject]@{
        J2_Nm = [Math]::Round([Math]::Abs($j2), 3)
        J3_Nm = [Math]::Round([Math]::Abs($j3), 3)
        J4_Nm = [Math]::Round([Math]::Abs($j4), 3)
    }
}

function Get-IKSolutions {
    param(
        [string]$Name,
        [double]$R,
        [double]$Z,
        [double]$PhiDegrees
    )

    $degree = 180.0 / [Math]::PI
    $phi = $PhiDegrees / $degree
    $wristR = $R - $Lt * [Math]::Cos($phi)
    $wristZ = $Z - $Lt * [Math]::Sin($phi)
    $dz = $wristZ - $H
    $cosQ3 = ($wristR * $wristR + $dz * $dz - $L1 * $L1 - $L2 * $L2) / (2.0 * $L1 * $L2)

    $solutions = foreach ($branch in @(-1.0, 1.0)) {
        $q3 = $branch * [Math]::Acos([Math]::Max(-1.0, [Math]::Min(1.0, $cosQ3)))
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

        [pscustomobject]@{
            Branch = if ($branch -lt 0) { 'negative-elbow' } else { 'positive-elbow' }
            Valid = $valid
            J2_deg = [Math]::Round($q2d, 2)
            J3_deg = [Math]::Round($q3d, 2)
            J4_deg = [Math]::Round($q4d, 2)
            Gravity_torque = Get-GravityTorque -Q2 $q2 -Q3 $q3 -Q4 $q4
        }
    }

    return [pscustomobject]@{
        Name = $Name
        R_mm = [Math]::Round(1000.0 * $R, 0)
        Z_mm = [Math]::Round(1000.0 * $Z, 0)
        Phi_deg = $PhiDegrees
        Solutions = $solutions
    }
}

$straightStatic = Get-GravityTorque -Q2 0.0 -Q3 0.0 -Q4 0.0

$iJ2 = (
    (1.0 / 3.0) * $mass.UpperLink * $L1 * $L1 +
    $mass.UpperCable * [Math]::Pow(0.5 * $L1, 2) +
    $mass.J3Assembly * $L1 * $L1 +
    $mass.ForearmLink * ($L1 * $L1 + $L1 * $L2 + $L2 * $L2 / 3.0) +
    $mass.ForearmCable * [Math]::Pow($L1 + 0.5 * $L2, 2) +
    $mass.WristAssembly * [Math]::Pow($L1 + $L2, 2) +
    $mass.ToolAssembly * [Math]::Pow($L1 + $L2 + 0.050, 2) +
    $mass.Payload * [Math]::Pow($L1 + $L2 + $Lt, 2)
)

$iJ3 = (
    (1.0 / 3.0) * $mass.ForearmLink * $L2 * $L2 +
    $mass.ForearmCable * [Math]::Pow(0.5 * $L2, 2) +
    $mass.WristAssembly * $L2 * $L2 +
    $mass.ToolAssembly * [Math]::Pow($L2 + 0.050, 2) +
    $mass.Payload * [Math]::Pow($L2 + $Lt, 2)
)

$iJ1 = $iJ2 + $j2Nema23Mass_kg * [Math]::Pow($j2Nema23YawRadius_m, 2)

$iJ4 = $mass.ToolAssembly * 0.050 * 0.050 + $mass.Payload * $Lt * $Lt
$alphaLower = 60.0 * [Math]::PI / 180.0
$alphaWrist = 120.0 * [Math]::PI / 180.0
$utilization = 0.67
$frictionFraction = 0.10

$jointRows = @(
    [pscustomobject]@{
        Joint = 'J1'
        Static_Nm = 0.0
        Acceleration_Nm = $iJ1 * $alphaLower
        Friction_Nm = 0.20
    },
    [pscustomobject]@{
        Joint = 'J2'
        Static_Nm = $straightStatic.J2_Nm
        Acceleration_Nm = $iJ2 * $alphaLower
        Friction_Nm = $frictionFraction * $straightStatic.J2_Nm
    },
    [pscustomobject]@{
        Joint = 'J3'
        Static_Nm = $straightStatic.J3_Nm
        Acceleration_Nm = $iJ3 * $alphaLower
        Friction_Nm = $frictionFraction * $straightStatic.J3_Nm
    },
    [pscustomobject]@{
        Joint = 'J4'
        Static_Nm = $straightStatic.J4_Nm
        Acceleration_Nm = $iJ4 * $alphaWrist
        Friction_Nm = $frictionFraction * $straightStatic.J4_Nm
    },
    [pscustomobject]@{
        Joint = 'J5'
        Static_Nm = 1.25 * $g * 0.025
        Acceleration_Nm = (1.25 * 0.025 * 0.025) * $alphaWrist
        Friction_Nm = $frictionFraction * (1.25 * $g * 0.025)
    }
)

$sizing = foreach ($row in $jointRows) {
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

$motorRunningTorqueAssumption = 0.50 * 0.80
$gearboxEfficiencyAssumption = 0.75
$screeningRatios = foreach ($jointName in @('J1', 'J2', 'J3')) {
    $capacity = ($sizing | Where-Object Joint -eq $jointName).Minimum_output_capacity_Nm
    [pscustomobject]@{
        Joint = $jointName
        Mathematical_minimum_ratio = [Math]::Round(
            $capacity / ($motorRunningTorqueAssumption * $gearboxEfficiencyAssumption),
            1
        )
        Preliminary_screening_ratio = switch ($jointName) {
            'J1' { 'at least 10:1' }
            'J2' { 'about 90:1 or higher before torque-speed validation' }
            'J3' { 'about 35:1 or higher before torque-speed validation' }
        }
    }
}

$summary = [pscustomobject]@{
    Model_status = 'Preliminary CAD-independent estimate'
    Distal_moving_mass_kg = [Math]::Round(($mass.Values | Measure-Object -Sum).Sum, 3)
    Mass_model_kg = $mass
    J2_motor_for_J1_yaw = [pscustomobject]@{
        Model = 'StepperOnline 23HS45-3504S NEMA 23'
        Mass_kg = $j2Nema23Mass_kg
        Yaw_radius_m = $j2Nema23YawRadius_m
        Inclusion = 'Included in J1 yaw inertia only; its mass is not distal to the J2 pitch axis.'
    }
    Full_horizontal_static_torque_Nm = $straightStatic
    Sizing = $sizing
    Screening_ratio_assumptions = [pscustomobject]@{
        Stepper_holding_torque_Nm = 0.80
        Assumed_running_fraction = 0.50
        Assumed_gearbox_efficiency = $gearboxEfficiencyAssumption
        Ratios = $screeningRatios
    }
    Required_poses = @(
        Get-IKSolutions -Name 'CP1' -R 0.500 -Z 0.000 -PhiDegrees -90.0
        Get-IKSolutions -Name 'CP2' -R 0.200 -Z 0.000 -PhiDegrees -90.0
        Get-IKSolutions -Name 'CP3' -R 0.300 -Z 0.650 -PhiDegrees 0.0
        Get-IKSolutions -Name 'R600' -R 0.600 -Z 0.180 -PhiDegrees 0.0
    )
}

$outputPath = Join-Path $PSScriptRoot 'torque_model_summary.json'
$summary | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath $outputPath -Encoding utf8
$summary | ConvertTo-Json -Depth 10
