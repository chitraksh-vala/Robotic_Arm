$ErrorActionPreference = 'Stop'

$g = 9.81
$deg = [Math]::PI / 180.0

$design = [ordered]@{
    H_mm = 180.0
    L1_mm = 280.0
    L2_mm = 260.0
    Lt_mm = 100.0
    Payload_kg = 0.750
    ToolAssembly_kg = 0.500
    DistalMovingMassForJ2_kg = 3.270
    J1_required_Nm = 1.414
    J2_required_Nm = 23.897
    J3_required_Nm = 8.945
    J4_required_Nm = 1.638
    J5_required_Nm = 0.506
}

$jointSpeed = [ordered]@{
    J1_deg_s = 30.0
    J2_deg_s = 30.0
    J3_deg_s = 30.0
    J4_deg_s = 60.0
    J5_deg_s = 60.0
}

$gear = [ordered]@{
    J1_ratio = 10.0
    J2_ratio = 100.0
    J3_ratio = 40.0
    Efficiency = 0.75
}

$torqueCurve = @(
    [pscustomobject]@{ rpm = 50.0; torque_Nm = 0.78 },
    [pscustomobject]@{ rpm = 100.0; torque_Nm = 0.76 },
    [pscustomobject]@{ rpm = 200.0; torque_Nm = 0.70 },
    [pscustomobject]@{ rpm = 300.0; torque_Nm = 0.65 },
    [pscustomobject]@{ rpm = 400.0; torque_Nm = 0.58 },
    [pscustomobject]@{ rpm = 500.0; torque_Nm = 0.53 },
    [pscustomobject]@{ rpm = 600.0; torque_Nm = 0.49 }
)

function Get-InterpTorque {
    param([double]$rpm)

    $points = $torqueCurve | Sort-Object rpm
    if ($rpm -le $points[0].rpm) { return $points[0].torque_Nm }
    if ($rpm -ge $points[-1].rpm) { return $points[-1].torque_Nm }

    for ($i = 0; $i -lt ($points.Count - 1); $i++) {
        $a = $points[$i]
        $b = $points[$i + 1]
        if ($rpm -ge $a.rpm -and $rpm -le $b.rpm) {
            $f = ($rpm - $a.rpm) / ($b.rpm - $a.rpm)
            return $a.torque_Nm + $f * ($b.torque_Nm - $a.torque_Nm)
        }
    }
}

function Get-StepperJointRow {
    param(
        [string]$Joint,
        [double]$RequiredNm,
        [double]$Ratio,
        [double]$SpeedDegS
    )

    $jointRpm = $SpeedDegS / 6.0
    $motorRpm = $jointRpm * $Ratio
    $requiredMotorTorque = $RequiredNm / ($Ratio * $gear.Efficiency)
    $availableMotorTorque = Get-InterpTorque -rpm $motorRpm
    $availableOutputTorque = $availableMotorTorque * $Ratio * $gear.Efficiency
    $margin = $availableOutputTorque / $RequiredNm
    $microstepDeg = 360.0 / (1600.0 * $Ratio)
    $tcpIdealStepMm = ($design.L1_mm + $design.L2_mm + $design.Lt_mm) * [Math]::Tan($microstepDeg * $deg)

    [pscustomobject]@{
        Joint = $Joint
        Selected_ratio = "$([int]$Ratio):1"
        Required_output_Nm = [Math]::Round($RequiredNm, 3)
        Target_joint_speed_deg_s = $SpeedDegS
        Joint_speed_rpm = [Math]::Round($jointRpm, 3)
        Motor_speed_rpm = [Math]::Round($motorRpm, 1)
        Required_motor_torque_Nm = [Math]::Round($requiredMotorTorque, 3)
        Curve_available_motor_torque_Nm = [Math]::Round($availableMotorTorque, 3)
        Curve_available_output_torque_Nm = [Math]::Round($availableOutputTorque, 3)
        Torque_margin = [Math]::Round($margin, 2)
        Ideal_microstep_output_deg = [Math]::Round($microstepDeg, 5)
        Ideal_TCP_step_at_full_reach_mm = [Math]::Round($tcpIdealStepMm, 4)
        Screen = if ($margin -ge 1.5) { 'Pass for concept' } elseif ($margin -ge 1.25) { 'Marginal' } else { 'Fail' }
    }
}

$stepperRows = @(
    Get-StepperJointRow -Joint 'J1' -RequiredNm $design.J1_required_Nm -Ratio $gear.J1_ratio -SpeedDegS $jointSpeed.J1_deg_s
    Get-StepperJointRow -Joint 'J2' -RequiredNm $design.J2_required_Nm -Ratio $gear.J2_ratio -SpeedDegS $jointSpeed.J2_deg_s
    Get-StepperJointRow -Joint 'J3' -RequiredNm $design.J3_required_Nm -Ratio $gear.J3_ratio -SpeedDegS $jointSpeed.J3_deg_s
)

$j4ServoStallNm = 30.0 * 0.0980665
$j5ServoStallNm = $j4ServoStallNm
$servoUsableFractions = @(0.30, 0.50, 0.67)
$servoRows = foreach ($joint in @(
    [pscustomobject]@{ Joint = 'J4'; RequiredNm = $design.J4_required_Nm; TargetSpeed = $jointSpeed.J4_deg_s },
    [pscustomobject]@{ Joint = 'J5'; RequiredNm = $design.J5_required_Nm; TargetSpeed = $jointSpeed.J5_deg_s }
)) {
    foreach ($fraction in $servoUsableFractions) {
        $usable = $j4ServoStallNm * $fraction
        [pscustomobject]@{
            Joint = $joint.Joint
            Required_output_Nm = [Math]::Round($joint.RequiredNm, 3)
            Target_speed_deg_s = $joint.TargetSpeed
            ST3215_stall_Nm = [Math]::Round($j4ServoStallNm, 3)
            Assumed_usable_fraction = $fraction
            Assumed_usable_Nm = [Math]::Round($usable, 3)
            Margin = [Math]::Round($usable / $joint.RequiredNm, 2)
            Screen = if (($usable / $joint.RequiredNm) -ge 1.0) { 'Pass under this assumption' } else { 'Fails under this assumption' }
        }
    }
}

$E_cf_Pa = 64e9
$densityCf_kg_m3 = 1600.0

function Get-RoundCfTube {
    param(
        [string]$Name,
        [double]$Length_mm,
        [double]$OuterDiameter_mm,
        [double]$InnerDiameter_mm,
        [double]$DesignMass_kg,
        [double]$TipForce_N
    )

    $wall_mm = ($OuterDiameter_mm - $InnerDiameter_mm) / 2.0
    $area_mm2 = [Math]::PI * ([Math]::Pow($OuterDiameter_mm, 2) - [Math]::Pow($InnerDiameter_mm, 2)) / 4.0
    $I_mm4 = [Math]::PI * ([Math]::Pow($OuterDiameter_mm, 4) - [Math]::Pow($InnerDiameter_mm, 4)) / 64.0
    $massTube_kg = ($area_mm2 * 1e-6) * ($Length_mm / 1000.0) * $densityCf_kg_m3
    $I_m4 = $I_mm4 * 1e-12
    $L_m = $Length_mm / 1000.0
    $deflection_m = $TipForce_N * [Math]::Pow($L_m, 3) / (3.0 * $E_cf_Pa * $I_m4)
    $moment_Nm = $TipForce_N * $L_m
    $stress_Pa = $moment_Nm * (($OuterDiameter_mm / 2.0) / 1000.0) / $I_m4

    [pscustomobject]@{
        Link = $Name
        Selected_section = "$([int]$OuterDiameter_mm) mm OD x $([int]$InnerDiameter_mm) mm ID ($wall_mm mm wall) 3K roll-wrapped carbon-fibre twill tube"
        Length_mm = $Length_mm
        Area_mm2 = [Math]::Round($area_mm2, 1)
        Tube_only_mass_kg = [Math]::Round($massTube_kg, 3)
        Design_mass_with_attachments_kg = $DesignMass_kg
        I_bending_mm4 = [Math]::Round($I_mm4, 0)
        Tip_force_used_N = [Math]::Round($TipForce_N, 1)
        End_deflection_mm = [Math]::Round($deflection_m * 1000.0, 3)
        Bending_stress_MPa = [Math]::Round($stress_Pa / 1e6, 2)
        Screen = 'Pass for concept; bonded inserts and joint brackets need CAD/FEA'
    }
}

$upperTipForce = (1.160 + 0.200 + 0.075 + 0.300 + 0.500 + 0.750) * $g
$forearmTipForce = (0.300 + 0.500 + 0.750) * $g
$linkRows = @(
    Get-RoundCfTube -Name 'L1 upper arm' -Length_mm 280.0 -OuterDiameter_mm 40.0 -InnerDiameter_mm 36.0 -DesignMass_kg 0.210 -TipForce_N $upperTipForce
    Get-RoundCfTube -Name 'L2 forearm' -Length_mm 260.0 -OuterDiameter_mm 40.0 -InnerDiameter_mm 36.0 -DesignMass_kg 0.200 -TipForce_N $forearmTipForce
)

$accuracyRows = @(
    [pscustomobject]@{
        Topic = 'TCP accuracy target'
        Value = '+/-5 mm'
        Calculation = 'At 640 mm reach this corresponds to about +/-0.447 deg total angular error.'
        Decision = 'Keep gearbox backlash preferably <=0.15 deg output per loaded joint, absolute maximum <=0.30 deg.'
    },
    [pscustomobject]@{
        Topic = 'TCP repeatability target'
        Value = '+/-3 mm'
        Calculation = 'At 640 mm reach this corresponds to about +/-0.269 deg repeatable angular error.'
        Decision = 'Backlash, open-loop step loss and joint compliance dominate over microstep resolution.'
    }
)

$gripperRows = @(
    [pscustomobject]@{
        Item = 'Mechanical gripper actuator'
        Value = '1x ST3215, third Waveshare servo'
        Calculation = '30 kg.cm = 2.942 N.m locked-rotor torque at 12 V.'
        Decision = 'Use conservatively; design linkage so normal holding does not require continuous stall.'
    },
    [pscustomobject]@{
        Item = 'Conservative pinch estimate'
        Value = 'about 21 N at 25 mm effective finger radius'
        Calculation = '0.30 * 2.942 N.m usable torque * 0.60 linkage efficiency / 0.025 m = 21.2 N.'
        Decision = 'Adequate for light objects if finger pads have high friction; validate with real objects.'
    },
    [pscustomobject]@{
        Item = 'Short-duration stall-limited pinch estimate'
        Value = 'about 71 N at 25 mm effective finger radius'
        Calculation = '2.942 N.m * 0.60 / 0.025 m = 70.6 N.'
        Decision = 'Do not use as continuous design force.'
    }
)

$interfaceRows = @(
    [pscustomobject]@{
        Item = 'Mechanical quick-change'
        Decision = 'Use two locating dowels plus one positive latch/screw/wedge. Pogo pins carry no structure.'
        Status = 'Final concept'
    },
    [pscustomobject]@{
        Item = 'Electrical contacts'
        Decision = 'Minimum 4 contacts: +12 V, GND, half-duplex serial, tool-ID. Prefer 6 contacts with duplicated +12 V and GND for current margin.'
        Status = '4-pin compatible, 6-pin preferred'
    },
    [pscustomobject]@{
        Item = 'Tool loads'
        Decision = 'Design interface for 1.25 kg vertical load and at least 0.75 N.m tool moment before safety factor.'
        Status = 'Preliminary'
    },
    [pscustomobject]@{
        Item = 'Soft gripper routing'
        Decision = 'Keep pump/regulator off-arm; route only lightweight tube and compact valve/control hardware to tool side.'
        Status = 'Final concept'
    }
)

$finalOutputs = [pscustomobject]@{
    Model_status = 'Provisional design package, gearbox-only J2'
    Final_geometry = $design
    Selected_gearboxes = $gear
    Stepper_drivetrain_check = $stepperRows
    Servo_check = $servoRows
    Link_structural_check = $linkRows
    Accuracy_and_backlash_targets = $accuracyRows
    Gripper_sizing = $gripperRows
    Modular_interface = $interfaceRows
}

$jsonPath = Join-Path $PSScriptRoot 'final_design_sizing_summary.json'
$finalOutputs | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $jsonPath -Encoding utf8
$finalOutputs | ConvertTo-Json -Depth 8
