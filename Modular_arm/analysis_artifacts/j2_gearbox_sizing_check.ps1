$ErrorActionPreference = 'Stop'

$jointSpeedDegPerSec = 30.0
$jointSpeedRpm = $jointSpeedDegPerSec / 360.0 * 60.0
$stepperHoldingTorqueNm = 0.80

$cases = @(
    [pscustomobject]@{ Name = 'Current no counterbalance'; OutputCapacityNm = 23.897 },
    [pscustomobject]@{ Name = '5 Nm counterbalance'; OutputCapacityNm = 15.688 },
    [pscustomobject]@{ Name = '8 Nm counterbalance'; OutputCapacityNm = 10.763 },
    [pscustomobject]@{ Name = 'Mass combo + 5 Nm counterbalance'; OutputCapacityNm = 12.960 }
)

$ratios = @(40, 50, 60, 80, 100, 120, 150)
$efficiencies = @(0.65, 0.75, 0.85)

$rows = foreach ($case in $cases) {
    foreach ($ratio in $ratios) {
        foreach ($efficiency in $efficiencies) {
            $requiredMotorTorque = $case.OutputCapacityNm / ($ratio * $efficiency)
            $holdingPercent = 100.0 * $requiredMotorTorque / $stepperHoldingTorqueNm
            $motorSpeedRpm = $jointSpeedRpm * $ratio
            $outputPowerW = $case.OutputCapacityNm * ($jointSpeedDegPerSec * [Math]::PI / 180.0)
            $motorMechanicalPowerW = $requiredMotorTorque * ($motorSpeedRpm * 2.0 * [Math]::PI / 60.0)

            $screen = if ($holdingPercent -gt 70.0) {
                'Reject for now'
            } elseif ($holdingPercent -gt 50.0) {
                'Risky without torque-speed curve'
            } elseif ($holdingPercent -gt 35.0) {
                'Conditional target'
            } else {
                'Good torque margin, check speed/backlash'
            }

            [pscustomobject]@{
                Case = $case.Name
                Output_capacity_Nm = [Math]::Round($case.OutputCapacityNm, 3)
                Gearbox_ratio = "$ratio`:1"
                Efficiency = $efficiency
                Joint_speed_deg_s = $jointSpeedDegPerSec
                Joint_speed_rpm = [Math]::Round($jointSpeedRpm, 3)
                Motor_speed_rpm = [Math]::Round($motorSpeedRpm, 1)
                Required_motor_torque_Nm = [Math]::Round($requiredMotorTorque, 3)
                Percent_of_holding_torque = [Math]::Round($holdingPercent, 1)
                Output_power_W = [Math]::Round($outputPowerW, 2)
                Motor_mechanical_power_W = [Math]::Round($motorMechanicalPowerW, 2)
                Screen = $screen
            }
        }
    }
}

$recommended = @(
    [pscustomobject]@{
        Case = 'Current no counterbalance'
        Recommended_ratio_window = '100:1 to 120:1'
        Reason = '80:1 asks for about 51 percent of holding torque at 400 rpm with 75 percent efficiency; 100:1 drops this to about 41 percent at 500 rpm.'
        Status = 'Usable only if torque-speed curve confirms enough torque near 500-600 rpm'
    },
    [pscustomobject]@{
        Case = '5 Nm counterbalance'
        Recommended_ratio_window = '60:1 to 80:1'
        Reason = 'At 75 percent efficiency, 60:1 asks for about 46 percent of holding torque at 300 rpm and 80:1 asks for about 34 percent at 400 rpm.'
        Status = 'Healthier than current baseline'
    },
    [pscustomobject]@{
        Case = '8 Nm counterbalance'
        Recommended_ratio_window = '50:1 to 60:1'
        Reason = 'At 75 percent efficiency, 50:1 asks for about 38 percent of holding torque at 250 rpm and 60:1 asks for about 32 percent at 300 rpm.'
        Status = 'Preferred if counterbalance packaging is acceptable'
    },
    [pscustomobject]@{
        Case = 'Mass combo + 5 Nm counterbalance'
        Recommended_ratio_window = '50:1 to 60:1'
        Reason = 'Similar load level to the 8 N.m counterbalance case while using a smaller counterbalance.'
        Status = 'Best balanced mechanical direction'
    }
)

$report = [pscustomobject]@{
    Model_status = 'Preliminary J2 gearbox sizing screen'
    Inputs = [pscustomobject]@{
        Stepper_holding_torque_Nm = $stepperHoldingTorqueNm
        Target_J2_speed_deg_s = $jointSpeedDegPerSec
        Target_J2_speed_rpm = [Math]::Round($jointSpeedRpm, 3)
        Ratios_checked = $ratios
        Efficiencies_checked = $efficiencies
    }
    Recommended_windows = $recommended
    Results = @($rows)
}

$jsonPath = Join-Path $PSScriptRoot 'j2_gearbox_sizing_check_summary.json'
$report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $jsonPath -Encoding utf8
$report | ConvertTo-Json -Depth 8
