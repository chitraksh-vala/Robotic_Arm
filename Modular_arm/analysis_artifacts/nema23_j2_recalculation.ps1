$ErrorActionPreference = 'Stop'

# Current 40 mm OD / 36 mm ID carbon-fibre arm with a NEMA 23 at J2.
# Values are in SI units unless a property name says otherwise.
$g = 9.81
$alphaLower = 60.0 * [Math]::PI / 180.0
$gearboxEfficiency = 0.75
$utilization = 0.67

$L1 = 0.280
$L2 = 0.260
$Lt = 0.100

$mass = [ordered]@{
    UpperLink = 0.210
    UpperCable = 0.075
    J3Assembly = 1.160 # NEMA 17 with internal encoder, gearbox, brackets/bearings
    ForearmLink = 0.200
    ForearmCable = 0.075
    WristAssembly = 0.300
    ToolAssembly = 0.500
    Payload = 0.750
}

# The J2 NEMA 23 is centred on the J2 pitch axis, so it produces no J2 pitch
# gravity torque. Its body centre is approximately half the 113 mm body length
# away from the J1 yaw axis, and is included in J1 inertia.
$j2Motor = [ordered]@{
    Model = 'StepperOnline 23HS45-3504S'
    HoldingTorque_Nm = 3.00
    Mass_kg = 1.600
    EffectiveJ1YawRadius_m = 0.0565
    DynamicTorqueScreeningFraction = 0.50
}

function Get-GravityTorques {
    $xUpper = 0.5 * $L1
    $xJ3 = $L1
    $xForearm = $L1 + 0.5 * $L2
    $xWrist = $L1 + $L2
    $xTool = $xWrist + 0.050
    $xPayload = $xWrist + $Lt
    [pscustomobject]@{
        J2_Nm = $g * ($mass.UpperLink*$xUpper + $mass.UpperCable*$xUpper + $mass.J3Assembly*$xJ3 + $mass.ForearmLink*$xForearm + $mass.ForearmCable*$xForearm + $mass.WristAssembly*$xWrist + $mass.ToolAssembly*$xTool + $mass.Payload*$xPayload)
        J3_Nm = $g * ($mass.ForearmLink*(0.5*$L2) + $mass.ForearmCable*(0.5*$L2) + $mass.WristAssembly*$L2 + $mass.ToolAssembly*($L2+0.050) + $mass.Payload*($L2+$Lt))
    }
}

$gravity = Get-GravityTorques
$iJ2 = (1.0/3.0)*$mass.UpperLink*$L1*$L1 + $mass.UpperCable*[Math]::Pow(0.5*$L1,2) + $mass.J3Assembly*$L1*$L1 + $mass.ForearmLink*($L1*$L1+$L1*$L2+$L2*$L2/3.0) + $mass.ForearmCable*[Math]::Pow($L1+0.5*$L2,2) + $mass.WristAssembly*[Math]::Pow($L1+$L2,2) + $mass.ToolAssembly*[Math]::Pow($L1+$L2+0.050,2) + $mass.Payload*[Math]::Pow($L1+$L2+$Lt,2)
$iJ1 = $iJ2 + $j2Motor.Mass_kg * [Math]::Pow($j2Motor.EffectiveJ1YawRadius_m,2)
$iJ3 = (1.0/3.0)*$mass.ForearmLink*$L2*$L2 + $mass.ForearmCable*[Math]::Pow(0.5*$L2,2) + $mass.WristAssembly*$L2*$L2 + $mass.ToolAssembly*[Math]::Pow($L2+0.050,2) + $mass.Payload*[Math]::Pow($L2+$Lt,2)

$requirements = @(
    [pscustomobject]@{ Joint='J1'; Static=0.0; Accel=$iJ1*$alphaLower; Friction=0.20 },
    [pscustomobject]@{ Joint='J2'; Static=$gravity.J2_Nm; Accel=$iJ2*$alphaLower; Friction=0.10*$gravity.J2_Nm },
    [pscustomobject]@{ Joint='J3'; Static=$gravity.J3_Nm; Accel=$iJ3*$alphaLower; Friction=0.10*$gravity.J3_Nm }
) | ForEach-Object {
    $demand = $_.Static + $_.Accel + $_.Friction
    [pscustomobject]@{
        Joint = $_.Joint
        Static_Nm = [Math]::Round($_.Static,3)
        Acceleration_Nm = [Math]::Round($_.Accel,3)
        Friction_Nm = [Math]::Round($_.Friction,3)
        PreliminaryDemand_Nm = [Math]::Round($demand,3)
        RequiredOutputCapacity_Nm = [Math]::Round($demand/$utilization,3)
        MinimumGearboxContinuousRating_Nm = [Math]::Round($demand/$utilization,1)
        PreferredGearboxContinuousRating_Nm = [Math]::Round(1.5*$demand/$utilization,1)
    }
}

# Original NEMA 17 approximate curve previously supplied by user.
$nema17Curve = @(
    [pscustomobject]@{rpm=50; torque=0.78}, [pscustomobject]@{rpm=100; torque=0.76},
    [pscustomobject]@{rpm=200; torque=0.70}, [pscustomobject]@{rpm=300; torque=0.65}
)
function Get-Nema17Torque([double]$rpm) {
    if ($rpm -le 50) { return 0.78 }
    for ($i=0; $i -lt $nema17Curve.Count-1; $i++) {
        $a=$nema17Curve[$i]; $b=$nema17Curve[$i+1]
        if ($rpm -ge $a.rpm -and $rpm -le $b.rpm) { return $a.torque + (($rpm-$a.rpm)/($b.rpm-$a.rpm))*($b.torque-$a.torque) }
    }
    return 0.65
}

function Get-N17GearRows([string]$joint,[double]$required,[int[]]$ratios) {
    foreach($ratio in $ratios) {
        $motorRpm = 5.0*$ratio
        $motorTorque = Get-Nema17Torque $motorRpm
        $available = $motorTorque*$ratio*$gearboxEfficiency
        [pscustomobject]@{Joint=$joint; Motor='42AM08 NEMA 17'; Ratio="$ratio`:1"; MotorRpm=$motorRpm; ConservativeMotorTorque_Nm=[Math]::Round($motorTorque,3); AvailableOutput_Nm=[Math]::Round($available,2); Margin_x=[Math]::Round($available/$required,2)}
    }
}
function Get-N23GearRows([double]$required,[int[]]$ratios) {
    foreach($ratio in $ratios) {
        $motorRpm = 5.0*$ratio
        $motorTorque = $j2Motor.HoldingTorque_Nm*$j2Motor.DynamicTorqueScreeningFraction
        $available = $motorTorque*$ratio*$gearboxEfficiency
        [pscustomobject]@{Joint='J2'; Motor='23HS45-3504S NEMA 23'; Ratio="$ratio`:1"; MotorRpm=$motorRpm; ConservativeMotorTorque_Nm=[Math]::Round($motorTorque,3); AvailableOutput_Nm=[Math]::Round($available,2); Margin_x=[Math]::Round($available/$required,2)}
    }
}

$j1Required = ($requirements | Where-Object Joint -eq 'J1').RequiredOutputCapacity_Nm
$j2Required = ($requirements | Where-Object Joint -eq 'J2').RequiredOutputCapacity_Nm
$j3Required = ($requirements | Where-Object Joint -eq 'J3').RequiredOutputCapacity_Nm
$gearboxScreen = @(
    Get-N17GearRows -joint 'J1' -required $j1Required -ratios @(10,15)
    Get-N23GearRows -required $j2Required -ratios @(25,30,35,40)
    Get-N17GearRows -joint 'J3' -required $j3Required -ratios @(35,40,50)
)

$report = [pscustomobject]@{
    Model_status = 'Current NEMA 23 J2 / carbon-fibre link preliminary sizing pass'
    Geometry_mm = [pscustomobject]@{ L1=280; L2=260; ToolOffset=100 }
    J2_motor = $j2Motor
    Mass_model_kg = $mass
    Distal_mass_carried_by_J2_kg = [Math]::Round(($mass.Values | Measure-Object -Sum).Sum,3)
    Torque_requirements = @($requirements)
    Gearbox_screen = @($gearboxScreen)
    Recommended_ranges = @(
        [pscustomobject]@{Joint='J1'; RatioRange='10:1 to 15:1'; MarginRange='4.1x to 6.1x'; Reason='Original NEMA 17 curve has strong margin; favour backlash and bearing quality.'},
        [pscustomobject]@{Joint='J2'; RatioRange='30:1 to 40:1'; MarginRange='1.4x to 1.9x'; Reason='Uses 50% holding-torque dynamic screen until the actual driver-voltage curve is validated; 35:1 is the centre screening point.'},
        [pscustomobject]@{Joint='J3'; RatioRange='35:1 to 50:1'; MarginRange='2.1x to 2.8x'; Reason='Original NEMA 17 curve; 40:1 remains a good centre point.'}
    )
    Limitations = @(
        'J2 NEMA 23 curve in the manufacturer literature is at a stated test drive condition; final selection must use the curve for the purchased J2 driver and supply voltage.',
        'The current 1.600 kg NEMA 23 motor is included for J1 yaw inertia only. J2 gearbox mass and its exact centre of mass are not yet selected and must be added from CAD.',
        'Internal encoders are treated as part of motor mass. No external encoder mass is added.'
    )
}

$path = Join-Path $PSScriptRoot 'nema23_j2_recalculation_summary.json'
$report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $path -Encoding utf8
$report | ConvertTo-Json -Depth 8
