$ErrorActionPreference = 'Stop'

# Product-specific sizing baseline (2026-10-14).
# Links: Vortex-RC 40 x 36 mm 3K roll-wrapped CF tube (listed 1.100 kg/m).
# J1/J3/J4/J5: Rtelligent 42AM08. The Robu URL says 0.71 Nm while the current
# Rtelligent sheet says 0.80 Nm typical, so this script uses 0.71 Nm conservatively.
# J2: StepperOnline 23HS45-3504S, 3.0 Nm holding torque, 1.60 kg.
$g = 9.81
$alpha = 60.0 * [Math]::PI / 180.0
$utilization = 0.67
$efficiency = 0.75
$L1 = 0.280; $L2 = 0.260; $Lt = 0.100
$tubeListedMassPerM = 1.100
$upperTubeMass = $tubeListedMassPerM*$L1
$forearmTubeMass = $tubeListedMassPerM*$L2

# Conservative packaging: both wrist motors and gearboxes travel with the
# wrist. J4 actuator is centered at wrist; J5 actuator is 50 mm distal of it.
$mass = [ordered]@{
    UpperLink = [math]::Round($upperTubeMass+0.102,3); UpperCable = 0.075; J3Assembly = 1.160
    ForearmLink = [math]::Round($forearmTubeMass+0.104,3); ForearmCable = 0.075
    J4Assembly = 0.910 # 42AM08 (0.510) + 10-15:1 gearbox (0.250) + support (0.150)
    J5Assembly = 0.760 # 42AM08 (0.510) + 5-10:1 gearbox (0.150) + support (0.100)
    ToolAssembly = 0.500; Payload = 0.750
}
$j2Nema23Mass = 1.600; $j2Nema23YawRadius = 0.0565

function Get-Gravity {
    $xU=0.5*$L1; $xJ3=$L1; $xF=$L1+0.5*$L2; $xW=$L1+$L2
    $xJ5=$xW+0.050; $xTool=$xW+0.050; $xPay=$xW+$Lt
    [pscustomobject]@{
        J2 = $g*($mass.UpperLink*$xU+$mass.UpperCable*$xU+$mass.J3Assembly*$xJ3+$mass.ForearmLink*$xF+$mass.ForearmCable*$xF+$mass.J4Assembly*$xW+$mass.J5Assembly*$xJ5+$mass.ToolAssembly*$xTool+$mass.Payload*$xPay)
        J3 = $g*($mass.ForearmLink*(0.5*$L2)+$mass.ForearmCable*(0.5*$L2)+$mass.J4Assembly*$L2+$mass.J5Assembly*($L2+0.050)+$mass.ToolAssembly*($L2+0.050)+$mass.Payload*($L2+$Lt))
        J4 = $g*($mass.J5Assembly*0.050+$mass.ToolAssembly*0.050+$mass.Payload*$Lt)
        J5 = 1.25*$g*0.025
    }
}
$grav=Get-Gravity
$iJ2=(1.0/3.0)*$mass.UpperLink*$L1*$L1+$mass.UpperCable*[Math]::Pow(0.5*$L1,2)+$mass.J3Assembly*$L1*$L1+$mass.ForearmLink*($L1*$L1+$L1*$L2+$L2*$L2/3.0)+$mass.ForearmCable*[Math]::Pow($L1+0.5*$L2,2)+$mass.J4Assembly*[Math]::Pow($L1+$L2,2)+$mass.J5Assembly*[Math]::Pow($L1+$L2+0.050,2)+$mass.ToolAssembly*[Math]::Pow($L1+$L2+0.050,2)+$mass.Payload*[Math]::Pow($L1+$L2+$Lt,2)
$iJ1=$iJ2+$j2Nema23Mass*[Math]::Pow($j2Nema23YawRadius,2)
$iJ3=(1.0/3.0)*$mass.ForearmLink*$L2*$L2+$mass.ForearmCable*[Math]::Pow(0.5*$L2,2)+$mass.J4Assembly*$L2*$L2+$mass.J5Assembly*[Math]::Pow($L2+0.050,2)+$mass.ToolAssembly*[Math]::Pow($L2+0.050,2)+$mass.Payload*[Math]::Pow($L2+$Lt,2)
$iJ4=$mass.J5Assembly*0.050*0.050+$mass.ToolAssembly*0.050*0.050+$mass.Payload*$Lt*$Lt

$raw=@(
    [pscustomobject]@{Joint='J1';Static=0.0;Accel=$iJ1*$alpha;Friction=0.20;Speed=30},
    [pscustomobject]@{Joint='J2';Static=$grav.J2;Accel=$iJ2*$alpha;Friction=0.10*$grav.J2;Speed=30},
    [pscustomobject]@{Joint='J3';Static=$grav.J3;Accel=$iJ3*$alpha;Friction=0.10*$grav.J3;Speed=30},
    [pscustomobject]@{Joint='J4';Static=$grav.J4;Accel=$iJ4*$alpha;Friction=0.10*$grav.J4;Speed=60},
    [pscustomobject]@{Joint='J5';Static=$grav.J5;Accel=(1.25*0.025*0.025)*$alpha;Friction=0.10*$grav.J5;Speed=60}
)
$requirements=$raw | ForEach-Object {$d=$_.Static+$_.Accel+$_.Friction;$cap=$d/$utilization;[pscustomobject]@{Joint=$_.Joint;TargetSpeed_deg_s=$_.Speed;Static_Nm=[math]::Round($_.Static,3);Acceleration_Nm=[math]::Round($_.Accel,3);Friction_Nm=[math]::Round($_.Friction,3);RequiredOutputCapacity_Nm=[math]::Round($cap,3);MinimumGearboxContinuousRating_Nm=[math]::Round($cap,1);PreferredGearboxContinuousRating_Nm=[math]::Round(1.5*$cap,1);PreferredPeakRating_Nm=[math]::Round(2*$cap,1)}}

$n17HoldConservative = 0.71
$n17CurveScale = $n17HoldConservative/0.80
$curve=@([pscustomobject]@{rpm=50;torque=0.78*$n17CurveScale},[pscustomobject]@{rpm=100;torque=0.76*$n17CurveScale},[pscustomobject]@{rpm=200;torque=0.70*$n17CurveScale},[pscustomobject]@{rpm=300;torque=0.65*$n17CurveScale},[pscustomobject]@{rpm=400;torque=0.58*$n17CurveScale},[pscustomobject]@{rpm=500;torque=0.53*$n17CurveScale},[pscustomobject]@{rpm=600;torque=0.49*$n17CurveScale})
function Get-N17Torque([double]$rpm){if($rpm -le 50){return $curve[0].torque};for($i=0;$i -lt $curve.Count-1;$i++){$a=$curve[$i];$b=$curve[$i+1];if($rpm -ge $a.rpm -and $rpm -le $b.rpm){return $a.torque+(($rpm-$a.rpm)/($b.rpm-$a.rpm))*($b.torque-$a.torque)}};return $curve[$curve.Count-1].torque}
function Screen-N17([string]$joint,[int[]]$ratios){$req=($requirements|Where-Object Joint -eq $joint);foreach($ratio in $ratios){$rpm=($req.TargetSpeed_deg_s/6.0)*$ratio;$motor=Get-N17Torque $rpm;$out=$motor*$ratio*$efficiency;[pscustomobject]@{Joint=$joint;Motor='42AM08 NEMA 17';Ratio="$ratio`:1";MotorRpm=[math]::Round($rpm,0);MotorTorque_Nm=[math]::Round($motor,3);OutputTorque_Nm=[math]::Round($out,2);Margin_x=[math]::Round($out/$req.RequiredOutputCapacity_Nm,2)}}}
function Screen-N23([int[]]$ratios){$req=($requirements|Where-Object Joint -eq 'J2');foreach($ratio in $ratios){$rpm=($req.TargetSpeed_deg_s/6.0)*$ratio;$out=1.50*$ratio*$efficiency;[pscustomobject]@{Joint='J2';Motor='23HS45-3504S NEMA 23';Ratio="$ratio`:1";MotorRpm=[math]::Round($rpm,0);MotorTorque_Nm=1.5;OutputTorque_Nm=[math]::Round($out,2);Margin_x=[math]::Round($out/$req.RequiredOutputCapacity_Nm,2)}}}

$screen=@(Screen-N17 'J1' @(10,15);Screen-N23 @(50,60);Screen-N17 'J3' @(50,60);Screen-N17 'J4' @(10,15);Screen-N17 'J5' @(5,8,10))
$report=[pscustomobject]@{Model_status='Current all-stepper arm sizing: NEMA 17 at J1/J3/J4/J5 and NEMA 23 at J2';Geometry_mm=[pscustomobject]@{L1=280;L2=260;ToolOffset=100};MassModel_kg=$mass;J2_carried_distal_mass_kg=[math]::Round(($mass.Values|Measure-Object -Sum).Sum,3);WristPackaging='Conservative wrist-mounted J4 and J5 motors/gearboxes';TorqueRequirements=@($requirements);GearboxScreen=@($screen);RecommendedRanges=@([pscustomobject]@{Joint='J1';RatioRange='10:1 to 15:1';MarginRange='2.8x to 4.1x'},[pscustomobject]@{Joint='J2';RatioRange='50:1 to 60:1';MarginRange='1.5x to 1.8x'},[pscustomobject]@{Joint='J3';RatioRange='50:1 to 60:1';MarginRange='1.6x to 1.9x'},[pscustomobject]@{Joint='J4';RatioRange='10:1 to 15:1';MarginRange='2.5x to 3.7x'},[pscustomobject]@{Joint='J5';RatioRange='5:1 to 10:1';MarginRange='5.8x to 11.3x'});Limitations=@('J2 NEMA 23 uses 50% holding-torque conservative dynamic screening until the exact driver/supply curve is validated.','J4/J5 mass model assumes local wrist mounting. Remote/belt-drive packaging can materially lower J2 and J3 requirements.','No external encoder mass is included; internal encoders are part of motor mass.')}
$report = [pscustomobject]@{
    Model_status = 'Selected-products arm sizing: 42AM08 at J1/J3/J4/J5; 23HS45-3504S at J2'
    ProductBasis = [pscustomobject]@{ Tube='Vortex-RC 40x36x1000 3K roll-wrapped CF'; TubeListedMass_kg_per_m=$tubeListedMassPerM; Nema17HoldingTorqueConservative_Nm=$n17HoldConservative; Nema23HoldingTorque_Nm=3.0; Nema23Mass_kg=1.6 }
    Geometry_mm = [pscustomobject]@{ L1=280; L2=260; ToolOffset=100 }
    MassModel_kg = $mass
    J2_carried_distal_mass_kg = [math]::Round(($mass.Values|Measure-Object -Sum).Sum,3)
    TorqueRequirements = @($requirements)
    GearboxScreen = @($screen)
    Limitations = @('Supplier tube mass is treated as 1.100 kg/m conservatively; weigh delivered tube.','J2 uses a 50%-of-holding-torque dynamic screen until a driver/supply curve is validated.','No external encoder mass is included; verify the purchased motor SKU if integrated feedback is required.')
}
$report|ConvertTo-Json -Depth 8|Set-Content -LiteralPath (Join-Path $PSScriptRoot 'nema17_wrist_recalculation_summary.json') -Encoding utf8
$report|ConvertTo-Json -Depth 8
