param(
    [string]$OutputDirectory = $PSScriptRoot
)

$ErrorActionPreference = 'Stop'

$L1 = 280.0
$L2 = 260.0
$H = 180.0
$Lt = 100.0

$J2Min = -90.0
$J2Max = 110.0
$J3Min = -140.0
$J3Max = 140.0
$J4Min = -130.0
$J4Max = 130.0

$Degree = 180.0 / [Math]::PI

function Get-WorkspaceState {
    param(
        [double]$R,
        [double]$Z,
        [double]$PhiDegrees
    )

    $phi = $PhiDegrees / $Degree
    $wristR = $R - $Lt * [Math]::Cos($phi)
    $wristZ = $Z - $Lt * [Math]::Sin($phi)
    $x = $wristR
    $dz = $wristZ - $H
    $distanceSquared = $x * $x + $dz * $dz
    $cosQ3 = ($distanceSquared - $L1 * $L1 - $L2 * $L2) / (2.0 * $L1 * $L2)

    if ($cosQ3 -lt -1.0 -or $cosQ3 -gt 1.0) {
        return [pscustomobject]@{
            Code = 0
            Classification = 'Geometrically unreachable'
            NegativeValid = $false
            PositiveValid = $false
        }
    }

    $negativeValid = $false
    $positiveValid = $false

    foreach ($branch in @(-1.0, 1.0)) {
        $q3 = $branch * [Math]::Acos([Math]::Max(-1.0, [Math]::Min(1.0, $cosQ3)))
        $q2 = [Math]::Atan2($dz, $x) - [Math]::Atan2(
            $L2 * [Math]::Sin($q3),
            $L1 + $L2 * [Math]::Cos($q3)
        )
        $q4 = $phi - $q2 - $q3

        $q2Degrees = $q2 * $Degree
        $q3Degrees = $q3 * $Degree
        $q4Degrees = $q4 * $Degree

        $valid = (
            $q2Degrees -ge $J2Min -and $q2Degrees -le $J2Max -and
            $q3Degrees -ge $J3Min -and $q3Degrees -le $J3Max -and
            $q4Degrees -ge $J4Min -and $q4Degrees -le $J4Max
        )

        if ($branch -lt 0) {
            $negativeValid = $valid
        }
        else {
            $positiveValid = $valid
        }
    }

    if ($negativeValid -and $positiveValid) {
        $code = 3
        $classification = 'Both elbow branches'
    }
    elseif ($negativeValid) {
        $code = 1
        $classification = 'Negative-elbow branch only'
    }
    elseif ($positiveValid) {
        $code = 2
        $classification = 'Positive-elbow branch only'
    }
    else {
        $code = 4
        $classification = 'Geometry valid but joint-limit excluded'
    }

    return [pscustomobject]@{
        Code = $code
        Classification = $classification
        NegativeValid = $negativeValid
        PositiveValid = $positiveValid
    }
}

function Get-MapDefinition {
    param([string]$Name)

    if ($Name -eq 'downward') {
        return [pscustomobject]@{
            Name = 'downward'
            Title = 'Vertically Downward Tool Workspace'
            PhiDegrees = -90.0
            CsvName = 'workspace_map_downward_5mm.csv'
            SvgName = 'workspace_map_downward.svg'
            EnvelopeName = 'workspace_envelope_downward_1mm.csv'
        }
    }

    return [pscustomobject]@{
        Name = 'horizontal'
        Title = 'Horizontal Forward Tool Workspace'
        PhiDegrees = 0.0
        CsvName = 'workspace_map_horizontal_5mm.csv'
        SvgName = 'workspace_map_horizontal.svg'
        EnvelopeName = 'workspace_envelope_horizontal_1mm.csv'
    }
}

function Write-WorkspaceSvg {
    param(
        [object[]]$Rows,
        [pscustomobject]$Definition,
        [string]$Path
    )

    $cell = 4
    $plotWidth = 81 * $cell
    $plotHeight = 131 * $cell
    $left = 70
    $top = 45
    $width = $left + $plotWidth + 210
    $height = $top + $plotHeight + 70
    $colors = @{
        0 = '#eeeeee'
        1 = '#1f77b4'
        2 = '#ff7f0e'
        3 = '#2ca02c'
        4 = '#d62728'
    }

    $builder = [System.Text.StringBuilder]::new()
    [void]$builder.AppendLine("<svg xmlns='http://www.w3.org/2000/svg' width='$width' height='$height' viewBox='0 0 $width $height'>")
    [void]$builder.AppendLine("<rect width='100%' height='100%' fill='white'/>")
    [void]$builder.AppendLine("<text x='$left' y='24' font-family='Arial' font-size='18' font-weight='bold'>$($Definition.Title)</text>")

    foreach ($row in $Rows) {
        $x = $left + [int](($row.R_mm - 200) / 5) * $cell
        $y = $top + [int]((650 - $row.Z_mm) / 5) * $cell
        $fill = $colors[[int]$row.Code]
        [void]$builder.AppendLine("<rect x='$x' y='$y' width='$cell' height='$cell' fill='$fill'/>")
    }

    [void]$builder.AppendLine("<rect x='$left' y='$top' width='$plotWidth' height='$plotHeight' fill='none' stroke='black'/>")
    [void]$builder.AppendLine("<text x='$($left + $plotWidth / 2 - 35)' y='$($top + $plotHeight + 42)' font-family='Arial' font-size='14'>R (mm)</text>")
    [void]$builder.AppendLine("<text x='18' y='$($top + $plotHeight / 2)' font-family='Arial' font-size='14' transform='rotate(-90 18 $($top + $plotHeight / 2))'>Z (mm)</text>")

    foreach ($rTick in @(200, 300, 400, 500, 600)) {
        $x = $left + (($rTick - 200) / 5) * $cell
        [void]$builder.AppendLine("<line x1='$x' y1='$($top + $plotHeight)' x2='$x' y2='$($top + $plotHeight + 6)' stroke='black'/>")
        [void]$builder.AppendLine("<text x='$($x - 12)' y='$($top + $plotHeight + 22)' font-family='Arial' font-size='11'>$rTick</text>")
    }

    foreach ($zTick in @(0, 100, 200, 300, 400, 500, 600, 650)) {
        $y = $top + ((650 - $zTick) / 5) * $cell
        [void]$builder.AppendLine("<line x1='$($left - 6)' y1='$y' x2='$left' y2='$y' stroke='black'/>")
        [void]$builder.AppendLine("<text x='$($left - 38)' y='$($y + 4)' font-family='Arial' font-size='11'>$zTick</text>")
    }

    $legendX = $left + $plotWidth + 24
    $legendY = $top + 20
    $legend = @(
        @{ Code = 3; Text = 'Both branches' },
        @{ Code = 1; Text = 'Negative only' },
        @{ Code = 2; Text = 'Positive only' },
        @{ Code = 4; Text = 'Joint-limit excluded' },
        @{ Code = 0; Text = 'Geometric unreachable' }
    )
    foreach ($entry in $legend) {
        $fill = $colors[$entry.Code]
        [void]$builder.AppendLine("<rect x='$legendX' y='$legendY' width='14' height='14' fill='$fill' stroke='#555'/>")
        [void]$builder.AppendLine("<text x='$($legendX + 22)' y='$($legendY + 12)' font-family='Arial' font-size='12'>$($entry.Text)</text>")
        $legendY += 24
    }

    $markers = if ($Definition.Name -eq 'downward') {
        @(
            @{ R = 500; Z = 0; Label = 'CP1' },
            @{ R = 200; Z = 0; Label = 'CP2' }
        )
    }
    else {
        @(
            @{ R = 300; Z = 650; Label = 'CP3' },
            @{ R = 600; Z = 180; Label = 'R600' }
        )
    }

    foreach ($marker in $markers) {
        $x = $left + (($marker.R - 200) / 5) * $cell + $cell / 2
        $y = $top + ((650 - $marker.Z) / 5) * $cell + $cell / 2
        [void]$builder.AppendLine("<circle cx='$x' cy='$y' r='5' fill='none' stroke='black' stroke-width='2'/>")
        [void]$builder.AppendLine("<text x='$($x + 7)' y='$($y - 5)' font-family='Arial' font-size='11' font-weight='bold'>$($marker.Label)</text>")
    }

    [void]$builder.AppendLine('</svg>')
    [System.IO.File]::WriteAllText($Path, $builder.ToString(), [System.Text.UTF8Encoding]::new($false))
}

function Invoke-WorkspaceMap {
    param([pscustomobject]$Definition)

    $coarseRows = [System.Collections.Generic.List[object]]::new()
    foreach ($z in 0..130) {
        $zValue = $z * 5
        foreach ($r in 0..80) {
            $rValue = 200 + $r * 5
            $state = Get-WorkspaceState -R $rValue -Z $zValue -PhiDegrees $Definition.PhiDegrees
            $coarseRows.Add([pscustomobject]@{
                R_mm = $rValue
                Z_mm = $zValue
                Code = $state.Code
                Classification = $state.Classification
                Negative_elbow_valid = $state.NegativeValid
                Positive_elbow_valid = $state.PositiveValid
            })
        }
    }

    $csvPath = Join-Path $OutputDirectory $Definition.CsvName
    $svgPath = Join-Path $OutputDirectory $Definition.SvgName
    $envelopePath = Join-Path $OutputDirectory $Definition.EnvelopeName
    $coarseRows | Export-Csv -LiteralPath $csvPath -NoTypeInformation
    Write-WorkspaceSvg -Rows $coarseRows -Definition $Definition -Path $svgPath

    $envelope = [System.Collections.Generic.List[object]]::new()
    $refinedCounts = @{ 0 = 0; 1 = 0; 2 = 0; 3 = 0; 4 = 0 }
    foreach ($rValue in 200..600) {
        $reachableZ = [System.Collections.Generic.List[int]]::new()
        foreach ($zValue in 0..650) {
            $state = Get-WorkspaceState -R $rValue -Z $zValue -PhiDegrees $Definition.PhiDegrees
            $refinedCounts[$state.Code]++
            if ($state.Code -in @(1, 2, 3)) {
                $reachableZ.Add($zValue)
            }
        }

        if ($reachableZ.Count -gt 0) {
            $envelope.Add([pscustomobject]@{
                R_mm = $rValue
                Min_reachable_Z_mm = ($reachableZ | Measure-Object -Minimum).Minimum
                Max_reachable_Z_mm = ($reachableZ | Measure-Object -Maximum).Maximum
                Reachable_Z_samples = $reachableZ.Count
            })
        }
        else {
            $envelope.Add([pscustomobject]@{
                R_mm = $rValue
                Min_reachable_Z_mm = $null
                Max_reachable_Z_mm = $null
                Reachable_Z_samples = 0
            })
        }
    }
    $envelope | Export-Csv -LiteralPath $envelopePath -NoTypeInformation

    $totalRefined = 401 * 651
    $reachableRefined = $refinedCounts[1] + $refinedCounts[2] + $refinedCounts[3]
    $coarseCounts = $coarseRows | Group-Object Code | ForEach-Object {
        [pscustomobject]@{ Code = [int]$_.Name; Count = $_.Count }
    }

    $keySlices = foreach ($zValue in @(0, 180, 650)) {
        $reachableR = [System.Collections.Generic.List[int]]::new()
        foreach ($rValue in 200..600) {
            $state = Get-WorkspaceState -R $rValue -Z $zValue -PhiDegrees $Definition.PhiDegrees
            if ($state.Code -in @(1, 2, 3)) {
                $reachableR.Add($rValue)
            }
        }
        [pscustomobject]@{
            Z_mm = $zValue
            Min_reachable_R_mm = if ($reachableR.Count) { ($reachableR | Measure-Object -Minimum).Minimum } else { $null }
            Max_reachable_R_mm = if ($reachableR.Count) { ($reachableR | Measure-Object -Maximum).Maximum } else { $null }
        }
    }

    return [pscustomobject]@{
        Name = $Definition.Name
        Phi_degrees = $Definition.PhiDegrees
        Coarse_grid_points = $coarseRows.Count
        Coarse_counts = $coarseCounts
        Refined_grid_points = $totalRefined
        Refined_reachable_points = $reachableRefined
        Refined_reachable_percent = [Math]::Round(100.0 * $reachableRefined / $totalRefined, 2)
        Refined_counts = [pscustomobject]@{
            Geometrically_unreachable = $refinedCounts[0]
            Negative_only = $refinedCounts[1]
            Positive_only = $refinedCounts[2]
            Both_branches = $refinedCounts[3]
            Joint_limit_excluded = $refinedCounts[4]
        }
        Key_Z_slices = $keySlices
        Files = [pscustomobject]@{
            Coarse_csv = $csvPath
            Svg = $svgPath
            Refined_envelope_csv = $envelopePath
        }
    }
}

$summaries = foreach ($name in @('downward', 'horizontal')) {
    Invoke-WorkspaceMap -Definition (Get-MapDefinition -Name $name)
}

$summaryPath = Join-Path $OutputDirectory 'workspace_map_summary.json'
$summaries | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $summaryPath -Encoding utf8
$summaries | ConvertTo-Json -Depth 8
