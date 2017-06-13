function Bench ([ScriptBlock]$Expr, [int]$Samples=10) {
    $total_ms = 0
    $max = 0
    $min = [double]::PositiveInfinity
    $cur_samples = $Samples
    do {
        $time = (Measure-Command $expr).TotalMilliseconds
        $total_ms += $time
        if ($time -gt $max) { $max = $time }
        if ($time -lt $min) { $min = $time }
        $cur_samples--
    } while ($cur_samples -gt 0)
    Write-Host "Tot: $total_ms"
    Write-Host "Avg: $($total_ms / $Samples)"
    Write-Host "Max: $max"
    Write-Host "Min: $min"
}

Export-ModuleMember Bench