[CmdletBinding()]
param(
    [string]$Port,
    [string]$Output,
    [int]$Baud = 460800,
    [int]$DurationSeconds = 0,
    [switch]$ListPorts
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if($ListPorts)
{
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if($ports.Count -eq 0)
    {
        Write-Host "No serial ports found."
    }
    else
    {
        $ports | ForEach-Object { Write-Host $_ }
    }
    exit 0
}

if([string]::IsNullOrWhiteSpace($Port))
{
    throw "Port is required. Example: -Port COM6"
}

if($Baud -le 0)
{
    throw "Baud must be positive."
}

if($DurationSeconds -lt 0)
{
    throw "DurationSeconds cannot be negative."
}

if([string]::IsNullOrWhiteSpace($Output))
{
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Output = "kart_$stamp.bin"
}

$outputPath = [System.IO.Path]::GetFullPath($Output)
$outputDirectory = [System.IO.Path]::GetDirectoryName($outputPath)
if(-not [string]::IsNullOrWhiteSpace($outputDirectory) -and
   -not [System.IO.Directory]::Exists($outputDirectory))
{
    [System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $Baud,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 200
$serial.ReadBufferSize = 65536

$file = $null
$buffer = New-Object byte[] 8192
$totalBytes = [int64]0
$flushWatch = [System.Diagnostics.Stopwatch]::StartNew()
$runWatch = [System.Diagnostics.Stopwatch]::StartNew()

try
{
    $serial.Open()
    $file = [System.IO.File]::Open(
        $outputPath,
        [System.IO.FileMode]::Create,
        [System.IO.FileAccess]::Write,
        [System.IO.FileShare]::Read
    )

    Write-Host "Capturing $Port at $Baud baud (8N1, no flow control)"
    Write-Host "Output: $outputPath"
    if($DurationSeconds -gt 0)
    {
        Write-Host "Duration: $DurationSeconds seconds"
    }
    else
    {
        Write-Host "Press Ctrl+C to stop."
    }

    while($true)
    {
        if($DurationSeconds -gt 0 -and $runWatch.Elapsed.TotalSeconds -ge $DurationSeconds)
        {
            break
        }

        $available = $serial.BytesToRead
        if($available -le 0)
        {
            Start-Sleep -Milliseconds 2
            continue
        }

        $wanted = [int][System.Math]::Min($buffer.Length, $available)
        $read = $serial.Read($buffer, 0, $wanted)
        if($read -gt 0)
        {
            $file.Write($buffer, 0, $read)
            $totalBytes += $read
        }

        if($flushWatch.ElapsedMilliseconds -ge 1000)
        {
            $file.Flush()
            $flushWatch.Restart()
        }
    }
}
finally
{
    if($null -ne $file)
    {
        $file.Flush()
        $file.Dispose()
    }
    if($serial.IsOpen)
    {
        $serial.Close()
    }
    $serial.Dispose()
    Write-Host "Saved $totalBytes bytes."
}
