<#
.SYNOPSIS
    Keep QGroundControl's saved Comm Link pointed at the current WSL2 interface IP.

.DESCRIPTION
    QGroundControl on Windows must connect to the SAS/mavlink-bridge stack running
    inside WSL2 via that instance's interface IP (e.g. 172.28.x.x), NOT `localhost` --
    WSL2's localhost-forwarding shim silently drops UDP, which is what actually broke
    inbound mission upload/emergency wipe for a long time before this was understood
    (see docs/IMPLEMENTATION_STATUS.md, "A long-standing assumption, disproved").

    That interface IP changes every time WSL restarts, so the Comm Link QGC has saved
    goes stale on every restart and has to be corrected by hand in the QGC UI
    (Application Settings -> Comm Links) before the link will connect again. This
    script automates that edit: it reads QGroundControl.ini directly, finds the UDP
    link on port 14550 (the single external port every mavlink-bridge node shares,
    per mavlink_router_node.py), and rewrites its host to the WSL2 IP right now.

    QGC reads this file at startup, so the intended flow is: run this script BEFORE
    launching QGC (or with QGC already closed), then start QGC -- its saved link (if
    `auto=true`, as QGC sets when "Automatically Connect" is checked) will connect
    immediately using the corrected IP. If QGC is already running when you edit the
    file, its own settings model is already loaded from disk and won't notice the
    change until QGC restarts or the link is manually reconnected, so this script
    warns (but does not refuse) if QGC is currently running.

    A .bak copy of the ini is written before any change, so this is always reversible.

.PARAMETER WslDistro
    The WSL distribution to query for its interface IP. Default: Ubuntu-24.04
    (matches every other script/doc in this project).

.PARAMETER Port
    The external MAVLink UDP port to match the Comm Link on. Default: 14550
    (mavlink_router_node's external bind -- see mavlink-bridge/launch_sas_qgc_integration.py).

.PARAMETER IniPath
    Path to QGroundControl.ini. Default: the standard per-user location
    ($env:APPDATA\QGroundControl\QGroundControl.ini).

.EXAMPLE
    .\scripts\sync_qgc_link_ip.ps1
        Sync using the defaults (Ubuntu-24.04, port 14550).

.EXAMPLE
    .\scripts\sync_qgc_link_ip.ps1 -WslDistro Ubuntu-24.04 -Port 14550
        Same as above, explicit.
#>

[CmdletBinding()]
param(
    [string]$WslDistro = "Ubuntu-24.04",
    [int]$Port = 14550,
    [string]$IniPath = "$env:APPDATA\QGroundControl\QGroundControl.ini"
)

$ErrorActionPreference = "Stop"

function Get-WslInterfaceIP {
    param([string]$Distro)

    # ip -4 addr show eth0 | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1
    # Run as one bash -c string so this works identically whether invoked from
    # PowerShell or (if ever needed) from a Git-Bash wrapper -- avoids the
    # inline-multiline-bash-c mangling this project has hit before with the
    # Git-Bash-to-wsl.exe path (see scratch script history); a single-line
    # bash -c with no embedded $variables of its own is safe from that.
    $raw = & wsl -d $Distro -- bash -c "ip -4 addr show eth0 | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}/[0-9]+' | head -1"
    if (-not $raw) {
        throw "Could not determine an IPv4 address for eth0 in WSL distro '$Distro'. Is it running? (wsl -d $Distro -- true)"
    }
    return ($raw.Trim() -split '/')[0]
}

function Read-IniLines {
    param([string]$Path)
    if (-not (Test-Path $Path)) {
        throw "QGroundControl.ini not found at '$Path'. Has QGC been run at least once to create a Comm Link?"
    }
    return Get-Content -Path $Path -Encoding UTF8
}

# --- 1. Find the current WSL2 interface IP ---
$currentIp = Get-WslInterfaceIP -Distro $WslDistro
Write-Host "WSL2 ($WslDistro) interface IP: $currentIp"

# --- 2. Warn (don't block) if QGC is currently running ---
$qgcProc = Get-Process -Name "*QGroundControl*" -ErrorAction SilentlyContinue
if ($qgcProc) {
    Write-Warning "QGroundControl is currently running (PID $($qgcProc.Id -join ', ')). It has already " +
                   "loaded its settings from disk, so this change won't take effect until QGC is " +
                   "restarted, or the link is disconnected/reconnected by hand."
}

# --- 3. Parse QGroundControl.ini's [LinkConfigurations] section ---
$lines = Read-IniLines -Path $IniPath

$sectionStart = -1
$sectionEnd = $lines.Count
for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i].Trim() -eq "[LinkConfigurations]") {
        $sectionStart = $i
        continue
    }
    if ($sectionStart -ge 0 -and $lines[$i] -match '^\[.+\]$') {
        $sectionEnd = $i
        break
    }
}

if ($sectionStart -lt 0) {
    throw "No [LinkConfigurations] section found in '$IniPath'. Create at least one UDP Comm " +
          "Link (port $Port) in QGC's Application Settings first -- this script updates an " +
          "existing link, it does not create one."
}

# --- 4. Find the link whose type is UDP (1) and port is $Port ---
$countLine = $lines[($sectionStart + 1)..($sectionEnd - 1)] | Where-Object { $_ -match '^count=(\d+)' }
$linkCount = if ($countLine) { [int]($Matches[1]) } else { 0 }

$targetIndex = -1
for ($n = 0; $n -lt $linkCount; $n++) {
    $typeMatch = $lines[$sectionStart..($sectionEnd - 1)] | Where-Object { $_ -match "^Link$n\\type=(\d+)" }
    $portMatch = $lines[$sectionStart..($sectionEnd - 1)] | Where-Object { $_ -match "^Link$n\\port=(\d+)" }
    if ($typeMatch -and $portMatch) {
        $type = [int]([regex]::Match($typeMatch, '=(\d+)').Groups[1].Value)
        $port = [int]([regex]::Match($portMatch, '=(\d+)').Groups[1].Value)
        if ($type -eq 1 -and $port -eq $Port) {
            $targetIndex = $n
            break
        }
    }
}

if ($targetIndex -lt 0) {
    throw "No UDP Comm Link on port $Port found in '$IniPath'. This script updates an existing " +
          "link's host, it does not create one -- add a UDP link on port $Port in QGC's " +
          "Application Settings -> Comm Links first, then re-run this script."
}

# --- 5. Update every hostN entry for that link (normally just host0) ---
$hostPattern = "^Link$targetIndex\\host(\d+)="
$changed = $false
$oldIp = $null

for ($i = $sectionStart; $i -lt $sectionEnd; $i++) {
    if ($lines[$i] -match $hostPattern) {
        $prefix = ($lines[$i] -split '=')[0]
        $existing = ($lines[$i] -split '=', 2)[1]
        if ($existing -ne $currentIp) {
            if (-not $oldIp) { $oldIp = $existing }
            $lines[$i] = "$prefix=$currentIp"
            $changed = $true
        }
    }
}

if (-not $changed) {
    Write-Host "Link$targetIndex (UDP, port $Port) already points at $currentIp -- nothing to do." -ForegroundColor Green
    exit 0
}

# --- 6. Back up, then write ---
$backupPath = "$IniPath.bak"
Copy-Item -Path $IniPath -Destination $backupPath -Force
Write-Host "Backed up existing ini to '$backupPath'"

Set-Content -Path $IniPath -Value $lines -Encoding UTF8
Write-Host "Updated Link$targetIndex (UDP, port $Port): $oldIp -> $currentIp" -ForegroundColor Green
Write-Host "Restart QGroundControl (or disconnect/reconnect the link) for the change to take effect."
