#!/usr/bin/env bash
# Pair/connect Bluetooth earphones (e.g. AirPods) and make them the default
# audio output, so hazard_detector (OpenAL -> PipeWire/PulseAudio) plays on them.
#
# Run it on the machine that owns the Bluetooth adapter (the UNO Q, or your
# laptop when testing with Docker), NOT inside the container.
set -euo pipefail

usage() {
  cat <<EOF
Usage:
  $0 scan              Scan for 20 s and list nearby Bluetooth devices
  $0 connect <MAC>     Pair (if needed), trust, connect and set as default output
  $0 status            Show connected devices and the default audio output
  $0 disconnect <MAC>  Disconnect the device

AirPods pairing mode: AirPods in the case, lid open, hold the button on the
back of the case until the LED flashes white.
EOF
}

die() { echo "Error: $*" >&2; exit 1; }

require() {
  for cmd in "$@"; do
    command -v "$cmd" >/dev/null || die "'$cmd' not found (install bluez / pulseaudio-utils or pipewire-pulseaudio)"
  done
}

check_mac() {
  [[ "${1:-}" =~ ^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$ ]] || die "invalid MAC address '${1:-}' (expected XX:XX:XX:XX:XX:XX)"
}

# First PipeWire/PulseAudio sink or card whose name contains the MAC (XX_XX_...)
find_by_mac() { # <sinks|cards> <MAC>
  local id="${2//:/_}"
  pactl list short "$1" | awk -v id="${id^^}" 'toupper($2) ~ id { print $2; exit }'
}

# Classic (BR/EDR) link up = audio possible. Older BlueZ only prints "Connected:".
is_connected() {
  bluetoothctl info "$1" 2>/dev/null | grep -qE "^\s*(BREDR\.)?Connected: yes"
}

cmd_scan() {
  require bluetoothctl
  bluetoothctl power on >/dev/null
  echo "Scanning for 20 s..."
  bluetoothctl --timeout 20 scan on >/dev/null || true
  bluetoothctl devices
}

cmd_connect() {
  local mac="${1:-}"
  check_mac "$mac"
  require bluetoothctl pactl

  bluetoothctl power on >/dev/null

  if ! bluetoothctl info "$mac" 2>/dev/null | grep -q "Paired: yes"; then
    echo "Device not paired yet: put it in pairing mode now."
    echo "Scanning for 15 s..."
    bluetoothctl --timeout 15 scan on >/dev/null || true
    bluetoothctl pair "$mac" || die "pairing failed (see README > Troubleshooting)"
  fi

  bluetoothctl trust "$mac" >/dev/null

  # AirPods are dual-mode: bluetoothctl may try LE first and report
  # "le-connection-abort-by-local" while the classic (BR/EDR) audio link, the
  # one we need, comes up anyway. So retry and check the real state.
  local i
  for i in 1 2 3; do
    is_connected "$mac" && break
    bluetoothctl connect "$mac" || true
    sleep 2
  done
  is_connected "$mac" || die "connection failed (see README > Troubleshooting: ControllerMode = bredr)"
  echo "Connected."

  echo "Waiting for the audio card..."
  local card=""
  for i in $(seq 1 10); do
    card="$(find_by_mac cards "$mac")"
    [[ -n "$card" ]] && break
    sleep 1
  done
  [[ -n "$card" ]] || die "no audio card for $mac (is the Bluetooth audio plugin installed? Debian: libspa-0.2-bluetooth or pulseaudio-module-bluetooth)"

  # Force A2DP (high quality, output only). PipeWire and PulseAudio name it differently.
  pactl set-card-profile "$card" a2dp-sink 2>/dev/null \
    || pactl set-card-profile "$card" a2dp_sink 2>/dev/null \
    || echo "Warning: could not force the A2DP profile, keeping the current one"

  local sink=""
  for i in $(seq 1 5); do
    sink="$(find_by_mac sinks "$mac")"
    [[ -n "$sink" ]] && break
    sleep 1
  done
  [[ -n "$sink" ]] || die "no audio output (sink) for $mac"

  pactl set-default-sink "$sink"
  echo "OK: default audio output is now $sink"
}

cmd_status() {
  require bluetoothctl pactl
  echo "Connected Bluetooth devices:"
  bluetoothctl devices Connected || true
  echo
  echo "Default audio output: $(pactl get-default-sink)"
}

cmd_disconnect() {
  check_mac "${1:-}"
  require bluetoothctl
  bluetoothctl disconnect "$1"
}

case "${1:-}" in
  scan) cmd_scan ;;
  connect) cmd_connect "${2:-}" ;;
  status) cmd_status ;;
  disconnect) cmd_disconnect "${2:-}" ;;
  *) usage; exit 1 ;;
esac
