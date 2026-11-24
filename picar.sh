#!/usr/bin/env bash
# picar.sh: find the PiCar-B (hostname picar-ros) on this network and log in to it with ssh.
# The Linux version of the PiCar.cmd icon on Ferenc's Windows desktop.
#
#   ./picar.sh            find the car and log in (user picar; the car asks for its password,
#                         unless ~/.ssh/picar_ros holds the key)
#   ./picar.sh --find     only print the car's address
#   ./picar.sh --install  add a "PiCar" icon to the app menu and the desktop
#
# The car and this PC must be on the same Wi-Fi (the home Wi-Fi or the team's phone hotspot).
# How it looks for the car, until its ssh port answers:
#   1. the address where it was found last time (~/.cache/picar-last-address)
#   2. its name, picar-ros.local (a PC without mDNS waits about 5 s for this one)
#   3. the Raspberry Pis this PC already knows on the network (network card b8:27:eb:...)
#   4. every address of this PC's networks (only /24 networks: 254 addresses)
set -u

NAME=picar-ros.local
CAR_USER=picar
PORT=${PICAR_SSH_PORT:-22}               # another port only for testing this script
LAST=${XDG_CACHE_HOME:-$HOME/.cache}/picar-last-address
KEY=$HOME/.ssh/picar_ros
PI_MAC='b8:27:eb'                        # the Raspberry Pi 3's network cards

ssh_open() {   # true if address $1 answers on the ssh port within 2 seconds
    timeout 2 bash -c "exec 3<>/dev/tcp/$1/$PORT" 2>/dev/null
}

known_addresses() {   # where else the car may be, best guesses first
    getent ahostsv4 "$NAME" 2>/dev/null | awk '{print $1}'
    if command -v avahi-resolve >/dev/null; then
        avahi-resolve -4 -n "$NAME" 2>/dev/null | awk '{print $2}'
    fi
    ip neigh show 2>/dev/null | awk -v mac="$PI_MAC" 'tolower($0) ~ mac {print $1}'
}

scan_networks() {   # ping every address of this PC's /24 networks, then list the Pis seen
    local net i
    for net in $(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | grep '/24$' | cut -d. -f1-3); do
        echo "  keresem a(z) $net.0 hálózaton ..." >&2
        for i in $(seq 1 254); do
            ping -c 1 -W 1 "$net.$i" >/dev/null 2>&1 &
        done
        wait
    done
    ip neigh show 2>/dev/null | awk -v mac="$PI_MAC" 'tolower($0) ~ mac {print $1}'
}

first_open() {   # the first address on stdin whose ssh port answers
    local ip
    while read -r ip; do
        [[ $ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || continue
        if ssh_open "$ip"; then
            echo "$ip"
            return 0
        fi
    done
    return 1
}

find_car() {
    # the last address first and on its own: the lookups below can take seconds
    [ -f "$LAST" ] && head -n 1 "$LAST" | first_open && return 0
    known_addresses | awk '!seen[$0]++' | first_open && return 0
    scan_networks | awk '!seen[$0]++' | first_open
}

install_icon() {
    local here apps desktop
    here=$(cd "$(dirname "$0")" && pwd)/$(basename "$0")
    chmod +x "$here"
    apps=${XDG_DATA_HOME:-$HOME/.local/share}/applications
    mkdir -p "$apps"
    cat > "$apps/picar.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=PiCar
Comment=Bejelentkezés a PiCar-B autóra
Exec="$here"
Terminal=true
Icon=utilities-terminal
Categories=Utility;
EOF
    chmod +x "$apps/picar.desktop"
    echo "Kész: a PiCar ikon az alkalmazások menüjében van."
    desktop=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")
    if [ -d "$desktop" ] && [ "$desktop" != "$HOME" ]; then
        cp "$apps/picar.desktop" "$desktop/picar.desktop"
        chmod +x "$desktop/picar.desktop"
        # GNOME runs a desktop icon only if it is marked as trusted
        command -v gio >/dev/null && gio set "$desktop/picar.desktop" metadata::trusted true 2>/dev/null
        echo "Az asztalon is ott van (első dupla kattintásnál lehet, hogy jóvá kell hagyni: \"Allow Launching\")."
    fi
}

fail() {   # a message, and wait so a window opened from the icon does not close at once
    echo "$1" >&2
    [ -t 0 ] && read -r -p "Nyomj Entert a bezáráshoz." _
    exit 1
}

case "${1:-}" in
    --install) install_icon; exit 0 ;;
    -h|--help) awk 'NR > 1 && /^#/ {sub(/^# ?/, ""); print; next} NR > 1 {exit}' "$0"; exit 0 ;;
    --find|"") ;;
    *) fail "Ismeretlen kapcsoló: $1 (segítség: ./picar.sh --help)" ;;
esac

echo "Keresem az autót ($NAME) ..." >&2
ip=$(find_car) || fail "Nem találom az autót. Be van kapcsolva (piros LED, induláskor zölden villog)? Ugyanazon a Wi-Fi-n van, mint ez a gép?"
mkdir -p "$(dirname "$LAST")" && echo "$ip" > "$LAST"

if [ "${1:-}" = "--find" ]; then
    echo "$ip"
    exit 0
fi

echo "Megvan: $ip. Bejelentkezés (kilépés: exit) ..." >&2
options=(-o HostKeyAlias=picar-ros)   # the address can change: the car's key stays the same
[ -f "$KEY" ] && options+=(-i "$KEY")
ssh "${options[@]}" -p "$PORT" "$CAR_USER@$ip"
status=$?
[ $status -ne 0 ] && fail "Az ssh hibával lépett ki ($status)."
exit 0
