#!/usr/bin/env bash
# setup_laptop.sh: sets up a Fedora laptop to run the PiCar-B's AI agent and its model.
# The car (the Pi) runs only its own nodes (camera, ultrasonic, picar_base); the agent and
# the model run here and talk to the car over Wi-Fi through ROS 2.
#
#   ./setup_laptop.sh             everything
#   ./setup_laptop.sh --dry-run   only prints what it would do, changes nothing
#   ./setup_laptop.sh --no-model  without Ollama and the local model (for a cloud model)
#
# Run it from the cloned repository (~/picar_ws/src/picar) as yourself, not as root: it
# asks for your password where it needs sudo. It can be run again any time: every step
# checks first and skips what is already done. Everything it prints also goes to
# ~/setup_laptop.log. Its messages are in Hungarian, like setup_car.sh's.
#
# ROS 2 Jazzy runs only on Ubuntu 24.04, so ROS and the agent live in a distrobox: an
# Ubuntu 24.04 container named 'picar' that shares your home folder and the network.
# Steps:
#   1. checks: Fedora, the repository in <workspace>/src/picar, free disk space
#   2. Ollama as a user service, and the model qwen3-vl:4b-instruct. Only the program:
#      Ollama's own installer would put NVIDIA's driver on Fedora
#   3. distrobox and podman, and the 'picar' box (Ubuntu 24.04)
#   4. in the box: ./setup_car.sh laptop (ROS 2 Jazzy, the agent's packages, ~/rai_venv,
#      the agent's .env)
#   5. the picar-agent command (~/.local/bin): starts the agent in the box
#   6. the firewall: lets ROS 2 (DDS, UDP 7400-7600, ROS_DOMAIN_ID 0) reach this laptop

BOX=picar
BOX_IMAGE=docker.io/library/ubuntu:24.04
MODEL=qwen3-vl:4b-instruct
OLLAMA_DOWNLOAD=https://ollama.com/download/ollama-linux-amd64.tar.zst
OLLAMA_API=http://127.0.0.1:11434
DDS_FIRST=7400 DDS_LAST=7600   # ROS_DOMAIN_ID 0: discovery from 7400, then two ports per program
LOG=$HOME/setup_laptop.log
OS_RELEASE=/etc/os-release      # the tests use another
SUDO=sudo

DRY_RUN=false MODEL_TOO=true FORCE=false

# ---------- output ----------

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
info() { printf '   %s\n' "$*"; }
warn() { printf '   \033[33mFIGYELEM:\033[0m %s\n' "$*"; }
fail() {
    printf '\n\033[31mHIBA:\033[0m %s\n' "$*"
    $DRY_RUN || printf 'Minden, amit kiírt: %s\n' "$LOG"
    exit 1
}

run() {   # run a command (or with --dry-run only print it); stop if it fails
    if $DRY_RUN; then printf '   [csak kiírás] %s\n' "$*"; return 0; fi
    printf '   $ %s\n' "$*"
    "$@" || fail "nem sikerült: $*"
}

# ---------- small, testable steps ----------

free_gb() { df -Pk "$1" | awk 'NR == 2 {print int($4 / 1048576)}'; }

needed_gb() {   # free space the missing parts need, with some room to spare
    local gb=5   # the box: Ubuntu, ROS 2 ros-base, the agent's Python packages
    if $MODEL_TOO; then
        command -v ollama > /dev/null || gb=$((gb + 3))
        [ -e "$HOME/.ollama/models/manifests/registry.ollama.ai/library/${MODEL%%:*}/${MODEL#*:}" ] || gb=$((gb + 4))
    fi
    echo "$gb"
}

ports_cover() {   # firewalld's port list, first, last: is first-last (UDP) inside one entry?
    local entry range low high
    for entry in $1; do
        [ "${entry#*/}" = udp ] || continue
        range=${entry%/*}
        low=${range%-*}
        high=${range#*-}
        [ "$low" -le "$2" ] && [ "$high" -ge "$3" ] && return 0
    done
    return 1
}

write_file() {   # path, content: (over)write a small file, or only print it with --dry-run
    if $DRY_RUN; then printf '   [csak kiírás] %s:\n%s\n' "$1" "$2"; return; fi
    { mkdir -p "$(dirname "$1")" && printf '%s\n' "$2" > "$1"; } || fail "nem sikerült írni: $1"
    info "Megírva: $1"
}

ollama_unit() {   # the user service: starts with your login, stops with it
    cat << EOF
[Unit]
Description=Ollama, the AI model for the PiCar agent (made by setup_laptop.sh)
After=network-online.target

[Service]
ExecStart=$1 serve
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
EOF
}

agent_command() {   # the picar-agent script: starts the agent in the box
    cat << EOF
#!/usr/bin/env bash
# picar-agent: starts the PiCar-B's AI agent (made by setup_laptop.sh). It runs in the
# '$BOX' distrobox (Ubuntu 24.04 with ROS 2 Jazzy), or right here if this is that box.
ws=$(printf %q "$WS")
agent=$(printf %q "$REPO/shared/picar_agent")
start="source /opt/ros/jazzy/setup.bash && source '\$ws/install/setup.bash' &&
    source ~/rai_venv/bin/activate && cd '\$agent' && exec python3 agent.py"
if [ -n "\${CONTAINER_ID:-}" ]; then
    exec bash -c "\$start"
fi
exec distrobox enter $BOX -- bash -c "\$start"
EOF
}

# ---------- the steps ----------

check_system() {
    say "1. Ellenőrzés"
    [ "$(id -u)" != 0 ] || fail "Ne root-ként (sudo-val) indítsd, hanem simán: ./setup_laptop.sh. Ahol kell, a jelszavadat kéri."
    # shellcheck disable=SC1090
    . "$OS_RELEASE"
    if [ "${ID:-}" != fedora ]; then
        $FORCE || fail "Ez a Fedorára való (most: ${PRETTY_NAME:-ismeretlen}). Ubuntu 24.04-en elég ennyi: ./setup_car.sh laptop"
        warn "Ez nem Fedora: ${PRETTY_NAME:-ismeretlen}."
    fi
    info "Rendszer: $PRETTY_NAME, $(uname -m)"
    if $MODEL_TOO && [ "$(uname -m)" != x86_64 ]; then
        fail "Az Ollamát itt csak x86-64 gépre telepítem. Felhő-modellel: ./setup_laptop.sh --no-model"
    fi

    [ -d "$REPO/shared/picar_agent" ] && [ -f "$REPO/setup_car.sh" ] ||
        fail "A setup_laptop.sh-t a letöltött repó mappájából indítsd (benne van a shared/picar_agent)."
    [ -d "$REPO/picar/picar_car" ] || fail "Ebben a kódváltozatban nincs picar/picar_car. Frissítsd a kódot: cd $REPO && git pull"
    local src
    src=$(dirname "$REPO")
    WS=$(dirname "$src")
    [ "$(basename "$src")" = src ] || fail "A repónak egy workspace src mappájában kell lennie, például így:
   mkdir -p ~/picar_ws/src && cd ~/picar_ws/src
   git clone -b Ferenc_Csiszér https://github.com/DergerNox/AI_Squad_AI_Robot.git picar"
    case $WS in
        "$HOME"/*) ;;
        *) fail "A workspace ($WS) legyen a saját mappádban (~): a doboz csak azt látja." ;;
    esac
    info "Repó: $REPO, workspace: $WS"

    local free need
    free=$(free_gb "$HOME")
    need=$(needed_gb)
    info "Szabad hely: $free GB, ami kell: kb. $need GB"
    if [ "$free" -lt "$need" ]; then
        $FORCE || $DRY_RUN || fail "Kevés a szabad hely: $free GB van, kb. $need GB kell. (Ha mégis: --force)"
        warn "Kevés a szabad hely."
    fi
    if ! $DRY_RUN; then
        info "A jelszavadat kérheti (sudo):"
        $SUDO -v || fail "sudo nélkül nem megy: a felhasználódnak rendszergazdának (sudo) kell lennie."
    fi
}

install_ollama() {
    say "2. Ollama és a modell ($MODEL)"
    if command -v ollama > /dev/null; then
        info "Az Ollama már telepítve van: $(ollama -v 2>&1 | tail -1)"
    else
        info "Az Ollama telepítése, csak maga a program (kb. 1.5 GB letöltés). Az Ollama saját"
        info "telepítője nem jó ide: NVIDIA-meghajtót tenne a Fedorára, ami elronthatja a képet."
        command -v zstd > /dev/null || run $SUDO dnf install -y zstd
        run bash -c "set -o pipefail; curl -fsSL '$OLLAMA_DOWNLOAD' | $SUDO tar --zstd -x -C /usr"
    fi
    ollama_service
    if ollama list 2> /dev/null | awk 'NR > 1 {print $1}' | grep -qxF "$MODEL"; then
        info "A modell már letöltve: $MODEL"
    else
        info "A modell letöltése: $MODEL (kb. 3.3 GB)"
        run ollama pull "$MODEL"
    fi
}

ollama_service() {
    if systemctl is-active --quiet ollama 2> /dev/null; then
        info "Az Ollama rendszerszolgáltatásként már fut."
        return
    fi
    if systemctl --user is-active --quiet ollama 2> /dev/null; then
        info "Az Ollama szolgáltatás már fut (indul a bejelentkezéseddel)."
        return
    fi
    if curl -fs "$OLLAMA_API/api/version" > /dev/null 2>&1; then
        fail "Már fut egy kézzel indított Ollama (ollama serve). Állítsd le (Ctrl+C a termináljában), aztán indítsd újra ezt."
    fi
    info "Az Ollama szolgáltatás: a bejelentkezéseddel indul, és amíg nem kérdezik, alig használ memóriát."
    local ollama
    ollama=$(command -v ollama || echo /usr/bin/ollama)
    write_file "$HOME/.config/systemd/user/ollama.service" "$(ollama_unit "$ollama")"
    run systemctl --user daemon-reload
    run systemctl --user enable --now ollama
    $DRY_RUN && return
    local _
    for _ in $(seq 30); do
        curl -fs "$OLLAMA_API/api/version" > /dev/null 2>&1 && return
        sleep 1
    done
    fail "Az Ollama nem indult el. Nézd meg: journalctl --user -u ollama"
}

make_box() {
    say "3. Az Ubuntu 24.04 doboz ('$BOX', distrobox)"
    if command -v distrobox > /dev/null && command -v podman > /dev/null; then
        info "A distrobox és a podman már telepítve van."
    else
        run $SUDO dnf install -y distrobox podman
    fi
    if ! $DRY_RUN && podman container exists "$BOX"; then
        local image
        image=$(podman inspect -f '{{.ImageName}}' "$BOX" 2> /dev/null)
        info "A '$BOX' doboz már megvan ($image)."
        case $image in
            *ubuntu:24.04*) ;;
            *) fail "A '$BOX' doboz nem Ubuntu 24.04. Töröld (distrobox rm $BOX), vagy nevezd át, aztán indítsd újra ezt." ;;
        esac
    else
        info "A doboz elkészítése: Ubuntu 24.04, a saját mappádat és a hálózatot közösen használja a Fedorával."
        run distrobox create --yes --name "$BOX" --image "$BOX_IMAGE"
    fi
}

setup_box() {
    say "4. A dobozban: ROS 2 Jazzy, az ágens csomagjai és Python-környezete (./setup_car.sh laptop)"
    info "Az első belépéskor a distrobox pár percig berendezi a dobozt; utána jön a setup_car.sh."
    local flags=''
    $DRY_RUN && flags=' --dry-run'
    run distrobox enter "$BOX" -- bash -c "cd $(printf %q "$REPO") && bash setup_car.sh laptop$flags"
}

install_agent_command() {
    say "5. A picar-agent parancs"
    write_file "$HOME/.local/bin/picar-agent" "$(agent_command)"
    run chmod +x "$HOME/.local/bin/picar-agent"
    case ":$PATH:" in
        *":$HOME/.local/bin:"*) ;;
        *) warn "A ~/.local/bin nincs a PATH-ban: indítsd így: ~/.local/bin/picar-agent" ;;
    esac
}

open_firewall() {
    say "6. Tűzfal: a ROS 2 (DDS) elérje ezt a gépet (UDP $DDS_FIRST-$DDS_LAST)"
    if ! command -v firewall-cmd > /dev/null || ! $SUDO firewall-cmd --state > /dev/null 2>&1; then
        info "Nem fut tűzfal (firewalld), nincs mit beállítani."
        return
    fi
    local zones zone changed=false
    # the default zone, and every zone a network connection uses right now
    # (--get-active-zones: a zone's name starts a line, maybe with ' (default)' after it)
    zones=$( { $SUDO firewall-cmd --get-default-zone; $SUDO firewall-cmd --get-active-zones | awk '/^[^ \t]/ {print $1}'; } | sort -u)
    for zone in $zones; do
        if ports_cover "$($SUDO firewall-cmd --zone="$zone" --list-ports)" "$DDS_FIRST" "$DDS_LAST"; then
            info "A '$zone' zóna már átengedi."
        else
            run $SUDO firewall-cmd --permanent --zone="$zone" --add-port="$DDS_FIRST-$DDS_LAST/udp"
            changed=true
        fi
    done
    if $changed; then run $SUDO firewall-cmd --reload; fi
}

summary() {
    say "Kész. Ami még hátravan:"
    info "- Nyiss egy új terminált."
    info "- Az ágens indítása: picar-agent (Ctrl+C: leállítja)"
    info "- A modellt a $REPO/shared/picar_agent/.env választja: alapból a laptop Ollamája ($MODEL),"
    info "  felhő-modellhez PICAR_LLM, PICAR_MODEL és a kulcs (a fájlban leírva)."
    info "- A valódi autóhoz: a laptop és a Pi ugyanazon a Wi-Fin legyen (a telefon hotspotja). Próba:"
    info "  distrobox enter $BOX, aztán: ros2 topic list (látszik a /camera/image_raw és a /ultrasonic/range?)"
    if $MODEL_TOO; then
        info "- Az Ollama leállítása, ha épp nem kell: systemctl --user stop ollama"
    fi
    if $DRY_RUN; then info "(Ez csak kiírás volt: semmi nem változott.)"; fi
}

usage() {
    sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

main() {
    for arg in "$@"; do
        case $arg in
            --dry-run) DRY_RUN=true ;;
            --no-model) MODEL_TOO=false ;;
            --force) FORCE=true ;;
            -h | --help) usage; exit 0 ;;
            *) usage; echo; fail "Ismeretlen: '$arg'" ;;
        esac
    done
    REPO=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
    if ! $DRY_RUN; then
        exec > >(tee -a "$LOG") 2>&1
        printf '\n----- %s: setup_laptop.sh %s\n' "$(date '+%F %T')" "$*"
    fi

    check_system
    if $MODEL_TOO; then install_ollama; else say "2. Ollama: kihagyva (--no-model)"; fi
    make_box
    setup_box
    install_agent_command
    open_firewall
    summary
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    main "$@"
fi
