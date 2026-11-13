#!/usr/bin/env bash
# setup_car.sh: installs and sets up everything a car needs, on Ubuntu 24.04 (Lubuntu
# 24.04 is the same system with another desktop).
#
#   ./setup_car.sh picar             the PiCar-B's Raspberry Pi (Ubuntu Server 24.04)
#   ./setup_car.sh pccar             the PC car's Toughbook (Lubuntu 24.04)
#   ./setup_car.sh laptop            the computer that runs the PiCar-B's AI agent (on Fedora,
#                                    setup_laptop.sh runs this in an Ubuntu 24.04 distrobox)
#   ./setup_car.sh picar --dry-run   only prints what it would do, changes nothing
#   ./setup_car.sh pccar --no-agent  everything but the AI agent's Python environment
#
# Run it from the cloned repository (~/picar_ws/src/picar) as yourself, not as root: it
# asks for your password where it needs sudo. It can be run again any time: every step
# checks first and skips what is already done. Everything it prints also goes to
# ~/setup_car.log. Its messages are in Hungarian, for the hardware team.
#
# Steps:
#   1. checks: Ubuntu 24.04, the repository in <workspace>/src/picar, the right computer
#   2. ROS 2 Jazzy (ros-base: no desktop, no Gazebo) from ROS's apt repository
#   3. this car's system packages and settings
#        picar: gpiozero + lgpio (distance sensor), camera_ros, i2c-tools; SPI off (it would
#               take the distance sensor's pins); the user in the GPIO/I2C/video groups
#        pccar: v4l2_camera, adb and DroidCam with its DKMS video driver (the phone camera
#               over USB); the user in the dialout (ESP32 serial) and video groups
#        laptop: nothing more (the agent only needs ROS, its messages and the car's settings)
#   4. builds this car's packages in the workspace
#   5. the AI agent: ~/rai_venv from shared/picar_agent/requirements.txt, and its .env
#   6. ~/.bashrc: ROS and the workspace in every new terminal

ROS_APT_SOURCE_FALLBACK=1.3.0   # used when GitHub does not say which version is the newest
DROIDCAM_URL=https://files.dev47apps.net/linux/droidcam_2.1.7.zip
DROIDCAM_SHA256=28b1af50f7a9cdb7e6035b3c2e85dbd8428dbc1ad881dbeb89cab3a6237fb66c   # from dev47apps.com
VENV=$HOME/rai_venv
LOG=$HOME/setup_car.log
SUDO=sudo   # the tests set it empty

CAR='' DRY_RUN=false AGENT=true FORCE=false
REBOOT=false RELOGIN=false SWAPFILE=''

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

try() {   # like run, but a failure is only a warning
    if $DRY_RUN; then printf '   [csak kiírás] %s\n' "$*"; return 0; fi
    printf '   $ %s\n' "$*"
    "$@" || { warn "nem sikerült: $*"; return 1; }
}

apt_install() { run $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"; }

# ---------- small, testable steps ----------

is_raspberry_pi() { grep -qs "Raspberry Pi" /proc/device-tree/model; }

add_line_once() {   # file, line, text that shows the line is already there
    local file=$1 line=$2 marker=$3
    if grep -qsF -- "$marker" "$file"; then info "Már benne van ($file): $marker"; return; fi
    if $DRY_RUN; then printf '   [csak kiírás] a(z) %s végére: %s\n' "$file" "$line"; return; fi
    printf '%s\n' "$line" >> "$file"
    info "Hozzáadva a(z) $file végére: $line"
}

spi_off() {   # the Pi's config.txt
    local file=$1
    if [ ! -f "$file" ]; then
        warn "Nincs $file: az SPI-t kézzel kell kikapcsolni (README 3. pont)."
        return
    fi
    if grep -q '^dtparam=spi=off' "$file"; then info "Az SPI már ki van kapcsolva."; return; fi
    info "Az SPI kikapcsolása: a távolságérzékelő lábait (GPIO 8 és 11) foglalná."
    run $SUDO cp "$file" "$file.before-setup_car"
    if grep -q '^dtparam=spi=on' "$file"; then
        run $SUDO sed -i 's/^dtparam=spi=on/dtparam=spi=off/' "$file"
    else
        run $SUDO sh -c "echo 'dtparam=spi=off' >> '$file'"
    fi
    REBOOT=true
}

join_group() {   # group, why
    local group=$1 why=$2
    if ! getent group "$group" > /dev/null; then info "Nincs '$group' csoport ezen a gépen, kihagyom."; return; fi
    if id -nG "$USER" | tr ' ' '\n' | grep -qx "$group"; then
        info "Már tagja vagy a '$group' csoportnak ($why)."
        return
    fi
    info "Felveszlek a '$group' csoportba: $why."
    run $SUDO usermod -aG "$group" "$USER"
    RELOGIN=true
}

join_device_group() {   # device, why
    local device=$1 why=$2 group
    if [ ! -e "$device" ]; then info "Nincs $device (még), kihagyom."; return; fi
    group=$(stat -c %G "$device")
    if [ "$group" = root ]; then
        warn "A $device csak a root-é. Ha később 'permission denied' hibát kapsz ($why), szólj Ferencnek."
        return
    fi
    join_group "$group" "$why"
}

car_packages() {   # the packages to build for the car
    local extra=''
    case $CAR in
        picar) echo "picar_interfaces picar_base picar_listener picar_car" ;;
        laptop) echo "picar_interfaces picar_base picar_car" ;;   # messages; car.yaml (needs picar_base)
        pccar)   # and whatever pccar/ holds by now (nothing built yet in 2026-11)
            if [ -d "$REPO/pccar" ]; then
                extra=$(find "$REPO/pccar" -name package.xml -exec sed -n 's:.*<name>\(.*\)</name>.*:\1:p' {} \; | tr '\n' ' ')
            fi
            echo "picar_interfaces picar_base picar_listener $extra" | sed 's/ *$//' ;;
    esac
}

# ---------- the steps ----------

check_system() {
    say "1. Ellenőrzés"
    [ "$(id -u)" != 0 ] || fail "Ne root-ként (sudo-val) indítsd, hanem simán: ./setup_car.sh $CAR. Ahol kell, a jelszavadat kéri."
    # shellcheck disable=SC1091
    . /etc/os-release
    [ "${VERSION_CODENAME:-}" = noble ] || fail "Ubuntu 24.04 kell hozzá (a Lubuntu 24.04 is jó), mert a ROS 2 Jazzy csak azon fut. Ezen a gépen: ${PRETTY_NAME:-ismeretlen}."
    info "Rendszer: $PRETTY_NAME, $(uname -m)"
    local others='' distro
    for distro in /opt/ros/*/; do
        [ -d "$distro" ] && [ "$(basename "$distro")" != jazzy ] && others="$others$(basename "$distro") "
    done
    [ -z "$others" ] || warn "Más ROS-változat is van a gépen ($others). A kód a Jazzyhoz készült; a ~/.bashrc csak a Jazzyt töltse be."

    if [ "$CAR" = picar ] && ! is_raspberry_pi; then
        $FORCE || $DRY_RUN || fail "Ez nem Raspberry Pi: a 'picar' beállítás a PiCar-B Pi-jére való. (Ha mégis ezt akarod: --force)"
        warn "Ez nem Raspberry Pi."
    fi
    if [ "$CAR" != picar ] && is_raspberry_pi; then
        $FORCE || $DRY_RUN || fail "Ez egy Raspberry Pi: a '$CAR' beállítás nem a Pi-re való. (Ha mégis ezt akarod: --force)"
        warn "Ez egy Raspberry Pi."
    fi

    [ -d "$REPO/shared/picar_base" ] || fail "A setup_car.sh-t a letöltött repó mappájából indítsd (benne van a shared/picar_base)."
    SRC=$(dirname "$REPO")
    WS=$(dirname "$SRC")
    [ "$(basename "$SRC")" = src ] || fail "A repónak egy workspace src mappájában kell lennie, például így:
   mkdir -p ~/picar_ws/src && cd ~/picar_ws/src
   git clone https://github.com/DergerNox/AI_Squad_AI_Robot.git picar"
    info "Repó: $REPO, workspace: $WS"
    if [ "$CAR" != pccar ] && [ ! -d "$REPO/picar/picar_car" ]; then
        fail "Ebben a kódváltozatban nincs picar/picar_car. Frissítsd a kódot: cd $REPO && git pull"
    fi
    if ! $DRY_RUN; then
        info "A jelszavadat kérheti (sudo):"
        sudo -v || fail "sudo nélkül nem megy: a felhasználódnak rendszergazdának (sudo) kell lennie."
    fi
}

temporary_swap() {   # a Pi 3B+ has 1 GB: building and pip need more
    local mem_kb swap_kb
    mem_kb=$(awk '/^MemTotal/ {print $2}' /proc/meminfo)
    swap_kb=$(awk '/^SwapTotal/ {print $2}' /proc/meminfo)
    if [ "$mem_kb" -ge 2000000 ] || [ "$swap_kb" -ge 1000000 ]; then return; fi
    info "Kevés a memória ($((mem_kb / 1024)) MB): a telepítés idejére ideiglenes cserefájlt (swap, 2 GB) készítek, a végén törlöm."
    SWAPFILE=/var/tmp/setup_car.swap
    run $SUDO fallocate -l 2G "$SWAPFILE"
    run $SUDO chmod 600 "$SWAPFILE"
    run $SUDO mkswap "$SWAPFILE"
    run $SUDO swapon "$SWAPFILE"
}

remove_swap() {
    [ -n "$SWAPFILE" ] && ! $DRY_RUN || return 0
    $SUDO swapoff "$SWAPFILE" 2> /dev/null
    $SUDO rm -f "$SWAPFILE"
}

install_ros() {
    say "2. ROS 2 Jazzy"
    if ! locale 2> /dev/null | grep -qi 'utf-8'; then
        info "UTF-8 nyelvi beállítás (a ROS ezt kéri)"
        apt_install locales
        run $SUDO locale-gen en_US en_US.UTF-8
        run $SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    fi
    # -R: ros2-apt-source puts a link there; older setups have a ros2.list file
    if dpkg -s ros2-apt-source > /dev/null 2>&1 || grep -Rqs "packages.ros.org/ros2" /etc/apt/sources.list.d/; then
        info "A ROS apt-forrása már be van állítva."
    else
        apt_install software-properties-common curl
        run $SUDO add-apt-repository -y universe
        local version
        version=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
                  grep -F '"tag_name"' | awk -F'"' '{print $4}')
        [ -n "$version" ] || version=$ROS_APT_SOURCE_FALLBACK
        info "A ROS apt-forrásának beállítása (ros2-apt-source $version)"
        run curl -fsSL -o /tmp/ros2-apt-source.deb \
            "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.noble_all.deb"
        run $SUDO dpkg -i /tmp/ros2-apt-source.deb
    fi
    info "A rendszer frissítése, aztán a ROS 2 Jazzy (asztali felület és Gazebo nélkül). Ez eltarthat egy ideig."
    run $SUDO apt-get update
    run $SUDO env DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
    local for_agent=()
    $AGENT && for_agent=(ros-jazzy-cv-bridge)   # RAI imports it (camera pictures); not in ros-base
    apt_install ros-jazzy-ros-base ros-dev-tools ros-jazzy-rai-interfaces \
        python3-pytest python3-venv python3-pip git curl "${for_agent[@]}"
}

setup_picar() {
    say "3. A PiCar-B csomagjai és beállításai"
    apt_install python3-gpiozero python3-lgpio ros-jazzy-camera-ros i2c-tools
    spi_off /boot/firmware/config.txt
    join_device_group /dev/gpiochip0 "a távolságérzékelő lábai"
    join_device_group /dev/i2c-1 "a szervóvezérlő (I2C)"
    join_group video "a kamera"
}

setup_pccar() {
    say "3. A PC-s autó csomagjai és beállításai"
    apt_install ros-jazzy-v4l2-camera v4l-utils adb dkms gcc make unzip "linux-headers-$(uname -r)"
    install_droidcam
    join_group dialout "az ESP32 soros portja (USB)"
    join_group video "a kamera"
}

setup_laptop() {
    say "3. Az MI-ágens gépe"
    info "Itt csak az ágens fut: a ROS-on, az üzeneteken és az autó beállításain kívül nem kell hozzá más."
}

install_droidcam() {
    if [ "$(uname -m)" != x86_64 ]; then warn "A DroidCam csak x86-64 gépen fut, kihagyom."; return; fi
    if command -v droidcam-cli > /dev/null && dkms status 2> /dev/null | grep -q v4l2loopback_dc; then
        info "A DroidCam már telepítve van."
        return
    fi
    info "A DroidCam telepítése: a telefon kamerája USB-n át, a gépen /dev/videoN-ként (640x480)"
    local dir=/tmp/droidcam-download
    $DRY_RUN || dir=$(mktemp -d)
    run curl -fsSL -o "$dir/droidcam.zip" "$DROIDCAM_URL"
    run sh -c "echo '$DROIDCAM_SHA256  $dir/droidcam.zip' | sha256sum -c -"
    run unzip -q "$dir/droidcam.zip" -d "$dir/droidcam"
    run sh -c "cd '$dir/droidcam' && $SUDO ./install-client"
    try sh -c "cd '$dir/droidcam' && $SUDO ./install-dkms 640 480" ||
        warn "A DroidCam videó-meghajtója nem töltődött be. Indítsd újra a gépet, aztán: lsmod | grep v4l2loopback_dc"
    if mokutil --sb-state 2> /dev/null | grep -q enabled; then
        warn "A Secure Boot be van kapcsolva: újraindításkor lehet, hogy jóvá kell hagyni a meghajtót (kék MOK képernyő, 'Enroll MOK'), különben nem töltődik be."
    fi
    $DRY_RUN || rm -rf "$dir"
}

build_workspace() {
    say "4. A kód fordítása ($WS)"
    local packages flags='--symlink-install'
    packages=$(car_packages)
    info "Ennek az autónak a csomagjai: $packages"
    # a Pi 3B+ has 1 GB: one package and one compiler job at a time
    [ "$CAR" = picar ] && flags="$flags --executor sequential"
    run bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && MAKEFLAGS=-j1 colcon build $flags --packages-select $packages"
}

setup_agent() {
    say "5. Az MI-ágens Python környezete ($VENV)"
    local agent=$REPO/shared/picar_agent
    if [ -d "$VENV" ]; then
        info "A $VENV már megvan, csak kiegészítem."
    else
        run python3 -m venv "$VENV" --system-site-packages
    fi
    [ "$CAR" = picar ] && info "A Pi-n ez akár egy óráig is eltarthat (sok csomag, kevés memória)."
    run "$VENV/bin/pip" install -r "$agent/requirements.txt"
    if [ -f "$agent/.env" ]; then
        info "A .env már megvan (a beállításaidat nem írom felül)."
    else
        run cp "$agent/.env.example" "$agent/.env"
        if [ "$CAR" != pccar ]; then
            run sed -i 's/^PICAR_CONFIG=$/PICAR_CONFIG=car/' "$agent/.env"
            info "PICAR_CONFIG=car beállítva: az ágens a valódi PiCar-B adataival dolgozik."
        fi
    fi
}

setup_bashrc() {
    say "6. Minden új terminál ismerje a ROS-t és a kódot (~/.bashrc)"
    local bashrc=$HOME/.bashrc ws_setup
    if grep -Eqs '/opt/ros/[a-z]+/setup' "$bashrc" && grep -Ev '^\s*#' "$bashrc" | grep -Eo '/opt/ros/[a-z]+/' | grep -vq '/opt/ros/jazzy/'; then
        warn "A ~/.bashrc egy másik ROS-változatot is betölt: azt a sort töröld vagy tedd # mögé (csak a Jazzy kell)."
    fi
    # Both lines check that ROS is there: on a Fedora laptop the same ~/.bashrc also runs
    # outside the Ubuntu box, where there is no ROS.
    local ros=/opt/ros/jazzy/setup.bash
    add_line_once "$bashrc" "[ -f $ros ] && source $ros" "$ros"
    ws_setup="${WS/#$HOME/\~}/install/setup.bash"   # ~/picar_ws/install/setup.bash
    # the marker matches both ~/picar_ws/... and /home/<user>/picar_ws/...
    add_line_once "$bashrc" "[ -f $ros ] && [ -f $ws_setup ] && source $ws_setup" "$(basename "$WS")/install/setup.bash"
}

summary() {
    say "Kész. Ami még hátravan:"
    if $REBOOT; then
        info "- Indítsd újra a gépet: sudo reboot (az SPI kikapcsolása és az új csoportok csak utána érvényesek)"
    elif $RELOGIN; then
        info "- Jelentkezz ki és be, vagy indítsd újra a gépet: az új csoportok csak utána érvényesek"
    fi
    info "- Nyiss egy új terminált: abban már megy a ROS és a kód."
    if [ "$CAR" = laptop ]; then
        info "- A modell: a $REPO/shared/picar_agent/.env fájlban (alapból ennek a gépnek az Ollamája)."
        info "- Az ágens indítása: picar-agent (ha a setup_laptop.sh telepítette), vagy:"
        info "  cd $REPO/shared/picar_agent && source $VENV/bin/activate && python3 agent.py"
    elif $AGENT; then
        info "- Írd be a $REPO/shared/picar_agent/.env fájlba, hol fut a modell (PICAR_BASE_URL, README 7-8. pont)."
    fi
    case $CAR in
        picar) info "- A távolságérzékelő próbája és az autó node-jai: README 3. pont." ;;
        pccar)
            info "- A telefonon: a DroidCam alkalmazás és az USB-hibakeresés (Fejlesztői beállítások)."
            info "- Próba: droidcam-cli adb 4747, egy másik terminálban: v4l2-ctl --list-devices" ;;
    esac
    if $DRY_RUN; then info "(Ez csak kiírás volt: semmi nem változott.)"; fi
}

usage() {
    sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

main() {
    for arg in "$@"; do
        case $arg in
            picar | pccar | laptop) CAR=$arg ;;
            --dry-run) DRY_RUN=true ;;
            --no-agent) AGENT=false ;;
            --force) FORCE=true ;;
            -h | --help) usage; exit 0 ;;
            *) usage; echo; fail "Ismeretlen: '$arg'" ;;
        esac
    done
    if [ -z "$CAR" ]; then
        usage
        echo
        if is_raspberry_pi; then fail "Melyik autó? Ez egy Raspberry Pi, valószínűleg: ./setup_car.sh picar"; fi
        fail "Melyik autó? ./setup_car.sh picar (a Pi-n), ./setup_car.sh pccar (a PC-s autó gépén) vagy ./setup_car.sh laptop (az ágens gépén)"
    fi
    [ "$CAR" = laptop ] && ! $AGENT && fail "A laptopon épp az ágens a lényeg: ott ne használd a --no-agent-et."
    REPO=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
    if ! $DRY_RUN; then
        exec > >(tee -a "$LOG") 2>&1
        printf '\n----- %s: setup_car.sh %s\n' "$(date '+%F %T')" "$*"
    fi
    trap remove_swap EXIT

    check_system
    [ "$CAR" = picar ] && temporary_swap
    install_ros
    "setup_$CAR"
    build_workspace
    $AGENT && setup_agent
    setup_bashrc
    summary
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    main "$@"
fi
