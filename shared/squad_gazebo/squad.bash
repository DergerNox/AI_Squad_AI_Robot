# The squad's terminal commands. One line in ~/.bashrc, after ROS and ~/picar_ws are sourced:
#   [ -f ~/picar_ws/install/squad_gazebo/share/squad_gazebo/squad.bash ] && source ~/picar_ws/install/squad_gazebo/share/squad_gazebo/squad.bash
#
#   sim                  asks car and world; starts the twin with its Gazebo window, waits
#                        for its camera, then starts the agent here. Ctrl+C stops everything.
#   sim world:=room      this tab's car without questions (any launch argument works).
#                        Rarely needed: headless:=true (no window), agent:=false (twin only),
#                        cars:=FILE (another cars.yaml)
#   car mini             only sets this tab up for a car: its folder, ROS domain and venv
#
# The cars, their folders, domains and agents are in squad_gazebo's config/cars.yaml.
# Messages of the twin (and of an agent with its own ask command) go to ~/.ros/squad/.

_squad_ros() {
  source /opt/ros/jazzy/setup.bash
  [ -f ~/picar_ws/install/setup.bash ] && source ~/picar_ws/install/setup.bash
}

car() {
  local SQ_CAR SQ_FOLDER SQ_DOMAIN SQ_VENV SQ_AGENT_DIR SQ_AGENT_RUN SQ_AGENT_ENV SQ_AGENT_ASK \
        SQ_AGENT_BEFORE SQ_AGENT_WAIT SQ_AGENT_PROBLEM info
  [ -n "$1" ] || { echo "usage: car <name>   (or just: sim)"; return 1; }
  if [ -n "$CAR_ENV" ] && [ "$CAR_ENV" != "$1" ]; then
    echo "This tab is set up for $CAR_ENV. Open a new tab for $1."; return 1
  fi
  _squad_ros
  info=$(python3 -m squad_gazebo.pick info "$1" ${SQUAD_CARS:+--cars "$SQUAD_CARS"}) || return 1
  eval "$info"
  if [ "$SQ_FOLDER" != ~/picar_ws ] && [ -f "$SQ_FOLDER/install/setup.bash" ]; then
    source "$SQ_FOLDER/install/setup.bash"
  fi
  [ -f "$SQ_VENV/bin/activate" ] && source "$SQ_VENV/bin/activate"
  export CAR_ENV=$1 ROS_DOMAIN_ID=$SQ_DOMAIN GZ_PARTITION=$1
  PS1="[$1] ${BASE_PS1:=$PS1}"
  cd "$SQ_FOLDER" || return 1
}

sim() {
  local SQ_CAR SQ_FOLDER SQ_DOMAIN SQ_VENV SQ_WORLD SQ_HEADLESS SQ_AGENT SQ_AGENT_DIR \
        SQ_AGENT_RUN SQ_AGENT_ENV SQ_AGENT_ASK SQ_AGENT_BEFORE SQ_AGENT_WAIT SQ_AGENT_PROBLEM
  local answer arg agent=true SQUAD_CARS=$SQUAD_CARS
  local args=()
  for arg in "$@"; do
    case "$arg" in
      agent:=*) agent=${arg#agent:=} ;;
      cars:=*) SQUAD_CARS=${arg#cars:=}; args+=("$arg") ;;
      *) args+=("$arg") ;;
    esac
  done

  if [ $# -eq 0 ]; then
    [ -n "$CAR_ENV" ] || _squad_ros
    answer=$(python3 -m squad_gazebo.pick ask $CAR_ENV) || return 1
    eval "$answer"
    [ "$CAR_ENV" = "$SQ_CAR" ] || car "$SQ_CAR" || return 1
    args=("world:=$SQ_WORLD" "headless:=$SQ_HEADLESS")
    agent=$SQ_AGENT
  else
    [ -n "$CAR_ENV" ] || { echo "Set this tab up first (car <name>), or type just: sim"; return 1; }
    answer=$(python3 -m squad_gazebo.pick info "$CAR_ENV" ${SQUAD_CARS:+--cars "$SQUAD_CARS"}) || return 1
    eval "$answer"
    if [ "$agent" = true ] && [ -n "$SQ_AGENT_PROBLEM" ]; then
      echo "Cannot start the agent: $SQ_AGENT_PROBLEM"; return 1
    fi
    if [ "$agent" = true ] && [ -z "$SQ_AGENT_RUN" ]; then
      echo "Agent: none set up for $CAR_ENV in cars.yaml, starting the twin only"
      agent=false
    fi
  fi

  if [ "$agent" != true ]; then
    GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
      ros2 launch squad_gazebo sim.launch.py car:="$CAR_ENV" "${args[@]}"
    return
  fi
  _squad_twin_and_agent "${args[@]}"
}

# The twin in the background, then the agent in this terminal. A subshell, so that
# Ctrl+C, Ctrl+D or the agent ending always stops the twin too.
_squad_twin_and_agent() (
  logs=~/.ros/squad
  mkdir -p "$logs"
  twin_log=$logs/$CAR_ENV-twin.log
  agent_log=$logs/$CAR_ENV-agent.log
  twin='' agent_pid='' waiter=''

  stop() {
    trap - EXIT INT TERM
    [ -n "$waiter" ] && kill -TERM "$waiter" 2>/dev/null
    if [ -n "$agent_pid" ] && kill -TERM "$agent_pid" 2>/dev/null; then
      for _ in $(seq 10); do kill -0 "$agent_pid" 2>/dev/null || break; sleep 0.5; done
      kill -KILL "$agent_pid" 2>/dev/null
    fi
    if [ -n "$twin" ] && kill -0 "$twin" 2>/dev/null; then
      echo "Stopping the twin..."
      kill -INT -- -"$twin" 2>/dev/null
      for _ in $(seq 20); do kill -0 "$twin" 2>/dev/null || break; sleep 0.5; done
      if kill -0 "$twin" 2>/dev/null; then
        kill -TERM -- -"$twin" 2>/dev/null; sleep 1; kill -KILL -- -"$twin" 2>/dev/null
      fi
    fi
    wait 2>/dev/null
    echo "Stopped."
  }
  trap stop EXIT
  trap 'echo; exit 130' INT TERM

  [ -n "$SQ_AGENT_BEFORE" ] && eval "$SQ_AGENT_BEFORE" > /dev/null 2>&1
  GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
    setsid ros2 launch squad_gazebo sim.launch.py car:="$CAR_ENV" "$@" > "$twin_log" 2>&1 < /dev/null &
  twin=$!
  echo "Starting the $CAR_ENV twin (its messages: $twin_log)"
  echo "Waiting for $SQ_AGENT_WAIT..."
  # In the background + wait: then Ctrl+C always interrupts the wait (bash would ignore it
  # while a foreground timeout runs, because timeout keeps the Ctrl+C from its command).
  while :; do
    timeout 5 ros2 topic echo --once "$SQ_AGENT_WAIT" --field header.stamp > /dev/null 2>&1 &
    waiter=$!
    wait "$waiter" && break
    if ! kill -0 "$twin" 2>/dev/null; then
      echo "The twin stopped. Last lines of $twin_log:"
      tail -n 15 "$twin_log"
      exit 1
    fi
    sleep 1 & wait $!
  done
  waiter=''

  cd "$SQ_AGENT_DIR" || exit 1
  if [ -n "$SQ_AGENT_ENV" ]; then set -a; source "$SQ_AGENT_ENV"; set +a; fi
  if [ -z "$SQ_AGENT_ASK" ]; then
    echo "Twin ready. Starting the agent: $SQ_AGENT_RUN"
    eval "$SQ_AGENT_RUN"          # it has its own prompt; when it ends, the twin stops
    exit
  fi
  eval "exec $SQ_AGENT_RUN" > "$agent_log" 2>&1 < /dev/null &
  agent_pid=$!
  echo "Twin ready, agent started (its messages: $agent_log)."
  echo "Type a task and press Enter. Ctrl+C stops everything."
  while kill -0 "$agent_pid" 2>/dev/null && IFS= read -r -e -p "> " task; do
    [ -n "$task" ] && eval "$SQ_AGENT_ASK" '"$task"'
  done
  if ! kill -0 "$agent_pid" 2>/dev/null; then
    echo "The agent stopped. Last lines of $agent_log:"
    tail -n 15 "$agent_log"
  fi
)

if [ -n "$CAR" ]; then car "$CAR"; fi    # Windows Terminal profiles: wsl.exe ... env CAR=picar bash -i
