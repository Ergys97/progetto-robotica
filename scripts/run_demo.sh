#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="progetto_robotica"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
VENV_DIR="${VENV_DIR:-$HOME/venv}"

usage() {
    cat <<'USAGE'
Uso:
  bash scripts/run_demo.sh flat [--headless] [--build]
  bash scripts/run_demo.sh obstacle [--headless] [--build]

Preset:
  flat                 Scenario piano regolare.
  obstacle             Scenario con rampa e piccoli step.

Opzioni:
  --headless           Avvia MuJoCo senza viewer grafico.
  --build              Ricompila prima del launch.
  -h, --help           Mostra questo messaggio.

Variabili ambiente:
  ROS_WS=~/ros2_ws     Workspace ROS 2 da usare.
  VENV_DIR=~/venv      Virtualenv Python da attivare, se presente.

Dashboard:
  http://localhost:5000
USAGE
}

safe_source() {
    local setup_file="$1"
    set +u
    # shellcheck source=/dev/null
    source "$setup_file"
    set -u
}

remove_colon_path_prefix() {
    local var_name="$1"
    local blocked_prefix="$2"
    local current="${!var_name:-}"
    local result=""
    local entry

    if [[ -z "$current" ]]; then
        return
    fi

    IFS=':' read -r -a entries <<< "$current"
    for entry in "${entries[@]}"; do
        if [[ -z "$entry" || "$entry" == "$blocked_prefix" || "$entry" == "$blocked_prefix/"* ]]; then
            continue
        fi
        if [[ -z "$result" ]]; then
            result="$entry"
        else
            result="${result}:${entry}"
        fi
    done
    export "${var_name}=${result}"
}

if [[ $# -lt 1 ]]; then
    usage
    exit 2
fi

scenario_arg="$1"
shift

scenario=""
case "$scenario_arg" in
    flat)
        scenario="flat"
        ;;
    obstacle|obstacle_course)
        scenario="obstacle_course"
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Scenario non supportato: $scenario_arg" >&2
        usage >&2
        exit 2
        ;;
esac

headless="False"
build_first="False"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless)
            headless="True"
            ;;
        --build)
            build_first="True"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Opzione non supportata: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

ros_setup="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
install_setup="${ROS_WS}/install/setup.bash"
wrong_src_ws="${HOME}/ros2_ws/src"
wrong_src_install="${wrong_src_ws}/install"

if [[ "$(realpath -m "$ROS_WS")" == "$(realpath -m "$wrong_src_ws")" ]]; then
    echo "ROS_WS punta a ${ROS_WS}, ma deve puntare alla root del workspace, non a src." >&2
    echo "Usa: unset ROS_WS   oppure   ROS_WS=${HOME}/ros2_ws bash scripts/run_demo.sh ${scenario_arg}" >&2
    exit 1
fi

remove_colon_path_prefix AMENT_PREFIX_PATH "$wrong_src_install"
remove_colon_path_prefix CMAKE_PREFIX_PATH "$wrong_src_install"
remove_colon_path_prefix COLCON_PREFIX_PATH "$wrong_src_install"
remove_colon_path_prefix PYTHONPATH "$wrong_src_install"
remove_colon_path_prefix PATH "$wrong_src_install"

if [[ -f "${VENV_DIR}/bin/activate" ]]; then
    safe_source "${VENV_DIR}/bin/activate"
else
    echo "Virtualenv non trovato in ${VENV_DIR}; continuo senza attivarlo." >&2
fi

if [[ ! -f "$ros_setup" ]]; then
    echo "Setup ROS non trovato: $ros_setup" >&2
    exit 1
fi
safe_source "$ros_setup"

if [[ "$build_first" == "True" ]]; then
    echo "Compilo ${PACKAGE_NAME} in ${ROS_WS}..."
    cd "$ROS_WS"
    colcon build --packages-select progetto_robotica
fi

if [[ ! -f "$install_setup" ]]; then
    echo "Workspace non compilato o setup mancante: $install_setup" >&2
    echo "Riprova con: bash scripts/run_demo.sh ${scenario_arg} --build" >&2
    exit 1
fi
safe_source "$install_setup"

echo "Scenario: ${scenario}"
echo "Headless: ${headless}"
echo "Dashboard: http://localhost:5000"

ros2 launch progetto_robotica teleop_sim_launch.py scenario:="${scenario}" headless:="${headless}"
