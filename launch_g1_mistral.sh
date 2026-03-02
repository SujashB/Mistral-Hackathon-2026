#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ws_g1"
VENV_BIN="$SCRIPT_DIR/.venv/bin"
MODEL_NAME="${G1_MODEL_NAME:-g1-controller}"
GGUF_FILE="${G1_GGUF_FILE:-$SCRIPT_DIR/training/output/g1-controller.gguf}"
MODELFILE="${G1_MODELFILE:-$SCRIPT_DIR/training/output/Modelfile}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
LAUNCH_LOG="$ROS_LOG_DIR/g1_mistral_launch.log"
VLA_MODE=false
LAUNCH_PID=""
COMMAND_EXECUTION_TIMEOUT="${G1_COMMAND_TIMEOUT:-18}"
ENABLE_RVIZ="${G1_ENABLE_RVIZ:-true}"

# ── ASCII banner ───────────────────────────────────────────────────────
show_banner() {
    local CYAN='\033[0;36m'
    local BOLD='\033[1m'
    local DIM='\033[2m'
    local YELLOW='\033[0;33m'
    local MAGENTA='\033[0;35m'
    local NC='\033[0m'

    cat <<'BANNER'

    ███╗   ███╗ ██████╗ ████████╗██╗ ██████╗ ███╗   ██╗
    ████╗ ████║██╔═══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║
    ██╔████╔██║██║   ██║   ██║   ██║██║   ██║██╔██╗ ██║
    ██║╚██╔╝██║██║   ██║   ██║   ██║██║   ██║██║╚██╗██║
    ██║ ╚═╝ ██║╚██████╔╝   ██║   ██║╚██████╔╝██║ ╚████║
    ╚═╝     ╚═╝ ╚═════╝    ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝
     ██████╗ ██████╗ ██████╗ ██████╗ ███████╗
    ██╔════╝██╔═══██╗██╔══██╗██╔══██╗██╔════╝
    ██║     ██║   ██║██████╔╝██████╔╝███████╗
    ██║     ██║   ██║██╔══██╗██╔═══╝ ╚════██║
    ╚██████╗╚██████╔╝██║  ██║██║     ███████║
     ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚═╝     ╚══════╝

BANNER
    printf "    ${CYAN}${BOLD}MotionCorps${NC} — ${DIM}Humanoid Robot Control powered by Mistral${NC}\n"
    printf "    ${DIM}Unitree G1 · 29-DOF · Vision-Language-Action${NC}\n"
    printf "    ${MAGENTA}Mistral LLM${NC} + ${YELLOW}LLaVA Vision${NC} + ${CYAN}ROS 2${NC}\n"
    echo
    printf "    ${DIM}─────────────────────────────────────────────${NC}\n"
    echo
}
if [[ -z "${G1_VISION_SYSTEM_PROMPT:-}" ]]; then
    VISION_SYSTEM_PROMPT='You are the visual grounding module for a Unitree G1 humanoid robot. The robot is standing upright facing a table about 0.95m ahead. The table surface is at waist height (0.78m). Describe objects relative to the robot: "directly ahead", "ahead and to the left", "ahead and to the right". Include approximate distance from the robot when possible. Be concrete and grounded strictly in what is visible.'
else
    VISION_SYSTEM_PROMPT="$G1_VISION_SYSTEM_PROMPT"
fi

if [[ -z "${G1_VISION_PROMPT:-}" ]]; then
    VISION_PROMPT='Analyze this tabletop scene from the robot'"'"'s perspective and return JSON with this schema: {"objects":[{"name":"...","color":"...","shape":"...","size_description":"...","position_description":"relative to the robot, e.g. ahead-left at ~0.8m","spatial_relationships":["...","..."]}],"scene_description":"..."}. Describe positions relative to the robot (ahead, left, right). The table is directly in front of the robot.'
else
    VISION_PROMPT="$G1_VISION_PROMPT"
fi

if [[ -z "${G1_FUSION_SYSTEM_PROMPT:-}" ]]; then
    FUSION_SYSTEM_PROMPT='You are a motion-decision fusion layer for a Unitree G1 humanoid robot standing upright facing a table ~0.95m ahead at waist height. You receive a user command, a structured scene description from a vision model, and a draft joint plan from the motion model. Reconcile them into one safe final decision. When the user refers to objects or the table, the robot should reach or point toward the correct direction (forward = negative shoulder pitch, left = positive shoulder roll, right = negative shoulder roll). Prefer the scene-grounded interpretation when resolving ambiguity, preserve user intent when consistent with the scene, and output ONLY valid JSON with a top-level "joint_positions" object containing joint angles in radians.'
else
    FUSION_SYSTEM_PROMPT="$G1_FUSION_SYSTEM_PROMPT"
fi

if [[ -z "${G1_DEMO_SCENE_JSON:-}" ]]; then
    DEMO_SCENE_JSON='{"objects":[{"name":"red bottle","position_description":"on the table, ahead and to the robot'"'"'s left at ~0.8m","color":"red","shape":"bottle","size_description":"tall","spatial_relationships":["to the left of center"]}],"scene_description":"A wooden table is directly in front of the robot at ~0.95m distance, at waist height. A tall red bottle sits on the left side of the table. The table is within arm'"'"'s reach."}'
else
    DEMO_SCENE_JSON="$G1_DEMO_SCENE_JSON"
fi

# Parse flags
VOICE_MODE=false
for arg in "$@"; do
    case "$arg" in
        --vla)   VLA_MODE=true ;;
        --voice) VOICE_MODE=true ;;
    esac
done

cleanup() {
    if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo
        echo "Stopping G1 launch..."
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Missing required command: $1" >&2
        exit 1
    fi
}

source_compat() {
    local nounset_was_enabled=0

    if [[ $- == *u* ]]; then
        nounset_was_enabled=1
        set +u
    fi

    # shellcheck disable=SC1090
    source "$1"

    if (( nounset_was_enabled )); then
        set -u
    fi
}

source_ros() {
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        source_compat /opt/ros/jazzy/setup.bash
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
        source_compat /opt/ros/humble/setup.bash
    else
        echo "ROS 2 setup.bash not found under /opt/ros" >&2
        exit 1
    fi
}

ensure_workspace() {
    if [[ ! -d "$WS_DIR/install" ]]; then
        echo "Workspace is not built. Run: cd \"$WS_DIR\" && colcon build" >&2
        exit 1
    fi
}

build_workspace() {
    mkdir -p "$ROS_LOG_DIR"
    local build_log="$ROS_LOG_DIR/g1_build.log"
    (
        cd "$WS_DIR"
        source_ros
        colcon build --packages-select g1_package --event-handlers console_direct+
    ) >"$build_log" 2>&1 || {
        echo "Workspace build failed. Tail of $build_log:" >&2
        tail -n 80 "$build_log" >&2 || true
        exit 1
    }
}

source_workspace() {
    source_compat "$WS_DIR/install/setup.bash"
}

ensure_ollama() {
    require_cmd ollama

    if ! curl -sf http://localhost:11434/api/tags >/dev/null; then
        echo "Ollama is not reachable at http://localhost:11434" >&2
        echo "Start it with: ollama serve" >&2
        exit 1
    fi

    if ! ollama list | awk '{print $1}' | grep -Fxq "$MODEL_NAME" && \
       ! ollama list | awk '{print $1}' | grep -Fxq "${MODEL_NAME}:latest"; then
        register_ollama_model
    fi

    local vision_model="${G1_VISION_MODEL:-llava:7b}"
    if ! ollama list | awk '{print $1}' | grep -Fxq "$vision_model"; then
        echo "Vision model '$vision_model' is not installed."
        echo "Pull it with: ollama pull $vision_model"
        echo "(Continuing anyway — vision grounder will use fallback world state)"
    fi
}

register_ollama_model() {
    local generated_modelfile

    echo "Ollama model '$MODEL_NAME' is not registered. Attempting local registration..."

    if [[ -f "$MODELFILE" ]]; then
        ollama create "$MODEL_NAME" -f "$MODELFILE"
        return 0
    fi

    if [[ ! -f "$GGUF_FILE" ]]; then
        echo "No Modelfile or GGUF found for '$MODEL_NAME'." >&2
        echo "Expected one of:" >&2
        echo "  $MODELFILE" >&2
        echo "  $GGUF_FILE" >&2
        echo "Run ./run_pipeline.sh step 7 first, or set G1_MODELFILE/G1_GGUF_FILE." >&2
        exit 1
    fi

    generated_modelfile="$(mktemp)"
    cat >"$generated_modelfile" <<EOF
FROM $GGUF_FILE

SYSTEM """You are a robot motion controller for a Unitree G1 humanoid robot with 29 DOF. Given a natural language motion command, output target joint positions as JSON. All values in radians. Output ONLY the JSON object with a 'joint_positions' key."""

PARAMETER temperature 0.1
PARAMETER top_p 0.9
PARAMETER num_predict 512
PARAMETER stop </s>
EOF
    ollama create "$MODEL_NAME" -f "$generated_modelfile"
    rm -f "$generated_modelfile"
}

kill_stale_processes() {
    local pattern
    pattern='ros2 launch g1_package g1_|joint_state_publisher_gui|joint_state_publisher|mistral_joint_commander.py|vision_grounder.py|energy_monitor.py|behavior_manager.py|scene_marker_publisher.py|robot_state_publisher|rviz2'

    if pgrep -af "$pattern" >/dev/null 2>&1; then
        pkill -f "$pattern" || true
        sleep 2
    fi
}

start_launch() {
    mkdir -p "$ROS_LOG_DIR"
    : > "$LAUNCH_LOG"

    # Source .env to pick up ELEVENLABS_API_KEY and other secrets
    if [[ -f "$SCRIPT_DIR/.env" ]]; then
        set -a
        # shellcheck disable=SC1090
        source "$SCRIPT_DIR/.env"
        set +a
    fi

    export PATH="$VENV_BIN:$PATH"
    export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
    export ROS_LOG_DIR
    export G1_INFERENCE_BACKEND="${G1_INFERENCE_BACKEND:-ollama}"
    export G1_MODEL_NAME="$MODEL_NAME"

    (
        set +e
        cd "$SCRIPT_DIR"
        source_ros
        source_compat "$WS_DIR/install/setup.bash"
        local launch_file="g1_mistral.launch.xml"
        if $VLA_MODE; then
            launch_file="g1_vla.launch.xml"
        fi

        if [[ -n "${ELEVENLABS_API_KEY:-}" ]]; then
            export ELEVENLABS_API_KEY
        fi

        exec stdbuf -oL -eL ros2 launch g1_package "$launch_file" enable_rviz:="$ENABLE_RVIZ"
    ) >"$LAUNCH_LOG" 2>&1 &

    LAUNCH_PID="$!"
}

wait_for_commander() {
    local waited=0
    local timeout=30

    while (( waited < timeout )); do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "G1 launch exited unexpectedly. Tail of launch log:" >&2
            tail -n 50 "$LAUNCH_LOG" >&2 || true
            exit 1
        fi

        if grep -q "Listening on /voice_command topic" "$LAUNCH_LOG"; then
            return 0
        fi

        sleep 1
        ((waited += 1))
    done

    echo "Timed out waiting for the commander to start. Tail of launch log:" >&2
    tail -n 50 "$LAUNCH_LOG" >&2 || true
    exit 1
}

publish_command() {
    local command="$1"
    local escaped_command
    escaped_command="${command//\'/\'\"\'\"\'}"
    ros2 topic pub --once /voice_command std_msgs/msg/String "{data: '$escaped_command'}" >/dev/null
}

log_has_since() {
    local start_line="$1"
    local pattern="$2"
    awk -v start="$start_line" -v pattern="$pattern" 'NR > start && $0 ~ pattern { found=1; exit } END { exit(found ? 0 : 1) }' "$LAUNCH_LOG"
}

log_extract_last_since() {
    local start_line="$1"
    local pattern="$2"
    awk -v start="$start_line" -v pattern="$pattern" 'NR > start && $0 ~ pattern { line=$0 } END { print line }' "$LAUNCH_LOG"
}

log_extract_new_since() {
    local start_line="$1"
    local pattern="$2"
    awk -v start="$start_line" -v pattern="$pattern" 'NR > start && $0 ~ pattern { print NR "\t" $0 }' "$LAUNCH_LOG"
}

log_extract_all_since() {
    local start_line="$1"
    local pattern="$2"
    awk -v start="$start_line" -v pattern="$pattern" 'NR > start && $0 ~ pattern { print $0 }' "$LAUNCH_LOG"
}

show_command_progress() {
    local start_line="$1"
    local show_prompt_after="${2:-false}"
    local last_stream_line="$start_line"
    local deadline=$((SECONDS + COMMAND_EXECUTION_TIMEOUT))
    local stream_pattern='LLaVA |MotionLLM |Heuristic motion fallback applied|Motion LLM final response|Vision summary: |Execution movement verified|Execution completed after observed motion settled|Target joint positions updated successfully|Could not parse|ERROR|WARN|WARNING'
    local last_state=""

    local DIM='\033[2m'
    local NC='\033[0m'
    printf "  [command] queued\n"

    while (( SECONDS < deadline )); do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "Launch exited while handling the command. Tail of launch log:"
            tail -n 40 "$LAUNCH_LOG" || true
            return 1
        fi

        local streamed_lines
        streamed_lines="$(log_extract_new_since "$last_stream_line" "$stream_pattern")"
        if [[ -n "$streamed_lines" ]]; then
            while IFS=$'\t' read -r line_no raw_line; do
                [[ -z "$line_no" ]] && continue
                last_stream_line="$line_no"
                printf "  %s\n" "${raw_line##*] }"
            done <<< "$streamed_lines"
        fi

        if log_has_since "$start_line" "State: .* -> STABILIZING"; then
            if [[ "$last_state" != "stabilizing posture" ]]; then
                printf "  [state] stabilizing posture\n"
                last_state="stabilizing posture"
            fi
        elif log_has_since "$start_line" "Execution completed after observed motion settled"; then
            printf "  [state] complete\n"
            if [[ "$show_prompt_after" == "true" ]]; then
                print_cli_prompt
            fi
            return 0
        elif log_has_since "$start_line" "State: .* -> EXECUTING"; then
            if [[ "$last_state" != "executing motion" ]]; then
                printf "  [state] executing motion\n"
                last_state="executing motion"
            fi
        elif log_has_since "$start_line" "State: .* -> PLANNING"; then
            if [[ "$last_state" != "planning motion" ]]; then
                printf "  [state] planning motion\n"
                last_state="planning motion"
            fi
        elif log_has_since "$start_line" "State: .* -> PERCEIVING"; then
            if [[ "$last_state" != "analyzing scene" ]]; then
                printf "  [state] analyzing scene\n"
                last_state="analyzing scene"
            fi
        fi
        sleep 0.25
    done

    printf "  [state] still running\n"
    local warn_lines
    warn_lines="$(log_extract_all_since "$start_line" "WARN\|ERROR\|Could not parse")"
    if [[ -n "$warn_lines" ]]; then
        printf "  ${DIM}Warnings:${NC}\n"
        while IFS= read -r wl; do
            printf "  ${DIM}%s${NC}\n" "${wl##*] }"
        done <<< "$warn_lines"
    fi
    return 0
}

print_cli_prompt() {
    local BOLD='\033[1m'
    local NC='\033[0m'
    printf "${BOLD}motioncorps>${NC} "
}

voice_chat_loop() {
    local BOLD='\033[1m'
    local CYAN='\033[0;36m'
    local NC='\033[0m'

    printf "${CYAN}Voice mode active.${NC} Press ${BOLD}ENTER${NC} to speak — silence stops recording.\n\n"

    local voice_script="$SCRIPT_DIR/voice_input.py"
    if [[ ! -f "$voice_script" ]]; then
        echo "voice_input.py not found at $voice_script" >&2
        exit 1
    fi

    # Run voice_input.py as a co-process; read commands from its stdout
    coproc VOICE_PROC { "$VENV_BIN/python3" "$voice_script"; }

    while true; do
        # Forward our stdin (Enter keypresses) to the voice process
        IFS= read -r -p "" _enter </dev/tty || break
        printf "%s\n" "$_enter" >&"${VOICE_PROC[1]}"

        # Read the transcribed command from voice process stdout
        IFS= read -r command <&"${VOICE_PROC[0]}" || break
        command="${command#"${command%%[![:space:]]*}"}"
        command="${command%"${command##*[![:space:]]}"}"
        [[ -z "$command" ]] && continue
        [[ "$command" == "quit" || "$command" == "exit" ]] && break

        printf "${BOLD}motioncorps>${NC} %s\n" "$command"
        local log_start
        log_start=$(wc -l < "$LAUNCH_LOG" 2>/dev/null || echo 0)
        publish_command "$command"
        show_command_progress "$log_start" "true"
    done

    kill "${VOICE_PROC_PID}" 2>/dev/null || true
}

chat_loop() {
    while true; do
        print_cli_prompt
        read -r command || break
        command="${command#"${command%%[![:space:]]*}"}"
        command="${command%"${command##*[![:space:]]}"}"

        if [[ -z "$command" ]]; then
            continue
        fi

        if [[ "$command" == "quit" || "$command" == "exit" ]]; then
            break
        fi

        local log_start
        log_start=$(wc -l < "$LAUNCH_LOG" 2>/dev/null || echo 0)

        publish_command "$command"
        show_command_progress "$log_start" "true"
    done
}

trap cleanup EXIT INT TERM

require_cmd curl
require_cmd awk
require_cmd grep
require_cmd pgrep
require_cmd pkill
source_ros
require_cmd ros2
ensure_workspace
build_workspace
source_workspace
ensure_ollama
kill_stale_processes
start_launch
wait_for_commander
show_banner
if $VOICE_MODE; then
    voice_chat_loop
else
    chat_loop
fi
