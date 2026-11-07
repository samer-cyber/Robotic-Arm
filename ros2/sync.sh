#!/usr/bin/env bash
# Copy the workspace from the Windows drive into the WSL filesystem, then build.
#
# Why not just colcon build on /mnt/d: the 9p bridge is slow enough to make
# builds painful, it does not carry executable bits reliably, and colcon
# symlink installs across it break in confusing ways. Source of truth stays on
# D:, builds happen on ext4.
#
#   bash ros2/sync.sh              (from the repo root, inside WSL)
#   bash ros2/sync.sh --no-build
#
# The repo root is found from this script's own location, so the checkout can
# live anywhere. Override with ARM_REPO=/mnt/d/... if you really need to.
#
# NOTE: deliberately no `set -u`. The ROS setup scripts read
# AMENT_TRACE_SETUP_FILES and several other variables without defaults, so
# nounset kills the shell the moment you source them:
#     /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
set -eo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WIN_ROOT="${ARM_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
WS="$HOME/vla_ws"

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "!! missing: $1" >&2
    echo "   install with: $2" >&2
    exit 1
  }
}
need rsync  "sudo apt install -y rsync"
need colcon "sudo apt install -y python3-colcon-common-extensions"

[ -d "$WIN_ROOT" ] || { echo "!! cannot see $WIN_ROOT - is the D: drive mounted?" >&2; exit 1; }
[ -f "$ROS_SETUP" ] || { echo "!! no ROS 2 Jazzy at $ROS_SETUP" >&2; exit 1; }

# Sourcing a ROS setup file is the one thing that must run with nounset off.
# Wrapped so the reason is documented at the point of use, not just at the top.
ros_source() {
  set +u                      # already off script-wide; explicit here so the
                              # reason lives next to the line that needs it
  # shellcheck disable=SC1090
  source "$1"
}

echo "==> syncing workspace into $WS"
mkdir -p "$WS/src"
rsync -a --delete \
      --exclude __pycache__ --exclude "*.pyc" \
      "$WIN_ROOT/ros2/vla_ws/src/" "$WS/src/"

# armlink lives in host/ and is the single source of truth. Vendor a copy into
# the armlink_py package so the built workspace is self-contained.
echo "==> vendoring armlink from host/"
rsync -a --delete \
      --exclude __pycache__ --exclude "*.pyc" \
      "$WIN_ROOT/host/armlink/" "$WS/src/armlink_py/armlink/"
echo "    (do not edit the copy - edit host/armlink and re-run this script)"

# Belt and braces: strip CR from everything that came off the Windows
# drive. A stray CR in a Python file is survivable; in a shell script it
# is not, and the error it produces scrolls over its own first half:
#     invalid option name/Robotic Arm/ROS2/sync.sh: line 16: set: pipefail
# which is really
#     .../sync.sh: line 16: set: pipefail: invalid option name
# with the CR sending the cursor back to column 0.
echo "==> normalising line endings"
find "$WS/src" -type f \
     \( -name "*.py" -o -name "*.sh" -o -name "*.yaml" -o -name "*.srv" \
        -o -name "*.xml" -o -name "*.txt" -o -name "*.cfg" \) \
     -exec sed -i 's/\r$//' {} +

if [ "${1:-}" = "--no-build" ]; then
  echo "==> skipping build"
  exit 0
fi

echo "==> building interfaces"
cd "$WS"
ros_source "$ROS_SETUP"
# arm_vla_interfaces first and alone: the Python packages import the generated
# srv stubs, so they cannot even be configured until those exist.
colcon build --packages-select arm_vla_interfaces

echo "==> building python packages"
ros_source "$WS/install/setup.bash"
colcon build --packages-select armlink_py arm_vla

echo
echo "==> done. In every new terminal:"
echo "    source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash"
