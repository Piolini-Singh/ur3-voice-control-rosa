#!/usr/bin/env bash
# Create the Python virtualenv for the voice agent (thesis, section 4.2.2).
#
# --system-site-packages is required: rclpy comes from ROS 2 Humble and is
# not pip-installable, so the venv must be able to see the system packages.
#
# Usage (from the repository root):
#     source /opt/ros/humble/setup.bash
#     ./scripts/setup_venv.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/venv_agente_vocale"

if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "rclpy not importable: run 'source /opt/ros/humble/setup.bash' first." >&2
  exit 1
fi

python3 -m venv --system-site-packages "${VENV_DIR}"
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python3 -m pip install --upgrade pip
# Exact package set used for the thesis experiments. To get the newest compatible
# versions instead, run: pip install -r voice_agent/requirements.txt
python3 -m pip install -r "${REPO_ROOT}/voice_agent/requirements.lock"

if [ ! -f "${REPO_ROOT}/.env" ]; then
  cp "${REPO_ROOT}/.env.example" "${REPO_ROOT}/.env"
  echo "Created .env from .env.example: put your OPENAI_API_KEY in it."
fi

echo "Done. Activate with: source ${VENV_DIR}/bin/activate"
