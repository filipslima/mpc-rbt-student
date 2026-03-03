#!/usr/bin/env bash
set -eo pipefail

# Set default LOG_LEVEL if not already set
export LOG_LEVEL=${LOG_LEVEL:-6}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${ROOT_DIR}/build/sender_node"
CFG="${1:-${ROOT_DIR}/config.json}"

if [[ ! -x "${BIN}" ]]; then
  echo "ERROR: ${BIN} not found or not executable. Run ./build.sh first."
  exit 1
fi

if [[ ! -f "${CFG}" ]]; then
  echo "ERROR: config not found at: ${CFG}"
  echo "Usage: ./run_sender.sh [path/to/config.json]"
  exit 1
fi

exec "${BIN}" "${CFG}"