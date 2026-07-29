#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/devel/setup.bash"
cd "${ROOT_DIR}/third_party/HPHS"
python3 ./scripts/explorer.py
