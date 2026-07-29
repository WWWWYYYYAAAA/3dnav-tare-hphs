#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHEEL_DIR="${ROOT_DIR}/.cache/torch_wheels"
PIP_WHEEL="${WHEEL_DIR}/pip-24.3.1-py3-none-any.whl"
POLICY_PATH="${ROOT_DIR}/third_party/rl_policy/a1/policy.pt"

if [ ! -f "${PIP_WHEEL}" ]; then
  echo "Missing pip wheel: ${PIP_WHEEL}" >&2
  echo "Create the wheel cache on the host, then rerun this script inside Docker." >&2
  exit 1
fi

if [ ! -f "${WHEEL_DIR}/torch-2.4.1+cpu-cp38-cp38-linux_x86_64.whl" ]; then
  echo "Missing torch wheel under ${WHEEL_DIR}" >&2
  echo "Expected torch-2.4.1+cpu for CPython 3.8 / linux_x86_64." >&2
  exit 1
fi

cd "${ROOT_DIR}"

python3 "${PIP_WHEEL}/pip" install \
  --no-index \
  --find-links "${WHEEL_DIR}" \
  pip==24.3.1 \
  torch==2.4.1+cpu

python3 -m pip --version

python3 - <<PY
import torch

policy_path = "${POLICY_PATH}"
print("TORCH_OK", torch.__version__)

model = torch.jit.load(policy_path, map_location="cpu")
output = model(torch.zeros(1, 45))
assert tuple(output.shape) == (1, 12), tuple(output.shape)
print("POLICY_OK", tuple(output.shape))
PY
