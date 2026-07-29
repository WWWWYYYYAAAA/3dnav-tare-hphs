#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_DIR="${ROOT_DIR}/third_party"
VENDOR_DIR="${ROOT_DIR}/src/vendor"

mkdir -p "${THIRD_PARTY_DIR}" "${VENDOR_DIR}"

clone_or_update() {
  local url="$1"
  local dir="$2"
  local branch="${3:-}"

  if [ -d "${dir}/.git" ]; then
    git -C "${dir}" fetch --all --prune
  else
    git clone "${url}" "${dir}"
  fi

  if [ -n "${branch}" ]; then
    git -C "${dir}" checkout "${branch}"
    git -C "${dir}" pull --ff-only origin "${branch}"
  else
    git -C "${dir}" pull --ff-only
  fi
}

link_dir() {
  local source_dir="$1"
  local link_path="$2"

  if [ -e "${link_path}" ] || [ -L "${link_path}" ]; then
    return
  fi

  ln -s "${source_dir}" "${link_path}"
}

clone_or_update "https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git" \
  "${THIRD_PARTY_DIR}/autonomous_exploration_development_environment" "noetic"
clone_or_update "https://github.com/caochao39/tare_planner.git" \
  "${THIRD_PARTY_DIR}/tare_planner" "melodic-noetic"
clone_or_update "https://github.com/bit-lsj/HPHS.git" \
  "${THIRD_PARTY_DIR}/HPHS"
clone_or_update "https://gitee.com/fdsf3e2342/3d-navi.git" \
  "${THIRD_PARTY_DIR}/3d-navi"

for pkg_dir in "${THIRD_PARTY_DIR}/autonomous_exploration_development_environment/src/"*; do
  [ -d "${pkg_dir}" ] || continue
  link_dir "${pkg_dir}" "${VENDOR_DIR}/$(basename "${pkg_dir}")"
done

link_dir "${THIRD_PARTY_DIR}/tare_planner/src/tare_planner" "${VENDOR_DIR}/tare_planner"
link_dir "${THIRD_PARTY_DIR}/HPHS" "${VENDOR_DIR}/HPHS"

for pkg_dir in "${THIRD_PARTY_DIR}/3d-navi/src/"*; do
  [ -d "${pkg_dir}" ] || continue
  link_dir "${pkg_dir}" "${VENDOR_DIR}/$(basename "${pkg_dir}")"
done

echo "Sources are ready under ${THIRD_PARTY_DIR}; catkin package links are under ${VENDOR_DIR}."
