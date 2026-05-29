#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
RES_DIR="${SCRIPT_DIR}/res"
INSTALL_ROOT="/opt/autoaim"
BIN_DIR="${INSTALL_ROOT}/bin"
TARGET_RES_DIR="${INSTALL_ROOT}/res"
DEFAULT_REMOTE_HOST="remote"
MEDIAMTX_VERSION="v1.18.2"

usage() {
    cat <<EOF
Usage:
  $(basename "$0") local
  $(basename "$0") remote [host]
EOF
}

require_command() {
    local command_name="$1"

    if ! command -v "${command_name}" >/dev/null 2>&1; then
        printf 'Missing command: %s\n' "${command_name}" >&2
        exit 1
    fi
}

run_privileged() {
    if command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        "$@"
    fi
}

download_mediamtx() {
    local version="$1"
    local output_dir="$2"
    local archive="mediamtx_${version}_linux_amd64.tar.gz"

    curl -L --fail \
        "https://github.com/bluenviron/mediamtx/releases/download/${version}/${archive}" \
        -o "${output_dir}/${archive}"
    tar -xzf "${output_dir}/${archive}" -C "${output_dir}"
}

append_path_to_zshrc() {
    local line='export PATH="/opt/autoaim/bin:$PATH"'
    local zshrc_path="${HOME}/.zshrc"

    touch "${zshrc_path}"
    if ! grep -Fq "${line}" "${zshrc_path}"; then
        printf '\n%s\n' "${line}" >>"${zshrc_path}"
  fi
}

installed_runtime_matches() {
  [[ -f "${BIN_DIR}/mediamtx" ]] \
    && "${BIN_DIR}/mediamtx" --version 2>/dev/null | grep -Fq "${MEDIAMTX_VERSION}"
}

install_runtime_dependencies() {
    local packages=(
        curl
        python3
        gstreamer1.0-tools
        gstreamer1.0-plugins-base
        gstreamer1.0-plugins-good
        gstreamer1.0-plugins-ugly
    )
    local missing_packages=()
    local package

    for package in "${packages[@]}"; do
        if ! dpkg -s "${package}" >/dev/null 2>&1; then
            missing_packages+=("${package}")
        fi
    done

    if [[ ${#missing_packages[@]} -eq 0 ]]; then
        return
    fi

    run_privileged apt update
    run_privileged apt install -y "${missing_packages[@]}"
}

install_from() {
    local payload_dir="$1"

    [[ -d "${payload_dir}" ]] || {
        printf 'Missing payload directory: %s\n' "${payload_dir}" >&2
        exit 1
    }

    [[ -f "${payload_dir}/bin/mediamtx" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/bin/mediamtx" >&2
        exit 1
    }

    [[ -f "${payload_dir}/bin/start-streamer" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/bin/start-streamer" >&2
        exit 1
    }

    [[ -f "${payload_dir}/res/mediamtx.yml" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/res/mediamtx.yml" >&2
        exit 1
    }

    [[ -f "${payload_dir}/res/playing.html" ]] || {
        printf 'Missing payload file: %s\n' "${payload_dir}/res/playing.html" >&2
        exit 1
    }

    install_runtime_dependencies

    run_privileged mkdir -p "${BIN_DIR}" "${TARGET_RES_DIR}"
    run_privileged install -m 755 "${payload_dir}/bin/mediamtx" "${BIN_DIR}/mediamtx"
    run_privileged install -m 755 "${payload_dir}/bin/start-streamer" "${BIN_DIR}/start-streamer"
    run_privileged install -m 644 "${payload_dir}/res/mediamtx.yml" "${TARGET_RES_DIR}/mediamtx.yml"
    run_privileged install -m 644 "${payload_dir}/res/playing.html" "${TARGET_RES_DIR}/playing.html"

    append_path_to_zshrc

    cat <<EOF
Install completed.

Binary directory:
  ${BIN_DIR}

Resource directory:
  ${TARGET_RES_DIR}

Next step:
  start-streamer
EOF
}

prepare_payload() {
  local payload_dir="$1"

  mkdir -p "${payload_dir}/bin" "${payload_dir}/res"

  install -m 755 "${RES_DIR}/start-streamer" "${payload_dir}/bin/start-streamer"
  install -m 644 "${RES_DIR}/mediamtx.yml" "${payload_dir}/res/mediamtx.yml"
  install -m 644 "${RES_DIR}/playing.html" "${payload_dir}/res/playing.html"

  if installed_runtime_matches; then
    printf 'Reusing installed mediamtx runtime: %s\n' "${MEDIAMTX_VERSION}"
    install -m 755 "${BIN_DIR}/mediamtx" "${payload_dir}/bin/mediamtx"
    return
  fi

  printf 'Downloading mediamtx runtime: %s\n' "${MEDIAMTX_VERSION}"
  download_mediamtx "${MEDIAMTX_VERSION}" "${payload_dir}/bin"
}

install_local() {
    local payload_dir

    payload_dir="$(mktemp -d /tmp/autoaim-install-local.XXXXXX)"
    prepare_payload "${payload_dir}"
    install_from "${payload_dir}"
}

install_remote() {
    local remote_host="$1"
    local local_payload_dir
    local remote_payload_dir="/tmp/autoaim-install-payload"

    require_command ssh
    require_command scp

    local_payload_dir="$(mktemp -d /tmp/autoaim-install-remote.XXXXXX)"
    prepare_payload "${local_payload_dir}"

    ssh "${remote_host}" "rm -rf '${remote_payload_dir}' && mkdir -p '${remote_payload_dir}'"
    scp -r "${local_payload_dir}/." "${remote_host}:${remote_payload_dir}/"
    scp "$0" "${remote_host}:${remote_payload_dir}/install-server.sh"
    ssh "${remote_host}" "bash '${remote_payload_dir}/install-server.sh' '${remote_payload_dir}'"
}

main() {
    case "${1:-}" in
    local)
        install_local
        ;;
    remote)
        install_remote "${2:-${DEFAULT_REMOTE_HOST}}"
        ;;
    *)
        if [[ $# -eq 1 && -d "$1" ]]; then
            install_from "$1"
            return
        fi
        usage >&2
        exit 1
        ;;
    esac
}

main "$@"
