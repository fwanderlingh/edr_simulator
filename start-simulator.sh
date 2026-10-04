#!/usr/bin/env bash
# Bootstrap a private runtime once, then run the simulator without activation.
set -euo pipefail

simulator_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_directory="$simulator_root/.conda"
tools_directory="$simulator_root/.tools"
mamba_executable="$tools_directory/bin/micromamba"
export MAMBA_ROOT_PREFIX="$simulator_root/.mamba"
export PYTHONNOUSERSITE=1
export CONDA_PREFIX="$runtime_directory"
export PATH="$runtime_directory/bin:$PATH"
python_executable="$runtime_directory/bin/python"

case "$(uname -s):$(uname -m)" in
    Linux:x86_64) platform=linux-64 ;;
    Linux:aarch64|Linux:arm64) platform=linux-aarch64 ;;
    Darwin:x86_64) platform=osx-64 ;;
    Darwin:arm64) platform=osx-arm64 ;;
    *) printf '%s\n' 'Unsupported platform. See README.md for manual environment setup.' >&2; exit 1 ;;
esac

if [[ ! -x "$mamba_executable" ]]; then
    printf '%s\n' 'First-time setup: downloading the local environment manager.' \
        'Internet is required for setup. Nothing is installed globally.'
    mkdir -p "$tools_directory"
    archive="$tools_directory/micromamba.tar.bz2"
    url="https://micro.mamba.pm/api/micromamba/$platform/latest"
    if command -v curl >/dev/null 2>&1; then
        curl --fail --location --retry 3 "$url" --output "$archive"
    elif command -v wget >/dev/null 2>&1; then
        wget --output-document="$archive" "$url"
    else
        printf '%s\n' 'Setup needs curl or wget to download the runtime.' >&2
        exit 1
    fi
    tar -xjf "$archive" -C "$tools_directory" bin/micromamba
fi

runtime_ready=false
if [[ -x "$runtime_directory/bin/python" ]]; then
    if "$python_executable" -c \
        'import sys, numpy, pybullet, tkinter; assert sys.version_info[:2] == (3, 12)'; then
        runtime_ready=true
    fi
fi
if [[ "$runtime_ready" == false ]]; then
    printf '%s\n' 'Setting up Python 3.12 and the precompiled physics engine. This may take a few minutes.'
    environment_file="$simulator_root/environment-$platform.lock"
    if [[ ! -f "$environment_file" ]]; then
        environment_file="$simulator_root/environment.yml"
    fi
    "$mamba_executable" create --yes --no-rc --prefix "$runtime_directory" --file "$environment_file"
    "$mamba_executable" clean --all --yes --no-rc > /dev/null
fi

cd -- "$simulator_root"
case "${1:-}" in
    --setup-only) printf '%s\n' 'Setup complete. Run bash start-simulator.sh to open the laboratory.' ;;
    --test)
        shift
        if ! "$python_executable" -c 'import pytest' 2> /dev/null; then
            "$mamba_executable" install --yes --no-rc --prefix "$runtime_directory" 'pytest>=8,<9'
            "$mamba_executable" clean --all --yes --no-rc > /dev/null
        fi
        exec "$python_executable" -m pytest tests "$@" ;;
    *) exec "$python_executable" -m robotics_sim "$@" ;;
esac
