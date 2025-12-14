#!/usr/bin/env bash
set -euo pipefail

# setup_devcontainer.sh (no venv)
# Installs python3, pip, build tools, and common Python developer tooling globally in the container.
#
# Usage:
#  - Place at scripts/setup_devcontainer.sh
#  - chmod +x scripts/setup_devcontainer.sh
#  - Run as root (recommended) or as user (script will use sudo for privileged ops)

# PACKAGES: apt packages required for building Python packages and common tooling
APT_PKGS=(
  build-essential
  git
  curl
  ca-certificates
  python3
  python3-pip
  python3-venv
  python3-dev
  python3-distutils
  libffi-dev
  libssl-dev
  pkg-config
  libsndfile1
  ffmpeg
  nano
)

# PIP_PACKAGES: packages to install globally (change as needed)
PIP_PACKAGES=(
  pip-tools
  pipx
  black
  isort
  flake8
  mypy
  pytest
  ipython
  jupyterlab
  numpy
  pandas
  matplotlib
  scipy
  scikit-learn
  seaborn
  python-dotenv
  setuptools
  wheel
)

# Helper to run apt-get (uses sudo if not root)
run_apt() {
  if [ "$(id -u)" -eq 0 ]; then
    apt-get update -y
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
  else
    sudo apt-get update -y
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
  fi
}

# Helper to run pip (uses sudo when installing to system)
run_pip() {
  # always call with: run_pip <packages...>
  if [ "$(id -u)" -eq 0 ]; then
    python3 -m pip install --no-cache-dir --upgrade "$@"
  else
    # install system-wide needs sudo
    sudo python3 -m pip install --no-cache-dir --upgrade "$@"
  fi
}

echo "=== setup_devcontainer.sh (no venv) starting ==="

# 1) Install apt packages
echo "Installing apt packages..."
run_apt "${APT_PKGS[@]}"

# 2) Upgrade pip / setuptools / wheel system-wide
echo "Upgrading pip, setuptools, wheel..."
run_pip pip setuptools wheel

# 3) Install pip packages globally
echo "Installing common Python dev packages globally (this may take a few minutes)..."
run_pip "${PIP_PACKAGES[@]}"

# 4) Ensure pipx is usable (pipx usually installs to /usr/local/bin when installed as root)
if command -v pipx >/dev/null 2>&1; then
  echo "Running pipx ensurepath (if needed)..."
  if [ "$(id -u)" -eq 0 ]; then
    # ensurepath may try to write to user shell files; skip when running as root in CI-like contexts
    pipx ensurepath || true
  else
    pipx ensurepath || true
  fi
else
  echo "pipx not found after install - skipping pipx ensurepath."
fi

# 5) Optional: set up convenient symlinks for python3/pip if not present
if [ ! -x /usr/local/bin/python ] && command -v python3 >/dev/null 2>&1; then
  echo "Creating /usr/local/bin/python -> python3 (if writable)..."
  if [ "$(id -u)" -eq 0 ]; then
    ln -sf "$(command -v python3)" /usr/local/bin/python || true
  else
    sudo ln -sf "$(command -v python3)" /usr/local/bin/python || true
  fi
fi
if [ ! -x /usr/local/bin/pip ] && command -v pip3 >/dev/null 2>&1; then
  echo "Creating /usr/local/bin/pip -> pip3 (if writable)..."
  if [ "$(id -u)" -eq 0 ]; then
    ln -sf "$(command -v pip3)" /usr/local/bin/pip || true
  else
    sudo ln -sf "$(command -v pip3)" /usr/local/bin/pip || true
  fi
fi

# 6) Clean up apt lists to keep image size small (if running in build step)
if [ "$(id -u)" -eq 0 ]; then
  apt-get clean
  rm -rf /var/lib/apt/lists/*
fi

echo "=== setup_devcontainer.sh complete ==="
echo ""
echo "Notes / next steps:"
echo " - Python is available at: $(command -v python3 || true)"
echo " - pip is available at:    $(command -v pip3 || true)"
echo " - pipx is available at:   $(command -v pipx || true)"
echo " - Common dev tools installed: ${PIP_PACKAGES[*]}"
echo ""
echo "If you'd like me to:"
echo " - bake these packages into the Dockerfile instead (recommended for reproducible images),"
echo " - or add optional installs like PyTorch (with CUDA-aware wheel selection),"
echo "then tell me which and I'll produce the Dockerfile snippet immediately."
