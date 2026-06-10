#!/usr/bin/env bash
# Sourced helper — resolve the conda/mamba base + the Isaac / ROS env prefixes
# WITHOUT hardcoding anyone's home directory, so the repo runs on any machine.
#
# Resolution order for the conda base (first hit wins):
#   1. $R2D3_CONDA_BASE              (explicit override)
#   2. `conda info --base`           (if conda is on PATH)
#   3. $MAMBA_ROOT_PREFIX
#   4. $CONDA_PREFIX                 (de-nested if it's an activated env)
#   5. common install locations under $HOME and /opt
#
# Env names default to `isaac` / `ros_humble`; override with R2D3_ISAAC_ENV /
# R2D3_ROS_ENV, or point R2D3_ISAAC_PY / R2D3_ROS_PREFIX straight at a prefix.
#
# After sourcing, call `r2d3_resolve_envs` then read:
#   R2D3_CONDA_BASE  R2D3_ISAAC_PREFIX  R2D3_ISAAC_PY  R2D3_ROS_PREFIX

r2d3_resolve_envs() {
  local base=""
  if [[ -n "${R2D3_CONDA_BASE:-}" ]]; then
    base="$R2D3_CONDA_BASE"
  elif command -v conda >/dev/null 2>&1 && base="$(conda info --base 2>/dev/null)" && [[ -n "$base" ]]; then
    :
  elif [[ -n "${MAMBA_ROOT_PREFIX:-}" ]]; then
    base="$MAMBA_ROOT_PREFIX"
  elif [[ -n "${CONDA_PREFIX:-}" ]]; then
    if [[ "$CONDA_PREFIX" == */envs/* ]]; then base="${CONDA_PREFIX%/envs/*}"; else base="$CONDA_PREFIX"; fi
  else
    local d
    for d in "$HOME/miniforge3" "$HOME/mambaforge" "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
      if [[ -d "$d/envs" ]]; then base="$d"; break; fi
    done
  fi

  local isaac_env="${R2D3_ISAAC_ENV:-isaac}"
  local ros_env="${R2D3_ROS_ENV:-ros_humble}"
  R2D3_CONDA_BASE="$base"
  R2D3_ISAAC_PREFIX="${R2D3_ISAAC_PREFIX:-$base/envs/$isaac_env}"
  R2D3_ISAAC_PY="${R2D3_ISAAC_PY:-$R2D3_ISAAC_PREFIX/bin/python}"
  R2D3_ROS_PREFIX="${R2D3_ROS_PREFIX:-$base/envs/$ros_env}"
  export R2D3_CONDA_BASE R2D3_ISAAC_PREFIX R2D3_ISAAC_PY R2D3_ROS_PREFIX
}
