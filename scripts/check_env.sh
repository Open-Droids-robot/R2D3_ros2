#!/usr/bin/env bash
# scripts/check_env.sh — quick sanity check of the host environment we depend on.
# Read-only. Safe to run any time.
set -uo pipefail

ok()    { printf "  \033[32m✓\033[0m %s\n" "$*"; }
warn()  { printf "  \033[33m!\033[0m %s\n" "$*"; }
fail()  { printf "  \033[31m✗\033[0m %s\n" "$*"; }
head()  { printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

head "Identity"
echo "  user:    $(whoami) (uid $(id -u))"
echo "  host:    $(hostname)"
echo "  groups:  $(id -Gn)"
[[ "$(id -Gn)" == *docker*    ]] && ok "in docker group"    || fail "NOT in docker group"
[[ "$(id -Gn)" == *vglusers*  ]] && ok "in vglusers group"  || warn "NOT in vglusers — VirtualGL won't work"

head "Disk"
df -h /usr0 /usr1 / 2>/dev/null | awk 'NR==1 || /\/usr[01]|^\/dev.*\s\/$/'
usr0_pct=$(df --output=pcent /usr0 2>/dev/null | tail -1 | tr -dc 0-9)
if   [[ -n "$usr0_pct" && "$usr0_pct" -ge 95 ]]; then fail "/usr0 is ${usr0_pct}% full — large docker pulls WILL fail"
elif [[ -n "$usr0_pct" && "$usr0_pct" -ge 85 ]]; then warn "/usr0 is ${usr0_pct}% full — tight"
else ok "/usr0 has headroom (${usr0_pct}%)"
fi

head "GPUs (nvidia-smi)"
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=index,name,driver_version,memory.used,memory.total,utilization.gpu \
             --format=csv,noheader | awk -F, '{printf "  GPU%s:%s | drv:%s | mem:%s/%s | util:%s\n",$1,$2,$3,$4,$5,$6}'
  ok "nvidia-smi responds"
else
  fail "nvidia-smi not found"
fi

head "Docker"
if docker info >/dev/null 2>&1; then
  ok "docker reachable without sudo"
  echo "  $(docker version --format 'client {{.Client.Version}} | server {{.Server.Version}}')"
  echo "  data-root: $(docker info --format '{{.DockerRootDir}}')"
  # Just the runtime *names*, not the full feature map Docker's Go template dumps.
  runtimes=$(docker info 2>/dev/null | awk -F: '/^ Runtimes:/ {print $2}' | sed 's/^ *//')
  echo "  runtimes:  $runtimes"
  if [[ "$runtimes" == *nvidia* ]]; then
    ok "nvidia runtime registered"
  else
    fail "nvidia runtime NOT registered in docker"
  fi
else
  fail "cannot reach docker daemon (in 'docker' group?)"
fi

head "GUI prerequisites"
[[ -x /opt/TurboVNC/bin/vncserver ]] && ok "TurboVNC at /opt/TurboVNC/bin/vncserver" || fail "TurboVNC missing"
[[ -x /opt/VirtualGL/bin/vglrun   ]] && ok "VirtualGL at /opt/VirtualGL/bin/vglrun" || fail "VirtualGL missing"
command -v Xvfb >/dev/null && ok "Xvfb available (fallback)"

head "Python / conda"
if [[ -x "$HOME/miniforge3/bin/conda" ]]; then
  ok "miniforge at $HOME/miniforge3"
  if "$HOME/miniforge3/bin/conda" env list | grep -q '^r2d3 '; then
    ok "conda env 'r2d3' exists"
  else
    warn "conda env 'r2d3' not found"
  fi
else
  warn "miniforge not installed at $HOME/miniforge3"
fi

head "Network reachability (HTTPS, 5s timeout each)"
for url in github.com nvcr.io download.docker.com pypi.org; do
  code=$(curl -sS -o /dev/null -w '%{http_code}' --max-time 5 "https://$url" 2>/dev/null || echo "ERR")
  case "$code" in
    2*|301|302|401) ok "$url -> $code" ;;
    *)              fail "$url -> $code" ;;
  esac
done

head "Done"
