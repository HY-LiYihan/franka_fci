#!/usr/bin/env bash
set -Eeuo pipefail

# One-shot user-directory deployment for the FR3 control service.
# No sudo, systemd, firewall, or auto-start configuration is performed.

REPO_URL="${FRANKA_REPO_URL:-https://github.com/HY-LiYihan/franka_fci.git}"
INSTALL_DIR="${FRANKA_INSTALL_DIR:-$HOME/franka_fci}"
PYTHON_VERSION="${FRANKA_PYTHON_VERSION:-3.11}"
API_HOST="${FRANKA_API_HOST:-}"
API_PORT="${FRANKA_API_PORT:-8000}"
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"

info() { printf '\n[franka-fci] %s\n' "$*"; }
die() { printf '\n[franka-fci] ERROR: %s\n' "$*" >&2; exit 1; }

command -v git >/dev/null 2>&1 || die "git is required; install it first."
command -v curl >/dev/null 2>&1 || die "curl is required; install it first."

if ! command -v uv >/dev/null 2>&1; then
  info "Installing uv"
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi
command -v uv >/dev/null 2>&1 || die "uv was installed but is not on PATH. Run: source \"$HOME/.local/bin/env\""

if [[ -e "$INSTALL_DIR/.git" ]]; then
  info "Updating existing checkout: $INSTALL_DIR"
  git -C "$INSTALL_DIR" pull --ff-only
elif [[ -e "$INSTALL_DIR" ]]; then
  die "$INSTALL_DIR exists but is not a git checkout. Set FRANKA_INSTALL_DIR to another path."
else
  info "Cloning repository into $INSTALL_DIR"
  git clone "$REPO_URL" "$INSTALL_DIR"
fi

cd "$INSTALL_DIR"
info "Preparing Python $PYTHON_VERSION environment"
uv python install "$PYTHON_VERSION"
uv python pin "$PYTHON_VERSION"
uv sync

if [[ ! -f .env ]]; then
  info "Creating .env"
  cp .env.example .env
  if [[ -z "$API_HOST" ]]; then
    API_HOST="127.0.0.1"
  fi
  python3 - "$API_HOST" "$API_PORT" "$ROBOT_IP" <<'PY'
from pathlib import Path
import sys

path = Path(".env")
text = path.read_text()
replacements = {
    "FRANKA_ROBOT_IP=172.16.0.2": f"FRANKA_ROBOT_IP={sys.argv[3]}",
    "FRANKA_API_HOST=127.0.0.1": f"FRANKA_API_HOST={sys.argv[1]}",
    "FRANKA_API_PORT=8000": f"FRANKA_API_PORT={sys.argv[2]}",
}
for old, new in replacements.items():
    text = text.replace(old, new)
path.write_text(text)
PY
  chmod 600 .env
else
  info "Keeping existing .env (not overwritten)"
fi

info "Running offline checks"
uv run pytest -q
uv run ruff check franka_fci tests

info "Deployment complete"
printf '%s\n' "Project: $INSTALL_DIR" "Config:  $INSTALL_DIR/.env" "Start:   cd '$INSTALL_DIR' && uv run python -m franka_fci.main"
