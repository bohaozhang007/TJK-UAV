#!/usr/bin/env bash
# Run after console init/takeoff. Required: --detector-host IP --img FILE --box FILE
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec python3 -m app.agent.logic.v22 "$@"
