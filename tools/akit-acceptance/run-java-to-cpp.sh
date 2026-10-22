#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source_log="${AKIT_JAVA_LOG_OUTPUT:-${repo_root}/build/akit-acceptance/java-akit.wpilog}"
source_log="$(realpath -m "${source_log}")"

if [[ "${1:-}" == "--generate" ]]; then
  shift
  AKIT_JAVA_LOG_OUTPUT="${source_log}" "${repo_root}/tools/akit-acceptance/write-java-fixture.sh"
fi

exec python3 "${repo_root}/tools/akit-acceptance/replay-java-fixture.py" \
  "${source_log}" "$@"
