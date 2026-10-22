#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
rebuilt_dir="${REBUILT_DIR:-${repo_root}/../2026-Rebuilt}"
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 <cpp-replay-output.wpilog> [phoenix|rev]" >&2
  exit 2
fi
input="$(realpath "$1")"
backend="${2:-phoenix}"

"${rebuilt_dir}/gradlew" -p "${rebuilt_dir}" \
  -I "${repo_root}/tools/akit-acceptance/akit-acceptance.init.gradle" \
  -Dakit.acceptance.source="${repo_root}/tools/akit-acceptance/AkitReplayOutputAcceptanceTest.java" \
  -Dakit.acceptance.output="${input}" \
  -Dakit.acceptance.backend="${backend}" \
  --no-daemon --console=plain \
  -x spotlessApply -x spotlessJava -x spotlessJavaApply -x spotlessGroovyGradle \
  -x spotlessGroovyGradleApply -x spotlessJson -x spotlessJsonApply -x spotlessMisc \
  -x createVersionFile -x spotlessMiscApply \
  test --tests AkitReplayOutputAcceptanceTest
