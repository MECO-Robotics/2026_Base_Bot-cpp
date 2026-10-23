#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
rebuilt_dir="${REBUILT_DIR:-${repo_root}/../2026-Rebuilt}"
if [[ $# -lt 1 ]]; then
  echo "usage: $0 <cpp-generated.wpilog> [path=LoggableType ...]" >&2
  exit 2
fi
input="$(realpath "$1")"
shift
required_keys=""
if [[ $# -gt 0 ]]; then
  printf -v required_keys '%s,' "$@"
  required_keys="${required_keys%,}"
fi

if [[ ! -x "${rebuilt_dir}/gradlew" ]]; then
  echo "Gradle wrapper not found at ${rebuilt_dir}/gradlew (set REBUILT_DIR)" >&2
  exit 2
fi

"${rebuilt_dir}/gradlew" -p "${rebuilt_dir}" \
  -I "${repo_root}/tools/akit-acceptance/akit-acceptance.init.gradle" \
  -Dakit.acceptance.source="${repo_root}/tools/akit-acceptance/AkitReaderAcceptanceTest.java" \
  -Dakit.acceptance.input="${input}" \
  -Dakit.acceptance.keys="${required_keys}" \
  --no-daemon --console=plain \
  -x spotlessApply -x spotlessJava -x spotlessJavaApply -x spotlessGroovyGradle \
  -x spotlessGroovyGradleApply -x spotlessJson -x spotlessJsonApply -x spotlessMisc \
  -x createVersionFile \
  -x spotlessMiscApply test --tests AkitReaderAcceptanceTest
