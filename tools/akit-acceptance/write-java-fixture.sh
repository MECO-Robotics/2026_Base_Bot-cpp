#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
rebuilt_dir="${REBUILT_DIR:-${repo_root}/../2026-Rebuilt}"
output="${AKIT_JAVA_LOG_OUTPUT:-${repo_root}/build/akit-acceptance/java-akit.wpilog}"
output="$(realpath -m "${output}")"

if [[ ! -x "${rebuilt_dir}/gradlew" ]]; then
  echo "Gradle wrapper not found at ${rebuilt_dir}/gradlew (set REBUILT_DIR)" >&2
  exit 2
fi

"${rebuilt_dir}/gradlew" -p "${rebuilt_dir}" \
  -I "${repo_root}/tools/akit-acceptance/akit-acceptance.init.gradle" \
  -Dakit.acceptance.source="${repo_root}/tools/akit-acceptance/AkitJavaLogFixtureTest.java" \
  -Dakit.acceptance.javaOutput="${output}" \
  --no-daemon --console=plain \
  -x spotlessApply -x spotlessJava -x spotlessJavaApply -x spotlessGroovyGradle \
  -x spotlessGroovyGradleApply -x spotlessJson -x spotlessJsonApply -x spotlessMisc \
  -x createVersionFile test --tests AkitJavaLogFixtureTest

test -s "${output}"
printf 'Java AdvantageKit source fixture: %s\n' "${output}"
