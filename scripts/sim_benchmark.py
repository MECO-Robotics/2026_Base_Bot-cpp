#!/usr/bin/env python3
"""Run model benchmarks with provenance and an enforceable p99 deadline."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def capture(command):
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    return (result.stdout + result.stderr).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, default=Path('build/jolt-release'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-p99-ms', type=float, default=4.)
    args = parser.parse_args()
    cache = {}
    for line in (args.build_dir / 'CMakeCache.txt').read_text().splitlines():
        if '=' in line and not line.startswith(('#', '//')):
            key, value = line.split('=', 1)
            cache[key.split(':', 1)[0]] = value
    executable = args.build_dir / ('simulation_benchmark.exe' if os.name == 'nt' else 'simulation_benchmark')
    results = json.loads(subprocess.check_output([str(executable.resolve())], text=True))
    compiler = cache.get('CMAKE_CXX_COMPILER', 'unknown')
    compiler_version = capture([compiler, '/?'] if os.name == 'nt' else [compiler, '--version'])
    cpu = platform.processor()
    if platform.system() == 'Darwin':
        cpu = capture(['sysctl', '-n', 'machdep.cpu.brand_string'])
    if not cpu and Path('/proc/cpuinfo').exists():
        cpu = next((line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')), '')
    passed = all(row['p99Seconds'] < args.max_p99_ms / 1000 for row in results)
    evidence = {'metadata': {'machine': platform.uname()._asdict(), 'cpu': cpu,
                             'compiler': compiler, 'compilerVersion': compiler_version,
                             'buildType': cache.get('CMAKE_BUILD_TYPE', ''),
                             'cxxFlags': {key: value for key, value in cache.items() if key.startswith('CMAKE_CXX_FLAGS')},
                             'revision': capture(['git', 'rev-parse', 'HEAD']),
                             'workingTreeChanges': capture(['git', 'status', '--porcelain']),
                             'measurement': 'Complete registered model/arena before -> Jolt -> after substep; excludes robot controls and telemetry'},
                'acceptance': {'maximumP99Milliseconds': args.max_p99_ms, 'passed': passed},
                'results': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps(evidence['acceptance']))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
