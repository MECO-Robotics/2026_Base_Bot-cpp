#!/usr/bin/env python3
"""Launch an isolated local HAL simulator on Windows or Linux.

Copies the executable and libraries before starting so later builds cannot replace
loaded files. Pass --smoke-output to own both simulator and smoke-driver lifetime.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import uuid
import subprocess
import sys
import tempfile
import time
import zipfile


def snapshot_windows_dlls(sources, destination):
    """Colocate release DLLs with the executable, as Windows' loader expects.

    Keep libraries immutable even when Gradle rebuilds. Reject conflicting DLL
    basenames rather than silently mixing different native configurations.
    """
    copied = {}
    for source in sources:
        if not source.exists():
            continue
        for library in sorted(source.rglob('*')):
            if not library.is_file() or library.suffix.lower() != '.dll':
                continue
            key = library.name.lower()
            digest = hashlib.sha256(library.read_bytes()).hexdigest()
            if key in copied and copied[key]['sha256'] != digest:
                raise RuntimeError(f'Conflicting release runtime DLLs: {library.name}')
            if key not in copied:
                shutil.copy2(library, destination / library.name)
                copied[key] = {'source': str(library), 'sha256': digest}
    if not copied:
        raise RuntimeError('No Windows runtime DLLs found in the Gradle release installation')
    return copied


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--smoke-output', type=Path)
    action.add_argument('--replay-log', type=Path, help='Rerun C++ commands against recorded sensor/DS inputs')
    parser.add_argument('--replay-output', type=Path, default=Path('artifacts/replayed.wpilog'))
    parser.add_argument('--replay-report', type=Path, default=Path('artifacts/replay-comparison.json'))
    parser.add_argument('--replay-timeout', type=float, default=120)
    parser.add_argument('--backend', choices=['phoenix', 'jolt-planar', 'jolt-3d'])
    args = parser.parse_args()
    if platform.system() not in ('Windows', 'Linux'):
        parser.error('Full robot simulation requires Windows or Linux with Phoenix; '
                     'on macOS use the standalone simulation/ CMake models and benchmarks')
    root = Path(__file__).resolve().parents[1]
    recording = None
    if args.replay_log:
        from sim_replay import load_recording
        recording = load_recording(args.replay_log)
    args.backend = args.backend or 'jolt-3d'
    if args.backend not in ('phoenix', 'jolt-planar', 'jolt-3d'):
        parser.error('Unsupported recorded backend; select --backend explicitly')
    import math
    if not math.isfinite(args.replay_timeout) or args.replay_timeout <= 0:
        parser.error('--replay-timeout must be positive and finite')
    if args.replay_log:
        paths = [args.replay_log.resolve(), args.replay_output.resolve(), args.replay_report.resolve()]
        if len(set(paths)) != len(paths):
            parser.error('Source, replay WPILOG, and audit report must be distinct files')
        if args.replay_output.exists():
            parser.error('Replay output already exists; refusing overwrite')
    windows = platform.system() == 'Windows'
    target = 'windowsx86-64' if windows else 'linuxx86-64'
    suffix = '.exe' if windows else ''
    binary = root / f'build/exe/frcUserProgram/{target}/release/frcUserProgram{suffix}'
    app_libraries = root / f'build/install/frcUserProgram/{target}/release/lib'
    test_libraries = root / f'build/install/frcUserProgramTest/{target}/release/lib'
    libraries = app_libraries if app_libraries.exists() else test_libraries
    wpilib = Path(os.environ.get('WPILIB_ROOT', Path.home() / 'wpilib/2026'))
    archive = wpilib / f'maven/edu/wpi/first/halsim/halsim_ws_server/2026.2.1/halsim_ws_server-2026.2.1-{target}.zip'
    if not archive.exists():
        candidates = list((Path.home() / '.gradle/caches').rglob(archive.name))
        if candidates:
            archive = candidates[0]
    required = (binary, libraries) if recording else (binary, libraries, archive)
    if any(not p.exists() for p in required):
        parser.error(f'Build native release and install WPILib 2026.2.1 HAL websocket archive. Missing: {[str(p) for p in required if not p.exists()]}')
    # Refuse to attach the smoke driver to another process already using these ports.
    for port in (() if recording else (3300, 5810)):
        with socket.socket() as probe:
            # A bind also rejects a recently closed server's TIME_WAIT sockets.
            # Check live listeners; the run token protects the startup race.
            probe.settimeout(.25)
            if probe.connect_ex(('127.0.0.1', port)) == 0:
                parser.error(f'Local simulator port {port} is occupied; stop its owner first')
    with tempfile.TemporaryDirectory(prefix='frc-jolt-sim-') as directory:
        snapshot = Path(directory)
        shutil.copy2(binary, snapshot / binary.name)
        shutil.copytree(libraries, snapshot / 'lib')
        extension = None
        if not recording:
            with zipfile.ZipFile(archive) as source:
                source.extractall(snapshot / 'extension')
            extension = next((snapshot / 'extension').rglob('*halsim_ws_server.dll' if windows else '*halsim_ws_server.so'))
        runtime_manifest = {}
        if windows:
            # Gradle's installation is the authoritative resolved runtime set.
            # DLLs beside the EXE work independently of PATH spelling/search rules.
            runtime_manifest = snapshot_windows_dlls([libraries] + ([extension.parent] if extension else []), snapshot)
            print('Windows runtime snapshot: ' + json.dumps(runtime_manifest, sort_keys=True), flush=True)
            dumpbin = shutil.which('dumpbin')
            if dumpbin:
                dependencies = subprocess.run([dumpbin, '/DEPENDENTS', str(snapshot / binary.name)],
                                              capture_output=True, text=True, check=False)
                print(dependencies.stdout + dependencies.stderr, flush=True)
        environment = os.environ.copy()
        if recording:
            # Replay owns its clock and recorded IO; load no HAL websocket or physics.
            environment.pop('HALSIM_EXTENSIONS', None)
            environment.pop('FRC_SIM_PHYSICS', None)
            environment.pop('FRC_SIM_FIELD', None)
            environment['FRC_REPLAY_LOG'] = str(args.replay_log.resolve())
            environment['FRC_REPLAY_OUTPUT'] = str(args.replay_output.resolve())
            environment['FRC_REPLAY_SOURCE_SHA256'] = recording['sha256']
        else:
            for key in ('FRC_REPLAY_LOG', 'FRC_REPLAY_OUTPUT', 'FRC_REPLAY_SOURCE_SHA256'):
                environment.pop(key, None)
            environment['HALSIM_EXTENSIONS'] = str(extension)
            environment['FRC_SIM_PHYSICS'] = args.backend
        token = uuid.uuid4().hex
        environment['FRC_SIM_RUN_TOKEN'] = token
        path_key = 'PATH' if windows else 'LD_LIBRARY_PATH'
        if windows:
            # GitHub/MSVC environments spell it `Path`. Windows names are case
            # insensitive, but Python dictionaries are not: emit exactly one key.
            existing_paths = [key for key in environment if key.upper() == 'PATH']
            inherited_path = next((environment[key] for key in existing_paths), '')
            for key in existing_paths:
                del environment[key]
            environment[path_key] = os.pathsep.join(
                [str(snapshot), str(snapshot / 'lib')] + ([str(extension.parent)] if extension else []) + [inherited_path])
        else:
            environment[path_key] = str(snapshot / 'lib') + os.pathsep + environment.get(path_key, '')
        process = subprocess.Popen([str(snapshot / binary.name)], cwd=root, env=environment)
        try:
            if recording:
                try:
                    result = process.wait(timeout=args.replay_timeout)
                except subprocess.TimeoutExpired as error:
                    raise RuntimeError('Owned sensor replay exceeded its timeout') from error
                if result:
                    raise RuntimeError(f'Owned sensor replay failed with exit code {result}')
                from sim_replay import audit
                return audit(args.replay_log, args.replay_output, args.replay_report)
            if args.smoke_output:
                deadline = time.monotonic() + 20
                while True:
                    if process.poll() is not None:
                        raise RuntimeError(f'Owned simulator exited during startup: {process.returncode:#x}; Windows DLL snapshot: {list(runtime_manifest)}')
                    with socket.socket() as probe:
                        ready = probe.connect_ex(('127.0.0.1', 3300)) == 0
                    if ready:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError('Owned simulator did not open its HAL port')
                    time.sleep(.05)
                command = [sys.executable, str(root / 'scripts/sim_smoke.py'), '--output', str(args.smoke_output), '--run-token', token]
                if args.backend != 'phoenix':
                    command.append('--expect-jolt')
                smoke = subprocess.Popen(command, cwd=root)
                try:
                    while smoke.poll() is None:
                        if process.poll() is not None:
                            raise RuntimeError('Owned simulator exited while smoke driver was active')
                        time.sleep(.05)
                    return smoke.returncode
                finally:
                    if smoke.poll() is None:
                        smoke.terminate()
                        smoke.wait(timeout=5)
            return process.wait()
        except KeyboardInterrupt:
            return 130
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    sys.exit(main())
