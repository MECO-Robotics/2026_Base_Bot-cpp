#!/usr/bin/env python3
"""Launch an isolated local HAL simulator on Windows or Linux.

Copies the executable and libraries before starting so later builds cannot replace
loaded files. Pass --smoke-output to own both simulator and smoke-driver lifetime.
"""
import argparse
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke-output', type=Path)
    parser.add_argument('--backend', choices=['phoenix', 'jolt-planar', 'jolt-3d'], default='jolt-planar')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    windows = platform.system() == 'Windows'
    target = 'windowsx86-64' if windows else 'linuxx86-64'
    suffix = '.exe' if windows else ''
    binary = root / f'build/exe/frcUserProgram/{target}/release/frcUserProgram{suffix}'
    libraries = root / f'build/install/frcUserProgramTest/{target}/release/lib'
    wpilib = Path(os.environ.get('WPILIB_ROOT', Path.home() / 'wpilib/2026'))
    archive = wpilib / f'maven/edu/wpi/first/halsim/halsim_ws_server/2026.2.1/halsim_ws_server-2026.2.1-{target}.zip'
    if not archive.exists():
        candidates = list((Path.home() / '.gradle/caches').rglob(archive.name))
        if candidates:
            archive = candidates[0]
    if not binary.exists() or not libraries.exists() or not archive.exists():
        parser.error(f'Build native release and install WPILib 2026.2.1 HAL websocket archive. Missing: {[str(p) for p in (binary, libraries, archive) if not p.exists()]}')
    # Refuse to attach the smoke driver to another process already using these ports.
    for port in (3300, 5810):
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
        with zipfile.ZipFile(archive) as source:
            source.extractall(snapshot / 'extension')
        extension = next((snapshot / 'extension').rglob('*halsim_ws_server.dll' if windows else '*halsim_ws_server.so'))
        environment = os.environ.copy()
        environment['HALSIM_EXTENSIONS'] = str(extension)
        environment['FRC_SIM_PHYSICS'] = args.backend
        token = uuid.uuid4().hex
        environment['FRC_SIM_RUN_TOKEN'] = token
        path_key = 'PATH' if windows else 'LD_LIBRARY_PATH'
        environment[path_key] = str(snapshot / 'lib') + os.pathsep + environment.get(path_key, '')
        process = subprocess.Popen([str(snapshot / binary.name)], cwd=root, env=environment)
        try:
            if args.smoke_output:
                deadline = time.monotonic() + 20
                while True:
                    if process.poll() is not None:
                        raise RuntimeError(f'Owned simulator exited during startup: {process.returncode}')
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
