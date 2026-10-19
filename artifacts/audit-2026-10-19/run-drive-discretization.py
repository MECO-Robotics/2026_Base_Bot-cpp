#!/usr/bin/env python3
"""Compile the standalone finite-step check against cached pinned WPILib 2026.2.1."""
from pathlib import Path
import subprocess

root = Path.home() / ".gradle/caches/8.11/transforms"
source = Path(__file__).with_name("drive-discretization.cpp")
output = Path("/tmp/base-drive-discretization")
includes = [f"-I{p}" for p in root.glob("*/transformed/*-headers")]
libraries = [next(root.glob(
    f"*/transformed/{name}-cpp-2026.2.1-linuxx86-64/linux/x86-64/shared/lib{name}.so"))
    for name in ("wpimath", "wpiutil")]
subprocess.run(["g++", "-std=c++23", str(source), *includes,
                *map(str, libraries),
                *[f"-Wl,-rpath,{p.parent}" for p in libraries],
                "-o", str(output)], check=True)
subprocess.run([str(output)], check=True)
