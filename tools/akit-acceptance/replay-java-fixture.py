#!/usr/bin/env python3
"""Run the built production C++ robot against a Java AdvantageKit fixture."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path,
                        default=root / "build/akit-acceptance/java-akit.wpilog")
    parser.add_argument("--output", type=Path,
                        default=root / "build/akit-acceptance/cpp-replayed-java.wpilog")
    parser.add_argument("--configuration", choices=("debug", "release"), default="release",
                        help="configuration used to select the default executable and runtime libraries")
    parser.add_argument("--binary", type=Path, default=None,
                        help="robot executable; if its path contains /debug/ or /release/, that configuration selects matching libraries")
    parser.add_argument("--backend", choices=("phoenix", "rev"), default="phoenix")
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    source, output = (p.resolve() for p in (args.source, args.output))
    binary = (args.binary or root / f"build/exe/frcUserProgram/linuxx86-64/{args.configuration}/frcUserProgram").resolve()
    configuration = args.configuration
    for candidate in (binary, *binary.parents):
        if candidate.name in ("debug", "release"):
            configuration = candidate.name
            break
    if not source.is_file() or source.stat().st_size == 0:
        parser.error(f"Java AdvantageKit source log is missing or empty: {source}")
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f"built C++ robot executable is missing: {binary}; build it before running this wrapper")
    if source == output or output.exists():
        parser.error(f"output must be a new file distinct from the source: {output}")

    libs = root / f"build/install/frcUserProgram/linuxx86-64/{configuration}/lib"
    if not libs.is_dir():
        parser.error(f"C++ release runtime libraries are missing: {libs}; install/build the release target first")

    output.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    env = os.environ.copy()
    env.update({
        "FRC_REPLAY_LOG": str(source),
        "FRC_REPLAY_OUTPUT": str(output),
        "FRC_REPLAY_SOURCE_SHA256": digest,
        "FRC_DRIVE_BACKEND": args.backend,
        "LD_LIBRARY_PATH": str(libs) + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""),
    })
    for key in ("HALSIMWS_HOST", "HALSIMWS_PORT", "FRC_SIM_RUN_TOKEN"):
        env.pop(key, None)

    # Run an immutable copy so a concurrent build cannot replace a loaded binary.
    with tempfile.TemporaryDirectory(prefix="akit-cpp-replay-") as directory:
        snapshot = Path(directory) / binary.name
        shutil.copy2(binary, snapshot)
        snapshot.chmod(snapshot.stat().st_mode | 0o111)
        result = subprocess.run([str(snapshot)], cwd=root, env=env,
                                text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=args.timeout)
    print(result.stdout, end="")
    if result.returncode != 0:
        raise SystemExit(f"C++ production replay exited with status {result.returncode}")
    if not output.is_file() or output.stat().st_size == 0:
        raise SystemExit("C++ production replay did not produce its output WPILOG")

    expected_times = [1_000_000, 1_020_000, 1_040_000]
    acceptance = subprocess.run(
        [str(root / "tools/akit-acceptance/read-replay-output.sh"), str(output), args.backend],
        cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    print(acceptance.stdout, end="")
    if acceptance.returncode != 0:
        raise SystemExit(f"AdvantageKit Java reader acceptance failed with status {acceptance.returncode}")

    report = {
        "result": "PASS",
        "scope": "Java AdvantageKit 26.0.2 fixture through production C++ replay, then actual AdvantageKit WPILOGReader acceptance; output records are filtered by that reader",
        "source": str(source), "output": str(output), "sourceSha256": digest,
        "frameCount": len(expected_times), "timestampsUs": expected_times,
        "readerAcceptance": "passed",
    }
    report_path = output.with_suffix(output.suffix + ".acceptance.json")
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, sort_keys=True))
    print(f"Acceptance report: {report_path}")


if __name__ == "__main__":
    main()
