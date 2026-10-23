#!/usr/bin/env python3
"""Link a tiny clean robot project against the staged MECO vendordep."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=("roborio", "desktop"), required=True)
    parser.add_argument("--maven-repo", type=Path, required=True)
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="meco-vendor-consumer-") as temporary:
        project = Path(temporary)
        (project / "vendordeps").mkdir()
        (project / "src/main/cpp").mkdir(parents=True)
        shutil.copy2(repo / "vendor/MecoBase.json", project / "vendordeps/MecoBase.json")
        shutil.copy2(repo / "settings.gradle", project / "settings.gradle")
        (project / "src/main/cpp/Robot.cpp").write_text(
            '#include "base/Drive.h"\n'
            '#include "base/Mechanism.h"\n'
            "int main() { base::DriveConfig{}.Validate(); base::Gains{}.Validate(false); "
            "return 0; }\n",
            encoding="utf-8",
        )
        platform = "wpi.platforms.roborio" if args.target == "roborio" else "wpi.platforms.desktop"
        (project / "build.gradle").write_text(
            f'''plugins {{ id "c"; id "cpp"; id "edu.wpi.first.GradleRIO" version "2026.2.1" }}
repositories {{ maven {{ url = uri(findProperty("mecoVendorRepo")) }} }}
nativeUtils.platformConfigs.all {{
    cppCompiler.args.removeAll {{ it.startsWith('-std=') || it.startsWith('/std:') }}
    cppCompiler.args.add(name.startsWith('windows') ? '/std:c++latest' : '-std=c++23')
}}
model {{ components {{ frcUserProgram(NativeExecutableSpec) {{
    targetPlatform {platform}
    sources.cpp {{ source {{ srcDir 'src/main/cpp'; include '**/*.cpp' }} }}
    lib library: 'mecoBase', linkage: 'static'
    wpi.cpp.vendor.cpp(it)
    wpi.cpp.deps.wpilib(it)
}} }} }}
''',
            encoding="utf-8",
        )

        gradle = repo / ("gradlew.bat" if os.name == "nt" else "gradlew")
        command = [str(gradle), "-p", str(project), "linkFrcUserProgramDebugExecutable",
                   f"-PmecoVendorRepo={args.maven_repo.resolve()}", "--max-workers=1"]
        if os.name == "nt":
            command.extend(["-Ptoolchain-optional-roboRio"])
            command = ["cmd", "/c", *command]
        subprocess.run(command, cwd=repo, check=True)


if __name__ == "__main__":
    main()
