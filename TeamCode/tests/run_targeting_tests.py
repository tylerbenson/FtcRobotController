#!/usr/bin/env python3
"""Run the Java integration tests with FTC SDK classes.

Requires Python 3 and JDK 17+. Downloads small pinned SDK test artifacts into a temporary
directory cache; no native OpenCV dependency.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.request
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    team = Path(__file__).resolve().parents[1]
    cache = Path(tempfile.gettempdir()) / "ftc-cell-targeting-tests"
    cache.mkdir(exist_ok=True)
    artifacts = {
        "json.jar": "org/json/json/20240303/json-20240303.jar",
        "Hardware.aar": "org/firstinspires/ftc/Hardware/12.0.0/Hardware-12.0.0.aar",
        "RobotCore.aar": "org/firstinspires/ftc/RobotCore/12.0.0/RobotCore-12.0.0.aar",
    }
    for name, artifact in artifacts.items():
        dest = cache / name
        if not dest.exists():
            print(f"Downloading {name}", flush=True)
            partial = dest.with_suffix(".download")
            urllib.request.urlretrieve("https://repo.maven.apache.org/maven2/" + artifact, partial)
            partial.replace(dest)
    for name in ("Hardware", "RobotCore"):
        with zipfile.ZipFile(cache / (name + ".aar")) as archive:
            (cache / (name + ".jar")).write_bytes(archive.read("classes.jar"))
    java_home = os.environ.get("JAVA_HOME")
    def executable(name):
        return str(Path(java_home) / "bin" / name) if java_home else shutil.which(name)
    classes = cache / "classes"
    classes.mkdir(exist_ok=True)
    classpath = os.pathsep.join(str(cache / name) for name in
                               ("json.jar", "Hardware.jar", "RobotCore.jar"))
    subprocess.run([executable("javac"), "-cp", classpath, "-d", str(classes),
                    str(team / "src/main/java/org/firstinspires/ftc/teamcode/LimelightCellTargeting.java"),
                    str(team / "tests/LimelightCellTargetingTest.java")], check=True)
    subprocess.run([executable("java"), "-cp", str(classes) + os.pathsep + classpath,
                    "org.firstinspires.ftc.teamcode.LimelightCellTargetingTest"], check=True)


if __name__ == "__main__":
    main()
