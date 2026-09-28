#!/usr/bin/env python3
"""Run the Java integration tests with real desktop OpenCV and FTC SDK classes.

Requires Python 3 and JDK 17+. Downloads pinned Maven artifacts into a temporary
directory cache (~110 MB for desktop OpenCV); does not alter Android dependencies.
"""
import argparse
import csv
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
import urllib.request
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, help="Replay a recorded session instead of synthetic tests")
    parser.add_argument("--calibration", type=Path, help="Override the session's calibration JSON")
    parser.add_argument("--output", type=Path, help="Replay CSV path (default SESSION/targeting.csv)")
    args = parser.parse_args()
    if not args.replay and (args.calibration or args.output):
        parser.error("--calibration/--output require --replay")
    team = Path(__file__).resolve().parents[1]
    cache = Path(tempfile.gettempdir()) / "ftc-cell-targeting-tests"
    cache.mkdir(exist_ok=True)
    artifacts = {
        "opencv.jar": "org/openpnp/opencv/4.9.0-0/opencv-4.9.0-0.jar",
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
    system = {"Darwin": "osx", "Linux": "linux"}.get(platform.system())
    arch = {"arm64": "ARMv8", "aarch64": "ARMv8", "x86_64": "x86_64"}.get(platform.machine())
    if system is None or arch is None:
        raise SystemExit("Host tests support macOS/Linux on arm64 or x86_64.")
    native = "libopencv_java490." + ("dylib" if system == "osx" else "so")
    with zipfile.ZipFile(cache / "opencv.jar") as archive:
        (cache / native).write_bytes(archive.read(f"nu/pattern/opencv/{system}/{arch}/{native}"))
    java_home = os.environ.get("JAVA_HOME")
    def executable(name):
        return str(Path(java_home) / "bin" / name) if java_home else shutil.which(name)
    classes = cache / "classes"
    classes.mkdir(exist_ok=True)
    classpath = os.pathsep.join(str(cache / name) for name in
                               ("opencv.jar", "json.jar", "Hardware.jar", "RobotCore.jar"))
    subprocess.run([executable("javac"), "-cp", classpath, "-d", str(classes),
                    str(team / "src/main/java/org/firstinspires/ftc/teamcode/LimelightCellTargeting.java"),
                    str(team / "tests/LimelightCellTargetingTest.java"),
                    str(team / "tests/LimelightReplay.java")], check=True)
    def run_java(main_class, *arguments):
        subprocess.run([executable("java"), "-Djava.library.path=" + str(cache), "-cp",
                        str(classes) + os.pathsep + classpath,
                        "org.firstinspires.ftc.teamcode." + main_class, *arguments], check=True)
    if args.replay:
        run_java("LimelightReplay", str(args.replay),
                 str(args.calibration) if args.calibration else "",
                 str(args.output) if args.output else "")
    else:
        with tempfile.TemporaryDirectory(prefix="ftc-replay-test-") as fixture:
            run_java("LimelightCellTargetingTest", fixture)
            run_java("LimelightReplay", fixture, "", "")
            with (Path(fixture) / "targeting.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            assert [row["status"] for row in rows] == [
                "candidate", "no_target", "no_target", "no_target", "read_error"]
            assert abs(float(rows[0]["distance_m"]) - math.sqrt(4.25)) < .001
            assert abs(float(rows[0]["distance_error_m"])) < .001
            print("PASS: recorded-session replay, filtering, staleness and distance error")


if __name__ == "__main__":
    main()
