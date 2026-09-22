#!/usr/bin/env python3
"""Build a dependency-free Camera2 APK using Google's SDK platform/build-tools 35.

SDK paths are workspace-local (tools/android-sdk); no Gradle or global SDK changes.
"""
import hashlib
import os
from pathlib import Path
import subprocess
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT / "tools/android-sdk"
ANDROID = ROOT / "phone_stereo/android"


def run(*args):
    subprocess.run([str(a) for a in args], check=True)


def main():
    SDK.mkdir(parents=True, exist_ok=True)
    for name, sha, expected in [
        ("platform-35_r02.zip", "0bb560a90a7a2cbd0dd8348224d518b638fe7949", "android-35/android.jar"),
        ("build-tools_r35_linux.zip", "2cfaa0bbb2336e9ec18ed3ecea84fa2e2af607bc", "android-15/d8"),
    ]:
        if (SDK / expected).exists():
            continue
        archive = SDK / name
        if not archive.exists():
            urllib.request.urlretrieve("https://dl.google.com/android/repository/" + name, archive)
        if hashlib.sha1(archive.read_bytes()).hexdigest() != sha:
            raise RuntimeError(f"SDK checksum mismatch: {archive}")
        with zipfile.ZipFile(archive) as z:
            z.extractall(SDK)
            for info in z.infolist():
                mode = info.external_attr >> 16
                if mode:
                    (SDK / info.filename).chmod(mode)
    jar = SDK / "android-35/android.jar"
    bt = SDK / "android-15"
    build = ANDROID / "build"
    classes = build / "classes"
    classes.mkdir(parents=True, exist_ok=True)
    for old in classes.rglob("*.class"):
        old.unlink()
    run("javac", "-source", "8", "-target", "8", "-classpath", jar, "-d", classes,
        *sorted((ANDROID / "src").rglob("*.java")))
    run(bt / "d8", "--lib", jar, "--min-api", "28", "--output", build, *sorted(classes.rglob("*.class")))
    unsigned = build / "unsigned.apk"
    run(bt / "aapt", "package", "-f", "-M", ANDROID / "AndroidManifest.xml", "-I", jar, "-F", unsigned)
    with zipfile.ZipFile(unsigned, "a", zipfile.ZIP_DEFLATED) as z:
        z.write(build / "classes.dex", "classes.dex")
    run(bt / "zipalign", "-f", "4", unsigned, build / "aligned.apk")
    key = build / "debug.keystore"
    if not key.exists():
        run("keytool", "-genkeypair", "-keystore", key, "-storepass", "android", "-keypass", "android",
            "-alias", "debug", "-keyalg", "RSA", "-validity", "3650", "-dname", "CN=PhoneStereo Development")
    apk = build / "phone-stereo.apk"
    run(bt / "apksigner", "sign", "--ks", key, "--ks-pass", "pass:android", "--out", apk, build / "aligned.apk")
    run(bt / "apksigner", "verify", apk)
    print(apk)


if __name__ == "__main__":
    main()
