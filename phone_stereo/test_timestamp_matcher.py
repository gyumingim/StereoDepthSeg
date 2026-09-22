"""Exercise the actual Android-independent matcher with reordered callbacks."""
import subprocess
import tempfile
from pathlib import Path
import unittest

class TimestampMatcherTests(unittest.TestCase):
    def test_java_matcher(self):
        root = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory() as build:
            subprocess.run(["javac", "-d", build,
                str(root / "android/src/com/camera/dualstream/CaptureTimestampMatcher.java"),
                str(root / "tests/CaptureTimestampMatcherTest.java")], check=True)
            subprocess.run(["java", "-cp", build,
                "com.camera.dualstream.CaptureTimestampMatcherTest"], check=True)
