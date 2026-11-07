#!/usr/bin/env python3
"""Static GUI-source wiring regression; no Qt, VTK or vendor SDK needed.

This is not a Windows GUI build: it detects accidental omissions of the M4
translation units from the final MotionLaserSync target source list.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
s = (root / "CMakeLists.txt").read_text(encoding="utf-8")


def block(name):
    match = re.search(r"set\(" + re.escape(name) + r"\s+([\s\S]*?)\n\)", s)
    assert match, f"Missing CMake set({name})"
    return match.group(1)

core = block("TWIN_CORE_SOURCES")
m4 = block("TWIN_M4_SOURCES")
widgets = block("WIDGET_SOURCES")
required_m4 = (
    "src/twin/TwinM4Comparison.cpp",
    "src/twin/TwinM4Calibration.cpp",
)
for name in required_m4:
    assert name in m4, f"M4 list is missing {name}"
    assert name not in core, f"{name} is accidentally compiled into GUI core library"
    assert (root / name).is_file(), f"Missing implementation file: {name}"
assert "${TWIN_M4_SOURCES}" in widgets, "GUI is not using M4 sources"
assert "src/twin/TwinM4Adapter.cpp" in widgets, "GUI lacks Qt M4 bridge"
assert "target_sources(digital_twin_core PRIVATE ${TWIN_M4_SOURCES}" in s, "CLI M4 regression tests lost their sources"
assert "${WIDGET_SOURCES}" in s and "target_link_libraries(MotionLaserSync PRIVATE" in s, "GUI target linking missing"
assert 'M4 GUI missing implementation' in s, "Missing GUI CMake configure-time M4 source guard"
assert 'D:/dev/vcpkg/installed/x64-windows-release' in s, "OCCT prefix unexpectedly changed"
print("M4 GUI source wiring verified: Comparison, Calibration, Qt Adapter")
