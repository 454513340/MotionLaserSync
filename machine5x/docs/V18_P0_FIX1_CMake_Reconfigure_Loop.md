# V18-P0 FIX1 — CMake/Ninja repeated reconfigure fix

This package contains the same V18-P0 runtime code. The archive file timestamps are normalized to a historical date so that Windows/Ninja does not treat `CMakeLists.txt` or sources as files from the future.

Symptom fixed:

```text
[0/1] Re-running CMake...
[0/2] Re-running CMake...
[0/3] Re-running CMake...
...
```

After extracting this package, delete the old `out/build` directory and let Visual Studio/CMake configure a fresh build tree.

The only text-level CMake change besides packaging metadata is removal of one duplicate `include/core/ProfileSessionEpoch.h` list entry; it had no runtime effect.
