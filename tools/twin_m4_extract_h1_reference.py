#!/usr/bin/env python3
"""Extract a small RAW, lossless-indexed regression fixture from H1 SCAN v3.

This is a TEST-ONLY parser for this specific Archive v5 fixture. Production
loading continues to use CompactScanIO::load with all existing safety checks.
Usage: python tools/twin_m4_extract_h1_reference.py H1.scan \
   tests/twin/data/h1_pose_subset.csv tests/twin/data/h1_real_100_profiles.bin
"""
import csv
import hashlib
import pathlib
import struct
import sys
import zlib


def extract(scan_path: pathlib.Path, pose_path: pathlib.Path, output: pathlib.Path):
    required = set()
    with pose_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(line for line in stream if line.strip() and not line.startswith("#")):
            required.add(int(row[0]))
    whole = scan_path.read_bytes()
    head = whole[:256]
    if len(head) != 256 or head[:4] != b"SCAN" or struct.unpack_from("<I", head, 4)[0] != 3:
        raise ValueError("Need SCAN v3 file")
    if zlib.crc32(head[:252]) != struct.unpack_from("<I", head, 252)[0]:
        raise ValueError("Outer SCAN header CRC mismatch")
    codec = head[112]
    plain_size, compressed_size = struct.unpack_from("<QQ", head, 116)
    compressed = whole[256:]
    if codec != 3 or len(compressed) != compressed_size or len(compressed) < 4:
        raise ValueError("Expected H1 QtZlib scan")
    if int.from_bytes(compressed[:4], "big") != plain_size:
        raise ValueError("Invalid qCompress uncompressed size")
    raw = zlib.decompress(compressed[4:])
    if len(raw) != plain_size or zlib.crc32(raw[:-4]) != struct.unpack_from("<I", raw, len(raw)-4)[0]:
        raise ValueError("Internal Archive CRC mismatch")
    if raw[:4] != b"A5D4" or struct.unpack_from("<I", raw, 4)[0] != 5:
        raise ValueError("Expected Archive v5")
    profile_count = struct.unpack_from("<Q", raw, 8)[0]
    cursor = 16
    selected = []
    for _ in range(profile_count):
        ordinal = struct.unpack_from("<Q", raw, cursor+8)[0]
        n = struct.unpack_from("<I", raw, cursor+56)[0]
        if n > 1280 or cursor+492+n*20 > len(raw)-4:
            raise ValueError("Invalid Archive v5 profile length")
        if ordinal in required:
            selected.append((ordinal, n, raw[cursor+492:cursor+492+n*20]))
        cursor += 492+n*20
    if cursor != len(raw)-4 or len(selected) != len(required):
        raise ValueError("Archive tail or requested ordinal mismatch")
    with output.open("wb") as stream:
        stream.write(b"H1M4RAW1")
        stream.write(struct.pack("<I", len(selected)))
        for ordinal, n, raw_points in selected:
            stream.write(struct.pack("<QI", ordinal, n))
            stream.write(raw_points)
    print("extracted",len(selected),"profiles,",sum(p[1] for p in selected),"raw indexed points")
    print("sha256",hashlib.sha256(output.read_bytes()).hexdigest())


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit("Usage: script.py input.scan pose_subset.csv output.bin")
    extract(*(pathlib.Path(s) for s in sys.argv[1:]))
