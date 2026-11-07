#!/usr/bin/env python3
"""Read new .sxraw per-Profile sensor-native X/Z binary archives.

Usage:
  python tools/inspect_sphere_exposure_raw.py EXP_0100_R01_PLUS.sxraw [output.csv]

SXP1 v1 / little endian layout:
  magic[4], uint32 version, int32 exposure/rate/repeat/direction,
  float32 commanded X feed mm/s;
  records: uint32 sensorProfileIndex, int64 callbackPcMicroseconds,
  uint32 N, N pairs (float32 sensorXmm, float32 sensorZmm).
"""
import csv, pathlib, struct, sys


def read_scan(path):
    with open(path, 'rb') as f:
        if f.read(4) != b'SXP1':
            raise ValueError('not a SphereExposureSweep .sxraw file')
        header = f.read(24)
        if len(header) != 24:
            raise ValueError('truncated file header')
        version, exposure, rate, repeat, direction, speed = struct.unpack('<Iiiiif', header)
        if version != 1:
            raise ValueError(f'unsupported version {version}')
        yield {'header': {'exposure10us': exposure, 'rateHz': rate,
                         'repeat': repeat, 'direction': direction, 'speedMmPerSec': speed}}
        while True:
            header = f.read(16)
            if not header:
                break
            if len(header) != 16:
                raise ValueError('truncated Profile header')
            profile_index, pc_us, n = struct.unpack('<IqI', header)
            if n > 100000:
                raise ValueError('implausible Profile point count')
            buf = f.read(n * 8)
            if len(buf) != n * 8:
                raise ValueError('truncated Profile points')
            yield {'profileIndex': profile_index, 'pcUs': pc_us,
                   'points': list(struct.iter_unpack('<ff', buf))}


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    path = pathlib.Path(sys.argv[1])
    dest = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else path.with_suffix('.xz.csv')
    frame_count = point_count = 0
    with open(dest, 'w', encoding='utf-8-sig', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['ProfileIndex', 'PcUs', 'PointIndex', 'SensorXmm', 'SensorZmm'])
        for frame in read_scan(path):
            if 'header' in frame:
                print('Metadata:', frame['header'])
                continue
            frame_count += 1
            for i, (x, z) in enumerate(frame['points']):
                writer.writerow([frame['profileIndex'], frame['pcUs'], i, x, z])
                point_count += 1
    print(f'Profiles={frame_count}; points={point_count}; CSV={dest}')


if __name__ == '__main__':
    main()
