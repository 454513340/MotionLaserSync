#!/usr/bin/env python3
"""Offline, stdlib-only SCAN v3 structural/CRC smoke check.
This is NOT a substitute for CompactScanIO's calibrated full-frame validation.
Usage: python tools/check_scan_v3_reference.py /path/to/6061_Alumina_Scan_H1.scan
"""
import sys, pathlib, struct, zlib

def check(path):
    blob=pathlib.Path(path).read_bytes()
    if len(blob)<276: raise ValueError('truncated SCAN')
    h,body=blob[:256],blob[256:]
    if h[:4]!=b'SCAN':raise ValueError('bad header magic')
    version, count=struct.unpack_from('<IQ',h,4)
    codec=h[112]
    unc,comp=struct.unpack_from('<QQ',h,116)
    if version!=3:raise ValueError('only SCAN v3 supported')
    if zlib.crc32(h[:252])!=struct.unpack_from('<I',h,252)[0]:raise ValueError('header CRC failed')
    if codec==3:
        if comp!=len(body):raise ValueError('compressed length mismatch')
        if int.from_bytes(body[:4],'big')!=unc:raise ValueError('QtZlib size mismatch')
        raw=zlib.decompress(body[4:])
    elif codec==0:
        raw=body
    else:raise ValueError('unsupported codec')
    if len(raw)!=unc or zlib.crc32(raw[:-4])!=struct.unpack_from('<I',raw,len(raw)-4)[0]:
        raise ValueError('payload size / CRC failed')
    magic,payload_version,payload_count=struct.unpack_from('<IIQ',raw,0)
    if magic!=0x34443541 or payload_version not in (4,5) or payload_count!=count:
        raise ValueError('archive header mismatch')
    offset=16
    points=0
    last_ordinal=-1
    session=None
    first_sensor=last_sensor=None
    for _ in range(count):
        # 492 bytes for archive v5, 468 for v4; profileIndex at fixed offset +32.
        fixed=492 if payload_version>=5 else 468
        if offset+fixed>len(raw)-4:raise ValueError('truncated frame')
        s,ordinal=struct.unpack_from('<QQ',raw,offset)
        n=struct.unpack_from('<I',raw,offset+56)[0]
        sensor_time=struct.unpack_from('<d',raw,offset+60)[0]
        if session is None:session=s;first_sensor=sensor_time
        if s!=session or ordinal<=last_ordinal:raise ValueError('nonmonotone or cross-session ordinal')
        last_ordinal=ordinal
        offset+=fixed+20*n
        if offset>len(raw)-4:raise ValueError('truncated sensor points')
        points+=n
        last_sensor=sensor_time
    if offset!=len(raw)-4:raise ValueError('trailing archive bytes')
    print('SCAN v3 integrity: OK')
    print('profiles:',count,'valid raw sample points:',points)
    print('first / last ordinal:',last_ordinal-count+1,last_ordinal)
    print('sensor time span (s):',round(last_sensor-first_sensor,6))
    return count,points

if __name__=='__main__':
    if len(sys.argv)!=2:
        print('Usage: python check_scan_v3_reference.py FILE.scan',file=sys.stderr)
        sys.exit(2)
    try:check(sys.argv[1])
    except Exception as exc:print(f'FAILED: {exc}',file=sys.stderr);sys.exit(1)
