; Program: V21 five-axis capture -- apply commissioned 17D scan calibration first
; SyncMode: Hardware
; Not a universal safe path. Check workpiece/laser-head clearance before use.
N10 M0
N20 G21 G91 G94
N30 M62 P"./scans" Q"fiveaxis" T"scan_zlib"
N40 M61
N50 M50
N60 G1 X0.5 C0.5 B-0.25 F15 S1
N70 G1 X0.5 C0.5 B-0.25 F15 S1
N80 G1 X-1 C-1 B0.5 F15 S1
N90 M51
N100 M60
N110 M63
N120 M30
