; V21 XYZCB incremental template. XYZ=mm; C/B=deg; G94 F=mm/min.
; DEMONSTRATION ONLY: verify actual start, swept volume and axis limits first.
; No universal safe Z is assumed. Explicit F15 overrides the UI default.
M0 L"Verify clearance, rotary references, limits and the applied 17D scan model"
G21 G91 G94
M62 P"@SAVE_DIR@" Q"@SAVE_PREFIX@" T"scan_zlib"
M61
M50
G1 X1 C1 B0.5 F15 S1
G1 Y0.2 F15 S1
G1 X-1 C-1 B-0.5 F15 S1
G1 Y0.2 F15 S1
G1 X1 C1 B0.5 F15 S1
G1 X-1 Y-0.4 C-1 B-0.5 F15 S1
M51
M60
M63
M30
