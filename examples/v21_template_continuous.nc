; V21 XYZCB incremental template. XYZ=mm; C/B=deg; G94 F=mm/min.
; DEMONSTRATION ONLY: verify actual start, swept volume and axis limits first.
; No universal safe Z is assumed. Explicit F15 overrides the UI default.
M0 L"Verify clearance, rotary references, limits and the applied 17D scan model"
G21 G91 G94
M62 P"@SAVE_DIR@" Q"@SAVE_PREFIX@" T"scan_zlib"
M61
; G5 is this project's joint-point-list extension, not NURBS/RTCP.
; First three segments are collinear; the reversal stops at the junction.
M50
G5 F15 S1 L"FiveAxisJointPath"
X0.2 Y0 Z0 C0.2 B0.1
X0.2 Y0 Z0 C0.2 B0.1
X0.2 Y0 Z0 C0.2 B0.1
X-0.6 Y0 Z0 C-0.6 B-0.3
M51
M60
M63
M30
