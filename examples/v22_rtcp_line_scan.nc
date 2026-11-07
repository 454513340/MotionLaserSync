; V22 calibrated scanner RTCP example.
; G43.4 XYZ means controlled-point machine/task coordinates, C/B remain the two rotary DOF.
; Because the single-unknown-sphere calibration does not fix absolute translation gauge,
; this example establishes a LOCAL task frame at the current physical pose with G92.
; Before real scanning: apply the same 17-parameter model to Scan + RTCP and verify OUT14->IN0/laser wiring.
G90 G94 G17
G43.4
G92 X0 Y0 Z0 C0 B0
M50
G01 X20 Y0 Z0 C5 B-3 F600 S1
G01 X40 Y10 Z-2 C12 B-8 F600 S1
G01 X60 Y10 Z-2 C18 B-12 F600 S1
M51
G49
M30
