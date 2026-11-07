; V22 RTCP orientation-hold example: sensor controlled point stays at local XYZ=0
; while C/B rotate; XYZ linear axes are automatically compensated by physical17 inverse kinematics.
G90 G94
G43.4
G92 X0 Y0 Z0 C0 B0
G01 X0 Y0 Z0 C20 B15 F120
G01 X0 Y0 Z0 C-15 B-10 F120
G01 X0 Y0 Z0 C0 B0 F120
G49
M30
