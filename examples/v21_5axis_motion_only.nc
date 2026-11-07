; Program: V21 XYZCB relative motion example -- NOT a universal safe path
; Confirm clearance, homing, mechanical limits and C/B absolute references first.
; S0 disables capture; F uses G94 (mm/min, pure rotary deg/min).
N10 M0
N20 G21 G91 G94
N30 G1 X0.5 C0.5 B-0.25 F30 S0
N40 G1 X0.5 C0.5 B-0.25 F30 S0
N50 G1 X-1 C-1 B0.5 F30 S0
N60 M30
