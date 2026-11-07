; Program: V21 custom G05 points and arc tests from assumed machine zero
; G05 is this project's point-list dialect, not a commercial CNC spline code.
N10 M0
N20 G21 G90 G94
N30 G0 X0 Y0 Z0 C0 B0 S0
N40 G5 F60 S0
X0.5 C0.5 B0.25
X1 C1 B0.5
N50 G17 G3 X0 Y1 Z0.2 C2 B1 I-1 J0 F30 S0
N60 G0 X0 Y0 Z1 C0 B0 S0
N70 G18 G3 X1 Z0 K-1 I0 F30 S0
N80 M30
