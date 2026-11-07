; Program: M6 offline five-axis trajectory - NOT FOR EXECUTION
; OfflineOnly: true
; SimulationVerified: false
; SyncMode: Hardware
; HardwareTopology: H1
; HardwareFrequencyHz: 200
; ProgramFrame: MACHINE_JOINTS_XYZCB
; CoordinateOrder: X Y Z C B
; LinearUnit: mm
; RotaryUnit: deg
; PartPoseId: UNVERIFIED
; CalibrationId: UNVERIFIED
; SourceCad: 
; KinematicsSource: 
; RegistrationVerified: false
; InitialJointAnchored: true
; Warning: First approach to the simulated path is not certified; no hardware start state has been verified
; Warning: M6 timestamps become nominal G93 segment times; controller jerk/accel planning may alter exposure positions
; Warning: 2950 multi-angle optics / full housing and fixture collisions are NOT certified
G21
G90
G93
; Initial machine state must be inspected before any real movement
G00 X0 Y0 Z0 C0 B0
; M6 TRANSFER — offline sampled XYZCB polyline
G05 S0 F60
 X1 Y0 Z0 C0 B0 F60
; end M6 G05 segment
M50
; M6 SCAN — offline sampled XYZCB polyline
G05 S1 F3000
 X1.1 Y0 Z0 C2 B1 F3000
 X1.2 Y0 Z0 C4 B2 F3000
; end M6 G05 segment
M51
M60 L"simulated_strip"
M61
; M6 TRANSFER — offline sampled XYZCB polyline
G05 S0 F60
 X0 Y0 Z0 C0 B0 F60
; end M6 G05 segment
M63
M30
