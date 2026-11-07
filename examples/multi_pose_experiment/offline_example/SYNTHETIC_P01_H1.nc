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
; ExperimentStudyId: SYNTHETIC_TEST_DO_NOT_EXECUTE
; RegistrationVerified: false
; InitialJointAnchored: false
; Warning: First approach to the simulated path is not certified; no hardware start state has been verified
; Warning: M6 timestamps become nominal G93 segment times; controller jerk/accel planning may alter exposure positions
; Warning: 2950 multi-angle optics / full housing and fixture collisions are NOT certified
G21
G90
G93
; Initial machine state must be inspected before any real movement
G00 X0 Y-10 Z-85 C0 B0
; M6 TRANSFER — offline sampled XYZCB polyline
G05 S0 F1200
 X0 Y-10 Z-86 C0 B0 F1200
 X0 Y-10 Z-87 C0 B0 F1200
 X0 Y-10 Z-88 C0 B0 F1200
 X0 Y-10 Z-89 C0 B0 F1200
 X0 Y-10 Z-90 C0 B0 F1200
 X0 Y-10 Z-91 C0 B0 F1200
 X0 Y-10 Z-92 C0 B0 F1200
 X0 Y-10 Z-93 C0 B0 F1200
 X0 Y-10 Z-94 C0 B0 F1200
 X0 Y-10 Z-95 C0 B0 F1200
; end M6 G05 segment
M50
; M6 SCAN — offline sampled XYZCB polyline
G05 S1 F1200
 X0 Y-9 Z-95 C0 B0 F1200
 X0 Y-8 Z-95 C0 B0 F1200
 X0 Y-7 Z-95 C0 B0 F1200
 X0 Y-6 Z-95 C0 B0 F1200
 X0 Y-5 Z-95 C0 B0 F1200
 X0 Y-4 Z-95 C0 B0 F1200
 X0 Y-3 Z-95 C0 B0 F1200
 X0 Y-2 Z-95 C0 B0 F1200
 X0 Y-1 Z-95 C0 B0 F1200
 X0 Y0 Z-95 C0 B0 F1200
 X0 Y1 Z-95 C0 B0 F1200
 X0 Y2 Z-95 C0 B0 F1200
 X0 Y3 Z-95 C0 B0 F1200
 X0 Y4 Z-95 C0 B0 F1200
 X0 Y5 Z-95 C0 B0 F1200
 X0 Y6 Z-95 C0 B0 F1200
 X0 Y7 Z-95 C0 B0 F1200
 X0 Y8 Z-95 C0 B0 F1200
 X0 Y9 Z-95 C0 B0 F1200
 X0 Y10 Z-95 C0 B0 F1200
; end M6 G05 segment
M51
M60 L"MP_P01_1"
M61
; M6 TRANSFER — offline sampled XYZCB polyline
G05 S0 F1200
 X0 Y10 Z-94 C0 B0 F1200
 X0 Y10 Z-93 C0 B0 F1200
 X0 Y10 Z-92 C0 B0 F1200
 X0 Y10 Z-91 C0 B0 F1200
 X0 Y10 Z-90 C0 B0 F1200
 X0 Y10 Z-89 C0 B0 F1200
 X0 Y10 Z-88 C0 B0 F1200
 X0 Y10 Z-87 C0 B0 F1200
 X0 Y10 Z-86 C0 B0 F1200
 X0 Y10 Z-85 C0 B0 F1200
; end M6 G05 segment
M63
M30
