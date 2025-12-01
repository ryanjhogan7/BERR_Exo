- ERR Exoskeleton Hardware Technical Reference
System Overview
Purpose: Resistive training exoskeleton operating in regenerative braking mode where human arm input provides mechanical power (~120W) and motor creates opposing resistive torque.
Core Hardware Components
Motor System

Motor: ODrive D6374 Brushless DC Motor

Torque Constant: 0.055 Nm/A
KV Rating: 150 KV
Base Torque: 2.8 Nm


Gearbox: 10:1 Planetary Gearbox

Output Torque: 28 Nm continuous (exceeds 25 Nm target)
Reduction Ratio: 10:1


Controller: ODrive Pro

Encoder Resolution: 12-bit (4096 CPR)
Response Time: 15-25ms (target <100ms)
API: Updated format uses axis.config.motor.X (not older axis.motor.config.X)
Critical: Requires proper save/reboot cycle handling


Encoder: AMT212B-V-OD

Type: Absolute encoder
Essential for precision torque control



Power & Thermal Management

Power Supply: 24V, 5A

Powers controller electronics (NOT high-current motor power)
Motor operates as generator in braking mode


Regenerative Braking: 50W, 2Ω Regen Clamp Module

Dissipates ~20W at 60 RPM
Handles regenerated electrical power from braking



Mechanical Components

Bearings: Skateboard bearings (test jig)
Frame: 3D-printed structural components + metal elements
Test Jig: Two-part benchtop design for motor validation

Key Performance Specs
ParameterTargetCurrent StatusOutput Torque0-25 Nm28 Nm capableTorque Precision±1 NmTesting requiredResponse Time<100 ms15-25ms achievedRange of Motion>95% naturalDesign validated 0-140°System WeightTarget TBD3.225 kg (motor+gearbox+electronics)Operating Duration>2 hoursBattery dependent
Operating Principles

Regenerative Braking Mode: Motor acts as generator, not traditional motor
Power Flow: Human → Mechanical Power → Motor → Electrical Power (regenerated) → Regen Clamp
Back-EMF: Motor creates back-EMF; controller forces current through to create opposing torque
Low RPM Operation: ~60 RPM typical, emphasizes torque over speed

Software/Control

Language: Python
Scripts:

Setup configuration
Main torque tester with real-time feedback
Supporting documentation


Features: Interactive torque control, safety limits, real-time torque/current/velocity display

Critical Design Learnings

✅ Motor operates in REVERSE as resistive brake (not traditional motoring)
✅ Power supply is 24V, 5A for electronics (not high-current low-voltage)
✅ Encoder feedback essential for clinical-grade precision
✅ ODrive API requires axis.config.motor.X format (newer API)
✅ Configuration saves trigger automatic reboots (expected behavior, not error)

Key Information Sources
Technical Documentation

ODrive Robotics Documentation: https://docs.odriverobotics.com/

Motor control implementation
API reference (current format)
Configuration best practices