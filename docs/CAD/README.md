# Mars Rover CAD Rules & Glossary

## 1. Core Rules
- **Minimal Nesting:** Minimize the number of folders to maintain a flat, manageable hierarchy.
- **No Duplicates:** Always use `[Link]` ("Insert into Current Design") for existing parts. Never copy-paste files.
- **One Truth:** `MR_00` is the main assembly. Everything else is a sub-assembly.
- **Sub-assemblies naming:** All sub-assembly files MUST end with `_00` (e.g., `MR_ARM_SHR_00`).
- **Shared Modules (SHR):** If multiple end-effectors use the same core parts, keep them in `MR_ARM_SHR_00`. Change it once, update everywhere.
- **COTs (Standard Parts):** ALL purchased hardware must live in the `MR_COT_00` folder.

## 2. Naming Convention
**Format:** `PROJECT_SYSTEM_SUBSYSTEM_INDEX` (e.g., `MR_ARM_BDY_01`)

## 3. Glossary of Abbreviations

### Main Systems
- **MR** = Mars Rover (Project Prefix)
- **FRM** = Frame (Chassis)
- **SUS** = Suspension
- **ARM** = Robotic Arm
- **COT** = Commercial Off-The-Shelf (Purchased/Standard parts)

### Arm Sub-systems
- **ATT** = Attachment (Connection to Rover)
- **BAS** = Base (Rotating Shoulder)
- **BDY** = Body (Main Arm Links)
- **SHR** = Shared (Shared End-Effector Core)
- **JAW** = Jaw (Pinch Gripper)
- **SMP** = Sampler (Clamshell Scoop / Drill)
- **PMP** = Pump (Liquid Handling)

### Frame Sub-systems
- **FME** = Frame Core (Actuall Frame From Aluminium Profiles)
- **BBX** = Brain Box (Box With Jetson)
- **PBX** = Power Box (Box With Voltage Converters)
- **EBX** = Electronics Box (Box With Other Frame Electronics)
- **TOP** = Top Cover (Top Plate Of Frame With External Electronics)
- **TFT** = Time Of Flight Tower (Tower With TOF Sensors)

### COTs Categories (Hardware)
- **MTR** = Motor / Actuator
- **BRG** = Bearing
- **CTR** = Controller (Microcontrollers, Raspberry Pi)
- **TUB** = Tube (Aluminum profiles/tubes)
- **SNS** = Sensor (Load cells, pH, rain sensors)
- **OTR** = Other (Clamps, Screws, Miscellaneous hardware)

## 4. Adding a New Part
1. Determine its type: Unique Custom, Shared Custom (SHR), or Purchased (COT).
2. Save it in the correct folder with the next available index (e.g., `_06`).
3. Name it in English.
4. Immediately update the `structure.md` file!