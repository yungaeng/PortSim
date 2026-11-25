# Hyundai Rotem reference exterior

The AGV exterior follows the three user-supplied blue Hyundai Rotem / Dongwon
terminal photographs. It is a reference-based reconstruction, not manufacturer
CAD or a change to the simulated equipment specification.

## Preserved simulation interfaces

- Two axles / four wheels matching the reference arrangement. The middle axle
  and its fender are removed, and the two service cabinets span the centre gap.
  Front/rear pivots remain at local Y = +/-480 cm, X = +/-170 cm, Z = 75 cm.
  The 1.4 m tire diameter and front/rear wheel spin/steering are retained.
- Original hidden `AGV_Deck`: position `(0,0,170)` cm, size `(300,1380,100)` cm,
  collision profile and 220 cm top surface.
- `DeckPosition` / `CargoPosition` and the 349.5 cm container centre, including
  moving cargo offsets, docking, attachment and emergency load handling.
- Original sensor components and their positions; their placeholder boxes are
  hidden. Payload and logistics specifications are unchanged.

The physical solver uses four active contacts with a 50/50 centred front/rear
load split, modified by CoG and pitch reactions. All four tires contribute force,
braking and odometry. Telemetry reports rear-left/right and front-left/right.
Per-wheel inertia and torque limits remain simulation assumptions; changing to
four wheels changes aggregate torque capacity and therefore measured performance.
This is not an identified Hyundai Rotem drivetrain. The shared reach-stacker
solver explicitly retains its three-axle/six-contact setting.

The body and wheel meshes are collision-free visual components. Their
`PortSim.AuthoredExterior` tag excludes them from the terminal's generic material
palette override. The body combines blue railings, arched fenders, battery/service
doors, latches, ventilation, lamps, LiDAR housing, hazard bumpers and lettering in
one shared mesh. Four text components display each actual vehicle ID. Tire and
hub assets use the original cylinder envelope and local Z axle.

## Sources and regeneration

Editable `.blend` and exported `.fbx` files live in
`PortSim/Content/PortSim/Assets/SourceFiles/RotemAGV/`; imported meshes and dedicated
materials live in `/Game/PortSim/Assets/Models/RotemAGV/`.

Run from the repository root (close the project editor before reimporting):

```powershell
& 'C:/Program Files/Blender Foundation/Blender 5.3/blender.exe' --background --python "$PWD/PortSim/Scripts/create_rotem_agv.py"
& 'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe' "$PWD/PortSim/PortSim.uproject" '-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities' -run=pythonscript "-script=$PWD/PortSim/Scripts/import_rotem_agv.py" -unattended -nullrhi -nosound
```

The importer reuses existing dedicated materials and validates mesh envelopes.
`preview_rotem_agv.py` creates a temporary editor studio, checks deck/wheel and
collision invariants, and captures `Saved/Screenshots/RotemAGV_Studio.png`.

## Verification

- UE 5.6 Development Editor build.
- `TestAGVDynamics.ps1`: four-contact load/moment conservation, visual wheel
  mapping, telemetry, shared six-contact reach-stacker compatibility, physics
  and integrated emergency benchmarks.
- `PortSim.Recovery.LoadedAGV`: reach-stacker recovery regression.
- `TestEquipment.ps1`: fleet layout, bounds and camera checks for 60 AGVs.
- `TestPickup.ps1 -Crane RMG -Case agv`: misaligned AGV safety/recovery case.
- `TestPickup.ps1 -Crane RMG -Case offset`: alignment correction, trial lift and
  final container placement.
- Editor studio invariants: four tire/hub pairs, unchanged deck transform
  and collision, and collision-free authored exterior.
- Rendered loaded motion with `-PortSimAGVProof -PortSimPlayback=4 -benchmark
  -fps=30`: attached cargo moved over 15 m and terminal validation passed.
  The initial variable-frame run moved the cargo but its global validation
  reported a 0.5116 cm secured ship-cargo position mismatch; fixed-step rerun passed.
