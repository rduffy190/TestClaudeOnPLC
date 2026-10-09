# Source

Structured Text for ctrlX PLC Engineering. **One file is one object in the device tree.** The comment markers `// ===== Declaration =====` and `// ===== Implementation =====` show which pane each part is pasted into.
- Methods and properties are child objects. Add them with *Add Object → Method / Property*, then paste the matching block.
- DUT and GVL files paste whole.

| Folder | Objects | Design doc |
|---|---|---|
| `State/` | `E_MachineState`, `ST_StateClientData`, `I_StateClient` (methods `Request`, `Report`), `FB_StateClient` (methods `Request`, `Report`, property `Data` with a Get only), `FB_StateRequest`, `FB_StateManager`, `GVL_State` | `design/Central_State_Management_Design.md` |
| `DriveHw/` | `ST_DriveFeedback`, `Fn_DriveFeedback`, `FB_DriveSupplyMgr` | `design/Unit_DriveHardwareStation_Design.md` |
| `Servo/` | `ST_AxisCyclicStatus`, `GVL_Axes`, `GVL_AxisRtMap`, `ST_AxisRtOffsets`, `ST_AxisRtConfig`, `Fn_RtCopy`, `FB_AxisRtMap`, `FB_AxisRtRead`, `E_ServoSeqState`, `FB_AxisCmdState`, `FB_ServoAxisMgr`, `FB_ServoMotionSeq` | `design/Unit_ServoStation_Design.md` |
| `Main/` | `GVL_IO`, `GVL_Hmi`, `PRG_Main` | |

**Paste in this order** (dependencies first): `E_MachineState` → `ST_StateClientData` → `I_StateClient` → `FB_StateClient` → `FB_StateRequest` → `FB_StateManager` → `GVL_State` → `ST_DriveFeedback` → `Fn_DriveFeedback` → `FB_DriveSupplyMgr` → `ST_AxisCyclicStatus` → `GVL_Axes` → `GVL_AxisRtMap` → `ST_AxisRtOffsets` → `ST_AxisRtConfig` → `Fn_RtCopy` → `FB_AxisRtMap` → `FB_AxisRtRead` → `E_ServoSeqState` → `FB_AxisCmdState` → `FB_ServoAxisMgr` → `FB_ServoMotionSeq` → `GVL_IO`, `GVL_Hmi` → `PRG_Main`.

**Task:** call `PRG_Main` from the cyclic PLC task.

**I/O mapping:** map `GVL_IO` to the SoE cyclic data:
- supply unit: status word S-0-2058 in the AT, control word S-0-2057 in the MDT. The supply unit must be configured for "external activation via field bus".
- drives 0 (Ax1, hosts the supply) and 1 (Ax2): status word in the AT, DC bus power control S-0-0240 in the MDT;
- drive 2 is a spare and isn't used. Only its status word is mapped, for diagnostics.

There's no separate fault reset output. On entry to Resetting the servo axis manager resets both axes, and the reset of drive 0 also resets the supply.

**Libraries:** `Standard` (TON), **`CXA_DriveUtil`** (`MB_PowerSupplyDriveIntegrated`), **`CXA_Motion`** (`ML_` functions, see `../CLAUDE.md`) **`CXA_Datalayer`** (`DL_MemUser`, `DL_NODEVALUE`) and **`CXA_AutomationCore_fbs`** (`comm_datalayer_MemoryMap`, `comm_datalayer_Variable`) for the RT axis status.

**Motion setup:**
- set `GVL_Axes.asAxisName` to the ctrlX MOTION axis names;
- enable the cyclic status data for each axis (`motion/axs/<name>/cfg/functions/cyclic-status-data/remanent`). There's no I/O mapping for it. At the top of `PRG_Main`, `FB_AxisRtMap` opens `motion/realtime/cyclicStatusData`, reads its memory map and fills `stAxisRtConfig` with each axis's field offsets and the map revision. Then one `FB_AxisRtRead` per axis copies that axis into `GVL_Axes.astStatus[i]`. Axes are matched by `GVL_Axes.asAxisName` and fields by `GVL_AxisRtMap.asFieldName`;
- diagnostics: `fbAxisRtMap.bError` / `uiErrorStep` / `eResult` (open, get map, or a reader failed; it retries every 1 s and rereads the map, so a reconfigured map recovers by itself), `fbAxisRtMap.bAllAxesFound`, `afbAxisRtRead[i].bNotInMap` (axis name wrong or cyclic status not enabled; `stAxisRtConfig.astAxis[i].abFound` shows which fields are missing). An axis that isn't read has `DataValid = FALSE`;
- set the dynamics constants in `FB_ServoMotionSeq` (`POS_*`, `VM_*`, `ABORT_DEC`), which are placeholders, and `SPEED_DEADBAND`.

## Status

| Part | State |
|---|---|
| Central state management | Written. Not compiled yet. |
| Drive hardware station | Written. Not compiled yet. |
| Servo station | Written. Not compiled. Uses the RT cyclic axis status instead of `ML_AxsGetPLCOpenState`. |
| RT axis status (`FB_AxisRtMap`, `FB_AxisRtRead`) | Written. Not compiled. The map API follows the user's EtherCAT map example (now in the `codesys-st` skill: `references/datalayer-rt/`). The name parsing and offsets were checked against the live map over REST (Ax1 at byte 0, Ax2 at 59, VM at 118). Not run on the PLC yet. |

The logic of the manager and the drive station was checked with a scan-by-scan model: turn on → start → drive fault → abort → reset with the fault still present → recover → Idle. It hasn't been run on a controller.

## Importing instead of pasting

`python3 tools/build_import.py` builds `build/PlcProject_Import.xml` and **`build/manual/`**, which holds the objects to create by hand (split into paste-ready files per pane) and a checklist, `build/manual/README.md`, with every manual step in order: libraries, hand-made objects, import, post-import checks (from the converter warnings) and project setup. Follow that checklist. A summary:

1. **Libraries:** add `CXA_DriveUtil`, `CXA_Motion`, `CXA_Datalayer` and `CXA_AutomationCore_fbs` in the Library Manager (`Standard` is already there).
2. **Create by hand** (the converter doesn't handle interfaces, methods or properties):
   - `I_StateClient`: *Add Object → Interface*, then add the methods `Request` and `Report` and paste their declarations from `src/State/I_StateClient.st`.
   - `FB_StateClient`: *Add Object → POU*, a function block that implements `I_StateClient`. The IDE generates the method stubs; paste the declarations and bodies from `src/State/FB_StateClient.st`. Then add the property `Data` with a Get accessor only (delete the Set).
3. **Import:** *Project → Import PLCopenXML…* → `build/PlcProject_Import.xml`, and select all objects.
4. **Check after import.** These are kept only in the plain-text declarations, so make sure they came through:
   - `{attribute 'qualified_only'}` / `{attribute 'strict'}` on `E_MachineState` and `E_ServoSeqState`;
   - the `VAR_GLOBAL CONSTANT` blocks in `GVL_State`, `GVL_IO`, `GVL_Axes` and `GVL_AxisRtMap`;
   - the `ARRAY[*]` inputs of `FB_StateManager` and `FB_DriveSupplyMgr`;
   - the initial values of `GVL_Axes.asAxisName` and `GVL_AxisRtMap.asFieldName`;
   - `FB_AxisRtRead.stConfig` is in a `VAR_IN_OUT CONSTANT` block.
5. **Build** (F11).
