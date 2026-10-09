# PLC project: ctrlX CORE

Structured Text for a Bosch Rexroth ctrlX CORE (ctrlX PLC Engineering, CODESYS-based).

## Use the `codesys-st` skill

Load the **`codesys-st`** skill (`~/.claude/skills/codesys-st/`) before writing or reviewing any PLC code. It holds:
- the house rules: the sequence structure, no rising edges, never reading your own outputs, naming conventions;
- the ctrlX library references.

Follow it exactly. The rules in it came from review of real code.

## Motion: use the Motion App commands (`ML_` functions)

For all axis and kinematics motion in this project, use the **Motion App PLC functions (`ML_...`, library `CXA_Motion`)**.

- **Don't use** `CXA_PLCopen` (`MC_*` / `MB_*` function blocks).
- **Don't use** `CXA_MotionInterface` (the AxisInterface / KinematicInterface template).

This overrides the skill's default of PLCopen FBs.

Where to look things up:
- **Command and function reference:** `~/.claude/skills/codesys-st/references/motion/INDEX.md`. Find the command there, then read only that section of the `07-*.md` files, including its "PLC:" part.
- **Behaviour, buffering, states and errors:** `~/.claude/skills/codesys-st/references/motion-essentials.md`.

Remember that `ML_` calls are **one-shot functions, not FBs**:
- Issue a command **once**, in a sequence's on-change (entry) section.
- Keep the returned `CmdID`, and track the command with `ML_GetCmdState` in the cyclic section.
- Never call a motion command every scan.

Check every struct and field name against the reference before using it. Don't guess.

## Design docs

The design is in **`design/`**. Start with `design/README.md`, which has the index, the implementation order, the house rules and the open decisions.

- Implement what the docs say. If code needs a decision the docs don't make, ask, and don't invent one.
- The design docs are library-agnostic on purpose. Library choices (such as the `ML_` functions above) belong in the code and here, not in the docs.
- When a design decision changes, update the doc as well as the code. Mark answered questions **Decided** in place, without renumbering.
