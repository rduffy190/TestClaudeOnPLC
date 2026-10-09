// FUNCTION_BLOCK: FB_ServoAxisMgr
// Servo station axis manager: axis power on/off from the machine state, torque-free at the
// end of Aborting, fault detection -> abort request, axis reset on entry to Resetting.
// No sequence of its own: power is a level per axis, issued as an ML_AxsPower command once
// each time the wanted level changes.
// Design: design/Unit_ServoStation_Design.md §3
// Library: CXA_Motion

// ===== Declaration =====
FUNCTION_BLOCK FB_ServoAxisMgr
VAR_INPUT
    itfClient   : I_StateClient;                        // GVL_State.aClients[GVL_State.CLIENT_SERVO_AXES]
    eMachineState : E_MachineState;   // machine state to follow (from the caller: the broadcast, or a sub-manager's state)
    asAxisName  : ARRAY[0..GVL_Axes.LAST_AXIS] OF STRING(15);    // ctrlX MOTION axis names: Ax1, Ax2, VM
END_VAR
VAR_IN_OUT CONSTANT
    astStatus   : ARRAY[0..GVL_Axes.LAST_AXIS] OF ST_AxisCyclicStatus;   // cyclic axis status
END_VAR
VAR_OUTPUT
    bAllPowered : BOOL;     // diag: all axes powered
    bAllOff     : BOOL;     // diag: all axes torque-free
    bAllAtRest  : BOOL;     // diag: all axes at rest (debounced)
    bAnyError   : BOOL;     // diag: some axis has an error
    uiAbortCode : UINT;     // abort code currently requested, 0 = none
END_VAR
VAR
    _eMachineState      : E_MachineState;
    _eLastMachineState  : E_MachineState;
    _iAxis              : INT;

    _bAllPowered        : BOOL;
    _bAllOff            : BOOL;
    _bAllAtRestNow      : BOOL;     // undebounced
    _bAnyError          : BOOL;
    _abPowerReady       : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;     // motion reports the axis ready for power up
    _abPowerConfirmed   : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;     // axis reported PowerOn since power was commanded
    _abCmdRejected      : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;     // an ML_ command was not accepted; cleared on entry to Resetting

    _bFaultAbortActive  : BOOL;
    _bAbort             : BOOL;
    _uiAbortCode        : UINT;
    _bDone              : BOOL;

    _bWantPower         : BOOL;     // machine-level power want (before per-axis conditions)
    _abWantPower        : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;
    _abPowerSent        : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;     // last power level commanded per axis
    _astPower           : ARRAY[0..GVL_Axes.LAST_AXIS] OF ML_AxsPowerData;
    _astReset           : ARRAY[0..GVL_Axes.LAST_AXIS] OF ML_AxsResetData;

    _fbRestDebounce     : TON;
    _fbStateReq         : FB_StateRequest;
END_VAR
VAR CONSTANT
    REST_DEBOUNCE       : TIME := T#100MS;          // all axes must be at rest this long before going torque-free
    SOURCE_NAME         : STRING(50) := 'FB_ServoAxisMgr';
    ABORT_NONE          : UINT := 0;
    ABORT_AXIS_ERROR    : UINT := 200;  // + axis index
    ABORT_POWER_LOST    : UINT := 210;  // + axis index
    ABORT_AXIS_CMD      : UINT := 220;  // + axis index: power/reset command not accepted
    ABORT_AXIS_DATA     : UINT := 230;  // + axis index: cyclic status not valid
END_VAR

// ===== Implementation =====
_eMachineState := eMachineState;     // read once per scan

// ===== 1. Read status =====
_bAllPowered   := TRUE;
_bAllOff       := TRUE;
_bAllAtRestNow := TRUE;
_bAnyError     := FALSE;
FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
    // Readiness from the motion feedback, not the drive status word: it also works for an axis
    // switched to a virtual/test axis. The drive word belongs to the drive hardware station.
    _abPowerReady[_iAxis]      := astStatus[_iAxis].PowerReady;
    _abPowerConfirmed[_iAxis]  := _abPowerSent[_iAxis]
                                  AND (_abPowerConfirmed[_iAxis] OR astStatus[_iAxis].PowerOn);

    _bAllPowered   := _bAllPowered AND astStatus[_iAxis].PowerOn;
    _bAllOff       := _bAllOff AND NOT astStatus[_iAxis].PowerOn;
    // At rest: standstill, disabled or in error (an axis in error is stopped by its drive)
    _bAllAtRestNow := _bAllAtRestNow AND (astStatus[_iAxis].Standstill
                                          OR NOT astStatus[_iAxis].PowerOn
                                          OR astStatus[_iAxis].HasError);
    _bAnyError     := _bAnyError OR astStatus[_iAxis].HasError;
END_FOR

// ===== 2. Faults -> abort request =====
// Not raised while aborting, aborted or resetting, so a fault can't block its own reset.
_bFaultAbortActive := _eMachineState <> E_MachineState.Aborting
                      AND _eMachineState <> E_MachineState.Aborted
                      AND _eMachineState <> E_MachineState.Resetting;

_uiAbortCode := ABORT_NONE;
FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
    IF _uiAbortCode = ABORT_NONE THEN
        IF NOT astStatus[_iAxis].DataValid THEN
            _uiAbortCode := ABORT_AXIS_DATA + INT_TO_UINT(_iAxis);
        ELSIF astStatus[_iAxis].HasError THEN
            _uiAbortCode := ABORT_AXIS_ERROR + INT_TO_UINT(_iAxis);
        ELSIF _abPowerConfirmed[_iAxis] AND NOT astStatus[_iAxis].PowerOn THEN
            _uiAbortCode := ABORT_POWER_LOST + INT_TO_UINT(_iAxis);
        ELSIF _abCmdRejected[_iAxis] THEN
            _uiAbortCode := ABORT_AXIS_CMD + INT_TO_UINT(_iAxis);
        END_IF
    END_IF
END_FOR
_bAbort := _bFaultAbortActive AND _uiAbortCode <> ABORT_NONE;

// ===== 3. Done for the current machine state =====
CASE _eMachineState OF
    E_MachineState.Off, E_MachineState.Aborted, E_MachineState.Aborting:
        _bDone := _bAllOff;                         // Aborting: torque-free before the supply goes off

    E_MachineState.TurningOn, E_MachineState.Resetting:
        _bDone := _bAllPowered AND NOT _bAnyError;

ELSE
    _bDone := TRUE;                                 // stay powered, nothing else to do
END_CASE

// ===== 4. On change of machine state: reset on entry to Resetting =====
IF _eMachineState <> _eLastMachineState THEN
    IF _eMachineState = E_MachineState.Resetting THEN
        // Any fault aborts, so Resetting always follows one: reset every axis unconditionally.
        // Drive 0 (Ax1) hosts the drive supply, so this also clears supply errors.
        FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
            _abCmdRejected[_iAxis] := FALSE;
            _astReset[_iAxis].In.AxisName := asAxisName[_iAxis];
            _astReset[_iAxis].In.Source   := SOURCE_NAME;
            IF NOT ML_AxsReset(Data := _astReset[_iAxis]) THEN
                _abCmdRejected[_iAxis] := TRUE;
            END_IF
        END_FOR
    END_IF
    _eLastMachineState := _eMachineState;
END_IF

// ===== 5. Command levels: power per axis =====
// Powered in every state except Off and Aborted; in Aborting only until all axes are at rest
_bWantPower := _eMachineState <> E_MachineState.Off
               AND _eMachineState <> E_MachineState.Aborted
               AND NOT (_eMachineState = E_MachineState.Aborting AND _fbRestDebounce.Q);

FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
    // Enable only once the drive's power section is ready (DC bus up) and the axis has no error
    _abWantPower[_iAxis] := _bWantPower AND _abPowerReady[_iAxis] AND NOT astStatus[_iAxis].HasError;

    // ML_AxsPower is one-shot: issue it once each time the wanted level changes
    IF _abWantPower[_iAxis] <> _abPowerSent[_iAxis] THEN
        _astPower[_iAxis].In.AxisName := asAxisName[_iAxis];
        _astPower[_iAxis].In.Source   := SOURCE_NAME;
        _astPower[_iAxis].In.SwitchOn := _abWantPower[_iAxis];
        IF NOT ML_AxsPower(Data := _astPower[_iAxis]) THEN
            _abCmdRejected[_iAxis] := TRUE;
        END_IF
        _abPowerSent[_iAxis] := _abWantPower[_iAxis];
    END_IF
END_FOR

// ===== 6. Outputs, then sub-FBs =====
bAllPowered := _bAllPowered;
bAllOff     := _bAllOff;
bAllAtRest  := _fbRestDebounce.Q;
bAnyError   := _bAnyError;
uiAbortCode := _uiAbortCode;

_fbRestDebounce(IN := _bAllAtRestNow, PT := REST_DEBOUNCE);
_fbStateReq(
    itfClient   := itfClient,
    bAbort      := _bAbort,
    uiAbortCode := _uiAbortCode,
    eStateEcho  := _eMachineState,
    bDone       := _bDone);
