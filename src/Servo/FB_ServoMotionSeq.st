// FUNCTION_BLOCK: FB_ServoMotionSeq
// Servo station motion: position Ax1/Ax2 to their offsets and the VM to 0, gear Ax1/Ax2 to the
// VM (absolute to set point, 1:1, slave offsets), run the VM at the speed command (on the fly),
// hold/unhold by stopping/restarting the VM, stop by halting the VM then the slaves, abort by
// stopping the VM then the slaves at the abort deceleration.
// House structure: jumps -> transitions -> on change (ML_ commands issued once, on entry)
// -> cyclic (outputs, command trackers, client).
// Design: design/Unit_ServoStation_Design.md §4
// Library: CXA_Motion

// ===== Declaration =====
FUNCTION_BLOCK FB_ServoMotionSeq
VAR_INPUT
    itfClient   : I_StateClient;                                // GVL_State.aClients[GVL_State.CLIENT_SERVO_MOTION]
    eMachineState : E_MachineState;   // machine state to follow (from the caller: the broadcast, or a sub-manager's state)
    asAxisName  : ARRAY[0..GVL_Axes.LAST_AXIS] OF STRING(15);   // ctrlX MOTION axis names: Ax1, Ax2, VM
    lrSpeedCmd  : LREAL;    // VM velocity in Running; limited by the caller; may change at any time
    lrAx1Offset : LREAL;    // Ax1 phase offset from the VM (positioning target and gear slave offset)
    lrAx2Offset : LREAL;    // Ax2 phase offset from the VM (positioning target and gear slave offset)
END_VAR
VAR_IN_OUT CONSTANT
    astStatus   : ARRAY[0..GVL_Axes.LAST_AXIS] OF ST_AxisCyclicStatus;   // cyclic axis status
END_VAR
VAR_OUTPUT
    eSeqState   : E_ServoSeqState;  // current step, for HMI / online view
    uiAbortCode : UINT;             // abort code currently requested, 0 = none
END_VAR
VAR
    _eMachineState  : E_MachineState;
    _eSeqState      : E_ServoSeqState;
    _eLastSeqState  : E_ServoSeqState;
    _iAxis          : INT;
    _lrSpeedLatched : LREAL;        // speed of the velocity command currently running

    // Command tracking, one command per axis at a time
    _abTrack        : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;
    _abAccepted     : ARRAY[0..GVL_Axes.LAST_AXIS] OF BOOL;
    _aulCmdID       : ARRAY[0..GVL_Axes.LAST_AXIS] OF ULINT;
    _afbCmd         : ARRAY[0..GVL_Axes.LAST_AXIS] OF FB_AxisCmdState;
    _iFailedAxis    : INT;

    // ML_ command data
    _astPosAbs      : ARRAY[0..GVL_Axes.LAST_AXIS] OF ML_AxsPosData;
    _astGearIn      : ARRAY[0..GVL_Axes.LAST_AXIS] OF ML_AxsGearInPos2Data;    // used for Ax1, Ax2
    _stVmVel        : ML_AxsVelData;
    _astAbort       : ARRAY[0..GVL_Axes.LAST_AXIS] OF ML_AxsAbortData;

    _bFaultAbortActive : BOOL;
    _bAbort         : BOOL;
    _uiAbortCode    : UINT;
    _bDone          : BOOL;
    _fbStateReq     : FB_StateRequest;
END_VAR
VAR CONSTANT
    NO_AXIS         : INT := -1;
    VM_POSITION     : LREAL := 0.0;     // VM start position
    POS_VEL         : LREAL := 90.0;    // start positioning: velocity (units/s)   -- set for the machine
    POS_ACC         : LREAL := 360.0;   // start positioning: acceleration
    POS_DEC         : LREAL := 360.0;   // start positioning and ungearing: deceleration
    VM_ACC          : LREAL := 360.0;   // VM ramp up
    VM_DEC          : LREAL := 360.0;   // VM normal ramp down (hold, stop)
    ABORT_DEC       : LREAL := 1440.0;  // abort deceleration, separate from VM_DEC
    NO_JERK_LIMIT   : LREAL := 0.0;     // jerk 0 on both = no jerk limit (trapezoid)
    SPEED_DEADBAND  : LREAL := 0.01;    // speed command change that re-issues the VM velocity
    GEAR_RATIO      : DINT := 1;        // 1:1
    MASTER_OFFSET   : LREAL := 0.0;
    MASTER_SYNC_POS : LREAL := 0.0;
    MASTER_START_DIST : LREAL := 0.0;
    FINE_ADJUST     : LREAL := 0.0;
    SOURCE_NAME     : STRING(50) := 'FB_ServoMotionSeq';
    ABORT_NONE      : UINT := 0;
    ABORT_MOTION_CMD: UINT := 300;      // + axis index: motion command rejected, aborted or failed
    ABORT_SEQ_INVALID : UINT := 399;    // unknown sequence state
END_VAR

// ===== Implementation =====
_eMachineState := eMachineState;     // read once per scan

// ===== 1. Jumps: abort first, then stop =====
IF _eMachineState = E_MachineState.Aborting THEN
    // From any active step: stop the VM first (slaves follow while geared)
    IF _eSeqState <> E_ServoSeqState.AbortVm
       AND _eSeqState <> E_ServoSeqState.AbortSlaves
       AND _eSeqState <> E_ServoSeqState.Idle THEN
        _eSeqState := E_ServoSeqState.AbortVm;
    END_IF
ELSIF _eMachineState = E_MachineState.Stopping THEN
    // From any active step not already on the stop (or abort) path: halt the VM, then ungear
    IF _eSeqState <> E_ServoSeqState.Idle
       AND _eSeqState <> E_ServoSeqState.VmHalting
       AND _eSeqState <> E_ServoSeqState.Ungearing
       AND _eSeqState <> E_ServoSeqState.AbortVm
       AND _eSeqState <> E_ServoSeqState.AbortSlaves THEN
        _eSeqState := E_ServoSeqState.VmHalting;
    END_IF
END_IF

// ===== 2. Transitions only =====
CASE _eSeqState OF
    E_ServoSeqState.Idle:
        IF _eMachineState = E_MachineState.Starting THEN
            _eSeqState := E_ServoSeqState.Positioning;
        END_IF

    E_ServoSeqState.Positioning:
        IF _afbCmd[GVL_Axes.AX1].bDone AND _afbCmd[GVL_Axes.AX2].bDone AND _afbCmd[GVL_Axes.VM].bDone THEN
            _eSeqState := E_ServoSeqState.GearingIn;
        END_IF

    E_ServoSeqState.GearingIn:
        IF astStatus[GVL_Axes.AX1].InSynchronization AND astStatus[GVL_Axes.AX2].InSynchronization THEN
            _eSeqState := E_ServoSeqState.Synced;
        END_IF

    E_ServoSeqState.Synced:
        IF _eMachineState = E_MachineState.Running OR _eMachineState = E_MachineState.UnHolding THEN
            _eSeqState := E_ServoSeqState.VmRunning;
        END_IF

    E_ServoSeqState.VmRunning:
        IF _eMachineState = E_MachineState.Holding THEN
            _eSeqState := E_ServoSeqState.VmHalting;
        ELSIF ABS(lrSpeedCmd - _lrSpeedLatched) > SPEED_DEADBAND THEN
            _eSeqState := E_ServoSeqState.VmSpeedChange;
        END_IF

    E_ServoSeqState.VmSpeedChange:
        // Back to VmRunning: its entry issues the velocity command with the new speed
        _eSeqState := E_ServoSeqState.VmRunning;

    E_ServoSeqState.VmHalting:
        IF _afbCmd[GVL_Axes.VM].bDone THEN
            IF _eMachineState = E_MachineState.Stopping THEN
                _eSeqState := E_ServoSeqState.Ungearing;
            ELSE
                _eSeqState := E_ServoSeqState.Synced;   // Holding: VM stopped, slaves stay geared
            END_IF
        END_IF

    E_ServoSeqState.Ungearing:
        IF _afbCmd[GVL_Axes.AX1].bDone AND _afbCmd[GVL_Axes.AX2].bDone THEN
            _eSeqState := E_ServoSeqState.Idle;
        END_IF

    E_ServoSeqState.AbortVm:
        // Stopped, or can't be commanded any more (unpowered / in error): move on, never hang
        IF _afbCmd[GVL_Axes.VM].bDone OR _afbCmd[GVL_Axes.VM].bFailed
           OR NOT astStatus[GVL_Axes.VM].PowerOn OR astStatus[GVL_Axes.VM].HasError THEN
            _eSeqState := E_ServoSeqState.AbortSlaves;
        END_IF

    E_ServoSeqState.AbortSlaves:
        IF (_afbCmd[GVL_Axes.AX1].bDone OR _afbCmd[GVL_Axes.AX1].bFailed
            OR NOT astStatus[GVL_Axes.AX1].PowerOn OR astStatus[GVL_Axes.AX1].HasError)
           AND (_afbCmd[GVL_Axes.AX2].bDone OR _afbCmd[GVL_Axes.AX2].bFailed
            OR NOT astStatus[GVL_Axes.AX2].PowerOn OR astStatus[GVL_Axes.AX2].HasError) THEN
            _eSeqState := E_ServoSeqState.Idle;
        END_IF

ELSE
    _eSeqState := E_ServoSeqState.AbortVm;
END_CASE

// ===== 3. On change: common actions, then entry actions (ML_ commands issued here, once) =====
IF _eSeqState <> _eLastSeqState THEN
    // Every change: stop tracking the previous step's commands
    FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
        _abTrack[_iAxis]    := FALSE;
        _abAccepted[_iAxis] := FALSE;
    END_FOR

    CASE _eSeqState OF
        E_ServoSeqState.Positioning:
            // Absolute moves, shortest way on the modulo axes
            _astPosAbs[GVL_Axes.AX1].In.Position := lrAx1Offset;
            _astPosAbs[GVL_Axes.AX2].In.Position := lrAx2Offset;
            _astPosAbs[GVL_Axes.VM].In.Position  := VM_POSITION;
            FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
                _astPosAbs[_iAxis].In.AxisName := asAxisName[_iAxis];
                _astPosAbs[_iAxis].In.Source   := SOURCE_NAME;
                _astPosAbs[_iAxis].In.Buffered := TRUE;
                _astPosAbs[_iAxis].In.DynLimits.Velocity     := POS_VEL;
                _astPosAbs[_iAxis].In.DynLimits.Acceleration := POS_ACC;
                _astPosAbs[_iAxis].In.DynLimits.Deceleration := POS_DEC;
                _astPosAbs[_iAxis].In.DynLimits.JerkAcc      := NO_JERK_LIMIT;
                _astPosAbs[_iAxis].In.DynLimits.JerkDec      := NO_JERK_LIMIT;
                _abAccepted[_iAxis] := ML_AxsPosAbs(Data := _astPosAbs[_iAxis]);
                _aulCmdID[_iAxis]   := _astPosAbs[_iAxis].Out.CmdID;
                _abTrack[_iAxis]    := TRUE;
            END_FOR

        E_ServoSeqState.GearingIn:
            // Absolute to the VM set point: slave = VM x 1 + slave offset, synchronised the shortest way
            _astGearIn[GVL_Axes.AX1].In.Parameters.SlaveOffset := lrAx1Offset;
            _astGearIn[GVL_Axes.AX2].In.Parameters.SlaveOffset := lrAx2Offset;
            FOR _iAxis := GVL_Axes.AX1 TO GVL_Axes.AX2 DO
                _astGearIn[_iAxis].In.AxisName            := asAxisName[_iAxis];
                _astGearIn[_iAxis].In.Source              := SOURCE_NAME;
                _astGearIn[_iAxis].In.MasterName          := asAxisName[GVL_Axes.VM];
                _astGearIn[_iAxis].In.SyncSource          := ML_SyncSource.SYNC_SETPOINT;
                _astGearIn[_iAxis].In.SyncMode            := ML_SyncMode.SYNC_ABSOLUTE;
                _astGearIn[_iAxis].In.DynSyncDirection    := ML_DynSyncDirection.SHORTEST_WAY;
                _astGearIn[_iAxis].In.MasterSyncPosition  := MASTER_SYNC_POS;
                _astGearIn[_iAxis].In.MasterStartDistance := MASTER_START_DIST;
                _astGearIn[_iAxis].In.Parameters.MasterOffset     := MASTER_OFFSET;
                _astGearIn[_iAxis].In.Parameters.RatioNumerator   := GEAR_RATIO;
                _astGearIn[_iAxis].In.Parameters.RatioDenominator := GEAR_RATIO;
                _astGearIn[_iAxis].In.Parameters.FineAdjust       := FINE_ADJUST;
                _astGearIn[_iAxis].In.Buffered            := TRUE;
                _abAccepted[_iAxis] := ML_AxsGearInPos2(Data := _astGearIn[_iAxis]);
                _aulCmdID[_iAxis]   := _astGearIn[_iAxis].Out.CmdID;
                _abTrack[_iAxis]    := TRUE;
            END_FOR

        E_ServoSeqState.VmRunning:
            // Unbuffered, so a new speed replaces the running command without stopping the VM
            _lrSpeedLatched := lrSpeedCmd;
            _stVmVel.In.AxisName := asAxisName[GVL_Axes.VM];
            _stVmVel.In.Source   := SOURCE_NAME;
            _stVmVel.In.Velocity := _lrSpeedLatched;
            _stVmVel.In.DriveVelMode := FALSE;
            _stVmVel.In.Buffered := FALSE;
            _stVmVel.In.DynLimits.Velocity     := ABS(_lrSpeedLatched);
            _stVmVel.In.DynLimits.Acceleration := VM_ACC;
            _stVmVel.In.DynLimits.Deceleration := VM_DEC;
            _stVmVel.In.DynLimits.JerkAcc      := NO_JERK_LIMIT;
            _stVmVel.In.DynLimits.JerkDec      := NO_JERK_LIMIT;
            _abAccepted[GVL_Axes.VM] := ML_AxsVelocity(Data := _stVmVel);
            _aulCmdID[GVL_Axes.VM]   := _stVmVel.Out.CmdID;
            _abTrack[GVL_Axes.VM]    := TRUE;

        E_ServoSeqState.VmHalting:
            _astAbort[GVL_Axes.VM].In.AxisName     := asAxisName[GVL_Axes.VM];
            _astAbort[GVL_Axes.VM].In.Source       := SOURCE_NAME;
            _astAbort[GVL_Axes.VM].In.Deceleration := VM_DEC;
            _astAbort[GVL_Axes.VM].In.JerkDec      := NO_JERK_LIMIT;
            _abAccepted[GVL_Axes.VM] := ML_AxsAbort(Data := _astAbort[GVL_Axes.VM]);
            _aulCmdID[GVL_Axes.VM]   := _astAbort[GVL_Axes.VM].Out.CmdID;
            _abTrack[GVL_Axes.VM]    := TRUE;

        E_ServoSeqState.Ungearing:
            // Slaves are already at zero speed with the VM: cancelling the gearing brings them to standstill
            FOR _iAxis := GVL_Axes.AX1 TO GVL_Axes.AX2 DO
                _astAbort[_iAxis].In.AxisName     := asAxisName[_iAxis];
                _astAbort[_iAxis].In.Source       := SOURCE_NAME;
                _astAbort[_iAxis].In.Deceleration := POS_DEC;
                _astAbort[_iAxis].In.JerkDec      := NO_JERK_LIMIT;
                _abAccepted[_iAxis] := ML_AxsAbort(Data := _astAbort[_iAxis]);
                _aulCmdID[_iAxis]   := _astAbort[_iAxis].Out.CmdID;
                _abTrack[_iAxis]    := TRUE;
            END_FOR

        E_ServoSeqState.AbortVm:
            // Slaves follow the VM down while still geared
            _astAbort[GVL_Axes.VM].In.AxisName     := asAxisName[GVL_Axes.VM];
            _astAbort[GVL_Axes.VM].In.Source       := SOURCE_NAME;
            _astAbort[GVL_Axes.VM].In.Deceleration := ABORT_DEC;
            _astAbort[GVL_Axes.VM].In.JerkDec      := NO_JERK_LIMIT;
            _abAccepted[GVL_Axes.VM] := ML_AxsAbort(Data := _astAbort[GVL_Axes.VM]);
            _aulCmdID[GVL_Axes.VM]   := _astAbort[GVL_Axes.VM].Out.CmdID;
            _abTrack[GVL_Axes.VM]    := TRUE;

        E_ServoSeqState.AbortSlaves:
            FOR _iAxis := GVL_Axes.AX1 TO GVL_Axes.AX2 DO
                _astAbort[_iAxis].In.AxisName     := asAxisName[_iAxis];
                _astAbort[_iAxis].In.Source       := SOURCE_NAME;
                _astAbort[_iAxis].In.Deceleration := ABORT_DEC;
                _astAbort[_iAxis].In.JerkDec      := NO_JERK_LIMIT;
                _abAccepted[_iAxis] := ML_AxsAbort(Data := _astAbort[_iAxis]);
                _aulCmdID[_iAxis]   := _astAbort[_iAxis].Out.CmdID;
                _abTrack[_iAxis]    := TRUE;
            END_FOR
    END_CASE

    _eLastSeqState := _eSeqState;
END_IF

// ===== 4. Cyclic =====
// Faults -> abort request. Command failures in the abort steps are expected (axes may be
// unpowered or in error) and don't count. Not raised while aborting, aborted or resetting.
_bFaultAbortActive := _eMachineState <> E_MachineState.Aborting
                      AND _eMachineState <> E_MachineState.Aborted
                      AND _eMachineState <> E_MachineState.Resetting;
_iFailedAxis := NO_AXIS;
IF _eSeqState <> E_ServoSeqState.AbortVm AND _eSeqState <> E_ServoSeqState.AbortSlaves THEN
    FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
        IF _afbCmd[_iAxis].bFailed AND _iFailedAxis = NO_AXIS THEN
            _iFailedAxis := _iAxis;
        END_IF
    END_FOR
END_IF
IF _iFailedAxis <> NO_AXIS THEN
    _uiAbortCode := ABORT_MOTION_CMD + INT_TO_UINT(_iFailedAxis);
ELSE
    _uiAbortCode := ABORT_NONE;
END_IF
_bAbort := _bFaultAbortActive AND _uiAbortCode <> ABORT_NONE;

// Done for the current machine state, from the sequence state
CASE _eMachineState OF
    E_MachineState.Starting, E_MachineState.Holding:
        _bDone := _eSeqState = E_ServoSeqState.Synced;

    E_MachineState.UnHolding:
        _bDone := _eSeqState = E_ServoSeqState.VmRunning AND astStatus[GVL_Axes.VM].InVelocity;

    E_MachineState.Stopping, E_MachineState.Aborting:
        _bDone := _eSeqState = E_ServoSeqState.Idle;

ELSE
    _bDone := TRUE;     // no motion work in this machine state
END_CASE

// Outputs
eSeqState   := _eSeqState;
uiAbortCode := _uiAbortCode;

// Sub-FBs: command trackers and the state client, every scan
FOR _iAxis := 0 TO GVL_Axes.LAST_AXIS DO
    _afbCmd[_iAxis](
        bActive   := _abTrack[_iAxis],
        sObjName  := asAxisName[_iAxis],
        ulCmdID   := _aulCmdID[_iAxis],
        bAccepted := _abAccepted[_iAxis]);
END_FOR
_fbStateReq(
    itfClient   := itfClient,
    bAbort      := _bAbort,
    uiAbortCode := _uiAbortCode,
    eStateEcho  := _eMachineState,
    bDone       := _bDone);
