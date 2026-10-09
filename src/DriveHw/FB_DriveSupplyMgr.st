// FUNCTION_BLOCK: FB_DriveSupplyMgr
// Drive hardware station: one drive-integrated supply unit feeding the servo drives.
// Switches the supply to operation when all drives are ready for power, enables DC bus
// power in the drives while the supply is in operation, switches off only when no axis is
// under torque, aborts on supply or drive faults. Faults are cleared by the drive reset of
// drive 0 (which hosts the supply), issued by the servo axis manager in Resetting; this block
// waits for the faults to clear.
// No internal state machine: everything is a level function of the machine state and the
// hardware status.
// Design: design/Unit_DriveHardwareStation_Design.md
// Library: CXA_DriveUtil (MB_PowerSupplyDriveIntegrated)

// ===== Declaration =====
FUNCTION_BLOCK FB_DriveSupplyMgr
VAR_INPUT
    itfClient       : I_StateClient;    // GVL_State.aClients[GVL_State.CLIENT_DRIVE_HW]
    eMachineState : E_MachineState;   // machine state to follow (from the caller: the broadcast, or a sub-manager's state)
    wSupplyStatus   : WORD;             // supply unit status word S-0-2058 (S-0-1720.0.2)
END_VAR
VAR_IN_OUT
    wSupplyCtrl     : WORD;             // supply unit control word S-0-2057 (S-0-1720.0.1)
    awDriveStatus   : ARRAY[*] OF WORD; // drive status words, one per drive
    awDriveDcBusCtrl: ARRAY[*] OF WORD; // drive DC bus power control S-0-0240, one per drive
END_VAR
VAR_OUTPUT
    bSupplyInOperation : BOOL;          // diag: supply in operation mode (DC bus up)
    bSupplyOff      : BOOL;             // diag: supply mains off
    bAllReadyForPower : BOOL;           // diag: every drive ready for power
    bAnyUnderTorque : BOOL;             // diag: some drive under torque
    bAnyDriveFault  : BOOL;             // diag: some drive faulted
    bSupplyFault    : BOOL;             // diag: supply internal error or supply control error
    uiAbortCode     : UINT;             // abort code currently requested, 0 = none
END_VAR
VAR
    _eMachineState  : E_MachineState;   // eMachineState, read once per scan
    _stDrive        : ST_DriveFeedback;
    _bAllReady      : BOOL;
    _bAnyTorque     : BOOL;
    _bAnyDriveFault : BOOL;
    _iFirstFaultDrive : DINT;
    _iDrive         : DINT;
    _bSupplyInOp    : BOOL;
    _bSupplyOff     : BOOL;
    _bSupplyError   : BOOL;             // internal supply unit error
    _bSupplyCtrlError : BOOL;           // supply control FB error (invalid input, mode timeout)

    _bFaultAbortActive : BOOL;          // fault aborts are raised in this state
    _bAbort         : BOOL;
    _uiAbortCode    : UINT;
    _bDone          : BOOL;

    _bWantOn        : BOOL;
    _bSupplyCmd     : BOOL;             // supply to operation (self-holding)
    _bSupplyEnable  : BOOL;
    _eTargetMode    : MB_POWERSUPPLY_TARGET_MODE;

    _fbSupply       : MB_PowerSupplyDriveIntegrated;
    _fbStateReq     : FB_StateRequest;
END_VAR
VAR CONSTANT
    MASK_DC_BUS_POWER   : WORD := 16#0001;  // S-0-0240 bit 0: DC bus power enabled
    NO_DRIVE            : DINT := -1;     // drives are numbered from 0
    SUPPLY_MODE_TIMEOUT : TIME := T#10S;    // max time per supply mode change (supervised by the FB)
    ABORT_NONE          : UINT := 0;
    ABORT_SUPPLY_CTRL   : UINT := 100;      // supply did not reach its target mode / invalid input
    ABORT_SUPPLY_FAULT  : UINT := 101;      // supply unit internal error
    ABORT_DRIVE_FAULT   : UINT := 110;      // + drive index
END_VAR

// ===== Implementation =====
_eMachineState := eMachineState;     // read once per scan

// ===== 1. Read status =====
_bAllReady        := TRUE;
_bAnyTorque       := FALSE;
_bAnyDriveFault   := FALSE;
_iFirstFaultDrive := NO_DRIVE;
FOR _iDrive := LOWER_BOUND(awDriveStatus, 1) TO UPPER_BOUND(awDriveStatus, 1) DO
    _stDrive := Fn_DriveFeedback(wStatus := awDriveStatus[_iDrive]);
    _bAllReady  := _bAllReady AND _stDrive.bReadyForPower;
    _bAnyTorque := _bAnyTorque OR _stDrive.bUnderTorque;
    IF _stDrive.bFault THEN
        _bAnyDriveFault := TRUE;
        IF _iFirstFaultDrive = NO_DRIVE THEN
            _iFirstFaultDrive := _iDrive;
        END_IF
    END_IF
END_FOR

// Supply state from the previous call of the supply FB
_bSupplyInOp      := _fbSupply.ActualMode = MB_POWERSUPPLY_ACTUAL_MODE.OPERATION_MODE;
_bSupplyOff       := _fbSupply.ActualMode = MB_POWERSUPPLY_ACTUAL_MODE.MAINS_OFF;
_bSupplyError     := _fbSupply.PowerSupplyError;
_bSupplyCtrlError := _fbSupply.Error;

// ===== 2. Faults -> abort request =====
// Not raised while aborting, aborted or resetting: a fault that is still present must not
// block the reset that clears it. A fault that survives the reset leaves this client not
// done, and the manager's Resetting timeout aborts again.
_bFaultAbortActive := _eMachineState <> E_MachineState.Aborting
                      AND _eMachineState <> E_MachineState.Aborted
                      AND _eMachineState <> E_MachineState.Resetting;

_uiAbortCode := ABORT_NONE;
IF _bSupplyError THEN
    _uiAbortCode := ABORT_SUPPLY_FAULT;
ELSIF _bAnyDriveFault THEN
    _uiAbortCode := ABORT_DRIVE_FAULT + DINT_TO_UINT(_iFirstFaultDrive);
ELSIF _bSupplyCtrlError THEN
    _uiAbortCode := ABORT_SUPPLY_CTRL;
END_IF
_bAbort := _bFaultAbortActive AND _uiAbortCode <> ABORT_NONE;

// ===== 3. Done for the current machine state =====
CASE _eMachineState OF
    E_MachineState.Off:
        _bDone := NOT _bSupplyCmd AND _bSupplyOff;

    E_MachineState.TurningOn:
        _bDone := _bSupplyInOp;

    E_MachineState.Aborting:
        _bDone := NOT _bAnyTorque AND NOT _bSupplyCmd AND _bSupplyOff;

    E_MachineState.Resetting:
        _bDone := NOT _bSupplyError AND NOT _bSupplyCtrlError AND NOT _bAnyDriveFault AND _bSupplyInOp;

ELSE
    _bDone := TRUE;     // nothing to do beyond holding the supply state
END_CASE

// ===== 4. Command levels =====
_bWantOn := _eMachineState <> E_MachineState.Off
            AND _eMachineState <> E_MachineState.Aborted
            AND _eMachineState <> E_MachineState.Aborting;

// Switch on only when every drive is ready, then hold; never switch off under torque
_bSupplyCmd := (_bWantOn AND (_bSupplyCmd OR _bAllReady))
               OR (_bSupplyCmd AND _bAnyTorque);

IF _bSupplyCmd THEN
    _eTargetMode := MB_POWERSUPPLY_TARGET_MODE.OPERATION_MODE;
ELSE
    _eTargetMode := MB_POWERSUPPLY_TARGET_MODE.MAINS_OFF_NO_DISCHARGE;
END_IF

// Keep the FB enabled until the supply is really off. In Resetting, a supply control error
// holds Enable off, which is the documented way to clear the FB's error.
_bSupplyEnable := (_bSupplyCmd OR NOT _bSupplyOff)
                  AND NOT (_eMachineState = E_MachineState.Resetting AND _bSupplyCtrlError);

// DC bus power to every drive while the supply is in operation
FOR _iDrive := LOWER_BOUND(awDriveDcBusCtrl, 1) TO UPPER_BOUND(awDriveDcBusCtrl, 1) DO
    IF _bSupplyInOp THEN
        awDriveDcBusCtrl[_iDrive] := awDriveDcBusCtrl[_iDrive] OR MASK_DC_BUS_POWER;
    ELSE
        awDriveDcBusCtrl[_iDrive] := awDriveDcBusCtrl[_iDrive] AND NOT MASK_DC_BUS_POWER;
    END_IF
END_FOR

// ===== 5. Outputs, then sub-FBs =====
bSupplyInOperation := _bSupplyInOp;
bSupplyOff         := _bSupplyOff;
bAllReadyForPower  := _bAllReady;
bAnyUnderTorque    := _bAnyTorque;
bAnyDriveFault     := _bAnyDriveFault;
bSupplyFault       := _bSupplyError OR _bSupplyCtrlError;
uiAbortCode        := _uiAbortCode;

_fbSupply(
    Enable       := _bSupplyEnable,
    TargetMode   := _eTargetMode,
    Timeout      := SUPPLY_MODE_TIMEOUT,
    InProcStatus := wSupplyStatus,
    OutProcCtrl  := wSupplyCtrl);
_fbStateReq(
    itfClient   := itfClient,
    bAbort      := _bAbort,
    uiAbortCode := _uiAbortCode,
    eStateEcho  := _eMachineState,
    bDone       := _bDone);
