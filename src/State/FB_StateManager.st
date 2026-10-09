// FUNCTION_BLOCK: FB_StateManager
// Owns the machine state. Reads every client, runs the machine state machine
// (house structure: read clients, jumps, transitions, on change, cyclic) and outputs
// the state for PRG_Main to broadcast through GVL_State.
// Design: design/Central_State_Management_Design.md

// ===== Declaration =====
FUNCTION_BLOCK FB_StateManager
VAR_IN_OUT
    aClients            : ARRAY[*] OF FB_StateClient;  // GVL_State.aClients; lower bound must be >= 1
END_VAR
VAR_OUTPUT
    eMachineState       : E_MachineState;   // the machine state
    uiAbortCode         : UINT;             // first abort cause, 0 = none
    iAbortClient        : DINT;             // client that caused the abort, 0 = none / manager
    eAbortFrom          : E_MachineState;   // state the machine aborted from
    bAbortUnacked       : BOOL;             // timeout abort: iAbortClient never acknowledged the state
    iAbortTimeoutClient : DINT;             // client that did not finish Aborting in time
    iNotDoneClient      : DINT;             // live: first client not done with the current state
END_VAR
VAR
    _eState             : E_MachineState;
    _eLastState         : E_MachineState;

    // Abort record: first cause only, cleared on entry to Idle
    _uiAbortCode        : UINT;
    _iAbortClient       : DINT;
    _eAbortFrom         : E_MachineState;
    _bAbortUnacked      : BOOL;
    _iAbortTimeoutClient: DINT;

    // Combined client data, rebuilt every scan
    _bReqOn             : BOOL;
    _bReqOff            : BOOL;
    _bReqStart          : BOOL;
    _bReqStop           : BOOL;
    _bReqHold           : BOOL;
    _bReqAbort          : BOOL;
    _bReqReset          : BOOL;
    _bAllDone           : BOOL;             // every enabled client is done with _eDoneState
    _eDoneState         : E_MachineState;   // state _bAllDone was evaluated for
    _iFirstAbortClient  : DINT;
    _uiFirstAbortCode   : UINT;
    _iFirstNotDone      : DINT;
    _bFirstNotDoneUnacked : BOOL;
    _iClient            : DINT;
    _stClient           : ST_StateClientData;

    _tStepTimeout       : TIME;             // timeout of the current state, T#0S = none
    _fbStepTimer        : TON;
END_VAR
VAR CONSTANT
    NO_CLIENT                : DINT := 0;
    ABORT_CODE_NONE          : UINT := 0;
    ABORT_CODE_TIMEOUT       : UINT := 16#FFF0;    // a transitional state timed out
    ABORT_CODE_INVALID_STATE : UINT := 16#FFFF;    // unknown state value (e.g. after online change)

    TURNING_ON_TIMEOUT  : TIME := T#30S;
    STARTING_TIMEOUT    : TIME := T#60S;
    STOPPING_TIMEOUT    : TIME := T#30S;
    HOLDING_TIMEOUT     : TIME := T#30S;
    UNHOLDING_TIMEOUT   : TIME := T#30S;
    ABORTING_TIMEOUT    : TIME := T#10S;
    RESETTING_TIMEOUT   : TIME := T#30S;
END_VAR

// ===== Implementation =====
// ===== 0. Read clients =====
_bReqOn    := FALSE;
_bReqOff   := FALSE;
_bReqStart := FALSE;
_bReqStop  := FALSE;
_bReqHold  := FALSE;
_bReqAbort := FALSE;
_bReqReset := FALSE;
_bAllDone  := TRUE;
_eDoneState := _eState;
_iFirstAbortClient    := NO_CLIENT;
_uiFirstAbortCode     := ABORT_CODE_NONE;
_iFirstNotDone        := NO_CLIENT;
_bFirstNotDoneUnacked := FALSE;

FOR _iClient := LOWER_BOUND(aClients, 1) TO UPPER_BOUND(aClients, 1) DO
    _stClient := aClients[_iClient].Data;
    IF _stClient.bEnabled THEN
        _bReqOn    := _bReqOn    OR _stClient.bOn;
        _bReqOff   := _bReqOff   OR _stClient.bOff;
        _bReqStart := _bReqStart OR _stClient.bStart;
        _bReqStop  := _bReqStop  OR _stClient.bStop;
        _bReqHold  := _bReqHold  OR _stClient.bHold;
        _bReqAbort := _bReqAbort OR _stClient.bAbort;
        _bReqReset := _bReqReset OR _stClient.bReset;

        IF _stClient.bAbort AND _iFirstAbortClient = NO_CLIENT THEN
            _iFirstAbortClient := _iClient;
            _uiFirstAbortCode  := _stClient.uiAbortCode;
        END_IF

        // Done only counts for the current state: a stale done from an old echo is ignored
        IF NOT (_stClient.eStateEcho = _eState AND _stClient.bDone) THEN
            _bAllDone := FALSE;
            IF _iFirstNotDone = NO_CLIENT THEN
                _iFirstNotDone        := _iClient;
                _bFirstNotDoneUnacked := _stClient.eStateEcho <> _eState;
            END_IF
        END_IF
    END_IF
END_FOR

// ===== 1. Jumps: abort, then timeout =====
IF _eState = E_MachineState.Aborting THEN
    // Reaching a resettable state matters more than waiting forever
    IF _fbStepTimer.Q THEN
        _iAbortTimeoutClient := _iFirstNotDone;
        _eState := E_MachineState.Aborted;
    END_IF
ELSIF _eState <> E_MachineState.Off AND _eState <> E_MachineState.Aborted THEN
    IF _bReqAbort THEN
        _uiAbortCode   := _uiFirstAbortCode;
        _iAbortClient  := _iFirstAbortClient;
        _bAbortUnacked := FALSE;
        _eAbortFrom    := _eState;
        _eState := E_MachineState.Aborting;
    ELSIF _fbStepTimer.Q THEN
        _uiAbortCode   := ABORT_CODE_TIMEOUT;
        _iAbortClient  := _iFirstNotDone;
        _bAbortUnacked := _bFirstNotDoneUnacked;
        _eAbortFrom    := _eState;
        _eState := E_MachineState.Aborting;
    END_IF
END_IF

// ===== 2. Transitions only =====
// _bAllDone was evaluated for _eDoneState (the state at the start of the scan). After a
// jump it says nothing about the new state, so every AllDone transition also checks that.
CASE _eState OF
    E_MachineState.Off:
        IF _bReqOn AND NOT _bReqOff THEN
            _eState := E_MachineState.TurningOn;
        END_IF

    E_MachineState.TurningOn:
        IF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Idle;
        END_IF

    E_MachineState.Idle:
        IF _bReqOff THEN
            _eState := E_MachineState.Off;
        ELSIF _bReqStart AND NOT _bReqStop THEN
            _eState := E_MachineState.Starting;
        END_IF

    E_MachineState.Starting:
        // A stop always wins, whether or not starting has completed
        IF _bReqStop THEN
            _eState := E_MachineState.Stopping;
        ELSIF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Running;
        END_IF

    E_MachineState.Running:
        // Stop wins over hold
        IF _bReqStop THEN
            _eState := E_MachineState.Stopping;
        ELSIF _bReqHold THEN
            _eState := E_MachineState.Holding;
        END_IF

    E_MachineState.Stopping:
        IF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Idle;
        END_IF

    E_MachineState.Holding:
        IF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Hold;
        END_IF

    E_MachineState.Hold:
        // Resume only when no client requests hold any more
        IF NOT _bReqHold THEN
            _eState := E_MachineState.UnHolding;
        END_IF

    E_MachineState.UnHolding:
        IF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Running;
        END_IF

    E_MachineState.Aborting:
        IF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Aborted;
        END_IF

    E_MachineState.Aborted:
        // A held abort can't be reset away
        IF _bReqReset AND NOT _bReqAbort THEN
            _eState := E_MachineState.Resetting;
        END_IF

    E_MachineState.Resetting:
        IF _bAllDone AND _eDoneState = _eState THEN
            _eState := E_MachineState.Idle;
        END_IF

ELSE
    _uiAbortCode   := ABORT_CODE_INVALID_STATE;
    _iAbortClient  := NO_CLIENT;
    _bAbortUnacked := FALSE;
    _eAbortFrom    := _eState;
    _eState := E_MachineState.Aborting;
END_CASE

// ===== 3. On change: common actions, then entry actions =====
IF _eState <> _eLastState THEN
    // Every change: restart the step timer, no timeout unless the new state sets one
    _fbStepTimer(IN := FALSE);
    _tStepTimeout := T#0S;

    CASE _eState OF
        E_MachineState.Idle:
            // Idle means nothing is wrong: drop the abort record
            _uiAbortCode         := ABORT_CODE_NONE;
            _iAbortClient        := NO_CLIENT;
            _eAbortFrom          := E_MachineState.Off;
            _bAbortUnacked       := FALSE;
            _iAbortTimeoutClient := NO_CLIENT;

        E_MachineState.TurningOn:
            _tStepTimeout := TURNING_ON_TIMEOUT;

        E_MachineState.Starting:
            _tStepTimeout := STARTING_TIMEOUT;

        E_MachineState.Stopping:
            _tStepTimeout := STOPPING_TIMEOUT;

        E_MachineState.Holding:
            _tStepTimeout := HOLDING_TIMEOUT;

        E_MachineState.UnHolding:
            _tStepTimeout := UNHOLDING_TIMEOUT;

        E_MachineState.Aborting:
            _tStepTimeout := ABORTING_TIMEOUT;

        E_MachineState.Resetting:
            _tStepTimeout := RESETTING_TIMEOUT;
    END_CASE

    _eLastState := _eState;
END_IF

// ===== 4. Cyclic: outputs, then sub-FBs =====
eMachineState       := _eState;
uiAbortCode         := _uiAbortCode;
iAbortClient        := _iAbortClient;
eAbortFrom          := _eAbortFrom;
bAbortUnacked       := _bAbortUnacked;
iAbortTimeoutClient := _iAbortTimeoutClient;
iNotDoneClient      := _iFirstNotDone;

_fbStepTimer(IN := _tStepTimeout > T#0S, PT := _tStepTimeout);
