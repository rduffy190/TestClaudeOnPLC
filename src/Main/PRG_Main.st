// PROGRAM: PRG_Main
// Reads the axis status, calls the state manager, broadcasts its state, then calls the units.
// Order matters: axis status first (it's an input), then the manager, so every unit sees the
// state settled on this scan.

// ===== Declaration =====
PROGRAM PRG_Main
VAR
    fbRtMem         : DL_MemUser;           // RT node access, shared by the axis map and readers
    stAxisRtConfig  : ST_AxisRtConfig;      // RT read configuration from fbAxisRtMap
    fbAxisRtMap     : FB_AxisRtMap;         // opens motion/realtime/cyclicStatusData, reads its map
    afbAxisRtRead   : ARRAY[0..GVL_Axes.LAST_AXIS] OF FB_AxisRtRead;   // one reader per axis -> GVL_Axes.astStatus
    bAxisRtReadFault : BOOL;                // some reader failed: remap
    iAxis           : INT;
    fbStateManager  : FB_StateManager;
    fbHmiStateReq   : FB_StateRequest;      // HMI is just another client
    fbDriveSupply   : FB_DriveSupplyMgr;
    fbServoAxes     : FB_ServoAxisMgr;
    fbServoMotion   : FB_ServoMotionSeq;
END_VAR
VAR CONSTANT
    HMI_ABORT_CODE  : UINT := 1;            // operator abort
    MODULO          : LREAL := 360.0;       // servo axes modulo
END_VAR

// ===== Implementation =====
// ----- Axis status from the Data Layer RT -----
// The map FB opens the node and builds the config; each reader then reads one axis.
// A reader failure (last scan) makes the map FB close, reopen and remap.
fbAxisRtMap(
    bEnable    := TRUE,
    bReadFault := bAxisRtReadFault,
    asAxisName := GVL_Axes.asAxisName,
    fbMem      := fbRtMem,
    stConfig   := stAxisRtConfig);
bAxisRtReadFault := FALSE;
FOR iAxis := 0 TO GVL_Axes.LAST_AXIS DO
    afbAxisRtRead[iAxis](
        iAxis     := iAxis,
        fbMem     := fbRtMem,
        stConfig  := stAxisRtConfig,
        astStatus := GVL_Axes.astStatus);
    bAxisRtReadFault := bAxisRtReadFault OR afbAxisRtRead[iAxis].bError;
END_FOR

// ----- State manager and broadcast -----
fbStateManager(aClients := GVL_State.aClients);

GVL_State.eMachineState       := fbStateManager.eMachineState;
GVL_State.uiAbortCode         := fbStateManager.uiAbortCode;
GVL_State.iAbortClient        := fbStateManager.iAbortClient;
GVL_State.eAbortFrom          := fbStateManager.eAbortFrom;
GVL_State.bAbortUnacked       := fbStateManager.bAbortUnacked;
GVL_State.iAbortTimeoutClient := fbStateManager.iAbortTimeoutClient;
GVL_State.iNotDoneClient      := fbStateManager.iNotDoneClient;

// ----- HMI client: requests only, always done -----
fbHmiStateReq(
    itfClient   := GVL_State.aClients[GVL_State.CLIENT_HMI],
    bOn         := GVL_Hmi.bOnCmd,
    bOff        := GVL_Hmi.bOffCmd,
    bStart      := GVL_Hmi.bStartCmd,
    bStop       := GVL_Hmi.bStopCmd,
    bHold       := GVL_Hmi.bHoldCmd,
    bAbort      := GVL_Hmi.bAbortCmd,
    uiAbortCode := HMI_ABORT_CODE,
    bReset      := GVL_Hmi.bResetCmd,
    eStateEcho  := GVL_State.eMachineState,
    bDone       := TRUE);

// ----- Drive hardware station -----
fbDriveSupply(
    itfClient        := GVL_State.aClients[GVL_State.CLIENT_DRIVE_HW],
    eMachineState    := GVL_State.eMachineState,
    wSupplyStatus    := GVL_IO.wDI_SupplyStatus,
    wSupplyCtrl      := GVL_IO.wDO_SupplyCtrl,
    awDriveStatus    := GVL_IO.awDI_DriveStatus,
    awDriveDcBusCtrl := GVL_IO.awDO_DriveDcBusCtrl);

// ----- Servo station -----
fbServoAxes(
    itfClient     := GVL_State.aClients[GVL_State.CLIENT_SERVO_AXES],
    eMachineState := GVL_State.eMachineState,
    asAxisName    := GVL_Axes.asAxisName,
    astStatus     := GVL_Axes.astStatus);
fbServoMotion(
    itfClient     := GVL_State.aClients[GVL_State.CLIENT_SERVO_MOTION],
    eMachineState := GVL_State.eMachineState,
    asAxisName    := GVL_Axes.asAxisName,
    lrSpeedCmd    := GVL_Hmi.lrSpeedCmd,
    lrAx1Offset   := GVL_Hmi.lrSlaveOffset,
    lrAx2Offset   := MODULO - GVL_Hmi.lrSlaveOffset,
    astStatus     := GVL_Axes.astStatus);
