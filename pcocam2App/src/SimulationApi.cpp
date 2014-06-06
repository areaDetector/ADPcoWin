/*
 * SimulationApi.cpp
 *
 * Revamped PCO area detector driver.
 *
 * A simulation of the PCO API library and hardware
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "SimulationApi.h"
#include "TraceStream.h"
#include "Pco.h"
#include "epicsExport.h"
#include "iocsh.h"

/**
 * Parameter names
 */
const char* SimulationApi::nameConnected = "SimConnected";
const char* SimulationApi::nameOpen = "SimOpen";
const char* SimulationApi::nameCameraType = "SimCameraType";
const char* SimulationApi::nameMaxHorzRes = "SimMaxHorzRes";
const char* SimulationApi::nameMaxVertRes = "SimMaxVertRes";
const char* SimulationApi::nameDynResolution = "SimDynResolution";
const char* SimulationApi::nameMaxBinHorz = "SimMaxBinHorz";
const char* SimulationApi::nameMaxBinVert = "SimMaxBinVert";
const char* SimulationApi::nameBinHorzStepping = "SimBinHorzStepping";
const char* SimulationApi::nameBinVertStepping = "SimBinVertStepping";
const char* SimulationApi::nameRoiHorSteps = "SimRoiHorSteps";
const char* SimulationApi::nameRoiVertSteps = "SimRoiVertSteps";
const char* SimulationApi::namePixelRate = "SimPixelRate";
const char* SimulationApi::nameConvFact = "SimConvFact";
const char* SimulationApi::nameGeneralCaps = "SimGeneralCaps";
const char* SimulationApi::nameRamSize = "SimRamSize";
const char* SimulationApi::namePageSize = "SimPageSize";
const char* SimulationApi::nameBaudRate = "SimBaudRate";
const char* SimulationApi::nameClockFrequency = "SimClockFrequency";
const char* SimulationApi::nameCamlinkLines = "SimCamlinkLines";
const char* SimulationApi::nameDataFormat = "SimDataFormat";
const char* SimulationApi::nameTransmit = "SimTransmit";
const char* SimulationApi::nameActualHorzRes = "SimActualHorzRes";
const char* SimulationApi::nameActualVertRes = "SimActualVertRes";
const char* SimulationApi::nameTimeYear = "SimTimeYear";
const char* SimulationApi::nameTimeMonth = "SimTimeMonth";
const char* SimulationApi::nameTimeDay = "SimTimeDay";
const char* SimulationApi::nameTimeHour = "SimTimeHour";
const char* SimulationApi::nameTimeMinute = "SimTimeMinute";
const char* SimulationApi::nameTimeSecond = "SimTimeSecond";
const char* SimulationApi::nameTempCcd = "SimTempCcd";
const char* SimulationApi::nameTempCamera = "SimTempCamera";
const char* SimulationApi::nameTempPsu = "SimTempPsu";
const char* SimulationApi::nameBitAlignment = "SimBitAlignment";
const char* SimulationApi::nameEdgeGlobalShutter = "SimEdgeGlobalShutter";
const char* SimulationApi::nameActualHorzBin = "SimActualHorzBin";
const char* SimulationApi::nameActualVertBin = "SimActualVertBin";
const char* SimulationApi::nameActualRoiX0 = "SimActualRoiX0";
const char* SimulationApi::nameActualRoiY0 = "SimActualRoiY0";
const char* SimulationApi::nameActualRoiX1 = "SimActualRoiX1";
const char* SimulationApi::nameActualRoiY1 = "SimActualRoiY1";
const char* SimulationApi::nameTriggerMode = "SimTriggerMode";
const char* SimulationApi::nameStorageMode = "SimStorageMode";
const char* SimulationApi::nameTimestampMode = "SimTimestampMode";
const char* SimulationApi::nameAcquireMode = "SimAcquireMode";
const char* SimulationApi::nameDelayTime = "SimDelayTime";
const char* SimulationApi::nameDelayTimebase = "SimDelayTimebase";
const char* SimulationApi::nameExposureTime = "SimExposureTime";
const char* SimulationApi::nameExposureTimebase = "SimExposureTimebase";
const char* SimulationApi::nameActualConvFact = "SimActualConvFact";
const char* SimulationApi::nameAdcOperation = "SimAdcOperation";
const char* SimulationApi::nameRecordingState = "SimRecordingState";
const char* SimulationApi::nameRecorderSubmode = "SimRecorderSubmode";
const char* SimulationApi::nameCamlinkHorzRes = "SimCamlinkHorzRes";
const char* SimulationApi::nameCamlinkVertRes = "SimCamlinkVertRes";
const char* SimulationApi::nameArmed = "SimArmed";
const char* SimulationApi::nameStateRecord = "SimStateRecord";
const char* SimulationApi::nameClearStateRecord = "SimClearStateRecord";
const char* SimulationApi::nameExternalTrigger = "SimExternalTrigger";

/**
 * Constants
 */
const int SimulationApi::edgeSetupDataLength = 1;
const int SimulationApi::edgeSetupDataType = 1;

/** State machine strings
 */
const char* SimulationApi::eventNames[] = {"ConnectionUp", "ConnectionDown",
        "Open", "Close", "StartRecording", "StopRecording", "Trigger", "Arm",
        "CancelImages"};
const char* SimulationApi::stateNames[] = {"Connected", "Open", "Disconnected",
        "Armed", "Recording"};

/**
 * Constructor
 */
SimulationApi::SimulationApi(Pco* pco, TraceStream* trace)
: DllApi(pco, trace)
, bufferQueue(DllApi::maxNumBuffers, sizeof(int))
, stateMachine(NULL)
, frameNumber(0)
{
    // Create parameters...
    this->pco->createParam(SimulationApi::nameConnected, asynParamInt32, &this->handleConnected);
    this->pco->createParam(SimulationApi::nameOpen, asynParamInt32, &this->handleOpen);
    this->pco->createParam(SimulationApi::nameCameraType, asynParamInt32, &this->handleCameraType);
    this->pco->createParam(SimulationApi::nameMaxHorzRes, asynParamInt32, &this->handleMaxHorzRes);
    this->pco->createParam(SimulationApi::nameMaxVertRes, asynParamInt32, &this->handleMaxVertRes);
    this->pco->createParam(SimulationApi::nameDynResolution, asynParamInt32, &this->handleDynResolution);
    this->pco->createParam(SimulationApi::nameMaxBinHorz, asynParamInt32, &this->handleMaxBinHorz);
    this->pco->createParam(SimulationApi::nameMaxBinVert, asynParamInt32, &this->handleMaxBinVert);
    this->pco->createParam(SimulationApi::nameBinHorzStepping, asynParamInt32, &this->handleBinHorzStepping);
    this->pco->createParam(SimulationApi::nameBinVertStepping, asynParamInt32, &this->handleBinVertStepping);
    this->pco->createParam(SimulationApi::nameRoiHorSteps, asynParamInt32, &this->handleRoiHorSteps);
    this->pco->createParam(SimulationApi::nameRoiVertSteps, asynParamInt32, &this->handleRoiVertSteps);
    this->pco->createParam(SimulationApi::namePixelRate, asynParamInt32, &this->handlePixelRate);
    this->pco->createParam(SimulationApi::nameConvFact, asynParamInt32, &this->handleConvFact);
    this->pco->createParam(SimulationApi::nameGeneralCaps, asynParamInt32, &this->handleGeneralCaps);
    this->pco->createParam(SimulationApi::nameRamSize, asynParamInt32, &this->handleRamSize);
    this->pco->createParam(SimulationApi::namePageSize, asynParamInt32, &this->handlePageSize);
    this->pco->createParam(SimulationApi::nameBaudRate, asynParamInt32, &this->handleBaudRate);
    this->pco->createParam(SimulationApi::nameClockFrequency, asynParamInt32, &this->handleClockFrequency);
    this->pco->createParam(SimulationApi::nameCamlinkLines, asynParamInt32, &this->handleCamlinkLines);
    this->pco->createParam(SimulationApi::nameDataFormat, asynParamInt32, &this->handleDataFormat);
    this->pco->createParam(SimulationApi::nameTransmit, asynParamInt32, &this->handleTransmit);
    this->pco->createParam(SimulationApi::nameActualHorzRes, asynParamInt32, &this->handleActualHorzRes);
    this->pco->createParam(SimulationApi::nameActualVertRes, asynParamInt32, &this->handleActualVertRes);
    this->pco->createParam(SimulationApi::nameTimeYear, asynParamInt32, &this->handleTimeYear);
    this->pco->createParam(SimulationApi::nameTimeMonth, asynParamInt32, &this->handleTimeMonth);
    this->pco->createParam(SimulationApi::nameTimeDay, asynParamInt32, &this->handleTimeDay);
    this->pco->createParam(SimulationApi::nameTimeHour, asynParamInt32, &this->handleTimeHour);
    this->pco->createParam(SimulationApi::nameTimeMinute, asynParamInt32, &this->handleTimeMinute);
    this->pco->createParam(SimulationApi::nameTimeSecond, asynParamInt32, &this->handleTimeSecond);
    this->pco->createParam(SimulationApi::nameTempCcd, asynParamInt32, &this->handleTempCcd);
    this->pco->createParam(SimulationApi::nameTempCamera, asynParamInt32, &this->handleTempCamera);
    this->pco->createParam(SimulationApi::nameTempPsu, asynParamInt32, &this->handleTempPsu);
    this->pco->createParam(SimulationApi::nameBitAlignment, asynParamInt32, &this->handleBitAlignment);
    this->pco->createParam(SimulationApi::nameEdgeGlobalShutter, asynParamInt32, &this->handleEdgeGlobalShutter);
    this->pco->createParam(SimulationApi::nameActualHorzBin, asynParamInt32, &this->handleActualHorzBin);
    this->pco->createParam(SimulationApi::nameActualVertBin, asynParamInt32, &this->handleActualVertBin);
    this->pco->createParam(SimulationApi::nameActualRoiX0, asynParamInt32, &this->handleActualRoiX0);
    this->pco->createParam(SimulationApi::nameActualRoiY0, asynParamInt32, &this->handleActualRoiY0);
    this->pco->createParam(SimulationApi::nameActualRoiX1, asynParamInt32, &this->handleActualRoiX1);
    this->pco->createParam(SimulationApi::nameActualRoiY1, asynParamInt32, &this->handleActualRoiY1);
    this->pco->createParam(SimulationApi::nameTriggerMode, asynParamInt32, &this->handleTriggerMode);
    this->pco->createParam(SimulationApi::nameStorageMode, asynParamInt32, &this->handleStorageMode);
    this->pco->createParam(SimulationApi::nameTimestampMode, asynParamInt32, &this->handleTimestampMode);
    this->pco->createParam(SimulationApi::nameAcquireMode, asynParamInt32, &this->handleAcquireMode);
    this->pco->createParam(SimulationApi::nameDelayTime, asynParamInt32, &this->handleDelayTime);
    this->pco->createParam(SimulationApi::nameExposureTime, asynParamInt32, &this->handleExposureTime);
    this->pco->createParam(SimulationApi::nameDelayTimebase, asynParamInt32, &this->handleDelayTimebase);
    this->pco->createParam(SimulationApi::nameExposureTimebase, asynParamInt32, &this->handleExposureTimebase);
    this->pco->createParam(SimulationApi::nameActualConvFact, asynParamInt32, &this->handleActualConvFact);
    this->pco->createParam(SimulationApi::nameAdcOperation, asynParamInt32, &this->handleAdcOperation);
    this->pco->createParam(SimulationApi::nameRecordingState, asynParamInt32, &this->handleRecordingState);
    this->pco->createParam(SimulationApi::nameRecorderSubmode, asynParamInt32, &this->handleRecorderSubmode);
    this->pco->createParam(SimulationApi::nameCamlinkHorzRes, asynParamInt32, &this->handleCamlinkHorzRes);
    this->pco->createParam(SimulationApi::nameCamlinkVertRes, asynParamInt32, &this->handleCamlinkVertRes);
    this->pco->createParam(SimulationApi::nameArmed, asynParamInt32, &this->handleArmed);
    this->pco->createParam(SimulationApi::nameStateRecord, asynParamOctet, &this->handleStateRecord);
    this->pco->createParam(SimulationApi::nameClearStateRecord, asynParamInt32, &this->handleClearStateRecord);
    this->pco->createParam(SimulationApi::nameExternalTrigger, asynParamInt32, &this->handleExternalTrigger);
    // ...and initialise them
    this->pco->setIntegerParam(this->handleConnected, true);
    this->pco->setIntegerParam(this->handleOpen, false);
    this->pco->setIntegerParam(this->handleCameraType, DllApi::cameraType4000);
    this->pco->setIntegerParam(this->handleMaxHorzRes, 1280);
    this->pco->setIntegerParam(this->handleMaxVertRes, 1024);
    this->pco->setIntegerParam(this->handleDynResolution, 14);
    this->pco->setIntegerParam(this->handleMaxBinHorz, 4);
    this->pco->setIntegerParam(this->handleMaxBinVert, 4);
    this->pco->setIntegerParam(this->handleBinHorzStepping, 1);
    this->pco->setIntegerParam(this->handleBinVertStepping, 1);
    this->pco->setIntegerParam(this->handleRoiHorSteps, 1);
    this->pco->setIntegerParam(this->handleRoiVertSteps, 1);
    this->pco->setIntegerParam(this->handlePixelRate, 4000000);
    this->pco->setIntegerParam(this->handleConvFact, 100);
    this->pco->setIntegerParam(this->handleGeneralCaps, 0);
    this->pco->setIntegerParam(this->handleRamSize, 0);
    this->pco->setIntegerParam(this->handlePageSize, 0);
    this->pco->setIntegerParam(this->handleBaudRate, 0);
    this->pco->setIntegerParam(this->handleClockFrequency, 0);
    this->pco->setIntegerParam(this->handleCamlinkLines, 0);
    this->pco->setIntegerParam(this->handleDataFormat, 0);
    this->pco->setIntegerParam(this->handleTransmit, 0);
    this->pco->setIntegerParam(this->handleActualHorzRes, 1280);
    this->pco->setIntegerParam(this->handleActualVertRes, 1024);
    this->pco->setIntegerParam(this->handleTimeYear, 0);
    this->pco->setIntegerParam(this->handleTimeMonth, 0);
    this->pco->setIntegerParam(this->handleTimeDay, 0);
    this->pco->setIntegerParam(this->handleTimeHour, 0);
    this->pco->setIntegerParam(this->handleTimeMinute, 0);
    this->pco->setIntegerParam(this->handleTimeSecond, 0);
    this->pco->setIntegerParam(this->handleTempCcd, 21);
    this->pco->setIntegerParam(this->handleTempCamera, 22);
    this->pco->setIntegerParam(this->handleTempPsu, 23);
    this->pco->setIntegerParam(this->handleBitAlignment, 0);
    this->pco->setIntegerParam(this->handleEdgeGlobalShutter, 0);
    this->pco->setIntegerParam(this->handleActualHorzBin, 1);
    this->pco->setIntegerParam(this->handleActualVertBin, 1);
    this->pco->setIntegerParam(this->handleActualRoiX0, 0);
    this->pco->setIntegerParam(this->handleActualRoiY0, 0);
    this->pco->setIntegerParam(this->handleActualRoiX1, 1280);
    this->pco->setIntegerParam(this->handleActualRoiY1, 1024);
    this->pco->setIntegerParam(this->handleTriggerMode, DllApi::triggerAuto);
    this->pco->setIntegerParam(this->handleStorageMode, DllApi::storageModeRecorder);
    this->pco->setIntegerParam(this->handleTimestampMode, DllApi::timestampModeOff);
    this->pco->setIntegerParam(this->handleAcquireMode, DllApi::acquireModeAuto);
    this->pco->setIntegerParam(this->handleDelayTime, 100);
    this->pco->setIntegerParam(this->handleDelayTimebase, DllApi::timebaseMilliseconds);
    this->pco->setIntegerParam(this->handleExposureTime, 100);
    this->pco->setIntegerParam(this->handleExposureTimebase, DllApi::timebaseMilliseconds);
    this->pco->setIntegerParam(this->handleActualConvFact, 100);
    this->pco->setIntegerParam(this->handleAdcOperation, DllApi::adcModeSingle);
    this->pco->setIntegerParam(this->handleRecordingState, DllApi::recorderStateOff);
    this->pco->setIntegerParam(this->handleRecorderSubmode, 0);
    this->pco->setIntegerParam(this->handleCamlinkHorzRes, 1280);
    this->pco->setIntegerParam(this->handleCamlinkVertRes, 1024);
    this->pco->setIntegerParam(this->handleArmed, false);
    this->pco->setStringParam(this->handleStateRecord, "");
    this->pco->setIntegerParam(this->handleClearStateRecord, 0);
    this->pco->setIntegerParam(this->handleExternalTrigger, 0);
    // Initialise the buffers
    for(int i=0; i<DllApi::maxNumBuffers; i++)
    {
        this->buffers[i].status = 0;
        this->buffers[i].buffer = NULL;
    }
    // Create the state machine
    this->stateMachine = new StateMachine("SimulationApi", this->pco,
            this->handleStateRecord, this, SimulationApi::stateConnected,
            SimulationApi::stateNames, SimulationApi::eventNames, trace);
}

/**
 * Destructor
 */
SimulationApi::~SimulationApi()
{
    delete this->stateMachine;
}

/**
 * Post a request
 */
void SimulationApi::post(Request req)
{
    this->stateMachine->post(req);
}

/**
 * Handle state transitions.
 */
int SimulationApi::doTransition(StateMachine* machine, int state, int event)
{
    if(machine == this->stateMachine)
    {
        switch(state)
        {
        case stateConnected:
            if(event == SimulationApi::requestConnectionDown)
            {
                state = SimulationApi::stateDisconnected;
            }
            else if(event == SimulationApi::requestOpen)
            {
                state = SimulationApi::stateOpen;
            }
            break;
        case stateOpen:
            if(event == SimulationApi::requestConnectionDown)
            {
                state = SimulationApi::stateDisconnected;
            }
            else if(event == SimulationApi::requestClose)
            {
                state = SimulationApi::stateConnected;
            }
            else if(event == SimulationApi::requestArm)
            {
                state = SimulationApi::stateArmed;
            }
            break;
        case stateDisconnected:
            if(event == SimulationApi::requestConnectionUp)
            {
                state = SimulationApi::stateConnected;
            }
            break;
        case stateArmed:
            if(event == SimulationApi::requestConnectionDown)
            {
                state = SimulationApi::stateDisconnected;
            }
            else if(event == SimulationApi::requestClose)
            {
                state = SimulationApi::stateConnected;
            }
            else if(event == SimulationApi::requestStartRecording)
            {
                this->startTriggerTimer();
                this->frameNumber = 0;
                state = SimulationApi::stateRecording;
            }
            else if(event == SimulationApi::requestCancelImages)
            {
                state = SimulationApi::stateOpen;
            }
            break;
        case stateRecording:
            if(event == SimulationApi::requestConnectionDown)
            {
                this->stateMachine->stopTimer();
                state = SimulationApi::stateDisconnected;
            }
            else if(event == SimulationApi::requestClose)
            {
                this->stateMachine->stopTimer();
                state = SimulationApi::stateConnected;
            }
            else if(event == SimulationApi::requestStopRecording)
            {
                this->stateMachine->stopTimer();
                state = SimulationApi::stateArmed;
            }
            else if(event == SimulationApi::requestTrigger)
            {
                this->generateFrame();
                this->startTriggerTimer();
                state = SimulationApi::stateRecording;
            }
            break;
        }
    }
    return state;
}

/**
 * Generate a simulated frame
 */
void SimulationApi::generateFrame()
{
    // Advance the frame number
    this->frameNumber++;
    if(this->frameNumber >= 100000000)
    {
        this->frameNumber = 0;
    }
    // Is there a buffer available in the queue
    if(this->bufferQueue.pending() > 0)
    {
        int bufferNumber;
        this->bufferQueue.tryReceive(&bufferNumber, sizeof(int));
        // Fill the frame with a pattern
        int xSize;
        int ySize;
        this->pco->getIntegerParam(this->handleActualHorzRes, &xSize);
        this->pco->getIntegerParam(this->handleActualVertRes, &ySize);
        for(int x=0; x<xSize; x++)
        {
            for(int y=0; y<ySize; y++)
            {
                bool dark = true;
                if(((x / 16) & 1) != 0)
                {
                    dark = !dark;
                }
                if(((y / 16) & 1) != 0)
                {
                    dark = !dark;
                }
                this->buffers[bufferNumber].buffer[y*xSize+x] = (dark ? 15 : 255);
            }
        }
        // Plant the BCD time stamp if enabled
        int timestampMode;
        this->pco->getIntegerParam(this->handleTimestampMode, &timestampMode);
        if(timestampMode == DllApi::timestampModeBinary ||
                timestampMode == DllApi::timestampModeBinaryAndAscii)
        {
            // The frame number
            int dynResolution;
            this->pco->getIntegerParam(this->handleDynResolution, &dynResolution);
            int shiftLowBcd = Pco::bitsPerShortWord - dynResolution;
            int shiftHighBcd = shiftLowBcd + Pco::bitsPerNybble;
            unsigned long n = this->frameNumber;
            unsigned long divisor = Pco::bcdDigitValue * Pco::bcdDigitValue *
                    Pco::bcdDigitValue * Pco::bcdDigitValue * Pco::bcdDigitValue *
                    Pco::bcdDigitValue * Pco::bcdDigitValue * Pco::bcdDigitValue;
            for(int i=0; i<Pco::bcdPixelLength; i++)
            {
                unsigned long n0 = n / divisor;
                n -= n0 * divisor;
                divisor /= 10;
                unsigned long n1 = n / divisor;
                n -= n1 * divisor;
                divisor /= 10;
                unsigned short pixel = (unsigned short)((n1 << shiftLowBcd) | (n0 << shiftHighBcd));
                this->buffers[bufferNumber].buffer[Pco::bcdPixelLength-i-1] = pixel;
            }
            // TODO: The time
        }
        // Give the buffer back to the driver
        this->buffers[bufferNumber].status |= DllApi::statusDllEventSet;
        this->pco->frameReceived(bufferNumber);
    }
}

/**
 * Start the trigger timer if the trigger mode is automatic
 */
void SimulationApi::startTriggerTimer()
{
    int triggerMode;
    this->pco->getIntegerParam(this->handleTriggerMode, &triggerMode);
    if(triggerMode == DllApi::triggerAuto)
    {
        // In auto trigger mode, start the trigger timer
        int delay;
        int delayTimebase;
        int exposure;
        int exposureTimebase;
        this->pco->getIntegerParam(this->handleDelayTime, &delay);
        this->pco->getIntegerParam(this->handleDelayTimebase, &delayTimebase);
        this->pco->getIntegerParam(this->handleExposureTime, &exposure);
        this->pco->getIntegerParam(this->handleExposureTimebase, &exposureTimebase);
        double period = (double)delay / DllApi::timebaseScaleFactor[delayTimebase] +
                (double)exposure / DllApi::timebaseScaleFactor[exposureTimebase];
        this->stateMachine->startTimer(period, SimulationApi::requestTrigger);
    }
}

/**
 * Changes to asyn parameters
 */
void SimulationApi::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int parameter = pasynUser->reason;
    if(parameter == this->handleConnected)
    {
        if(value)
        {
            this->post(SimulationApi::requestConnectionUp);
        }
        else
        {
            this->post(SimulationApi::requestConnectionDown);
        }
    }
    else if(parameter == this->handleExternalTrigger)
    {
        int triggerMode;
        this->pco->getIntegerParam(this->handleTriggerMode, &triggerMode);
        if(triggerMode == DllApi::triggerExternal ||
                triggerMode == triggerExternalExposure)
        {
            this->post(SimulationApi::requestTrigger);
        }
    }
}

/**
 * Connect to the camera
 * Camera number is currently ignored.
 */
int SimulationApi::doOpenCamera(Handle* handle, unsigned short camNum)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && !open)
    {
        this->pco->setIntegerParam(this->handleOpen, true);
        this->post(SimulationApi::requestOpen);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Disconnect from the camera
 */
int SimulationApi::doCloseCamera(Handle handle)
{
    this->pco->setIntegerParam(this->handleOpen, false);
    this->post(SimulationApi::requestClose);
    return DllApi::errorNone;
}

/**
 * Reboot the camera
 */
int SimulationApi::doRebootCamera(Handle handle)
{
    return DllApi::errorNone;
}

/**
 * Get general information from the camera
 */
int SimulationApi::doGetGeneral(Handle handle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the camera type information
 */
int SimulationApi::doGetCameraType(Handle handle, unsigned short* camType)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int localCamType;
        this->pco->getIntegerParam(this->handleCameraType, &localCamType);
        *camType = localCamType;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get sensor information
 */
int SimulationApi::doGetSensorStruct(Handle handle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get camera description
 */
int SimulationApi::doGetCameraDescription(Handle handle, Description* description)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleMaxHorzRes, &v);
        description->maxHorzRes = (unsigned short)v;
        this->pco->getIntegerParam(this->handleMaxVertRes, &v);
        description->maxVertRes = (unsigned short)v;
        this->pco->getIntegerParam(this->handleDynResolution, &v);
        description->dynResolution = (unsigned short)v;
        this->pco->getIntegerParam(this->handleMaxBinHorz, &v);
        description->maxBinHorz = (unsigned short)v;
        this->pco->getIntegerParam(this->handleMaxBinVert, &v);
        description->maxBinVert = (unsigned short)v;
        this->pco->getIntegerParam(this->handleBinHorzStepping, &v);
        description->binHorzStepping = (unsigned short)v;
        this->pco->getIntegerParam(this->handleBinVertStepping, &v);
        description->binVertStepping = (unsigned short)v;
        this->pco->getIntegerParam(this->handleRoiHorSteps, &v);
        description->roiHorSteps = (unsigned short)v;
        this->pco->getIntegerParam(this->handleRoiVertSteps, &v);
        description->roiVertSteps = (unsigned short)v;
        this->pco->getIntegerParam(this->handlePixelRate, &v);
        description->pixelRate[0] = (unsigned long)v;
        description->pixelRate[1] = 0;
        description->pixelRate[2] = 0;
        description->pixelRate[3] = 0;
        this->pco->getIntegerParam(this->handleConvFact, &v);
        description->convFact = (unsigned short)v;
        this->pco->getIntegerParam(this->handleGeneralCaps, &v);
        description->generalCaps = (unsigned long)v;
        description->minCoolingSetpoint = 0;
        description->maxCoolingSetpoint = 0;
        description->defaultCoolingSetpoint = 0;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get camera storage information
 */
int SimulationApi::doGetStorageStruct(Handle handle, unsigned long* ramSize,
        unsigned int* pageSize)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleRamSize, &v);
        *ramSize = (unsigned long)v;
        this->pco->getIntegerParam(this->handlePageSize, &v);
        *pageSize = (unsigned int)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get camera recording information
 */
int SimulationApi::doGetRecordingStruct(Handle handle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Reset the camera's settings
 */
int SimulationApi::doResetSettingsToDefault(Handle handle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        // TODO: Actually reset the settings
        this->post(SimulationApi::requestStopRecording);
        this->pco->setIntegerParam(this->handleRecordingState, DllApi::recorderStateOff);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the camera's transfer parameters
 */
int SimulationApi::doGetTransferParameters(Handle handle, Transfer* transfer)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleBaudRate, &v);
        transfer->baudRate = (unsigned long)v;
        this->pco->getIntegerParam(this->handleClockFrequency, &v);
        transfer->clockFrequency = (unsigned long)v;
        this->pco->getIntegerParam(this->handleCamlinkLines, &v);
        transfer->camlinkLines = (unsigned long)v;
        this->pco->getIntegerParam(this->handleDataFormat, &v);
        transfer->dataFormat = (unsigned long)v;
        this->pco->getIntegerParam(this->handleTransmit, &v);
        transfer->transmit = (unsigned long)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the camera's transfer parameters
 */
int SimulationApi::doSetTransferParameters(Handle handle, Transfer* transfer)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleBaudRate, (int)transfer->baudRate);
        this->pco->setIntegerParam(this->handleClockFrequency, (int)transfer->clockFrequency);
        this->pco->setIntegerParam(this->handleCamlinkLines, (int)transfer->camlinkLines);
        this->pco->setIntegerParam(this->handleDataFormat, (int)transfer->dataFormat);
        this->pco->setIntegerParam(this->handleTransmit, (int)transfer->transmit);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * The the camera's current and maximum resolutions
 */
int SimulationApi::doGetSizes(Handle handle, Sizes* sizes)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleActualHorzRes, &v);
        sizes->xResActual = (unsigned short)v;
        this->pco->getIntegerParam(this->handleActualVertRes, &v);
        sizes->yResActual = (unsigned short)v;
        this->pco->getIntegerParam(this->handleMaxHorzRes, &v);
        sizes->xResMaximum = (unsigned short)v;
        this->pco->getIntegerParam(this->handleMaxVertRes, &v);
        sizes->yResMaximum = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the camera's date and time
 */
int SimulationApi::doSetDateTime(Handle handle, struct tm* currentTime)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleTimeYear, (int)currentTime->tm_year);
        this->pco->setIntegerParam(this->handleTimeMonth, (int)currentTime->tm_mon);
        this->pco->setIntegerParam(this->handleTimeDay, (int)currentTime->tm_mday);
        this->pco->setIntegerParam(this->handleTimeHour, (int)currentTime->tm_hour);
        this->pco->setIntegerParam(this->handleTimeMinute, (int)currentTime->tm_min);
        this->pco->setIntegerParam(this->handleTimeSecond, (int)currentTime->tm_sec);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get camera temperatures
 */
int SimulationApi::doGetTemperature(Handle handle, short* ccd,
        short* camera, short* psu)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleTempCcd, &v);
        *ccd = (short)v;
        this->pco->getIntegerParam(this->handleTempCamera, &v);
        *camera = (short)v;
        this->pco->getIntegerParam(this->handleTempPsu, &v);
        *psu = (short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the camera's cooling setpoint
 */
int SimulationApi::doSetCoolingSetpoint(Handle handle, short setPoint)
{
    return DllApi::errorNone;
}

/**
 * Get the camera's cooling setpoint.
 */
int SimulationApi::doGetCoolingSetpoint(Handle handle, short* setPoint)
{
    return DllApi::errorNone;
}

/**
 * Set the camera's operating pixel rate
 */
int SimulationApi::doSetPixelRate(Handle handle, unsigned long pixRate)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handlePixelRate, (int)pixRate);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the camera's operating pixel rate
 */
int SimulationApi::doGetPixelRate(Handle handle, unsigned long* pixRate)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handlePixelRate, &v);
        *pixRate = (unsigned long)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the operating bit alignment
 */
int SimulationApi::doGetBitAlignment(Handle handle, unsigned short* bitAlignment)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleBitAlignment, &v);
        *bitAlignment = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the operating bit alignment
 */
int SimulationApi::doSetBitAlignment(Handle handle, unsigned short bitAlignment)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleBitAlignment, (int)bitAlignment);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Return an Edge's camera setup information
 */
int SimulationApi::doGetCameraSetup(Handle handle, unsigned short* setupType,
        unsigned long* setupData, unsigned short* setupDataLen)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleEdgeGlobalShutter, &v);
        *setupType = (unsigned short)SimulationApi::edgeSetupDataType;
        *setupData = (unsigned long)v;
        *setupDataLen = (unsigned short)SimulationApi::edgeSetupDataLength;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the binning parameters.
 */
int SimulationApi::doSetBinning(Handle handle, unsigned short binHorz, unsigned short binVert)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleActualHorzBin, (int)binHorz);
        this->pco->setIntegerParam(this->handleActualVertBin, (int)binVert);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the binning parameters.
 */
int SimulationApi::doGetBinning(Handle handle, unsigned short* binHorz, unsigned short* binVert)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleActualHorzBin, &v);
        *binHorz = (unsigned short)v;
        this->pco->getIntegerParam(this->handleActualVertBin, &v);
        *binVert = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the region of interest parameters
 */
int SimulationApi::doSetRoi(Handle handle, unsigned short x0, unsigned short y0,
        unsigned short x1, unsigned short y1)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleActualRoiX0, (int)x0);
        this->pco->setIntegerParam(this->handleActualRoiY0, (int)y0);
        this->pco->setIntegerParam(this->handleActualRoiX1, (int)x1);
        this->pco->setIntegerParam(this->handleActualRoiY1, (int)y1);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the region of interest parameters
 */
int SimulationApi::doGetRoi(Handle handle, unsigned short* x0, unsigned short* y0,
        unsigned short* x1, unsigned short* y1)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleActualRoiX0, &v);
        *x0 = (unsigned short)v;
        this->pco->getIntegerParam(this->handleActualRoiY0, &v);
        *y0 = (unsigned short)v;
        this->pco->getIntegerParam(this->handleActualRoiX1, &v);
        *x1 = (unsigned short)v;
        this->pco->getIntegerParam(this->handleActualRoiY1, &v);
        *y1 = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the trigger mode
 */
int SimulationApi::doSetTriggerMode(Handle handle, unsigned short mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleTriggerMode, (int)mode);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the trigger mode
 */
int SimulationApi::doGetTriggerMode(Handle handle, unsigned short* mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleTriggerMode, &v);
        *mode = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the storage mode
 */
int SimulationApi::doSetStorageMode(Handle handle, unsigned short mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleStorageMode, (int)mode);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the storage mode
 */
int SimulationApi::doGetStorageMode(Handle handle, unsigned short* mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleStorageMode, &v);
        *mode = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the time stamp mode
 */
int SimulationApi::doSetTimestampMode(Handle handle, unsigned short mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleTimestampMode, (int)mode);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the time stamp mode
 */
int SimulationApi::doGetTimestampMode(Handle handle, unsigned short* mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleTimestampMode, &v);
        *mode = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the acquire mode
 */
int SimulationApi::doSetAcquireMode(Handle handle, unsigned short mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleAcquireMode, (int)mode);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the acquire mode
 */
int SimulationApi::doGetAcquireMode(Handle handle, unsigned short* mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleAcquireMode, &v);
        *mode = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the delay and exposure times
 */
int SimulationApi::doSetDelayExposureTime(Handle handle, unsigned long delay,
        unsigned long exposure, unsigned short timeBaseDelay,
        unsigned short timeBaseExposure)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleDelayTime, (int)delay);
        this->pco->setIntegerParam(this->handleDelayTimebase, (int)timeBaseDelay);
        this->pco->setIntegerParam(this->handleExposureTime, (int)exposure);
        this->pco->setIntegerParam(this->handleExposureTimebase, (int)timeBaseExposure);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the delay and exposure times
 */
int SimulationApi::doGetDelayExposureTime(Handle handle, unsigned long* delay,
        unsigned long* exposure, unsigned short* timeBaseDelay,
        unsigned short* timeBaseExposure)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleDelayTime, &v);
        *delay = (unsigned long)v;
        this->pco->getIntegerParam(this->handleDelayTimebase, &v);
        *timeBaseDelay = (unsigned short)v;
        this->pco->getIntegerParam(this->handleExposureTime, &v);
        *exposure = (unsigned long)v;
        this->pco->getIntegerParam(this->handleExposureTimebase, &v);
        *timeBaseExposure = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the sensor gain
 */
int SimulationApi::doSetConversionFactor(Handle handle, unsigned short factor)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleActualConvFact, (int)factor);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the ADC operating mode
 */
int SimulationApi::doGetAdcOperation(Handle handle, unsigned short* mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleAdcOperation, &v);
        *mode = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the ADC operating mode
 */
int SimulationApi::doSetAdcOperation(Handle handle, unsigned short mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleAdcOperation, (int)mode);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the camera's recording state
 */
int SimulationApi::doGetRecordingState(Handle handle, unsigned short* state)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleRecordingState, &v);
        *state = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the camera's recording state
 */
int SimulationApi::doSetRecordingState(Handle handle, unsigned short state)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        if(state == DllApi::recorderStateOn)
        {
            this->post(SimulationApi::requestStartRecording);
        }
        else
        {
            this->post(SimulationApi::requestStopRecording);
        }
        this->pco->setIntegerParam(this->handleRecordingState, (int)state);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the recorder submode
 */
int SimulationApi::doGetRecorderSubmode(Handle handle, unsigned short* mode)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int v;
        this->pco->getIntegerParam(this->handleRecorderSubmode, &v);
        *mode = (unsigned short)v;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Allocate a buffer
 */
int SimulationApi::doAllocateBuffer(Handle handle, short* bufferNumber, unsigned long size,
        unsigned short** buffer, Handle* eventHandle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        // Identify a buffer number
        bool found = *bufferNumber != DllApi::bufferUnallocated;
        for(int i=0; i<DllApi::maxNumBuffers && !found; i++)
        {
            if((this->buffers[i].status & DllApi::statusDllBufferAllocated) == 0)
            {
                found = true;
                *bufferNumber = (short)i;
                this->buffers[i].status = DllApi::statusDllBufferAllocated;
            }
        }
        if(found)
        {
            // Allocate the memory
            if(*buffer == NULL)
            {
                this->buffers[*bufferNumber].buffer = new unsigned short[size/sizeof(unsigned short)];
                *buffer = this->buffers[*bufferNumber].buffer;
            }
            else
            {
                this->buffers[*bufferNumber].buffer = *buffer;
                this->buffers[*bufferNumber].status |= DllApi::statusDllExternalBuffer;
            }
            // Create the event (the simulation doesn't use this, so just indicate created)
            if(*eventHandle == NULL)
            {
                this->buffers[*bufferNumber].status |= DllApi::statusDllEventCreated;
            }
            // Return result
            result = DllApi::errorNone;
        }
    }
    return result;
}

/**
 * Cancel all image buffers
 */
int SimulationApi::doCancelImages(Handle handle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        // Empty the buffer queue
        while(this->bufferQueue.pending() > 0)
        {
            int bufferNumber;
            this->bufferQueue.tryReceive(&bufferNumber, sizeof(int));
        }
        this->post(SimulationApi::requestCancelImages);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Set the image parameters for the image buffer transfer inside the CamLink and GigE interface.
 * While using CamLink or GigE this function must be called, before the user tries to get images
 * from the camera and the sizes have changed. With all other interfaces this is a dummy call.
 */
int SimulationApi::doCamlinkSetImageParameters(Handle handle, unsigned short xRes, unsigned short yRes)
{
    this->pco->setIntegerParam(this->handleCamlinkHorzRes, (int)xRes);
    this->pco->setIntegerParam(this->handleCamlinkHorzRes, (int)yRes);
    return DllApi::errorNone;
}

/**
 * Arm the camera ready for taking images.
 */
int SimulationApi::doArm(Handle handle)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        this->pco->setIntegerParam(this->handleArmed, (int)true);
        this->post(SimulationApi::requestArm);
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Add a buffer to the receive queue.
 */
int SimulationApi::doAddBufferEx(Handle handle, unsigned long firstImage, unsigned long lastImage,
        short bufferNumber, unsigned short xRes, unsigned short yRes,
        unsigned short bitRes)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        // Are the parameters correct?
        int actualXRes;
        int actualYRes;
        this->pco->getIntegerParam(this->handleActualHorzRes, &actualXRes);
        this->pco->getIntegerParam(this->handleActualVertRes, &actualYRes);
        if((int)xRes == actualXRes && (int)yRes == actualYRes && firstImage==0 && lastImage==0)
        {
            // Put the buffer on the queue
            int v = bufferNumber;
            this->bufferQueue.trySend(&v, sizeof(int));
        }
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Get the status of a buffer
 */
int SimulationApi::doGetBufferStatus(Handle handle, short bufferNumber,
        unsigned long* statusDll, unsigned long* statusDrv)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        *statusDll = this->buffers[bufferNumber].status;
        *statusDrv = 0;
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Force a trigger.
 */
int SimulationApi::doForceTrigger(Handle handle, unsigned short* triggered)
{
    int result = DllApi::errorAny;
    int connected;
    int open;
    this->pco->getIntegerParam(this->handleConnected, &connected);
    this->pco->getIntegerParam(this->handleOpen, &open);
    if(connected && open)
    {
        int triggerMode;
        this->pco->getIntegerParam(this->handleTriggerMode, &triggerMode);
        if(triggerMode == DllApi::triggerSoftware ||
                triggerMode == DllApi::triggerExternal)
        {
            this->post(SimulationApi::requestTrigger);
        }
        // TODO: Fill in the triggered return parameter
        result = DllApi::errorNone;
    }
    return result;
}

/**
 * Free a buffer.
 */
int SimulationApi::doFreeBuffer(Handle handle, short bufferNumber)
{
    if((this->buffers[bufferNumber].status & DllApi::statusDllExternalBuffer) == 0 &&
        this->buffers[bufferNumber].buffer != NULL)
    {
        delete[] this->buffers[bufferNumber].buffer;
        this->buffers[bufferNumber].buffer = NULL;
    }
    return DllApi::errorNone;
}

/**
 * Get the active RAM segment
 */
int SimulationApi::doGetActiveRamSegment(Handle handle, unsigned short* segment)
{
    *segment = 1;
    return DllApi::errorNone;
}

/**
 * Get the number of images in a segment
 */
int SimulationApi::doGetNumberOfImagesInSegment(Handle handle, unsigned short segment,
        unsigned long* validImageCount, unsigned long* maxImageCount)
{
    *validImageCount = 0;
    *maxImageCount = 0;
    return DllApi::errorNone;
}

/**
 * Set the active lookup table
 */
int SimulationApi::doSetActiveLookupTable(Handle handle, unsigned short identifier)
{
    return DllApi::errorNone;
}

// C entry point for iocinit
extern "C" int simulationApiConfig(const char* portName)
{
    Pco* pco = Pco::getPco(portName);
    if(pco != NULL)
    {
        new SimulationApi(pco, &pco->apiTrace);
    }
    else
    {
        printf("simulationApiConfig: Pco \"%s\" not found\n", portName);
    }
    return asynSuccess;
}
static const iocshArg simulationApiConfigArg0 = {"Port Name", iocshArgString};
static const iocshArg* const simulationApiConfigArgs[] =
    {&simulationApiConfigArg0};
static const iocshFuncDef configSimulationApi =
    {"simulationApiConfig", 1, simulationApiConfigArgs};
static void configSimulationApiCallFunc(const iocshArgBuf *args)
{
    simulationApiConfig(args[0].sval);
}

/** Register the functions */
static void simulationApiRegister(void)
{
    iocshRegister(&configSimulationApi, configSimulationApiCallFunc);
}

extern "C" { epicsExportRegistrar(simulationApiRegister); }


