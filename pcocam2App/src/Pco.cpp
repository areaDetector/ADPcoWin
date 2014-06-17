/* Pco.h
 *
 * Revamped PCO area detector driver.
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */
#include "Pco.h"
#include "DllApi.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include "epicsExport.h"
#include "epicsThread.h"
#include "iocsh.h"
#include "db_access.h"
#include <iostream>

/** Parameter names */
const char* Pco::namePixRate = "PCO_PIX_RATE";
const char* Pco::nameAdcMode = "PCO_ADC_MODE";
const char* Pco::nameCamRamUse = "PCO_CAM_RAM_USE";
const char* Pco::nameElectronicsTemp = "PCO_ELECTRONICS_TEMP";
const char* Pco::namePowerTemp = "PCO_POWER_TEMP";
const char* Pco::nameStorageMode = "PCO_STORAGE_MODE";
const char* Pco::nameRecorderSubmode = "PCO_RECORDER_SUBMODE";
const char* Pco::nameTimestampMode = "PCO_TIMESTAMP_MODE";
const char* Pco::nameAcquireMode = "PCO_ACQUIRE_MODE";
const char* Pco::nameArmMode = "PCO_ARM_MODE";
const char* Pco::nameDelayTime = "PCO_DELAY_TIME";
const char* Pco::nameImageNumber = "PCO_IMAGE_NUMBER";
const char* Pco::nameCameraSetup = "PCO_CAMERA_SETUP";
const char* Pco::nameBitAlignment = "PCO_BIT_ALIGNMENT";
const char* Pco::nameStateRecord = "PCO_STATERECORD";
const char* Pco::nameClearStateRecord = "PCO_CLEARSTATERECORD";
const char* Pco::nameOutOfNDArrays = "PCO_OUTOFNDARRAYS";
const char* Pco::nameBufferQueueReadFailures = "PCO_BUFFERQUEUEREADFAILURES";
const char* Pco::nameBuffersWithNoData = "PCO_BUFFERSWITHNODATA";
const char* Pco::nameMisplacedBuffers = "PCO_MISPLACEDBUFFERS";
const char* Pco::nameMissingFrames = "PCO_MISSINGFRAMES";
const char* Pco::nameDriverLibraryErrors = "PCO_DRIVERLIBRARYERRORS";
const char* Pco::nameHwBinX = "PCO_HWBINX";
const char* Pco::nameHwBinY = "PCO_HWBINY";
const char* Pco::nameHwRoiX1 = "PCO_HWROIX1";
const char* Pco::nameHwRoiY1 = "PCO_HWROIY1";
const char* Pco::nameHwRoiX2 = "PCO_HWROIX2";
const char* Pco::nameHwRoiY2 = "PCO_HWROIY2";
const char* Pco::nameXCamSize = "PCO_XCAMSIZE";
const char* Pco::nameYCamSize = "PCO_YCAMSIZE";
const char* Pco::nameCamlinkClock = "PCO_CAMLINKCLOCK";
const char* Pco::nameMinCoolingSetpoint = "PCO_MINCOOLINGSETPOINT";
const char* Pco::nameMaxCoolingSetpoint = "PCO_MAXCOOLINGSETPOINT";
const char* Pco::nameDefaultCoolingSetpoint = "PCO_DEFAULTCOOLINGSETPOINT";
const char* Pco::nameCoolingSetpoint = "PCO_COOLINGSETPOINT";
const char* Pco::nameDelayTimeMin = "PCO_DELAYTIMEMIN";
const char* Pco::nameDelayTimeMax = "PCO_DELAYTIMEMAX";
const char* Pco::nameDelayTimeStep = "PCO_DELAYTIMESTEP";
const char* Pco::nameExpTimeMin = "PCO_EXPTIMEMIN";
const char* Pco::nameExpTimeMax = "PCO_EXPTIMEMAX";
const char* Pco::nameExpTimeStep = "PCO_EXPTIMESTEP";
const char* Pco::nameMaxBinHorz = "PCO_MAXBINHORZ";
const char* Pco::nameMaxBinVert = "PCO_MAXBINVERT";
const char* Pco::nameBinHorzStepping = "PCO_BINHORZSTEPPING";
const char* Pco::nameBinVertStepping = "PCO_BINVERTSTEPPING";
const char* Pco::nameRoiHorzSteps = "PCO_ROIHORZSTEPS";
const char* Pco::nameRoiVertSteps = "PCO_ROIVERTSTEPS";
const char* Pco::nameReboot = "PCO_REBOOT";

/** Constants
 */
const int Pco::traceFlagsDllApi = 0x0100;
const int Pco::traceFlagsPcoState = 0x0200;
const int Pco::requestQueueCapacity = 10;
const int Pco::numHandles = 300;
const double Pco::reconnectPeriod = 5.0;
const double Pco::rebootPeriod = 10.0;
const double Pco::connectPeriod = 1.0;
const double Pco::statusPollPeriod = 2.0;
const double Pco::acquisitionStatusPollPeriod = 5.0;
const int Pco::bitsPerShortWord = 16;
const int Pco::bitsPerNybble = 4;
const long Pco::nybbleMask = 0x0f;
const long Pco::bcdDigitValue = 10;
const int Pco::bcdPixelLength = 4;
const int Pco::defaultHorzBin = 1;
const int Pco::defaultVertBin = 1;
const int Pco::defaultRoiMinX = 1;
const int Pco::defaultRoiMinY = 1;
const int Pco::defaultExposureTime = 50;
const int Pco::defaultDelayTime = 0;
const int Pco::edgeXSizeNeedsReducedCamlink = 1920;
const int Pco::edgePixRateNeedsReducedCamlink = 286000000;
const int Pco::edgeBaudRate = 115200;
const double Pco::timebaseNanosecondsThreshold = 0.001;
const double Pco::timebaseMicrosecondsThreshold = 1.0;
const double Pco::oneNanosecond = 1e-9;
const double Pco::oneMillisecond = 1e-3;
const double Pco::triggerRetryPeriod = 0.01;
const int Pco::statusMessageSize = 256;


/** State machine strings
 */
const char* Pco::eventNames[] = {"Initialise", "TimerExpiry", "Acquire",
    "Stop", "Arm", "ImageReceived", "Disarm", "Trigger", "Reboot"};
const char* Pco::stateNames[] =  {"Uninitialised", "Unconnected", "Idle",
    "Armed", "Acquiring", "UnarmedAcquiring", "ExternalAcquiring", "Rebooting"};

/** The PCO object map
 */
std::map<std::string, Pco*> Pco::thePcos;

/**
 * Constructor
 * \param[in] portName ASYN Port name
 * \param[in] maxSizeX frame size width
 * \param[in] maxSizeY frame size height
 * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
 */
Pco::Pco(const char* portName, int maxBuffers, size_t maxMemory)
: ADDriver(portName, 1, Pco::numHandles, maxBuffers, maxMemory,
        asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask | asynInt16ArrayMask | asynEnumMask,
        asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask | asynInt16ArrayMask | asynEnumMask,
        0,          // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0
        1,          // autoConnect = 1
        -1,         // Default priority
        -1)         // Default stack size
, stateMachine(NULL)
, triggerTimer(NULL)
, api(NULL)
, errorTrace(getAsynUser(), ASYN_TRACE_ERROR)
, apiTrace(getAsynUser(), Pco::traceFlagsDllApi)
, stateTrace(getAsynUser(), Pco::traceFlagsPcoState)
, receivedFrameQueue(maxBuffers, sizeof(NDArray*))
{
    // Put in global map
    Pco::thePcos[portName] = this;

    // Create the parameters...
    this->createParam(Pco::namePixRate, asynParamInt32, &this->handlePixRate);
    this->createParam(Pco::nameAdcMode, asynParamInt32, &this->handleAdcMode);
    this->createParam(Pco::nameCamRamUse, asynParamInt32, &this->handleCamRamUse);
    this->createParam(Pco::nameElectronicsTemp, asynParamFloat64, &this->handleElectronicsTemp);
    this->createParam(Pco::namePowerTemp, asynParamFloat64, &this->handlePowerTemp);
    this->createParam(Pco::nameStorageMode, asynParamInt32, &this->handleStorageMode);
    this->createParam(Pco::nameRecorderSubmode, asynParamInt32, &this->handleRecorderSubmode);
    this->createParam(Pco::nameTimestampMode, asynParamInt32, &this->handleTimestampMode);
    this->createParam(Pco::nameAcquireMode, asynParamInt32, &this->handleAcquireMode);
    this->createParam(Pco::nameDelayTime, asynParamFloat64, &this->handleDelayTime);
    this->createParam(Pco::nameArmMode, asynParamInt32, &this->handleArmMode);
    this->createParam(Pco::nameImageNumber, asynParamInt32, &this->handleImageNumber);
    this->createParam(Pco::nameCameraSetup, asynParamInt32, &this->handleCameraSetup);
    this->createParam(Pco::nameBitAlignment, asynParamInt32, &this->handleBitAlignment);
    this->createParam(Pco::nameStateRecord, asynParamOctet, &this->handleStateRecord);
    this->createParam(Pco::nameClearStateRecord, asynParamInt32, &this->handleClearStateRecord);
    this->createParam(Pco::nameOutOfNDArrays, asynParamInt32, &this->handleOutOfNDArrays);
    this->createParam(Pco::nameBufferQueueReadFailures, asynParamInt32, &this->handleBufferQueueReadFailures);
    this->createParam(Pco::nameBuffersWithNoData, asynParamInt32, &this->handleBuffersWithNoData);
    this->createParam(Pco::nameMisplacedBuffers, asynParamInt32, &this->handleMisplacedBuffers);
    this->createParam(Pco::nameMissingFrames, asynParamInt32, &this->handleMissingFrames);
    this->createParam(Pco::nameDriverLibraryErrors, asynParamInt32, &this->handleDriverLibraryErrors);
    this->createParam(Pco::nameHwBinX, asynParamInt32, &this->handleHwBinX);
    this->createParam(Pco::nameHwBinY, asynParamInt32, &this->handleHwBinY);
    this->createParam(Pco::nameHwRoiX1, asynParamInt32, &this->handleHwRoiX1);
    this->createParam(Pco::nameHwRoiY1, asynParamInt32, &this->handleHwRoiY1);
    this->createParam(Pco::nameHwRoiX2, asynParamInt32, &this->handleHwRoiX2);
    this->createParam(Pco::nameHwRoiY2, asynParamInt32, &this->handleHwRoiY2);
    this->createParam(Pco::nameXCamSize, asynParamInt32, &this->handleXCamSize);
    this->createParam(Pco::nameYCamSize, asynParamInt32, &this->handleYCamSize);
    this->createParam(Pco::nameCamlinkClock, asynParamInt32, &this->handleCamlinkClock);
    this->createParam(Pco::nameMinCoolingSetpoint, asynParamInt32, &this->handleMinCoolingSetpoint);
    this->createParam(Pco::nameMaxCoolingSetpoint, asynParamInt32, &this->handleMaxCoolingSetpoint);
    this->createParam(Pco::nameDefaultCoolingSetpoint, asynParamInt32, &this->handleDefaultCoolingSetpoint);
    this->createParam(Pco::nameCoolingSetpoint, asynParamInt32, &this->handleCoolingSetpoint);
    this->createParam(Pco::nameDelayTimeMin, asynParamFloat64, &this->handleDelayTimeMin);
    this->createParam(Pco::nameDelayTimeMax, asynParamFloat64, &this->handleDelayTimeMax);
    this->createParam(Pco::nameDelayTimeStep, asynParamFloat64, &this->handleDelayTimeStep);
    this->createParam(Pco::nameExpTimeMin, asynParamFloat64, &this->handleExpTimeMin);
    this->createParam(Pco::nameExpTimeMax, asynParamFloat64, &this->handleExpTimeMax);
    this->createParam(Pco::nameExpTimeStep, asynParamFloat64, &this->handleExpTimeStep);
    this->createParam(Pco::nameMaxBinHorz, asynParamInt32, &this->handleMaxBinHorz);
    this->createParam(Pco::nameMaxBinVert, asynParamInt32, &this->handleMaxBinVert);
    this->createParam(Pco::nameBinHorzStepping, asynParamInt32, &this->handleBinHorzStepping);
    this->createParam(Pco::nameBinVertStepping, asynParamInt32, &this->handleBinVertStepping);
    this->createParam(Pco::nameRoiHorzSteps, asynParamInt32, &this->handleRoiHorzSteps);
    this->createParam(Pco::nameRoiVertSteps, asynParamInt32, &this->handleRoiVertSteps);
    this->createParam(Pco::nameReboot, asynParamInt32, &this->handleReboot);

    // ...and initialise them
    this->setIntegerParam(this->NDDataType, NDUInt16);
    this->setIntegerParam(this->ADNumExposures, 1);
    this->setIntegerParam(this->handleArmMode, 0);
    this->setIntegerParam(this->handleImageNumber, 0);
    this->setStringParam(this->ADManufacturer, "PCO");
    this->setStringParam(this->ADModel, "4000");
    this->setIntegerParam(this->ADMaxSizeX, 1280);
    this->setIntegerParam(this->ADMaxSizeY, 1024);
    this->setIntegerParam(this->NDArraySize, 0);
    this->setStringParam(this->handleStateRecord, "");
    this->setIntegerParam(this->handleClearStateRecord, 0);
    this->setIntegerParam(this->handleOutOfNDArrays, 0);
    this->setIntegerParam(this->handleBufferQueueReadFailures, 0);
    this->setIntegerParam(this->handleBuffersWithNoData, 0);
    this->setIntegerParam(this->handleMisplacedBuffers, 0);
    this->setIntegerParam(this->handleMissingFrames, 0);
    this->setIntegerParam(this->handleDriverLibraryErrors, 0);
    this->setIntegerParam(this->handleHwBinX, 0);
    this->setIntegerParam(this->handleHwBinY, 0);
    this->setIntegerParam(this->handleHwRoiX1, 0);
    this->setIntegerParam(this->handleHwRoiY1, 0);
    this->setIntegerParam(this->handleHwRoiX2, 0);
    this->setIntegerParam(this->handleHwRoiY2, 0);
    this->setIntegerParam(this->handleXCamSize, 1280);
    this->setIntegerParam(this->handleYCamSize, 1024);
    this->setIntegerParam(this->handleBitAlignment, 1);
    this->setIntegerParam(this->handleMinCoolingSetpoint, 0);
    this->setIntegerParam(this->handleMaxCoolingSetpoint, 0);
    this->setIntegerParam(this->handleDefaultCoolingSetpoint, 0);
    this->setIntegerParam(this->handleCoolingSetpoint, 0);
    this->setIntegerParam(this->handleMaxBinHorz, 0);
    this->setIntegerParam(this->handleMaxBinVert, 0);
    this->setIntegerParam(this->handleBinHorzStepping, 0);
    this->setIntegerParam(this->handleBinVertStepping, 0);
    this->setIntegerParam(this->handleRoiHorzSteps, 0);
    this->setIntegerParam(this->handleRoiVertSteps, 0);
    this->setIntegerParam(this->handleReboot, 1);
    // We are not connected to a camera
    this->camera = NULL;
    // Initialise the buffers
    for(int i=0; i<Pco::numApiBuffers; i++)
    {
        this->buffers[i].bufferNumber = DllApi::bufferUnallocated;
        this->buffers[i].buffer = NULL;
        this->buffers[i].eventHandle = NULL;
        this->buffers[i].ready = false;
    }
    // Initialise the enum strings
    for(int i=0; i<DllApi::descriptionNumPixelRates; i++)
    {
        this->pixRateEnumValues[i] = 0;
        this->pixRateEnumStrings[i] = (char *)calloc(MAX_ENUM_STRING_SIZE, sizeof(char));
        this->pixRateEnumSeverities[i] = 0;
    }
    // Create the state machine
    this->stateMachine = new StateMachine("Pco", this,
            this->handleStateRecord, this, Pco::stateUninitialised,
            Pco::stateNames, Pco::eventNames, &this->stateTrace,
            Pco::requestQueueCapacity);
    this->triggerTimer = new StateMachine::Timer(this->stateMachine);
}

/**
 * Destructor
 */
Pco::~Pco()
{
    try
    {
        this->api->setRecordingState(this->camera, DllApi::recorderStateOff);
        this->api->cancelImages(this->camera);
        for(int i=0; i<Pco::numApiBuffers; i++)
        {
            this->api->freeBuffer(this->camera, i);
        }
        this->api->closeCamera(this->camera);
    }
    catch(PcoException&)
    {
    }
    for(int i=0; i<Pco::numApiBuffers; i++)
    {
        if(this->buffers[i].buffer != NULL)
        {
            delete[] this->buffers[i].buffer;
        }
    }
    delete this->triggerTimer;
    delete this->stateMachine;
}

/**
 * Connects the DLL API to the main PCO class.  This call triggers
 * the initialisation of the camera.
 */
void Pco::registerDllApi(DllApi* api)
{
    this->api = api;
    this->post(Pco::requestInitialise);
}

/**
 * Return the pco corresponding to the port name
 * \param[in] p The port name
 * \return The pco object, NULL if not found
 */
Pco* Pco::getPco(const char* portName)
{
    Pco* result = NULL;
    std::map<std::string, Pco*>::iterator pos = Pco::thePcos.find(portName);
    if(pos != Pco::thePcos.end())
    {
        result = pos->second;
    }
    return result;
}

/**
 * Handle state machine transitions
 */
int Pco::doTransition(StateMachine* machine, int state, int event)
{
    if(machine == this->stateMachine)
    {
        switch(state)
        {
        case Pco::stateUninitialised:
            if(event == Pco::requestInitialise)
            {
                this->stateMachine->startTimer(Pco::connectPeriod, Pco::requestTimerExpiry);
                state = Pco::stateUnconnected;
            }
            break;
        case Pco::stateUnconnected:
            if(event == Pco::requestTimerExpiry)
            {
                if(this->connectToCamera())
                {
                	this->initialiseCamera();
                    this->stateMachine->startTimer(Pco::statusPollPeriod, Pco::requestTimerExpiry);
                    this->discardImages();
                    state = Pco::stateIdle;
                }
                else
                {
                    // Try again soon
                    this->stateMachine->startTimer(Pco::reconnectPeriod, Pco::requestTimerExpiry);
                    state = Pco::stateUnconnected;
                }
            }
            break;
        case Pco::stateRebooting:
            if(event == Pco::requestTimerExpiry)
            {
				this->initialiseCamera();
				this->stateMachine->startTimer(Pco::statusPollPeriod, Pco::requestTimerExpiry);
				this->discardImages();
				state = Pco::stateIdle;
            }
            break;
        case Pco::stateIdle:
            if(event == Pco::requestTimerExpiry)
            {
                this->pollCameraNoAcquisition();
                this->pollCamera();
                this->stateMachine->startTimer(Pco::statusPollPeriod, Pco::requestTimerExpiry);
                state = Pco::stateIdle;
            }
            else if(event == Pco::requestArm)
            {
                try
                {
                    this->doArm();
                    this->stateMachine->startTimer(Pco::statusPollPeriod, Pco::requestTimerExpiry);
                    state = Pco::stateArmed;
                    this->outputStatusMessage("");
                }
                catch(std::bad_alloc& e)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    this->errorTrace << "Failed to arm due to out of memory, " << e.what() << std::endl;
                    state = Pco::stateIdle;
                    this->outputStatusMessage(e.what());
                }
                catch(PcoException& e)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    this->errorTrace << "Failed to arm due DLL error, " << e.what() << std::endl;
                    state = Pco::stateIdle;
                    this->outputStatusMessage(e.what());
                }
            }
            else if(event == Pco::requestAcquire)
            {
                try
                {
                    this->doArm();
                    this->nowAcquiring();
                    this->startCamera();
                    this->stateMachine->startTimer(Pco::acquisitionStatusPollPeriod, Pco::requestTimerExpiry);
                    state = Pco::statedUnarmedAcquiring;
                    this->outputStatusMessage("");
                }
                catch(std::bad_alloc& e)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    this->errorTrace << "Failed to arm due to out of memory, " << e.what() << std::endl;
                    state = Pco::stateIdle;
                    this->outputStatusMessage(e.what());
                }
                catch(PcoException& e)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    this->errorTrace << "Failed to arm due DLL error, " << e.what() << std::endl;
                    state = Pco::stateIdle;
                    this->outputStatusMessage(e.what());
                }
            }
            else if(event == Pco::requestImageReceived)
            {
                this->discardImages();
            }
            else if(event == Pco::requestReboot)
            {
            	// We need to stop the poll timer and discard any events that have
            	// already been passed to the state machine
            	this->stateMachine->stopTimer();
            	this->stateMachine->clear();
            	// Now do the reboot
            	this->doReboot();
                this->stateMachine->startTimer(Pco::rebootPeriod, Pco::requestTimerExpiry);
                state = Pco::stateUnconnected;
            }
            break;
        case Pco::stateArmed:
            if(event == Pco::requestTimerExpiry)
            {
                this->pollCamera();
                this->stateMachine->startTimer(Pco::statusPollPeriod, Pco::requestTimerExpiry);
                state = Pco::stateArmed;
            }
            else if(event == Pco::requestAcquire)
            {
                this->nowAcquiring();
                this->startCamera();
                this->stateMachine->startTimer(Pco::acquisitionStatusPollPeriod, Pco::requestTimerExpiry);
                state = Pco::stateAcquiring;
            }
            else if(event == Pco::requestImageReceived)
            {
                if(this->triggerMode != DllApi::triggerSoftware)
                {
                    this->nowAcquiring();
                    if(!this->receiveImages())
                    {
                        state = Pco::stateExternalAcquiring;
                    }
                    else if(this->triggerMode == DllApi::triggerAuto)
                    {
                        this->acquisitionComplete();
                        this->doDisarm();
                        state = Pco::stateIdle;
                    }
                    else
                    {
                        this->acquisitionComplete();
                        state = Pco::stateArmed;
                    }
                }
                else
                {
                    this->discardImages();
                    state = Pco::stateArmed;
                }
            }
            else if(event == Pco::requestDisarm)
            {
                this->doDisarm();
                this->discardImages();
                state = Pco::stateIdle;
            }
            else if(event == Pco::requestStop)
            {
                this->doDisarm();
                this->discardImages();
                state = Pco::stateIdle;
            }
            break;
        case Pco::stateAcquiring:
            if(event == Pco::requestTimerExpiry)
            {
                this->pollCamera();
                this->stateMachine->startTimer(Pco::acquisitionStatusPollPeriod, Pco::requestTimerExpiry);
                state = Pco::stateAcquiring;
            }
            else if(event == Pco::requestImageReceived)
            {
                if(!this->receiveImages())
                {
                    this->startCamera();
                    state = Pco::stateAcquiring;
                }
                else if(this->triggerMode != DllApi::triggerSoftware)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    state = Pco::stateIdle;
                }
                else
                {
                    this->acquisitionComplete();
                    state = Pco::stateArmed;
                }
            }
            else if(event == Pco::requestTrigger)
            {
                this->startCamera();
                state = Pco::stateAcquiring;
            }
            else if(event == Pco::requestStop)
            {
                if(this->triggerMode != DllApi::triggerSoftware)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    state = Pco::stateIdle;
                }
                else
                {
                    this->acquisitionComplete();
                    state = Pco::stateArmed;
                }
            }
            break;
        case Pco::stateExternalAcquiring:
            if(event == Pco::requestTimerExpiry)
            {
                this->pollCamera();
                this->stateMachine->startTimer(Pco::acquisitionStatusPollPeriod, Pco::requestTimerExpiry);
                state = Pco::stateExternalAcquiring;
            }
            else if(event == Pco::requestImageReceived)
            {
                if(!this->receiveImages())
                {
                    state = Pco::stateExternalAcquiring;
                }
                else if(this->triggerMode == DllApi::triggerAuto)
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    state = Pco::stateIdle;
                }
                else
                {
                    this->acquisitionComplete();
                    state = Pco::stateArmed;
                }
            }
            else if(event == Pco::requestStop)
            {
				this->acquisitionComplete();
				this->doDisarm();
				state = Pco::stateIdle;
            }
            break;
        case Pco::statedUnarmedAcquiring:
            if(event == Pco::requestTimerExpiry)
            {
                this->pollCamera();
                this->stateMachine->startTimer(Pco::acquisitionStatusPollPeriod, Pco::requestTimerExpiry);
                state = Pco::statedUnarmedAcquiring;
            }
            else if(event == Pco::requestImageReceived)
            {
                if(!this->receiveImages())
                {
                    this->startCamera();
                    state = Pco::statedUnarmedAcquiring;
                }
                else
                {
                    this->acquisitionComplete();
                    this->doDisarm();
                    this->discardImages();
                    state = Pco::stateIdle;
                }
            }
            else if(event == Pco::requestTrigger)
            {
                this->startCamera();
                state = Pco::statedUnarmedAcquiring;
            }
            else if(event == Pco::requestStop)
            {
                this->acquisitionComplete();
                this->doDisarm();
                this->discardImages();
                state = Pco::stateIdle;
            }
            break;
        }
    }
    return state;
}

/**
 * Reboot the camera
 */
void Pco::doReboot()
{
	this->api->setTimeouts(this->camera, 2000, 3000, 250);
	switch(this->camType)
	{
	case DllApi::cameraTypeEdge:
	case DllApi::cameraTypeEdgeGl:
		this->api->rebootCamera(this->camera);
		break;
	default:
		break;
	}
    this->api->closeCamera(this->camera);
    this->camera = 0;
}

/**
 * Output a message to the status PV.
 * \param[in] text The message to output
 */
void Pco::outputStatusMessage(const char* text)
{
    this->lock();
    this->setStringParam(this->ADStatusMessage, text);
    this->callParamCallbacks();
    this->unlock();
}

/**
 * Connect to the camera
 * \return Returns true for success.
 */
bool Pco::connectToCamera()
{
    bool result = true;
    this->lock();
    // Close the camera if we think it might be open
    if(this->camera != NULL)
    {
        try
        {
            this->api->closeCamera(this->camera);
        }
        catch(PcoException&)
        {
            // Swallow errors from this
        }
    }
    // Now try to open it again
    try
    {
        // Open the camera
        this->camera = 0;
        this->api->openCamera(&this->camera, 0);
    }
    catch(PcoException&)
    {
        result = false;
    }
    this->unlock();
    return result;
}

/** Initialise the camera
 * \return Returns true for success
 */
bool Pco::initialiseCamera()
{
    bool result = true;
    this->lock();
    // Initialise the camera if it opened
	try
	{
        // Get various camera data
        this->api->getGeneral(this->camera);
        this->api->getCameraType(this->camera, &this->camType);
        this->api->getSensorStruct(this->camera);
        this->api->getCameraDescription(this->camera, &this->camDescription);
        this->api->getStorageStruct(this->camera, &this->camRamSize, &this->camPageSize);
        this->api->getRecordingStruct(this->camera);

        // Corrections for values that appear to be incorrectly returned by the SDK
        switch(this->camType)
        {
        case DllApi::cameraTypeDimaxStd:
        case DllApi::cameraTypeDimaxTv:
        case DllApi::cameraTypeDimaxAutomotive:
            this->camDescription.roiVertSteps = 4;
            break;
        default:
            break;
        }

        // reset the camera
        try
        {
            this->api->setRecordingState(this->camera, DllApi::recorderStateOff);
            this->api->resetSettingsToDefault(this->camera);
        }
        catch(PcoException&)
        {
            // Swallow errors from this
        }

        // Record binning and roi capabilities
        this->setIntegerParam(this->handleMaxBinHorz, (int)this->camDescription.maxBinHorz);
        this->setIntegerParam(this->handleMaxBinVert, (int)this->camDescription.maxBinVert);
        this->setIntegerParam(this->handleBinHorzStepping, (int)this->camDescription.binHorzStepping);
        this->setIntegerParam(this->handleBinVertStepping, (int)this->camDescription.binVertStepping);
        this->setIntegerParam(this->handleRoiHorzSteps, (int)this->camDescription.roiHorSteps);
        this->setIntegerParam(this->handleRoiVertSteps, (int)this->camDescription.roiVertSteps);

        // Build the set of binning values
        this->setValidBinning(this->availBinX, this->camDescription.maxBinHorz,
                this->camDescription.binHorzStepping);
        this->setValidBinning(this->availBinY, this->camDescription.maxBinVert,
                this->camDescription.binVertStepping);

        // Get more camera information
        this->api->getTransferParameters(this->camera, &this->camTransfer);
        this->api->getSizes(this->camera, &this->camSizes);
        this->setIntegerParam(this->ADMaxSizeX, (int)this->camSizes.xResActual);
        this->setIntegerParam(this->ADMaxSizeY, (int)this->camSizes.yResActual);
        this->setIntegerParam(this->ADSizeX, (int)this->camSizes.xResActual);
        this->setIntegerParam(this->ADSizeY, (int)this->camSizes.yResActual);
        this->setIntegerParam(this->handleCamlinkClock, (int)this->camTransfer.clockFrequency);

        // Initialise the cooling setpoint information
        this->setIntegerParam(this->handleMinCoolingSetpoint, this->camDescription.minCoolingSetpoint);
        this->setIntegerParam(this->handleMaxCoolingSetpoint, this->camDescription.maxCoolingSetpoint);
        this->setIntegerParam(this->handleDefaultCoolingSetpoint, this->camDescription.defaultCoolingSetpoint);
        this->setIntegerParam(this->handleCoolingSetpoint, this->camDescription.defaultCoolingSetpoint);
        this->setCoolingSetpoint();

        // Acquisition timing parameters
        this->setDoubleParam(this->handleDelayTimeMin, (double)this->camDescription.minDelayNs * 1e-9);
        this->setDoubleParam(this->handleDelayTimeMax, (double)this->camDescription.maxDelayMs * 1e-3);
        this->setDoubleParam(this->handleDelayTimeStep, (double)this->camDescription.minDelayStepNs * 1e-9);
        this->setDoubleParam(this->handleExpTimeMin, (double)this->camDescription.minExposureNs * 1e-9);
        this->setDoubleParam(this->handleExpTimeMax, (double)this->camDescription.maxExposureMs * 1e-3);
        this->setDoubleParam(this->handleExpTimeStep, (double)this->camDescription.minExposureStepNs * 1e-9);

        // Update area detector information strings
        switch(this->camType)
        {
        case DllApi::cameraType1200Hs:
            this->setStringParam(ADModel, "PCO.Camera 1200");
            break;
        case DllApi::cameraType1300:
            this->setStringParam(ADModel, "PCO.Camera 1300");
            break;
        case DllApi::cameraType1600:
            this->setStringParam(ADModel, "PCO.Camera 1600");
            break;
        case DllApi::cameraType2000:
            this->setStringParam(ADModel, "PCO.Camera 2000");
            break;
        case DllApi::cameraType4000:
            this->setStringParam(ADModel, "PCO.Camera 4000");
            break;
        case DllApi::cameraTypeEdge:
        case DllApi::cameraTypeEdgeGl:
            this->setStringParam(ADModel, "PCO.Camera Edge");
            break;
        case DllApi::cameraTypeDimaxStd:
        case DllApi::cameraTypeDimaxTv:
        case DllApi::cameraTypeDimaxAutomotive:
            this->setStringParam(ADModel, "PCO.Camera Dimax");
            break;
        default:
            this->setStringParam(ADModel, "PCO.Camera");
            break;
        }
        this->setStringParam(ADManufacturer, "PCO");

        // Work out how to decode the BCD frame number in the image
        this->shiftLowBcd = Pco::bitsPerShortWord - this->camDescription.dynResolution;
        this->shiftHighBcd = this->shiftLowBcd + Pco::bitsPerNybble;

        // Set the camera clock
        this->setCameraClock();

        // Handle the pixel rates
        this->initialisePixelRate();

        // Make Edge specific function calls
        if(this->camType == DllApi::cameraTypeEdge || this->camType == DllApi::cameraTypeEdgeGl)
        {
            // Get Edge camera setup mode
            unsigned long setupData[DllApi::cameraSetupDataSize];
            unsigned short setupDataLen = DllApi::cameraSetupDataSize;
            unsigned short setupType;
            this->api->getCameraSetup(this->camera, &setupType, setupData, &setupDataLen);
            this->setIntegerParam(this->handleCameraSetup, setupData[0]);
        }

        // Set the default binning
        this->api->setBinning(this->camera, Pco::defaultHorzBin, Pco::defaultVertBin);
        this->setIntegerParam(this->ADBinX, Pco::defaultHorzBin);
        this->setIntegerParam(this->ADBinY, Pco::defaultVertBin);

        // Set the default ROI (apparently a must do step)
        int roix1, roix2, roiy1, roiy2; // region of interest
        // to maximise in x dimension
        roix1 = Pco::defaultRoiMinX;
        roix2 = this->camDescription.maxHorzRes/Pco::defaultHorzBin/
                this->camDescription.roiHorSteps;
        roix2 *= this->camDescription.roiHorSteps;
        // to maximise in y dimension
        roiy1 = Pco::defaultRoiMinY;
        roiy2 = this->camDescription.maxVertRes/Pco::defaultVertBin/
                this->camDescription.roiVertSteps;
        roiy2 *= this->camDescription.roiVertSteps;
        this->api->setRoi(this->camera,
                (unsigned short)roix1, (unsigned short)roiy1,
                (unsigned short)roix2, (unsigned short)roiy2);
        this->setIntegerParam(this->ADMinX, roix1-1);
        this->setIntegerParam(this->ADMinY, roiy1-1);
        this->setIntegerParam(this->ADSizeX, roix2-roix1+1);
        this->setIntegerParam(this->ADSizeY, roiy2-roiy1+1);

        // Set initial trigger mode to auto
        this->api->setTriggerMode(this->camera, DllApi::triggerExternal);

        // Set the storage mode to FIFO
        this->api->setStorageMode(this->camera, DllApi::storageModeFifoBuffer);

        // Set our preferred time stamp mode.
        if((this->camDescription.generalCaps & DllApi::generalCapsNoTimestamp) != 0)
        {
            this->api->setTimestampMode(this->camera, DllApi::timestampModeOff);
        }
        else if((this->camDescription.generalCaps & DllApi::generalCapsTimestampAsciiOnly) != 0)
        {
            this->api->setTimestampMode(this->camera, DllApi::timestampModeAscii);
        }
        else
        {
            this->api->setTimestampMode(this->camera, DllApi::timestampModeBinaryAndAscii);
        }

        // Set the acquire mode.
        this->api->setAcquireMode(this->camera, DllApi::acquireModeAuto);
        this->setIntegerParam(this->handleAcquireMode, DllApi::acquireModeAuto);

        // Set the delay and exposure times
        this->api->setDelayExposureTime(this->camera,
                Pco::defaultDelayTime, Pco::defaultExposureTime,
                DllApi::timebaseMilliseconds, DllApi::timebaseMilliseconds);
        this->setDoubleParam(this->ADAcquireTime,
                Pco::defaultExposureTime * Pco::oneMillisecond);

        // Set the gain
        if(this->camDescription.convFact > 0)
        {
            this->api->setConversionFactor(this->camera, this->camDescription.convFact);
            this->setDoubleParam(this->ADGain, this->camDescription.convFact);
        }

        // Set the ADC mode for the cameras that support it
        if(this->camType == DllApi::cameraType1600 ||
                this->camType == DllApi::cameraType2000 ||
                this->camType == DllApi::cameraType4000)
        {
            this->api->setAdcOperation(this->camera, DllApi::adcModeSingle);
        }

        // Default data type
        this->setIntegerParam(this->NDDataType, NDUInt16);

        // Camera booted
        this->setIntegerParam(this->handleReboot, 0);

        // Lets have a look at the status of the camera
        unsigned short recordingState;
        this->api->getRecordingState(this->camera, &recordingState);

        // refresh everything
        this->pollCameraNoAcquisition();
        this->pollCamera();
    }
    catch(PcoException&)
    {
        result = false;
    }
    // Update EPICS
    callParamCallbacks();
    this->unlock();
    return result;
}

/**
 * Initialise the pixel rate information.
 * The various members are used as follows:
 *   camDescription.pixelRate[] contains the available pixel rates in Hz, zeroes
 *                              for unused locations.
 *   pixRateEnumValues[] contains indices into the camDescription.pixelRate[] array
 *                       for the mbbx PV values.
 *   pixRateEnumStrings[] contains the mbbx strings
 *   pixRateEnumSeverities[] contains the severity codes for the mbbx PV.
 *   pixRate contains the current setting in Hz.
 *   pixRateValue contains the mbbx value of the current setting
 *   pixRateMax contains the maximum available setting in Hz.
 *   pixRateMaxValue contains the mbbx value of the maximum setting.
 *   pixRateNumEnums is the number of valid rates
 */
void Pco::initialisePixelRate()
{
    // Work out the information
    this->pixRateMax = 0;
    this->pixRateMaxValue = 0;
    this->pixRateNumEnums = 0;
    for(int i = 0; i<DllApi::descriptionNumPixelRates; i++)
    {
        if(this->camDescription.pixelRate[i] > 0)
        {
            epicsSnprintf(this->pixRateEnumStrings[this->pixRateNumEnums], MAX_ENUM_STRING_SIZE,
                "%ld Hz", this->camDescription.pixelRate[i]);
            this->pixRateEnumValues[this->pixRateNumEnums] = i;
            this->pixRateEnumSeverities[this->pixRateNumEnums] = 0;
            if((int)this->camDescription.pixelRate[i] > this->pixRateMax)
            {
                this->pixRateMax = this->camDescription.pixelRate[i];
                this->pixRateMaxValue = this->pixRateNumEnums;
            }
            this->pixRateNumEnums++;
        }
    }
    // Give the enum strings to the PV
    this->doCallbacksEnum(this->pixRateEnumStrings, this->pixRateEnumValues, this->pixRateEnumSeverities,
        this->pixRateNumEnums,  this->handlePixRate, 0);
    // Get the current rate
    unsigned long r;
    this->api->getPixelRate(this->camera, &r);
    this->pixRate = (int)r;
    this->setIntegerParam(this->handlePixRate, this->pixRateValue);
}

/**
 * Populate a binning validity set
 */
void Pco::setValidBinning(std::set<int>& valid, int max, int step) throw()
{
    valid.clear();
    int bin = 1;
    while(bin <= max)
    {
        valid.insert(bin);
        if(step == DllApi::binSteppingLinear)
        {
            bin += 1;
        }
        else
        {
            bin *= 2;
        }
    }
}

/**
 * Poll the camera for status information.  This function may only be called
 * while the camera is not acquiring.
 */
bool Pco::pollCameraNoAcquisition()
{
    bool result = true;
    try
    {
        unsigned short storageMode;
        this->api->getStorageMode(this->camera, &storageMode);
        this->lock();
        this->setIntegerParam(this->handleStorageMode, (int)storageMode);
        this->unlock();

        unsigned short recorderSubmode;
        this->api->getRecorderSubmode(this->camera, &recorderSubmode);
        this->lock();
        this->setIntegerParam(this->handleRecorderSubmode, (int)recorderSubmode);
        this->unlock();
    }
    catch(PcoException& e)
    {
        this->errorTrace << "Failure: " << e.what() << std::endl;
        result = false;
    }
    this->lock();
    callParamCallbacks();
    this->unlock();
    return result;
}

/**
 * Poll the camera for status information that can be gathered at any time
 */
bool Pco::pollCamera()
{
    bool result = true;
    this->lock();
    try
    {
        // Get the temperature information
        short ccdtemp, camtemp, powtemp;
        this->api->getTemperature(this->camera, &ccdtemp, &camtemp, &powtemp);
        this->setDoubleParam(this->ADTemperature, (double)ccdtemp/DllApi::ccdTemperatureScaleFactor);
        this->setDoubleParam(this->handleElectronicsTemp, (double)camtemp);
        this->setDoubleParam(this->handlePowerTemp, (double)powtemp);
        // Get memory usage
        this->setIntegerParam(this->handleCamRamUse, this->checkMemoryBuffer());
    }
    catch(PcoException& e)
    {
        this->errorTrace << "Failure: " << e.what() << std::endl;
        result = false;
    }
    callParamCallbacks();
    this->unlock();
    return result;
}

/**
 * Report the percentage of camera on board memory that contains images.
 * For cameras without on board memory this will always return 0%.
 * Note for a camera with a single image in memory the percentage returned will
 * be at least 1% even if the camera has a massive memory containing a small image.
 */
int Pco::checkMemoryBuffer() throw(PcoException)
{
    int percent = 0;
    if(this->camRamSize > 0)
    {
        unsigned short segment;
        unsigned long validImages;
        unsigned long maxImages;
        try
        {
            this->api->getActiveRamSegment(this->camera, &segment);
            this->api->getNumberOfImagesInSegment(this->camera, segment, &validImages,
                    &maxImages);
            if(maxImages > 0)
            {
                percent = (validImages*100)/maxImages;
                if(validImages > 0 && percent == 0)
                {
                    percent = 1;
                }
            }
        }
        catch(PcoException&)
        {
        }
    }
    return percent;
}

/**
 * Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 */
asynStatus Pco::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int parameter = pasynUser->reason;

    // Base class does most of the work including updating the parameters
    asynStatus status = ADDriver::writeInt32(pasynUser, value);

    // Special things to be done
    if (parameter == ADAcquire)
    {
        if(value)
        {
            // Start an acquisition
            this->post(Pco::requestAcquire);
        }
        else if(!value)
        {
            // Stop the acquisition
            this->post(Pco::requestStop);
        }
    }
    else if(parameter == this->handleArmMode)
    {
        // Perform an arm/disarm
        if(value)
        {
            this->post(Pco::requestArm);
        }
        else
        {
            this->post(Pco::requestDisarm);
        }
    }
    else if(parameter == this->handleClearStateRecord)
    {
        if(value)
        {
            setStringParam(handleStateRecord, "");
            setIntegerParam(handleClearStateRecord, 0);
        }
    }
    else if(parameter == this->handleCoolingSetpoint)
    {
        this->setCoolingSetpoint();
        this->callParamCallbacks();
    }
    else if(parameter == this->handleReboot)
    {
    	this->post(Pco::requestReboot);
    }

    // Show other components
    this->api->writeInt32(pasynUser, value);

    return status;
}

/**
 * Called when asyn clients call pasynFloat64->write().
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 */
asynStatus Pco::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int parameter = pasynUser->reason;

    // Base class does most of the work including updating the parameters
    asynStatus status = ADDriver::writeFloat64(pasynUser, value);

    if(parameter == this->ADTemperature)
    {
        // Interpret an attempt to set the temperature as setting the
        // cooling set point
        this->setIntegerParam(this->handleCoolingSetpoint, (int)value);
        this->setCoolingSetpoint();
        this->callParamCallbacks();
    }

    return status;
}

/**
 * Called when asyn clients call pasynOctet->write().
 * This function performs actions for some parameters, including ADFilePath, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Address of the string to write.
 * \param[in] nChars Number of characters to write.
 * \param[out] nActual Number of characters actually written.
 */
asynStatus Pco::writeOctet(asynUser *pasynUser, const char *value,
        size_t nChars, size_t *nActual)
{
    //int parameter = pasynUser->reason;

    // Base class does most of the work including updating the parameters
    asynStatus status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);

    return status;
}

/**
 * Post a request
 */
void Pco::post(Request req)
{
    this->stateMachine->post(req);
}

/**
 * A frame has been received
 */
void Pco::frameReceived(int bufferNumber)
{
    // Get an ND array
    size_t maxDims[] = {this->xCamSize, this->yCamSize};
    NDArray* image = this->pNDArrayPool->alloc(sizeof(maxDims)/sizeof(size_t),
            maxDims, NDUInt16, 0, NULL);
    if(image == NULL)
    {
        // Out of area detector NDArrays
        this->lock();
        this->setIntegerParam(this->handleOutOfNDArrays, ++this->outOfNDArrays);
        this->callParamCallbacks();
        this->unlock();
    }
    else
    {
        // Copy the image into an NDArray
        ::memcpy(image->pData, this->buffers[bufferNumber].buffer,
                this->xCamSize*this->yCamSize*sizeof(unsigned short));
        // Post the NDarray to the state machine thread
        if(this->receivedFrameQueue.send(&image, sizeof(NDArray*)) != 0)
        {
        	// Failed to put on queue, better free the NDArray to avoid leaks
        	image->release();
        }
        this->post(Pco::requestImageReceived);
    }
    // Give the buffer back to the API
    this->lock();
    this->api->addBufferEx(this->camera, /*firstImage=*/0,
        /*lastImage=*/0, bufferNumber,
        this->xCamSize, this->yCamSize, this->camDescription.dynResolution);
    this->unlock();
}

/**
 * Return my asyn user object for use in tracing etc.
 */
asynUser* Pco::getAsynUser()
{
    return this->pasynUserSelf;
}

/**
 * Allocate image buffers and give them to the SDK.  We allocate actual memory here,
 * rather than using the NDArray memory because the SDK hangs onto the buffers, it only
 * shows them to us when there is a frame ready.  We must copy the frame out of the buffer
 * into an NDArray for use by the rest of the system.
 */
void Pco::allocateImageBuffers() throw(std::bad_alloc, PcoException)
{
    // How big?
	int bufferSize = this->camSizes.xResActual * this->camSizes.yResActual;
    // Now allocate the memory and tell the SDK
    try
    {
        for(int i=0; i<Pco::numApiBuffers; i++)
        {
            if(this->buffers[i].buffer != NULL)
            {
                delete[] this->buffers[i].buffer;
            }
            this->buffers[i].buffer = new unsigned short[bufferSize];
            this->buffers[i].bufferNumber = DllApi::bufferUnallocated;
            this->buffers[i].eventHandle = NULL;
            this->api->allocateBuffer(this->camera, &this->buffers[i].bufferNumber,
                    bufferSize * sizeof(short), &this->buffers[i].buffer,
                    &this->buffers[i].eventHandle);
            this->buffers[i].ready = true;
            assert(this->buffers[i].bufferNumber == i);
        }
    }
    catch(std::bad_alloc& e)
    {
        // Recover from memory allocation failure
        this->freeImageBuffers();
        throw e;
    }
    catch(PcoException& e)
    {
        // Recover from PCO camera failure
        this->freeImageBuffers();
        throw e;
    }
}

/**
 * Free the image buffers
 */
void Pco::freeImageBuffers() throw()
{
    // Free the buffers in the camera.  Since we are recovering,
    // ignore any SDK error this may cause.
    try
    {
        this->api->cancelImages(this->camera);
        // JAT: We didn't originally free the buffers from the DLL routinely.
        //      However, for the Dimax it seems to be essential.
        for(int i=0; i<Pco::numApiBuffers; i++)
        {
            this->api->freeBuffer(this->camera, i);
        }
    }
    catch(PcoException& e)
    {
        this->errorTrace << "Failure: " << e.what() << std::endl;
    }
}

/**
 * Depending on the camera, pixel rate and x image size we may have to adjust the transfer parameters
 * in order to achieve the frame rate required across camlink. 
 * The Edge in rolling shutter mode and > 50fps we have to select 12 bit transfer and a look up 
 * table to do the compression.
 * By experiment the following formats appear to work/not work with the Edge:
 *  Global shutter  : PCO_CL_DATAFORMAT_5x12 works
 *                  : PCO_CL_DATAFORMAT_5x16 PCO_CL_DATAFORMAT_5x12L doesn't work
 *  Rolling Shutter : PCO_CL_DATAFORMAT_5x12L PCO_CL_DATAFORMAT_5x12R PCO_CL_DATAFORMAT_5x16 PCO_CL_DATAFORMAT_5x12 works
 */
void Pco::adjustTransferParamsAndLut() throw(PcoException)
{
	unsigned short lutIdentifier = 0;
    // Configure according to camera type
    switch(this->camType)
    {
    case DllApi::cameraTypeEdge:
    case DllApi::cameraTypeEdgeGl:
        // Set the camlink transfer parameters, reading them back
        // again to make sure.
        if(this->cameraSetup == DllApi::edgeSetupGlobalShutter)
        {
            // Works in global and rolling modes
            this->camTransfer.dataFormat = DllApi::camlinkDataFormat5x12 | 
                DllApi:: sccmosFormatTopCenterBottomCenter;
            lutIdentifier = DllApi::camlinkLutNone;
        }
        else 
        {
            if(this->xCamSize>=Pco::edgeXSizeNeedsReducedCamlink &&
                    this->pixRate>=Pco::edgePixRateNeedsReducedCamlink)
            {
                // Options for edge are PCO_CL_DATAFORMAT_5x12L (uses sqrt LUT) and 
                // PCO_CL_DATAFORMAT_5x12 (data shifted 2 LSBs lost)
                this->camTransfer.dataFormat = DllApi::camlinkDataFormat5x12L | 
                    DllApi::sccmosFormatTopCenterBottomCenter;
                lutIdentifier = DllApi::camLinkLutSqrt;
            } 
            else 
            {
                // Doesn't work in global, works in rolling
                this->camTransfer.dataFormat = DllApi::camlinkDataFormat5x16 | 
                    DllApi::sccmosFormatTopCenterBottomCenter;
                lutIdentifier = DllApi::camlinkLutNone;
            }
        }
        this->camTransfer.baudRate = Pco::edgeBaudRate;
        this->api->setTransferParameters(this->camera, &this->camTransfer);
        this->api->getTransferParameters(this->camera, &this->camTransfer);
        this->api->setActiveLookupTable(this->camera, lutIdentifier);
        break;
    default:
        break;
    }
}

/**
 * Set the camera clock to match EPICS time.
 */
void Pco::setCameraClock() throw(PcoException)
{
    epicsTimeStamp epicsTime;
    epicsTimeGetCurrent(&epicsTime);
    unsigned long nanoSec;
    struct tm currentTime;
    epicsTimeToTM(&currentTime, &nanoSec, &epicsTime);
    this->api->setDateTime(this->camera, &currentTime);
    // Record the year for timestamp correction purposes
    this->cameraYear = currentTime.tm_year;
}

/**
 * Set the camera cooling set point.  This function expects
 * the lock to already be taken.
 */
void Pco::setCoolingSetpoint()
{
    int minCoolingSetpoint;
    int maxCoolingSetpoint;
    int desiredCoolingSetpoint;
    this->getIntegerParam(this->handleCoolingSetpoint, &desiredCoolingSetpoint);
    this->getIntegerParam(this->handleMinCoolingSetpoint, &minCoolingSetpoint);
    this->getIntegerParam(this->handleMaxCoolingSetpoint, &maxCoolingSetpoint);
    if(minCoolingSetpoint == 0 && maxCoolingSetpoint == 0)
    {
        // Min and max = 0 means there is no cooling available for this camera.
    }
    else
    {
        try
        {
            this->api->setCoolingSetpoint(this->camera, (short)desiredCoolingSetpoint);
        }
        catch(PcoException&)
        {
            // Not much we can do with this error
        }
        try
        {
            short actualCoolingSetpoint;
            this->api->getCoolingSetpoint(this->camera, &actualCoolingSetpoint);
            this->setIntegerParam(this->handleCoolingSetpoint, (int)actualCoolingSetpoint);
        }
        catch(PcoException&)
        {
            // Not much we can do with this error
        }
    }
}

/**
 * Pass buffer to SDK so it can populate it 
 */
void Pco::addAvailableBuffer(int index) throw(PcoException)
{
    if(this->buffers[index].ready)
    {
        this->api->addBufferEx(this->camera, /*firstImage=*/0,
            /*lastImage=*/0, this->buffers[index].bufferNumber, 
            this->xCamSize, this->yCamSize, this->camDescription.dynResolution);
        this->buffers[index].ready = false;
    }
}

/**
 * Pass all buffers to the SDK so it can populate them 
 */
void Pco::addAvailableBufferAll() throw(PcoException)
{
    for(int i=0; i<Pco::numApiBuffers; i++)
    {
        addAvailableBuffer(i);
    }
}

/**
 * Arm the camera, ie. prepare the camera for acquisition.
 * Throws exceptions on failure.
 */
void Pco::doArm() throw(std::bad_alloc, PcoException)
{
    this->lock();
    try
    {
        // Camera now busy
        this->setIntegerParam(this->ADStatus, ADStatusReadout);
        // Get configuration information
        this->getIntegerParam(this->ADTriggerMode, &this->triggerMode);
        this->getIntegerParam(this->ADNumImages, &this->numImages);
        this->getIntegerParam(this->ADImageMode, &this->imageMode);
        this->getIntegerParam(this->handleTimestampMode, &this->timestampMode);
        this->getIntegerParam(ADMaxSizeX, &this->xMaxSize);
        this->getIntegerParam(ADMaxSizeY, &this->yMaxSize);
        this->getIntegerParam(ADMinX, &this->reqRoiStartX);
        this->getIntegerParam(ADMinY, &this->reqRoiStartY);
        this->getIntegerParam(ADSizeX, &this->reqRoiSizeX);
        this->getIntegerParam(ADSizeY, &this->reqRoiSizeY);
        this->getIntegerParam(ADBinX, &this->reqBinX);
        this->getIntegerParam(ADBinY, &this->reqBinY);
        this->getIntegerParam(this->handleAdcMode, &this->adcMode);
        this->getIntegerParam(this->handleBitAlignment, &this->bitAlignmentMode);
        this->getIntegerParam(this->handleAcquireMode, &this->acquireMode);
        this->getIntegerParam(this->handlePixRate, &this->pixRateValue);
        this->pixRate = this->camDescription.pixelRate[this->pixRateEnumValues[this->pixRateValue]];
        this->getDoubleParam(this->ADAcquireTime, &this->exposureTime);
        this->getDoubleParam(this->ADAcquirePeriod, &this->acquisitionPeriod);
        this->getDoubleParam(this->handleDelayTime, &this->delayTime);
        this->getIntegerParam(this->handleCameraSetup, &this->cameraSetup);
        this->getIntegerParam(this->NDDataType, &this->dataType);
        this->getIntegerParam(this->ADReverseX, &this->reverseX);
        this->getIntegerParam(this->ADReverseY, &this->reverseY);
        this->getDoubleParam(this->handleExpTimeMin, &this->minExposureTime);
        this->getDoubleParam(this->handleExpTimeMax, &this->maxExposureTime);
        this->getDoubleParam(this->handleDelayTimeMin, &this->minDelayTime);
        this->getDoubleParam(this->handleDelayTimeMax, &this->maxDelayTime);

        // Configure the camera (reading back the actual settings)
        this->cfgBinningAndRoi();    // Also sets camera image size
        this->cfgTriggerMode();
        this->cfgTimestampMode();
        this->cfgAcquireMode();
        this->cfgAdcMode();
        this->cfgBitAlignmentMode();
        this->cfgPixelRate();
        this->cfgAcquisitionTimes();
        this->allocateImageBuffers();
        this->adjustTransferParamsAndLut();

        // Update what we have really set
        this->setIntegerParam(this->ADMinX, this->reqRoiStartX);
        this->setIntegerParam(this->ADMinY, this->reqRoiStartY);
        this->setIntegerParam(this->ADSizeX, this->reqRoiSizeX);
        this->setIntegerParam(this->ADSizeY, this->reqRoiSizeY);
        this->setIntegerParam(this->ADTriggerMode, this->triggerMode);
        this->setIntegerParam(this->handleTimestampMode, this->timestampMode);
        this->setIntegerParam(this->handleAcquireMode, this->acquireMode);
        this->setIntegerParam(this->handleAdcMode, this->adcMode);
        this->setIntegerParam(this->handleBitAlignment, this->bitAlignmentMode);
        this->setIntegerParam(this->handlePixRate, this->pixRateValue);
        this->setDoubleParam(this->ADAcquireTime, this->exposureTime);
        this->setDoubleParam(this->ADAcquirePeriod, this->acquisitionPeriod);
        this->setDoubleParam(this->handleDelayTime, this->delayTime);
        this->setIntegerParam(this->handleHwBinX, this->hwBinX);
        this->setIntegerParam(this->handleHwBinY, this->hwBinY);
        this->setIntegerParam(this->handleHwRoiX1, this->hwRoiX1);
        this->setIntegerParam(this->handleHwRoiY1, this->hwRoiY1);
        this->setIntegerParam(this->handleHwRoiX2, this->hwRoiX2);
        this->setIntegerParam(this->handleHwRoiY2, this->hwRoiY2);
        this->setIntegerParam(this->handleXCamSize, this->xCamSize);
        this->setIntegerParam(this->handleYCamSize, this->yCamSize);

        // Set the image parameters for the image buffer transfer inside the CamLink and GigE interface.
        // While using CamLink or GigE this function must be called, before the user tries to get images
        // from the camera and the sizes have changed. With all other interfaces this is a dummy call.
        this->api->camlinkSetImageParameters(this->camera, this->xCamSize, this->yCamSize);

        // Make sure the pco camera clock is correct
        this->setCameraClock();

        // Give the buffers to the camera
        this->addAvailableBufferAll();
        this->lastImageNumber = 0;
        this->lastImageNumberValid = false;

        // Now Arm the camera, so it is ready to take images, all settings should have been made by now
        this->api->arm(this->camera);

        // Start the camera recording
        this->api->setRecordingState(this->camera, DllApi::recorderStateOn);

        // The PCO4000 appears to output 1,2 or 3 dodgy frames immediately on
        // getting the arm.  This bit of code tries to drop them.
        if(this->camType == DllApi::cameraType4000)
        {
            this->unlock();
            epicsThreadSleep(0.3);
            this->lock();
            this->discardImages();        // Dump any images
            this->stateMachine->clear();  // Dump any frame received messages
        }
        
        // Update EPICS
        this->callParamCallbacks();
        this->unlock();
    }
    catch(PcoException& e)
    {
        this->callParamCallbacks();
        this->unlock();
        throw e;
    }
    catch(std::bad_alloc& e)
    {
        this->callParamCallbacks();
        this->unlock();
        throw e;
    }
}

/**
 * Configure the ADC mode
 */
void Pco::cfgAdcMode() throw(PcoException)
{
    unsigned short v;
    if(this->camType == DllApi::cameraType1600 ||
            this->camType == DllApi::cameraType2000 ||
            this->camType == DllApi::cameraType4000)
    {
        this->api->setAdcOperation(this->camera, this->adcMode);
        this->api->getAdcOperation(this->camera, &v);
        this->adcMode = v;
    }
    else
    {
        this->adcMode = DllApi::adcModeSingle;
    }
}

/**
 * Configure the acquire mode
 */
void Pco::cfgAcquireMode() throw(PcoException)
{
    unsigned short v;
    this->api->setAcquireMode(this->camera, this->acquireMode);
    this->api->getAcquireMode(this->camera, &v);
    this->acquireMode = v;
}

/**
 * Configure the bit alignment mode
 */
void Pco::cfgBitAlignmentMode() throw(PcoException)
{
    unsigned short v;
    this->api->setBitAlignment(this->camera, this->bitAlignmentMode);
    this->api->getBitAlignment(this->camera, &v);
    this->bitAlignmentMode = v;
}

/**
 * Configure the timestamp mode
 */
void Pco::cfgTimestampMode() throw(PcoException)
{
    unsigned short v;
    if(this->camDescription.generalCaps & DllApi::generalCapsNoTimestamp)
    {
        // No timestamp available
        this->timestampMode = DllApi::timestampModeOff;
    }
    else if(this->camDescription.generalCaps & DllApi::generalCapsTimestampAsciiOnly)
    {
        // All timestamp modes are available
        this->api->setTimestampMode(this->camera, this->timestampMode);
        this->api->getTimestampMode(this->camera, &v);
        this->timestampMode = v;
    }
    else
    {
        // No ASCII only timestamps available
        if(this->timestampMode == DllApi::timestampModeAscii)
        {
            this->timestampMode = DllApi::timestampModeBinaryAndAscii;
        }
        this->api->setTimestampMode(this->camera, this->timestampMode);
        this->api->getTimestampMode(this->camera, &v);
        this->timestampMode = v;
    }
}

/**
 * Configure the trigger mode.
 * Handle the external only trigger mode by translating to the
 * regular external trigger mode.
 */
void Pco::cfgTriggerMode() throw(PcoException)
{
    unsigned short v;
    if(this->triggerMode == DllApi::triggerExternalOnly)
    {
        this->api->setTriggerMode(this->camera, DllApi::triggerExternal);
        this->api->getTriggerMode(this->camera, &v);
        if(v != DllApi::triggerExternal)
        {
            this->triggerMode = (int)v;
        }
    }
    else
    {
        this->api->setTriggerMode(this->camera, this->triggerMode);
        this->api->getTriggerMode(this->camera, &v);
        this->triggerMode = (int)v;
    }
}

/**
 * Configure the binning and region of interest.
 */
void Pco::cfgBinningAndRoi() throw(PcoException)
{
    // Work out the software and hardware binning
    if(this->availBinX.find(this->reqBinX) == this->availBinX.end())
    {
        // Not a binning the camera can do
        this->hwBinX = Pco::defaultHorzBin;
        this->swBinX = this->reqBinX;
    }
    else
    {
        // A binning the camera can do
        this->hwBinX = this->reqBinX;
        this->swBinX = Pco::defaultHorzBin;
    }
    if(this->availBinY.find(this->reqBinY) == this->availBinY.end())
    {
        // Not a binning the camera can do
        this->hwBinY = Pco::defaultVertBin;
        this->swBinY = this->reqBinY;
    }
    else
    {
        // A binning the camera can do
        this->hwBinY = this->reqBinY;
        this->swBinY = Pco::defaultHorzBin;
    }
    this->api->setBinning(this->camera, this->hwBinX, this->hwBinY);
    this->xCamSize = this->camSizes.xResActual / this->hwBinX;
    this->yCamSize = this->camSizes.yResActual / this->hwBinY;

    // Make requested ROI valid
    this->reqRoiStartX = std::max(this->reqRoiStartX, 0);
    this->reqRoiStartX = std::min(this->reqRoiStartX, this->xCamSize-1);
    this->reqRoiStartY = std::max(this->reqRoiStartY, 0);
    this->reqRoiStartY = std::min(this->reqRoiStartY, this->yCamSize-1);
    this->reqRoiSizeX = std::max(this->reqRoiSizeX, 0);
    this->reqRoiSizeX = std::min(this->reqRoiSizeX, this->xCamSize-this->reqRoiStartX);
    this->reqRoiSizeY = std::max(this->reqRoiSizeY, 0);
    this->reqRoiSizeY = std::min(this->reqRoiSizeY, this->yCamSize-this->reqRoiStartY);

    // Get the desired hardware ROI (zero based, end not inclusive)
    this->hwRoiX1 = this->reqRoiStartX;
    this->hwRoiX2 = this->reqRoiStartX+this->reqRoiSizeX;
    this->hwRoiY1 = this->reqRoiStartY;
    this->hwRoiY2 = this->reqRoiStartY+this->reqRoiSizeY;

    // Enforce horizontal symmetry requirements
    if(this->adcMode == DllApi::adcModeDual ||
            this->camType == DllApi::cameraTypeDimaxStd ||
            this->camType == DllApi::cameraTypeDimaxTv ||
            this->camType == DllApi::cameraTypeDimaxAutomotive)
    {
        if(this->hwRoiX1 <= this->xCamSize-this->hwRoiX2)
        {
            this->hwRoiX2 = this->xCamSize - this->hwRoiX1;
        }
        else
        {
            this->hwRoiX1 = this->xCamSize - this->hwRoiX2;
        }
    }

    // Enforce vertical symmetry requirements
    if(this->camType == DllApi::cameraTypeEdge ||
            this->camType == DllApi::cameraTypeEdgeGl ||
            this->camType == DllApi::cameraTypeDimaxStd ||
            this->camType == DllApi::cameraTypeDimaxTv ||
            this->camType == DllApi::cameraTypeDimaxAutomotive)
    {
        if(this->hwRoiY1 <= this->yCamSize-this->hwRoiY2)
        {
            this->hwRoiY2 = this->yCamSize - this->hwRoiY1;
        }
        else
        {
            this->hwRoiY1 = this->yCamSize - this->hwRoiY2;
        }
    }

    // Enforce stepping requirements
    this->hwRoiX1 = (this->hwRoiX1 / this->camDescription.roiHorSteps) *
            this->camDescription.roiHorSteps;
    this->hwRoiY1 = (this->hwRoiY1 / this->camDescription.roiVertSteps) *
            this->camDescription.roiVertSteps;
    this->hwRoiX2 = ((this->hwRoiX2+this->camDescription.roiHorSteps-1) /
            this->camDescription.roiHorSteps) * this->camDescription.roiHorSteps;
    this->hwRoiY2 = ((this->hwRoiY2+this->camDescription.roiVertSteps-1) /
            this->camDescription.roiVertSteps) * this->camDescription.roiVertSteps;

    // Work out the software ROI that cuts off the remaining bits in coordinates
    // relative to the hardware ROI
    this->swRoiStartX = this->reqRoiStartX - this->hwRoiX1;
    this->swRoiStartY = this->reqRoiStartY - this->hwRoiY1;
    this->swRoiSizeX = this->reqRoiSizeX;
    this->swRoiSizeY = this->reqRoiSizeY;

    // Record the size of the frame coming from the camera
    this->xCamSize = this->hwRoiX2 - this->hwRoiX1;
    this->yCamSize = this->hwRoiY2 - this->hwRoiY1;

    // Now change to 1 based coordinates and inclusive end, set the ROI
    // in the hardware
    this->hwRoiX1 += 1;
    this->hwRoiY1 += 1;
    this->api->setRoi(this->camera,
            (unsigned short)this->hwRoiX1, (unsigned short)this->hwRoiY1,
            (unsigned short)this->hwRoiX2, (unsigned short)this->hwRoiY2);

    // Set up the software ROI
    ::memset(this->arrayDims, 0, sizeof(NDDimension_t) * Pco::numDimensions);
    this->arrayDims[Pco::xDimension].offset = this->swRoiStartX;
    this->arrayDims[Pco::yDimension].offset = this->swRoiStartY;
    this->arrayDims[Pco::xDimension].size = this->swRoiSizeX;
    this->arrayDims[Pco::yDimension].size = this->swRoiSizeY;
    this->arrayDims[Pco::xDimension].binning = this->swBinX;
    this->arrayDims[Pco::yDimension].binning = this->swBinY;
    this->arrayDims[Pco::xDimension].reverse = this->reverseX;
    this->arrayDims[Pco::yDimension].reverse = this->reverseY;
    this->roiRequired =
            this->arrayDims[Pco::xDimension].offset != 0 ||
            this->arrayDims[Pco::yDimension].offset != 0 ||
            (int)this->arrayDims[Pco::xDimension].size != this->xCamSize ||
            (int)this->arrayDims[Pco::yDimension].size != this->yCamSize ||
            this->arrayDims[Pco::xDimension].binning != 1 ||
            this->arrayDims[Pco::yDimension].binning != 1 ||
            this->arrayDims[Pco::xDimension].reverse != 0 ||
            this->arrayDims[Pco::yDimension].reverse != 0 ||
            (NDDataType_t)this->dataType != NDUInt16;
}

/**
 * Configure the pixel rate
 */
void Pco::cfgPixelRate() throw(PcoException)
{
    this->api->setPixelRate(this->camera, (unsigned long)this->pixRate);
    unsigned long v;
    this->api->getPixelRate(this->camera, &v);
    this->pixRate = (int)v;
}

/**
 * Write the acquisition times to the camera
 */
void Pco::cfgAcquisitionTimes() throw(PcoException)
{
    // Get the information
    // Work out the delay time to achieve the desired period.  Note that the
    // configured delay time is used unless it is zero, in which case the
    // acquisition period is used.
    double exposureTime = this->exposureTime;
    double delayTime = this->delayTime;
    if(delayTime == 0.0)
    {
        delayTime = std::max(this->acquisitionPeriod - this->exposureTime, 0.0);
    }
    // Check them against the camera's constraints;
    if(delayTime < this->minDelayTime)
    {
        delayTime = this->minDelayTime;
    }
    if(delayTime > this->maxDelayTime)
    {
        delayTime = this->maxDelayTime;
    }
    if(exposureTime < this->minExposureTime)
    {
        exposureTime = this->minExposureTime;
    }
    if(exposureTime > this->maxExposureTime)
    {
        exposureTime = this->maxExposureTime;
    }
    // Work out the best ranges to use to represent to the camera
    unsigned short exposureBase;
    unsigned long exposure;
    unsigned short delayBase;
    unsigned long delay;
    if(this->exposureTime < Pco::timebaseNanosecondsThreshold)
    {
        exposureBase = DllApi::timebaseNanoseconds;
    }
    else if(this->exposureTime < Pco::timebaseMicrosecondsThreshold)
    {
        exposureBase = DllApi::timebaseMicroseconds;
    }
    else
    {
        exposureBase = DllApi::timebaseMilliseconds;
    }
    if(delayTime < Pco::timebaseNanosecondsThreshold)
    {
        delayBase = DllApi::timebaseNanoseconds;
    }
    else if(delayTime < Pco::timebaseMicrosecondsThreshold)
    {
        delayBase = DllApi::timebaseMicroseconds;
    }
    else
    {
        delayBase = DllApi::timebaseMilliseconds;
    }
    // Set the camera
    delay = (unsigned long)(delayTime * DllApi::timebaseScaleFactor[delayBase]);
    exposure = (unsigned long)(exposureTime * DllApi::timebaseScaleFactor[exposureBase]);
    this->api->setDelayExposureTime(this->camera, delay, exposure,
            delayBase, exposureBase);
    // Read back what the camera is actually set to
    this->api->getDelayExposureTime(this->camera, &delay, &exposure,
            &delayBase, &exposureBase);
    this->exposureTime = (double)exposure / DllApi::timebaseScaleFactor[exposureBase];
    delayTime = (double)delay / DllApi::timebaseScaleFactor[delayBase];
    if(this->delayTime != 0.0)
    {
        this->delayTime = delayTime;
    }
    this->acquisitionPeriod = this->exposureTime + delayTime;
}

/**
 * Indicate to EPICS that acquisition has begun.
 */
void Pco::nowAcquiring() throw()
{
    this->lock();
    // Get info
    this->getIntegerParam(this->NDArrayCounter, &this->arrayCounter);
    this->getIntegerParam(this->ADNumImages, &this->numImages);
    this->getIntegerParam(this->ADNumExposures, &this->numExposures);
    if(this->imageMode == ADImageSingle)
    {
        this->numImages = 1;
    }
    // Clear counters
    this->numImagesCounter = 0;
    this->numExposuresCounter = 0;
    this->outOfNDArrays = 0;
    this->bufferQueueReadFailures = 0;
    this->buffersWithNoData = 0;
    this->misplacedBuffers = 0;
    this->missingFrames = 0;
    this->driverLibraryErrors = 0;
    // Set info
    this->setIntegerParam(this->ADStatus, ADStatusReadout);
    this->setIntegerParam(this->ADAcquire, 1);
    this->setIntegerParam(this->NDArraySize, this->xCamSize*this->yCamSize*sizeof(unsigned short));
    this->setIntegerParam(this->NDArraySizeX, this->xCamSize);
    this->setIntegerParam(this->NDArraySizeY, this->yCamSize);
    this->setIntegerParam(this->ADNumImagesCounter, this->numImagesCounter);
    this->setIntegerParam(this->ADNumExposuresCounter, this->numExposuresCounter);
    // Update EPICS
    this->callParamCallbacks();
    this->unlock();
    this->updateErrorCounters();
}

/**
 * An acquisition has completed
 */
void Pco::acquisitionComplete() throw()
{
    this->lock();
    this->setIntegerParam(this->ADStatus, ADStatusIdle);
    this->setIntegerParam(this->ADAcquire, 0);
    this->callParamCallbacks();
    this->triggerTimer->stop();
    this->unlock();
}

/**
 * Exit the armed state
 */
void Pco::doDisarm() throw()
{
    this->lock();
    this->setIntegerParam(this->handleArmMode, 0);
    this->callParamCallbacks();
    try
    {
        this->api->setRecordingState(this->camera, DllApi::recorderStateOff);
    }
    catch(PcoException&)
    {
        // Not much we can do with this error
    }
    this->freeImageBuffers();
    this->unlock();
}

/**
 * Update EPICS with the state of the error counters
 */
void Pco::updateErrorCounters() throw()
{
    this->lock();
    this->setIntegerParam(this->handleOutOfNDArrays, this->outOfNDArrays);
    this->setIntegerParam(this->handleBufferQueueReadFailures, this->bufferQueueReadFailures);
    this->setIntegerParam(this->handleBuffersWithNoData, this->buffersWithNoData);
    this->setIntegerParam(this->handleMisplacedBuffers, this->misplacedBuffers);
    this->setIntegerParam(this->handleMissingFrames, this->missingFrames);
    this->setIntegerParam(this->handleDriverLibraryErrors, this->driverLibraryErrors);
    this->callParamCallbacks();
    this->unlock();
}

/**
 * Start the camera by sending a software trigger if we are in one
 * of the soft modes
 */
void Pco::startCamera() throw()
{
    // Start the camera if we are in one of the soft modes
    if(this->triggerMode == DllApi::triggerSoftware ||
        this->triggerMode == DllApi::triggerExternal)
    {
        unsigned short triggerState = 0;
        try
        {
            this->api->forceTrigger(this->camera, &triggerState);
        }
        catch(PcoException&)
        {
            this->driverLibraryErrors++;
            this->updateErrorCounters();
        }
        // Schedule a retry if it fails
        if(!triggerState)
        {
            // Trigger did not succeed, try again soon
            this->triggerTimer->start(Pco::triggerRetryPeriod, Pco::requestTrigger);
        }
    }
}

/**
 * Discard all images waiting in the queue.
 */
void Pco::discardImages() throw()
{
    while(this->receivedFrameQueue.pending() > 0)
    {
        NDArray* image = NULL;
        this->receivedFrameQueue.tryReceive(&image, sizeof(NDArray*));
        if(image != NULL)
        {
            image->release();
        }
    }
}

/**
 * Receive all available images from the camera.  This function is called in
 * response to an image ready event, but we read all images and cope if there are
 * none so that missing image ready events don't stall the system.  Receiving
 * stops when the queue is empty or the acquisition is complete.  Returns
 * true if the acquisition is complete.
 */
bool Pco::receiveImages() throw()
{
    // Poll the buffer queue
    // Note that the API has aready reset the event so the event status bit
    // returned by getBufferStatus will already be clear.  However, for
    // buffers that do have data ready return a statusDrv of zero.
    while(this->receivedFrameQueue.pending() > 0 &&
            this->numImagesCounter < this->numImages)
    {
        // Get the image
        NDArray* image = NULL;
        this->receivedFrameQueue.tryReceive(&image, sizeof(NDArray*));
        if(image != NULL)
        {
            // What is the number of the image?  If the image does not
            // contain the BCD image number
            // use the dead reckoning number instead.
            long imageNumber = this->lastImageNumber + 1;
            if(this->timestampMode == DllApi::timestampModeBinary ||
                this->timestampMode == DllApi::timestampModeBinaryAndAscii)
            {
                imageNumber = this->extractImageNumber(
                        (unsigned short*)image->pData);
            }
            // If this is the image we are expecting?
            if(this->lastImageNumberValid && imageNumber != this->lastImageNumber+1)
            {
                this->missingFrames++;
                printf("Missing frame, got=%ld, exp=%ld\n", imageNumber, this->lastImageNumber+1);
                this->lock();
                this->setIntegerParam(this->handleMissingFrames, this->missingFrames);
                this->unlock();
            }
            this->lastImageNumber = imageNumber;
            // Do software ROI, binning and reversal if required
            if(this->roiRequired)
            {
                NDArray* scratch;
                this->pNDArrayPool->convert(image, &scratch,
                        (NDDataType_t)this->dataType, this->arrayDims);
                image->release();
                image = scratch;
            }
            // Handle summing of multiple exposures
            bool nextImageReady = false;
            if(this->numExposures > 1)
            {
                this->numExposuresCounter++;
                if(this->numExposuresCounter > 1)
                {
                    switch(image->dataType)
                    {
                    case NDUInt8:
                    case NDInt8:
                        sumArray<epicsUInt8>(image, this->imageSum);
                        break;
                    case NDUInt16:
                    case NDInt16:
                        sumArray<epicsUInt16>(image, this->imageSum);
                        break;
                    case NDUInt32:
                    case NDInt32:
                        sumArray<epicsUInt32>(image, this->imageSum);
                        break;
                    default:
                        break;
                    }
                    // throw away the previous accumulator
                    this->imageSum->release();
                }
                // keep the sum of previous images for the next iteration
                this->imageSum = image;
                if(this->numExposuresCounter >= this->numExposures)
                {
                    // we have finished accumulating
                    nextImageReady = true;
                    this->numExposuresCounter = 0;
                }
            }
            else
            {
                nextImageReady = true;
            }
            if(nextImageReady)
            {
                // Attach the image information
                image->uniqueId = this->arrayCounter;
                epicsTimeStamp imageTime;
                if(this->timestampMode == DllApi::timestampModeBinary ||
                        this->timestampMode == DllApi::timestampModeBinaryAndAscii)
                {
                    this->extractImageTimeStamp(&imageTime,
                            (unsigned short*)image->pData);
                }
                else
                {
                    epicsTimeGetCurrent(&imageTime);
                }
                image->timeStamp = imageTime.secPastEpoch +
                        imageTime.nsec / Pco::oneNanosecond;
                this->getAttributes(image->pAttributeList);
                // Update statistics
                this->arrayCounter++;
                if(this->imageMode != ADImageContinuous)
                {
                    this->numImagesCounter++;
                }
                // Pass the array on
                this->doCallbacksGenericPointer(image, NDArrayData, 0);
                image->release();
            }
        }
    }
    this->lock();
    this->setIntegerParam(this->NDArrayCounter, arrayCounter);
    this->setIntegerParam(this->ADNumImagesCounter, this->numImagesCounter);
    this->setIntegerParam(this->ADNumExposuresCounter, this->numExposuresCounter);
    this->setIntegerParam(this->handleImageNumber, this->lastImageNumber);
    this->callParamCallbacks();
    this->unlock();
    return this->numImagesCounter >= this->numImages;
}

/**
 * Convert BCD coded number in image to int 
 */
long Pco::bcdToInt(unsigned short pixel) throw()
{
    int shiftLowBcd = Pco::bitsPerShortWord - this->camDescription.dynResolution;
    int shiftHighBcd = shiftLowBcd + Pco::bitsPerNybble;
    long p1 = (pixel>>shiftLowBcd)&(Pco::nybbleMask);
    long p2 = (pixel>>shiftHighBcd)&(Pco::nybbleMask);
    return p2*bcdDigitValue + p1;    
}

/**
 * Convert bcd number in first 4 pixels of image to extract image counter value
 */
long Pco::extractImageNumber(unsigned short* imagebuffer) throw()
{

    long imageNumber = 0;
    for(int i=0; i<Pco::bcdPixelLength; i++) 
    {
        imageNumber *= Pco::bcdDigitValue * Pco::bcdDigitValue;
        imageNumber += bcdToInt(imagebuffer[i]);
    };
    return imageNumber;
}

/**
 * Convert bcd numbers in pixels 5 to 14 of image to extract time stamp 
 */
void Pco::extractImageTimeStamp(epicsTimeStamp* imageTime,
        unsigned short* imageBuffer) throw()
{
    unsigned long nanoSec = 0;
    struct tm ct;
    ct.tm_year = bcdToInt(imageBuffer[4])*100 + bcdToInt(imageBuffer[5] - 1900);
    ct.tm_mon = bcdToInt(imageBuffer[6])-1;
    ct.tm_mday = bcdToInt(imageBuffer[7]);
    ct.tm_hour = bcdToInt(imageBuffer[8]);
    ct.tm_min = bcdToInt(imageBuffer[9]);
    ct.tm_sec = bcdToInt(imageBuffer[10]);
    nanoSec = (bcdToInt(imageBuffer[11])*10000 + bcdToInt(imageBuffer[12])*100 +
            bcdToInt(imageBuffer[13]))*1000;
   
#if 0
    // JAT: We'll comment this out for now and see what the PCO
    //      does return.
    // fix year if necessary
    // (pco4000 seems to always return 2036 for year)
    if(ct.tm_year >= 2036)
    {
        ct.tm_year = this->cameraYear;
    }
#endif
        
    epicsTimeFromTM (imageTime, &ct, nanoSec );
}

/**
 * Helper function to sum 2 NDArrays
 */
template<typename T> void Pco::sumArray(NDArray* startingArray,
        NDArray* addArray) throw()
{
    T* inOutData = reinterpret_cast<T*>(startingArray->pData);
    T* addData = reinterpret_cast<T*>(addArray->pData);
    NDArrayInfo_t inInfo;

    startingArray->getInfo(&inInfo);
    for(int i=0; i<this->xCamSize*this->yCamSize; i++)
    {
        *inOutData += *addData;
        inOutData++;
        addData++;
    }
}

// IOC shell configuration command
extern "C" int pcoConfig(const char* portName, int maxBuffers, size_t maxMemory)
{
    Pco* existing = Pco::getPco(portName);
    if(existing == NULL)
    {
        new Pco(portName, maxBuffers, maxMemory);
    }
    else
    {
        printf("Error: port name \"%s\" already exists\n", portName);
    }
    return asynSuccess;
}
static const iocshArg pcoConfigArg0 = {"Port name", iocshArgString};
static const iocshArg pcoConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg pcoConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg * const pcoConfigArgs[] = {&pcoConfigArg0, &pcoConfigArg1,
        &pcoConfigArg2};
static const iocshFuncDef configPco = {"pcoConfig", 3, pcoConfigArgs};
static void configPcoCallFunc(const iocshArgBuf *args)
{
    pcoConfig(args[0].sval, args[1].ival, args[2].ival);
}

/** Register the commands */
static void pcoRegister(void)
{
    iocshRegister(&configPco, configPcoCallFunc);
}
extern "C" { epicsExportRegistrar(pcoRegister); }

