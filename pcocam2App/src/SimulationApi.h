/*
 * SimulationApi.h
 *
 * Revamped PCO area detector driver.
 *
 * A simulation of the PCO API library and hardware
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#ifndef SIMULATIONAPI_H_
#define SIMULATIONAPI_H_

#include <string>
#include <epicsMessageQueue.h>
#include "StateMachine.h"
#include "DllApi.h"
class Pco;
class TraceStream;

class SimulationApi: public DllApi, public StateMachine::User
{
// Construction
public:
    SimulationApi(Pco* pco, TraceStream* trace);
    virtual ~SimulationApi();

// Overrides of DllApi
protected:
    virtual int doOpenCamera(Handle* handle, unsigned short camNum);
    virtual int doCloseCamera(Handle handle);
    virtual int doRebootCamera(Handle handle);
    virtual int doGetGeneral(Handle handle);
    virtual int doGetCameraType(Handle handle, unsigned short* camType);
    virtual int doGetSensorStruct(Handle handle);
    virtual int doGetCameraDescription(Handle handle, Description* description);
    virtual int doGetStorageStruct(Handle handle, unsigned long* ramSize, unsigned int* pageSize);
    virtual int doGetRecordingStruct(Handle handle);
    virtual int doResetSettingsToDefault(Handle handle);
    virtual int doGetTransferParameters(Handle handle, Transfer* transfer);
    virtual int doSetTransferParameters(Handle handle, Transfer* transfer);
    virtual int doGetSizes(Handle handle, Sizes* sizes);
    virtual int doSetDateTime(Handle handle, struct tm* currentTime);
    virtual int doGetTemperature(Handle handle, short* ccd, short* camera, short* psu);
    virtual int doSetPixelRate(Handle handle, unsigned long pixRate);
    virtual int doGetPixelRate(Handle handle, unsigned long* pixRate);
    virtual int doGetBitAlignment(Handle handle, unsigned short* bitAlignment);
    virtual int doSetBitAlignment(Handle handle, unsigned short bitAlignment);
    virtual int doGetCameraSetup(Handle handle, unsigned short* setupType,
            unsigned long* setupData, unsigned short* setupDataLen);
    virtual int doSetBinning(Handle handle, unsigned short binHorz, unsigned short binVert);
    virtual int doGetBinning(Handle handle, unsigned short* binHorz, unsigned short* binVert);
    virtual int doSetRoi(Handle handle, unsigned short x0, unsigned short y0,
            unsigned short x1, unsigned short y1);
    virtual int doGetRoi(Handle handle, unsigned short* x0, unsigned short* y0,
            unsigned short* x1, unsigned short* y1);
    virtual int doSetTriggerMode(Handle handle, unsigned short mode);
    virtual int doGetTriggerMode(Handle handle, unsigned short* mode);
    virtual int doSetStorageMode(Handle handle, unsigned short mode);
    virtual int doGetStorageMode(Handle handle, unsigned short* mode);
    virtual int doSetTimestampMode(Handle handle, unsigned short mode);
    virtual int doGetTimestampMode(Handle handle, unsigned short* mode);
    virtual int doSetAcquireMode(Handle handle, unsigned short mode);
    virtual int doGetAcquireMode(Handle handle, unsigned short* mode);
    virtual int doSetDelayExposureTime(Handle handle, unsigned long delay,
            unsigned long exposure, unsigned short timeBaseDelay,
            unsigned short timeBaseExposure);
    virtual int doGetDelayExposureTime(Handle handle, unsigned long* delay,
            unsigned long* exposure, unsigned short* timeBaseDelay,
            unsigned short* timeBaseExposure);
    virtual int doSetConversionFactor(Handle handle, unsigned short factor);
    virtual int doSetAdcOperation(Handle handle, unsigned short mode);
    virtual int doGetAdcOperation(Handle handle, unsigned short* mode);
    virtual int doGetRecordingState(Handle handle, unsigned short* state);
    virtual int doSetRecordingState(Handle handle, unsigned short state);
    virtual int doGetRecorderSubmode(Handle handle, unsigned short* mode);
    virtual int doAllocateBuffer(Handle handle, short* bufferNumber, unsigned long size,
            unsigned short** buffer, Handle* event);
    virtual int doCancelImages(Handle handle);
    virtual int doCamlinkSetImageParameters(Handle handle, unsigned short xRes, unsigned short yRes);
    virtual int doArm(Handle handle);
    virtual int doAddBufferEx(Handle handle, unsigned long firstImage, unsigned long lastImage, 
        short bufferNumber, unsigned short xRes, unsigned short yRes, unsigned short bitRes);
    virtual int doGetBufferStatus(Handle handle, short bufferNumber, unsigned long* statusDll,
        unsigned long* statusDrv);
    virtual int doForceTrigger(Handle handle, unsigned short* triggered);
    virtual int doFreeBuffer(Handle handle, short bufferNumber);
    virtual int doGetActiveRamSegment(Handle handle, unsigned short* segment);
    virtual int doGetNumberOfImagesInSegment(Handle handle, unsigned short segment,
            unsigned long* validImageCount, unsigned long* maxImageCount);

// Override of StateMachine::User
public:
    virtual int doTransition(StateMachine* machine, int state, int event);

// Overrides of asynPortDriver
public:
	virtual void writeInt32(asynUser *pasynUser, epicsInt32 value);

// Parameter names
private:
	static const char* nameConnected;
    static const char* nameOpen;
    static const char* nameCameraType;
    static const char* nameMaxHorzRes;
    static const char* nameMaxVertRes;
    static const char* nameDynResolution;
    static const char* nameMaxBinHorz;
    static const char* nameMaxBinVert;
    static const char* nameBinHorzStepping;
    static const char* nameBinVertStepping;
    static const char* nameRoiHorSteps;
    static const char* nameRoiVertSteps;
    static const char* namePixelRate;
    static const char* nameConvFact;
    static const char* nameGeneralCaps;
	static const char* nameRamSize;
	static const char* namePageSize;
    static const char* nameBaudRate;      
    static const char* nameClockFrequency;
    static const char* nameCamlinkLines;  
    static const char* nameDataFormat;    
    static const char* nameTransmit;      
    static const char* nameActualHorzRes;
    static const char* nameActualVertRes;
    static const char* nameTimeYear;
    static const char* nameTimeMonth;
    static const char* nameTimeDay;
    static const char* nameTimeHour;
    static const char* nameTimeMinute;
    static const char* nameTimeSecond;
    static const char* nameTempCcd;
    static const char* nameTempCamera;
    static const char* nameTempPsu;
    static const char* nameBitAlignment;
    static const char* nameEdgeGlobalShutter;
    static const char* nameActualHorzBin;
    static const char* nameActualVertBin;
    static const char* nameActualRoiX0;
    static const char* nameActualRoiY0;
    static const char* nameActualRoiX1;
    static const char* nameActualRoiY1;
    static const char* nameTriggerMode;
    static const char* nameStorageMode;
    static const char* nameTimestampMode;
    static const char* nameAcquireMode;
    static const char* nameDelayTime;
    static const char* nameDelayTimebase;
    static const char* nameExposureTime;
    static const char* nameExposureTimebase;
    static const char* nameActualConvFact;
    static const char* nameAdcOperation;
    static const char* nameRecordingState;
    static const char* nameRecorderSubmode;
    static const char* nameCamlinkHorzRes;
    static const char* nameCamlinkVertRes;
    static const char* nameArmed;
    static const char* nameStateRecord;
    static const char* nameClearStateRecord;
    static const char* nameExternalTrigger;

// Parameter handles
private:
    int handleConnected;
    int handleOpen;;
    int handleCameraType;
    int handleMaxHorzRes;
    int handleMaxVertRes;
    int handleDynResolution;
    int handleMaxBinHorz;
    int handleMaxBinVert;
    int handleBinHorzStepping;
    int handleBinVertStepping;
    int handleRoiHorSteps;
    int handleRoiVertSteps;
    int handlePixelRate;
    int handleConvFact;
    int handleGeneralCaps;
	int handleRamSize;
	int handlePageSize;
    int handleBaudRate;
    int handleClockFrequency;
    int handleCamlinkLines;  
    int handleDataFormat;    
    int handleTransmit;      
    int handleActualHorzRes;
    int handleActualVertRes;
    int handleTimeYear;
    int handleTimeMonth;
    int handleTimeDay;
    int handleTimeHour;
    int handleTimeMinute;
    int handleTimeSecond;
    int handleTempCcd;
    int handleTempCamera;
    int handleTempPsu;
    int handleBitAlignment;
    int handleEdgeGlobalShutter;
    int handleActualHorzBin;
    int handleActualVertBin;
    int handleActualRoiX0;
    int handleActualRoiY0;
    int handleActualRoiX1;
    int handleActualRoiY1;
    int handleTriggerMode;
    int handleStorageMode;
    int handleTimestampMode;
    int handleAcquireMode;
    int handleDelayTime;
    int handleDelayTimebase;
    int handleExposureTime;
    int handleExposureTimebase;
    int handleActualConvFact;
    int handleAdcOperation;
    int handleRecordingState;
    int handleRecorderSubmode;
    int handleCamlinkHorzRes;
    int handleCamlinkVertRes;
    int handleArmed;
    int handleStateRecord;
    int handleClearStateRecord;
    int handleExternalTrigger;

// Types
public:
    enum Request {requestConnectionUp=0, requestConnectionDown, requestOpen,
        requestClose, requestStartRecording, requestStopRecording,
        requestTrigger, requestArm, requestCancelImages};
    enum State {stateConnected=0, stateOpen, stateDisconnected, stateArmed,
        stateRecording};

// Constants
protected:
    static const int edgeSetupDataLength;
    static const int edgeSetupDataType;
    static const char* stateNames[];
    static const char* eventNames[];

// Members
protected:
    struct
    {
        unsigned short* buffer;
        unsigned long status;
    } buffers[DllApi::maxNumBuffers];
    epicsMessageQueue bufferQueue;
    StateMachine* stateMachine;
    int frameNumber;

// Functions
protected:
    void post(Request req);
    void generateFrame();
    void startTriggerTimer();
};

#endif /* SIMULATIONAPI_H_ */
