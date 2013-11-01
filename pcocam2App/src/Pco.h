/* Pco.h
 *
 * Revamped PCO area detector driver.
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */
#ifndef PCO_H_
#define PCO_H_

#include <ADDriver.h>
#include <string>
#include <map>
#include <set>
#include "StateMachine.h"
#include "DllApi.h"
#include "TraceStream.h"
#include "NDArrayException.h"

class Pco: public ADDriver, public StateMachine::User
{
// Construction
public:
    Pco(const char* portName, int maxBuffers, size_t maxMemory);
    virtual ~Pco();

// Overrides of ADDriver
public:
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser,
        const char *value, size_t nChars, size_t *nActual);

// Override of StateMachine::User
public:
    virtual int doTransition(StateMachine* machine, int state, int event);

// Parameter handles
protected:
    int handlePixRate;
    int handleAdcMode;
    int handleCamRamUse;
    int handleElectronicsTemp;
    int handlePowerTemp;
    int handleStorageMode;
    int handleRecorderSubmode;
    int handleTimestampMode;
    int handleAcquireMode;
    int handleDelayTime;
    int handleArmMode;
    int handleImageNumber;
    int handleCameraSetup;
    int handleBitAlignment;
    int handleStateRecord;
    int handleClearStateRecord;
    int handleOutOfNDArrays;
    int handleBufferQueueReadFailures;
    int handleBuffersWithNoData;
    int handleMisplacedBuffers;
    int handleMissingFrames;
    int handleDriverLibraryErrors;
    int handleHwBinX;
    int handleHwBinY;
    int handleHwRoiX1;
    int handleHwRoiY1;
    int handleHwRoiX2;
    int handleHwRoiY2;
    int handleXCamSize;
    int handleYCamSize;

// Parameter names
private:
    static const char* namePixRate;
    static const char* nameAdcMode;
    static const char* nameCamRamUse;
    static const char* nameElectronicsTemp;
    static const char* namePowerTemp;
    static const char* nameStorageMode;
    static const char* nameRecorderSubmode;
    static const char* nameTimestampMode;
    static const char* nameAcquireMode;
    static const char* nameDelayTime;
    static const char* nameArmMode;
    static const char* nameImageNumber;
    static const char* nameCameraSetup;
    static const char* nameBitAlignment;
    static const char* nameStateRecord;
    static const char* nameClearStateRecord;
    static const char* nameOutOfNDArrays;
    static const char* nameBufferQueueReadFailures;
    static const char* nameBuffersWithNoData;
    static const char* nameMisplacedBuffers;
    static const char* nameMissingFrames;
    static const char* nameDriverLibraryErrors;
    static const char* nameHwBinX;
    static const char* nameHwBinY;
    static const char* nameHwRoiX1;
    static const char* nameHwRoiY1;
    static const char* nameHwRoiX2;
    static const char* nameHwRoiY2;
    static const char* nameXCamSize;
    static const char* nameYCamSize;

// Constants
public:
    static const int traceFlagsDllApi;
    static const int traceFlagsPcoState;
    static const int requestQueueCapacity;
    static const int numHandles;
    static const double reconnectPeriod;
    static const double connectPeriod;
    static const double statusPollPeriod;
    static const double acquisitionStatusPollPeriod;
    static const char* stateNames[];
    static const char* eventNames[];
    static const int bitsPerShortWord;
    static const int bitsPerNybble;
    static const long nybbleMask;
    static const long bcdDigitValue;
    static const int bcdPixelLength;
    static const int defaultHorzBin;
    static const int defaultVertBin;
    static const int defaultRoiMinX;
    static const int defaultRoiMinY;
    static const int defaultExposureTime;
    static const int defaultDelayTime;
    enum {numApiBuffers=8};
    static const int edgeXSizeNeedsReducedCamlink;
    static const int edgePixRateNeedsReducedCamlink;
    static const int edgeBaudRate;
    static const double timebaseNanosecondsThreshold;
    static const double timebaseMicrosecondsThreshold;
    static const double oneNanosecond;
    static const double oneMillisecond;
    static const double triggerRetryPeriod;
    static const int statusMessageSize;
    enum {xDimension=0, yDimension=1, numDimensions=2};
// Types
public:
    enum Request {requestInitialise=0, requestTimerExpiry, requestAcquire,
        requestStop, requestArm, requestImageReceived, requestDisarm,
        requestTrigger};
    enum State {stateUninitialised=0, stateUnconnected, stateIdle,
        stateArmed, stateAcquiring, statedUnarmedAcquiring, stateExternalAcquiring};

// API for use by component classes
public:
    void post(Request req);
    void frameReceived(int bufferNumber);
    void trace(int flags, const char* format, ...);
    asynUser* getAsynUser();
    void registerDllApi(DllApi* api);

// Member variables
private:
    StateMachine* stateMachine;
    StateMachine::Timer* triggerTimer;
    DllApi* api;
    DllApi::Handle camera;
    unsigned short camType;
    DllApi::Description camDescription;
    unsigned long camRamSize;
    unsigned int camPageSize;
    DllApi::Transfer camTransfer;
    DllApi::Sizes camSizes;
    int shiftLowBcd;         // Shift for decoding the BCD frame number in image
    int shiftHighBcd;        // Shift for decoding the BCD frame number in image
    TraceStream errorTrace;
public:
    TraceStream apiTrace;
private:
    TraceStream stateTrace;
    struct
    {
        short bufferNumber;
        unsigned short* buffer;
        DllApi::Handle eventHandle;
        bool ready;
    } buffers[Pco::numApiBuffers];
    long lastImageNumber;
    bool lastImageNumberValid;
    epicsMessageQueue receivedFrameQueue;
    int numImagesCounter;
    int numExposuresCounter;
    int numImages;
    int numExposures;
    int cameraYear;
    int arrayCounter;
    std::set<int> availBinX;
    std::set<int> availBinY;
    // Pixel rate information
    char* pixRateEnumStrings[DllApi::descriptionNumPixelRates];
    int pixRateEnumValues[DllApi::descriptionNumPixelRates];
    int pixRateEnumSeverities[DllApi::descriptionNumPixelRates];
    int pixRateNumEnums;
    int pixRateMaxRateEnum;
    int pixRate;
    int pixRateValue;
    int pixRateMax;
    int pixRateMaxValue;
    // Config information for an acquisition
    int xMaxSize;        // The sensor size...
    int yMaxSize;        // ...as returned by camera info
    int xCamSize;        // The size being generated by the camera...
    int yCamSize;        // ...which is max size / binning - ROI
    int triggerMode;
    int imageMode;
    int timestampMode;
    double exposureTime;
    double acquisitionPeriod;
    double delayTime;
    int reverseX;
    int reverseY;
    int adcMode;
    int bitAlignmentMode;
    int acquireMode;
    int cameraSetup;
    int dataType;
    int hwBinX;
    int hwBinY;
    int swBinX;
    int swBinY;
    int reqBinX;
    int reqBinY;
    int hwRoiX1;
    int hwRoiY1;
    int hwRoiX2;
    int hwRoiY2;
    int swRoiStartX;
    int swRoiStartY;
    int swRoiSizeX;
    int swRoiSizeY;
    int reqRoiStartX;
    int reqRoiStartY;
    int reqRoiSizeX;
    int reqRoiSizeY;
    NDArray* imageSum;
    NDDimension_t arrayDims[numDimensions];
    bool roiRequired;
    // Error counters
    int outOfNDArrays;
    int bufferQueueReadFailures;
    int buffersWithNoData;
    int misplacedBuffers;
    int missingFrames;
    int driverLibraryErrors;
public:
    static std::map<std::string, Pco*> thePcos;
    static Pco* getPco(const char* portName);

// Utility functions
private:
    bool connectToCamera();
    bool pollCameraNoAcquisition();
    bool pollCamera();
    void doArm() throw(std::bad_alloc, PcoException);
    void doDisarm() throw();
    void nowAcquiring() throw();
    void startCamera() throw();
    void allocateImageBuffers() throw(std::bad_alloc, PcoException);
    void freeImageBuffers() throw();
    void adjustTransferParamsAndLut() throw(PcoException);
    void setCameraClock() throw(PcoException);
    void addAvailableBuffer(int index) throw(PcoException);
    void addAvailableBufferAll() throw(PcoException);
    bool receiveImages() throw();
    void discardImages() throw();
    long extractImageNumber(unsigned short* imagebuffer) throw();
    long bcdToInt(unsigned short pixel) throw();
    void extractImageTimeStamp(epicsTimeStamp* imageTime, unsigned short* imageBuffer) throw();
    void updateErrorCounters() throw();
    void acquisitionComplete() throw();
    int checkMemoryBuffer() throw(PcoException);
    void setValidBinning(std::set<int>& valid, int max, int step) throw();
    void cfgBinningAndRoi() throw(PcoException);
    void cfgTriggerMode() throw(PcoException);
    void cfgTimestampMode() throw(PcoException);
    void cfgAcquireMode() throw(PcoException);
    void cfgAdcMode() throw(PcoException);
    void cfgBitAlignmentMode() throw(PcoException);
    void cfgPixelRate() throw(PcoException);
    void cfgAcquisitionTimes() throw(PcoException);
    template<typename T> void sumArray(NDArray* startingArray,
            NDArray* addArray) throw();
    void initialisePixelRate();
};

#endif
