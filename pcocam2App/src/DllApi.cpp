/*
 * Api.cpp
 *
 * Revamped pixium area detector driver.
 *
 * Virtual base class for the PU API library
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "DllApi.h"
#include <ctime>
#include "TraceStream.h"
#include "Pco.h"

/* Constants */
const double DllApi::ccdTemperatureScaleFactor = 10.0;
const double DllApi::timebaseScaleFactor[DllApi::numTimebases] =
    {1000000000.0, 1000000.0, 1000.0};

/**
 * Constructor
 */
DllApi::DllApi(Pco* pco, TraceStream* trace)
: pco(pco)
, trace(trace)
{
    this->pco->registerDllApi(this);
}

/**
 * Destructor
 */
DllApi::~DllApi()
{
}

/**
 * Connect to the camera
 */
void DllApi::openCamera(Handle* handle, unsigned short camNum) throw(PcoException)
{
    int result = doOpenCamera(handle, camNum);
    *this->trace << "DllApi->OpenCamera(" << *handle << ", " <<
            camNum << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("openCamera", result);
    }
}

/**
 * Disconnect from the camera
 */
void DllApi::closeCamera(Handle handle) throw(PcoException)
{
    int result = doCloseCamera(handle);
    *this->trace << "DllApi->CloseCamera(" << handle << ") = " <<
            result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("closeCamera", result);
    }
}

/**
 * Get general information from the camera
 */
void DllApi::getGeneral(Handle handle) throw(PcoException)
{
    int result = doGetGeneral(handle);
    *this->trace << "DllApi->GetGeneral(" << handle << ") = " <<
            result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getGeneral", result);
    }
}

/**
 * Get the camera type information
 */
void DllApi::getCameraType(Handle handle, unsigned short* camType) throw(PcoException)
{
    int result = doGetCameraType(handle, camType);
    *this->trace << "DllApi->GetCameraType(" << handle << ", " <<
            *camType << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getCameraType", result);
    }
}

/**
 * Get sensor information
 */
void DllApi::getSensorStruct(Handle handle) throw(PcoException)
{
    int result = doGetSensorStruct(handle);
    *this->trace << "DllApi->GetSensorStruct(" << handle << ") = " <<
            result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getSensorStruct", result);
    }
}

/**
 * Get camera description
 */
void DllApi::getCameraDescription(Handle handle, Description* description) throw(PcoException)
{
    int result = doGetCameraDescription(handle, description);
    *this->trace << "DllApi->GetCameraDescription(" << handle << ", {" <<
        description->maxHorzRes << "," << description->maxVertRes << ", " <<
        description->maxBinHorz << "," << description->maxBinVert << ", " <<
        description->binHorzStepping << "," << description->binVertStepping << ", " <<
        description->roiHorSteps << "," << description->roiVertSteps << ", {" <<
        description->pixelRate[0] << "," << description->pixelRate[1] << "," <<
        description->pixelRate[2] << "," << description->pixelRate[3] << "}, " <<
        description->convFact << "}) = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getCameraDescription", result);
    }
}

/**
 * Get camera storage information
 */
void DllApi::getStorageStruct(Handle handle, unsigned long* ramSize, unsigned int* pageSize) throw(PcoException)
{
    int result = doGetStorageStruct(handle, ramSize, pageSize);
    *this->trace << "DllApi->GetStorageStruct(" << handle << ", " <<
        *ramSize << ", " << *pageSize << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getStorageStruct", result);
    }
}

/**
 * Get camera recording information
 */
void DllApi::getRecordingStruct(Handle handle) throw(PcoException)
{
    int result = doGetRecordingStruct(handle);
    *this->trace << "DllApi->GetRecordingStruct(" << handle << ") = " <<
            result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getRecordingStruct", result);
    }
}

/**
 * Reset the camera's settings
 */
void DllApi::resetSettingsToDefault(Handle handle) throw(PcoException)
{
    int result = doResetSettingsToDefault(handle);
    *this->trace << "DllApi->ResetSettingsToDefault(" << handle << ") = " <<
            result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("resetSettingsToDefault", result);
    }
}

/**
 * Get the camera's transfer parameters
 */
void DllApi::getTransferParameters(Handle handle, Transfer* transfer) throw(PcoException)
{
    int result = doGetTransferParameters(handle, transfer);
    *this->trace << "DllApi->GetTransferParameters(" << handle << ", " <<
        transfer->baudRate << ", " << transfer->clockFrequency << ", " <<
        transfer->camlinkLines << ", " << transfer->dataFormat << ", " <<
        transfer->transmit << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getTransferParameters", result);
    }
}

/**
 * Set the camera's transfer parameters
 */
void DllApi::setTransferParameters(Handle handle, Transfer* transfer) throw(PcoException)
{
    int result = doSetTransferParameters(handle, transfer);
    *this->trace << "DllApi->SetTransferParameters(" << handle << ", " <<
        transfer->baudRate << ", " << transfer->clockFrequency << ", " <<
        transfer->camlinkLines << ", " << transfer->dataFormat << ", " <<
        transfer->transmit << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setTransferParameters", result);
    }
}

/**
 * The the camera's current and maximum resolutions
 */
void DllApi::getSizes(Handle handle, Sizes* sizes) throw(PcoException)
{
    int result = doGetSizes(handle, sizes);
    *this->trace << "DllApi->GetSizes(" << handle << ", " <<
        sizes->xResActual << ", " << sizes->yResActual << ", " <<
        sizes->xResMaximum << ", " << sizes->yResMaximum << ") = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getSizes", result);
    }
}

/**
 * Set the camera's date and time
 */
void DllApi::setDateTime(Handle handle, struct tm* currentTime) throw(PcoException)
{
    int result = doSetDateTime(handle, currentTime);
    *this->trace << "DllApi->SetDateTime(" << handle << ", " <<
        currentTime->tm_mday << ", " << currentTime->tm_mon << ", " <<
        currentTime->tm_year << ", " << currentTime->tm_hour << ", " <<
        currentTime->tm_min << ", " << currentTime->tm_sec << ") = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setDateTime", result);
    }
}

/**
 * Get camera temperatures
 */
void DllApi::getTemperature(Handle handle, short* ccd,
        short* camera, short* psu) throw(PcoException)
{
    int result = doGetTemperature(handle, ccd, camera, psu);
    *this->trace << "DllApi->GetTemperature(" << handle << ", " <<
        *ccd << ", " << *camera << ", " << *psu << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getTemperature", result);
    }
}

/**
 * Set the camera's operating pixel rate
 */
void DllApi::setPixelRate(Handle handle, unsigned long pixRate) throw(PcoException)
{
    int result = doSetPixelRate(handle, pixRate);
    *this->trace << "DllApi->SetPixelRate(" << handle << ", " <<
        pixRate << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setPixelRate", result);
    }
}

/**
 * Get the camera's operating pixel rate
 */
void DllApi::getPixelRate(Handle handle, unsigned long* pixRate) throw(PcoException)
{
    int result = doGetPixelRate(handle, pixRate);
    *this->trace << "DllApi->GetPixelRate(" << handle << ", " <<
        *pixRate << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getPixelRate", result);
    }
}

/**
 * Set the operating bit alignment
 */
void DllApi::setBitAlignment(Handle handle, unsigned short bitAlignment) throw(PcoException)
{
    int result = doSetBitAlignment(handle, bitAlignment);
    *this->trace << "DllApi->SetBitAlignment(" << handle << ", " <<
        bitAlignment << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setBitAlignment", result);
    }
}

/**
 * Get the operating bit alignment
 */
void DllApi::getBitAlignment(Handle handle, unsigned short* bitAlignment) throw(PcoException)
{
    int result = doGetBitAlignment(handle, bitAlignment);
    *this->trace << "DllApi->GetBitAlignment(" << handle << ", " <<
        *bitAlignment << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getBitAlignment", result);
    }
}

/**
 * Return an Edge's camera setup information
 */
void DllApi::getCameraSetup(Handle handle, unsigned short* setupType,
        unsigned long* setupData, unsigned short* setupDataLen) throw(PcoException)
{
    int result = doGetCameraSetup(handle, setupType, setupData, setupDataLen);
    *this->trace << "DllApi->GetCameraSetup(" << handle << ", " <<
        *setupType << ", " << setupData[0] << ", " << *setupDataLen << ") = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getCameraSetup", result);
    }
}

/**
 * Set the binning parameters.
 */
void DllApi::setBinning(Handle handle, unsigned short binHorz, unsigned short binVert) throw(PcoException)
{
    int result = doSetBinning(handle, binHorz, binVert);
    *this->trace << "DllApi->SetBinning(" << handle << ", " <<
        binHorz << ", " << binVert << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setBinning", result);
    }
}

/**
 * Get the binning parameters.
 */
void DllApi::getBinning(Handle handle, unsigned short* binHorz, unsigned short* binVert) throw(PcoException)
{
    int result = doGetBinning(handle, binHorz, binVert);
    *this->trace << "DllApi->GetBinning(" << handle << ", " <<
        *binHorz << ", " << *binVert << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getBinning", result);
    }
}

/**
 * Set the region of interest parameters
 */
void DllApi::setRoi(Handle handle, unsigned short x0, unsigned short y0,
        unsigned short x1, unsigned short y1) throw(PcoException)
{
    int result = doSetRoi(handle, x0, y0, x1, y1);
    *this->trace << "DllApi->SetRoi(" << handle << ", " <<
        x0 << "," << y0 << ", " << x1 << "," << y1 << ") = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setRoi", result);
    }
}

/**
 * Get the region of interest parameters
 */
void DllApi::getRoi(Handle handle, unsigned short* x0, unsigned short* y0,
        unsigned short* x1, unsigned short* y1) throw(PcoException)
{
    int result = doGetRoi(handle, x0, y0, x1, y1);
    *this->trace << "DllApi->GetRoi(" << handle << ", " <<
        *x0 << "," << *y0 << ", " << *x1 << "," << *y1 << ") = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getRoi", result);
    }
}

/**
 * Set the trigger mode
 */
void DllApi::setTriggerMode(Handle handle, unsigned short mode) throw(PcoException)
{
    int result = doSetTriggerMode(handle, mode);
    *this->trace << "DllApi->SetTriggerMode(" << handle << ", " <<
        mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setTriggerMode", result);
    }
}

/**
 * Get the trigger mode
 */
void DllApi::getTriggerMode(Handle handle, unsigned short* mode) throw(PcoException)
{
    int result = doGetTriggerMode(handle, mode);
    *this->trace << "DllApi->GetTriggerMode(" << handle << ", " <<
        *mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getTriggerMode", result);
    }
}

/**
 * Set the storage mode
 */
void DllApi::setStorageMode(Handle handle, unsigned short mode) throw(PcoException)
{
    int result = doSetStorageMode(handle, mode);
    *this->trace << "DllApi->SetStorageMode(" << handle << ", " <<
        mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setStorageMode", result);
    }
}

/**
 * Get the storage mode
 */
void DllApi::getStorageMode(Handle handle, unsigned short* mode) throw(PcoException)
{
    int result = doGetStorageMode(handle, mode);
    *this->trace << "DllApi->GetStorageMode(" << handle << ", " <<
        *mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getStorageMode", result);
    }
}

/**
 * Set the time stamp mode
 */
void DllApi::setTimestampMode(Handle handle, unsigned short mode) throw(PcoException)
{
    int result = doSetTimestampMode(handle, mode);
    *this->trace << "DllApi->SetTimestampMode(" << handle << ", " <<
        mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setTimestampMode", result);
    }
}

/**
 * Get the time stamp mode
 */
void DllApi::getTimestampMode(Handle handle, unsigned short* mode) throw(PcoException)
{
    int result = doGetTimestampMode(handle, mode);
    *this->trace << "DllApi->GetTimestampMode(" << handle << ", " <<
        *mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getTimestampMode", result);
    }
}

/**
 * Set the acquire mode
 */
void DllApi::setAcquireMode(Handle handle, unsigned short mode) throw(PcoException)
{
    int result = doSetAcquireMode(handle, mode);
    *this->trace << "DllApi->SetAcquireMode(" << handle << ", " <<
        mode << ", " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setAcquireMode", result);
    }
}

/**
 * Get the acquire mode
 */
void DllApi::getAcquireMode(Handle handle, unsigned short* mode) throw(PcoException)
{
    int result = doGetAcquireMode(handle, mode);
    *this->trace << "DllApi->GetAcquireMode(" << handle << ", " <<
        *mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getAcquireMode", result);
    }
}

/**
 * Set the delay and exposure times
 */
void DllApi::setDelayExposureTime(Handle handle, unsigned long delay,
        unsigned long exposure, unsigned short timeBaseDelay,
        unsigned short timeBaseExposure) throw(PcoException)
{
    int result = doSetDelayExposureTime(handle, delay, exposure,
            timeBaseDelay, timeBaseExposure);
    *this->trace << "DllApi->SetDelayExposureTime(" << handle << ", " <<
        delay << ", " << exposure << ", " << timeBaseDelay << ", " <<
        timeBaseExposure << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setDelayExposureTime", result);
    }
}

/**
 * Get the delay and exposure times
 */
void DllApi::getDelayExposureTime(Handle handle, unsigned long* delay,
        unsigned long* exposure, unsigned short* timeBaseDelay,
        unsigned short* timeBaseExposure) throw(PcoException)
{
    int result = doGetDelayExposureTime(handle, delay, exposure,
            timeBaseDelay, timeBaseExposure);
    *this->trace << "DllApi->GetDelayExposureTime(" << handle << ", " <<
        *delay << ", " << *exposure << ", " << *timeBaseDelay << ", " <<
        *timeBaseExposure << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getDelayExposureTime", result);
    }
}

/**
 * Set the sensor gain
 */
void DllApi::setConversionFactor(Handle handle, unsigned short factor) throw(PcoException)
{
    int result = doSetConversionFactor(handle, factor);
    *this->trace << "DllApi->SetConversionFactor(" << handle << ", " <<
        factor << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setConversionFactor", result);
    }
}

/**
 * Get the ADC operating mode
 */
void DllApi::getAdcOperation(Handle handle, unsigned short* mode) throw(PcoException)
{
    int result = doGetAdcOperation(handle, mode);
    *this->trace << "DllApi->SetAdcOperation(" << handle << ", " <<
        *mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getAdcOperation", result);
    }
}

/**
 * Set the ADC operating mode
 */
void DllApi::setAdcOperation(Handle handle, unsigned short mode) throw(PcoException)
{
    int result = doSetAdcOperation(handle, mode);
    *this->trace << "DllApi->SetAdcOperation(" << handle << ", " <<
        mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setAdcOperation", result);
    }
}

/**
 * Get the camera's recording state
 */
void DllApi::getRecordingState(Handle handle, unsigned short* state) throw(PcoException)
{
    int result = doGetRecordingState(handle, state);
    *this->trace << "DllApi->GetRecordingState(" << handle << ", " <<
        *state << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getRecordingState", result);
    }
}

/**
 * Set the camera's recording state
 */
void DllApi::setRecordingState(Handle handle, unsigned short state) throw(PcoException)
{
    int result = doSetRecordingState(handle, state);
    *this->trace << "DllApi->SetRecordingState(" << handle << ", " <<
        state << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("setRecordingState", result);
    }
}

/**
 * Get the recorder submode
 */
void DllApi::getRecorderSubmode(Handle handle, unsigned short* mode) throw(PcoException)
{
    int result = doGetRecorderSubmode(handle, mode);
    *this->trace << "DllApi->GetRecorderSubmode(" << handle << ", " <<
        *mode << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getRecorderSubmode", result);
    }
}

/**
 * Allocate a buffer
 */
void DllApi::allocateBuffer(Handle handle, short* bufferNumber, unsigned long size,
        unsigned short** buffer, Handle* event) throw(PcoException)
{
    int result = doAllocateBuffer(handle, bufferNumber, size, buffer, event);
    *this->trace << "DllApi->AllocateBuffer(" << handle << ", " <<
        *bufferNumber << ", " << size << ", " << *buffer << ", " << event << ") = " <<
        result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("allocateBuffer", result);
    }
}

/**
 * Cancel all image buffers
 */
void DllApi::cancelImages(Handle handle) throw(PcoException)
{
    int result = doCancelImages(handle);
    *this->trace << "DllApi->CancelImages(" << handle <<
        ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("cancelImages", result);
    }
}

/**
 * Set the image parameters for the image buffer transfer inside the CamLink and GigE interface.
 * While using CamLink or GigE this function must be called, before the user tries to get images
 * from the camera and the sizes have changed. With all other interfaces this is a dummy call.
 */
void DllApi::camlinkSetImageParameters(Handle handle, unsigned short xRes, unsigned short yRes)
    throw(PcoException)
{
    int result = doCamlinkSetImageParameters(handle, xRes, yRes);
    *this->trace << "DllApi->CamlinkSetImageParameters(" << handle <<
        ", " << xRes << ", " << yRes << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("camlinkSetImageParameters", result);
    }
}

/**
 * Arm the camera ready for taking images.
 */
void DllApi::arm(Handle handle) throw(PcoException)
{
    int result = doArm(handle);
    *this->trace << "DllApi->Arm(" << handle <<
        ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("arm", result);
    }
}

/**
 * Add a buffer to the receive queue.
 */
void DllApi::addBufferEx(Handle handle, unsigned long firstImage, unsigned long lastImage, 
    short bufferNumber, unsigned short xRes, unsigned short yRes, 
    unsigned short bitRes) throw(PcoException)
{
    int result = doAddBufferEx(handle, firstImage, lastImage, bufferNumber, xRes, yRes, bitRes);
    this->trace->printf("DllApi->AddBufferEx(%p, %lu, %lu, %hd, %hu, %hu, %hu) = %x\n",
            handle, firstImage, lastImage, bufferNumber, xRes, yRes, bitRes, result);
    if(result != DllApi::errorNone)
    {
        throw PcoException("addBufferEx", result);
    }
}

/**
 * Get the status of a buffer.
 */
void DllApi::getBufferStatus(Handle handle, short bufferNumber, unsigned long* statusDll, 
    unsigned long* statusDrv) throw(PcoException)
{
    int result = doGetBufferStatus(handle, bufferNumber, statusDll, statusDrv);
    this->trace->printf("DllApi->GetBufferStatus(%p, %hd, %lx, %lx) = %x\n",
            handle, bufferNumber, *statusDll, *statusDrv, result);
    if(result != DllApi::errorNone)
    {
        throw PcoException("getBufferStatus", result);
    }
}

/**
 * Force a trigger if one not already running
 */
void DllApi::forceTrigger(Handle handle, unsigned short* triggered) throw(PcoException)
{
    int result = doForceTrigger(handle, triggered);
    *this->trace << "DllApi->ForceTrigger(" << handle <<
        ", " << *triggered << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("forceTrigger", result);
    }
}

/**
 * Free a buffer.  Need to do this before closing the camera.
 */
void DllApi::freeBuffer(Handle handle, short bufferNumber) throw(PcoException)
{
    int result = doFreeBuffer(handle, bufferNumber);
    *this->trace << "DllApi->FreeBuffer(" << handle <<
        ", " << bufferNumber << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("freeBuffer", result);
    }
}

/**
 * Get the active RAM segment.
 */
void DllApi::getActiveRamSegment(Handle handle, unsigned short* segment) throw(PcoException)
{
    int result = doGetActiveRamSegment(handle, segment);
    *this->trace << "DllApi->GetActiveRamSegment(" << handle <<
        ", " << *segment << ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getActiveRamSegment", result);
    }
}

/**
 * Get the number of images in a segment.
 */
void DllApi::getNumberOfImagesInSegment(Handle handle, unsigned short segment,
        unsigned long* validImageCount, unsigned long* maxImageCount) throw(PcoException)
{
    int result = doGetNumberOfImagesInSegment(handle, segment, validImageCount,
            maxImageCount);
    *this->trace << "DllApi->GetNumberOfImagesInSegment(" << handle <<
        ", " << segment << ", " << *validImageCount << ", " << *maxImageCount <<
        ") = " << result << std::endl;
    if(result != DllApi::errorNone)
    {
        throw PcoException("getNumberOfImagesInSegment", result);
    }
}

