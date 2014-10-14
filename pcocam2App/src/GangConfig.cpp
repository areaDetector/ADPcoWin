/* GangConfig.cpp
 *
 * Revamped PCO area detector driver.
 * A class used to transfer configuration to clients
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "GangConfig.h"
#include "Pco.h"

// Constructor
GangConfig::GangConfig()
	: exposure(0.0)
	, acqPeriod(0.0)
	, expPerImage(0)
	, numImages(0)
	, imageMode(0)
	, triggerMode(0)
	, dataType(0)
{
}

// Destructor
GangConfig::~GangConfig()
{
}

// Return a pointer to the data
void* GangConfig::data()
{
	return this;
}

// Copy the configuration from the given PCO object
void GangConfig::fromPco(Pco* pco)
{
	pco->getDoubleParam(pco->ADAcquireTime, &exposure);
	pco->getDoubleParam(pco->ADAcquirePeriod, &acqPeriod);
	pco->getIntegerParam(pco->ADNumExposures, &expPerImage);
	pco->getIntegerParam(pco->ADNumImages, &numImages);
	pco->getIntegerParam(pco->ADImageMode, &imageMode);
	pco->getIntegerParam(pco->ADTriggerMode, &triggerMode);
	pco->getIntegerParam(pco->NDDataType, &dataType);
}

// Copy the configuration to the given PCO object
void GangConfig::toPco(Pco* pco)
{
	pco->lock();
	pco->setDoubleParam(pco->ADAcquireTime, exposure);
	pco->setDoubleParam(pco->ADAcquirePeriod, acqPeriod);
	pco->setIntegerParam(pco->ADNumExposures, expPerImage);
	pco->setIntegerParam(pco->ADNumImages, numImages);
	pco->setIntegerParam(pco->ADImageMode, imageMode);
	pco->setIntegerParam(pco->ADTriggerMode, triggerMode);
	pco->setIntegerParam(pco->NDDataType, dataType);
	pco->callParamCallbacks();
	pco->unlock();
}
