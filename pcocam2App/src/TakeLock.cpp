/* TakeLock.cpp
 * See .h file header for description.
 *
 * Author:  Jonathan Thompson
 *
 */

#include "TakeLock.h"
#include "FreeLock.h"
#include "asynPortDriver.h"

/**
 * Constructor.  Use this to take the lock (or represent an already
 * taken lock if alreadyTaken is true).
 */
TakeLock::TakeLock(asynPortDriver* driver, bool alreadyTaken)
	: driver(driver)
	, initiallyTaken(alreadyTaken)
{
	if(!alreadyTaken)
	{
		driver->lock();
	}
}

/**
 * Constructor.  Use this to take a lock that is represented by a FreeLock.
 */
TakeLock::TakeLock(FreeLock& freeLock)
	: driver(freeLock.driver)
	, initiallyTaken(false)
{
	driver->lock();
}

/**
 * Destructor.  Call parameter call backs (with the lock taken) and
 * return the lock to its initial state.
 */
TakeLock::~TakeLock()
{
	driver->callParamCallbacks();
	if(!initiallyTaken)
	{
		driver->unlock();
	}
}

