/* TakeLock.cpp
 * See .h file header for description.
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "TakeLock.h"
#include "asynPortDriver.h"

/**
 * Constructor.  Use this to take the lock (or represent an already
 * taken lock if alreadyTaken is true).
 */
TakeLock::TakeLock(asynPortDriver* driver, bool alreadyTaken)
	: driver(driver)
	, initiallyTaken(alreadyTaken)
	, locked(alreadyTaken)
{
	lock();
}

/**
 * Constructor.  Use this constructor to create another take
 * lock that changes the state of the lock.
 */
TakeLock::TakeLock(TakeLock& takeLock, bool unlock)
	: driver(takeLock.driver)
	, initiallyTaken(takeLock.locked)
	, locked(takeLock.locked)
{
	if(unlock)
	{
		unlock();
	}
	else
	{
		lock();
	}
}


/**
 * Destructor.  Call parameter call backs (with the lock taken) and
 * return the lock to its initial state.
 */
TakeLock::~TakeLock()
{
	callParamCallbacks();
	if(!initiallyTaken)
	{
		unlock();
	}
}

/**
 * Make sure the lock is taken.
 */
void TakeLock::lock()
{
	if(!locked)
	{
		driver->lock();
		locked = true;
	}
}

/**
 * Make sure the lock is released.
 */
void TakeLock::unlock()
{
	if(locked)
	{
		driver->unlock();
		locked = false;
	}
}

/**
 * Update EPICS parameters.  The function must be
 * called with the lock taken, so this is ensured and
 * the lock returned to its original state after.
 */
void TakeLock::callParamCallbacks()
{
	bool wasLocked = locked;
	lock();
	driver->callParamCallbacks();
	if(!wasLocked)
	{
		unlock();
	}
}

