/* TakeLock.cpp
 * See .h file header for description.
 *
 * Author:  Jonathan Thompson
 *
 */

#include "FreeLock.h"
#include "TakeLock.h"
#include "asynPortDriver.h"

// Constructor.  Free a taken lock
FreeLock::FreeLock(TakeLock& takeLock)
	: driver(takeLock.driver)
{
	driver->unlock();
}

// Destructor.  Return the lock to the taken state.
FreeLock::~FreeLock()
{
	driver->lock();
}

