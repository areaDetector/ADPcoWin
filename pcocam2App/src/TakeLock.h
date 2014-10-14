/* TakeLock.h
 *
 * Revamped PCO area detector driver.
 * A class that tracks the acquisition of the lock belonging to
 * the asynPortDriver base class.  On destruction, parameter callbacks
 * are made and the lock is returned to its original state.  Use it
 * by creating an instance on the stack when the lock is required.
 * Then, when the instance goes out of scope, the lock is automatically
 * returned to its initial state.  This scoping mechanism allows
 * exceptions to be thrown without losing track of the lock.
 *
 * The lock() and unlock() functions are for the special cases
 * where the lock may be temporarily released while something that
 * takes time is performed.  They are not normally required, the
 * scoping does the whole job.
 *
 * It is a good idea to pass the TakeLock
 * object on to called functions that require the lock to be taken
 * even though those functions may not actually access it.  The fact
 * that it is in the parameter list confirms the lock is taken.
 *
 * It can be used in functions where the lock is already taken on entry
 * (the writeXXX functions for example) by passing alreadyTaken as true.
 *
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#ifndef TAKELOCK_H_
#define TAKELOCK_H_

class asynPortDriver;

class TakeLock {
public:
	TakeLock(asynPortDriver* driver, bool alreadyTaken=false);
	TakeLock(TakeLock& takeLock, bool unlock);
	virtual ~TakeLock();
	void lock();
	void unlock();
	void callParamCallbacks();
private:
	TakeLock();
	TakeLock(const TakeLock& other);
	TakeLock& operator=(const TakeLock& other);
	asynPortDriver* driver;
	bool initiallyTaken;
	bool locked;
};

#endif /* TAKELOCK_H_ */
