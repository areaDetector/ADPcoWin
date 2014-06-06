/**
 * State machine template class
 */

#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include <string>
#include "epicsMessageQueue.h"
#include "epicsThread.h"
#include "epicsTimer.h"
#include "asynPortDriver.h"
class TraceStream;

class StateMachine: public epicsThreadRunable
{
public:
    class User
    {
    public:
        User() {}
        virtual ~User() {}
        virtual int doTransition(StateMachine* machine, int state, int event) = 0;
    };
public:
    class Timer: public epicsTimerNotify
    {
    private:
        epicsTimer& timer;
        StateMachine* machine;
        int expiryEvent;
    public:
        Timer(StateMachine* machine);
        virtual ~Timer();
        void start(double delay, int expiryEvent);
        void stop();
        virtual expireStatus expire(const epicsTime& currentTime);
    };
private:
    int state;
    std::string name;
    TraceStream* tracer;
    asynPortDriver* portDriver;
    int handleRecord;
    User* user;
    enum {maxStateRecordLength=40};
    const char** stateNames;
    const char** eventNames;
    epicsMessageQueue requestQueue;
    epicsThread thread;
    epicsTimerQueueActive& timerQueue;
    Timer timer;
public:
    StateMachine(const char* name, asynPortDriver* portDriver,
            int handleRecord, User* user, int initial,
            const char** stateNames, const char** eventNames,
            TraceStream* tracer=NULL, int requestQueueCapacity=10);
    virtual ~StateMachine();
    void post(int req);
    void startTimer(double delay, int expiryEvent);
    void stopTimer();
    virtual void run();
    int pending();
    void clear();
};

#endif

