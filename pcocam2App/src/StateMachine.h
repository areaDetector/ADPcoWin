/**
 * State machine template class
 */

#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include <string>
#include <map>
#include "epicsMessageQueue.h"
#include "epicsThread.h"
#include "epicsTimer.h"
#include "asynPortDriver.h"
class TraceStream;
class StringParam;

class StateMachine: public epicsThreadRunable
{
public:
    enum StateSelector {firstState=0, secondState, thirdState, fourthState};
	class AbstractAct
	{
	public:
		AbstractAct() {}
		virtual StateSelector operator()() = 0;
	};
	template<class Target>
	class Act: public AbstractAct
	{
	public:
		Act(Target* target, StateSelector (Target::*fn)()) :
			target(target), fn(fn) {}
		virtual StateSelector operator()() {return (target->*fn)();}
	private:
		Target* target;
		StateSelector (Target::*fn)();
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
	class TransitionAct
	{
	private:
		AbstractAct* act;
		int s1;
		int s2;
		int s3;
		int s4;
	public: 
		TransitionAct(AbstractAct* act, int s1, int s2, int s3, int s4);
		~TransitionAct();
		int execute() const;
	};
	class TransitionKey
	{
	private:
		int st;
		int ev;
	public:
		TransitionKey(int st, int ev);
		TransitionKey();
		TransitionKey(const TransitionKey& other);
		~TransitionKey();
		TransitionKey& operator=(const TransitionKey& other);
		bool operator<(const TransitionKey& other) const;
	};
	std::map<TransitionKey, TransitionAct*> transitions;
    int state;
    std::string name;
    TraceStream* tracer;
    asynPortDriver* portDriver;
    StringParam* paramRecord;
    enum {maxStateRecordLength=40};
    const char** stateNames;
    const char** eventNames;
    epicsMessageQueue requestQueue;
    epicsThread thread;
    epicsTimerQueueActive& timerQueue;
    Timer timer;
public:
    StateMachine(const char* name, asynPortDriver* portDriver,
            StringParam* paramRecord, int initial,
            const char** stateNames, const char** eventNames,
            TraceStream* tracer=NULL, int requestQueueCapacity=10);
    virtual ~StateMachine();
    void post(int req);
    void startTimer(double delay, int expiryEvent);
    void stopTimer();
    virtual void run();
    int pending();
    void clear();
    bool isState(int s);
	void transition(int initialState, int ev, AbstractAct* action, int firstState, int secondState=-1, int thirdState=-1, int fourthState=-1);
};

#endif

