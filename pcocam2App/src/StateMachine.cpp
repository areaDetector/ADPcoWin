/**
 * State machine template class
 */

#include "StateMachine.h"
#include "TraceStream.h"
#include <string.h>

/**
 * Timer class constructor.
 * \param[in] machine The owning state machine
 * \param[in] queue The timer queue to place the timer on
 */
StateMachine::Timer::Timer(StateMachine* machine)
            : timer(machine->timerQueue.createTimer())
            , machine(machine)
            , expiryEvent(0)
{
}

/**
 * Timer class destructor.
 */
StateMachine::Timer::~Timer()
{
    this->timer.destroy();
}

/**
 * Start the timer.
 * \param[in] delay Time in seconds to timer expiry
 * \param[in] expiryEvent The event to post to the state machine on expiry
 */
void StateMachine::Timer::start(double delay, int expiryEvent)
{
    this->expiryEvent = expiryEvent;
    this->timer.start(*this, delay);
}

/**
 * Stop the timer.  If the timer is already stopped, nothing happens.  Note
 * that this function won't remove any expiry events that may have already been
 * posted to the state machines message queue.
 */
void StateMachine::Timer::stop()
{
    this->timer.cancel();
}

/**
 * This function is called when the timer expires.  Post the expiry event
 * to the state machine's queue.  Note that state machine timers are never
 * restarted automatically.
 * \param[in] currentTime The current time
 * \return Indicate whether the timer should be restarted.
 */
StateMachine::Timer::expireStatus StateMachine::Timer::expire(const epicsTime& currentTime)
{
    this->machine->post(this->expiryEvent);
    return noRestart;
}

/**
 * Constructor.
 * \param[in] name The name of the machine, used in trace messages.
 * \param[in] portUser An asynUser object to use when outputting trace.
 * \param[in] portDriver The asynPortDriver object that contains the state record.
 * \param[in] traceFlag The asyn trace flag to use when outputting trace.
 * \param[in] handleRecord The asyn string parameter handle of the state record.
 * \param[in] user Pointer to the state machine user that handles transitions.
 * \param[in] initial The initial state of the machine.
 * \param[in] stateNames An array of state name strings.
 * \param[in] eventNames An array of event name strings.
 * \param[in] requestQueueCapacity The size of the event queue.
 */
StateMachine::StateMachine(const char* name,
        asynPortDriver* portDriver,
        int handleRecord, User* user, int initial,
        const char** stateNames, const char** eventNames,
        TraceStream* tracer, int requestQueueCapacity)
    : state(initial)
    , name(name)
    , tracer(tracer)
    , portDriver(portDriver)
    , handleRecord(handleRecord)
    , user(user)
    , stateNames(stateNames)
    , eventNames(eventNames)
    , requestQueue(requestQueueCapacity, sizeof(int))
    , thread(*this, name, epicsThreadGetStackSize(epicsThreadStackMedium))
    , timerQueue(epicsTimerQueueActive::allocate(true))
    , timer(this)
{
    // Start the thread
    this->thread.start();
}

/**
 * Destructor.
 */
StateMachine::~StateMachine()
{
    this->timerQueue.release();
}

/**
 * Post an event to the request queue.
 * \param[in] req The event to post.
 */
void StateMachine::post(int req)
{
    if(this->tracer != NULL)
    {
        this->tracer->printf("%s: post request = %s[%d]\n",
                this->name.c_str(), this->eventNames[req], req);
        //*this->tracer << this->name << ": post request = " << this->eventNames[req] <<
        //        "[" << req << "]" << std::endl;
    }
    this->requestQueue.trySend(&req, sizeof(int));
}

/**
 * Start the timer.
 * \param[in] delay Time in seconds to timer expiry
 * \param[in] expiryEvent The event to post to the state machine on expiry
 */
void StateMachine::startTimer(double delay, int expiryEvent)
{
    this->timer.stop();
    this->timer.start(delay, expiryEvent);
}

/**
 * Stop the timer/
 */
void StateMachine::stopTimer()
{
    this->timer.stop();
}

/**
 * The function that is run by the thread.  Runs forever processing
 * events from the message queue when available.
 */
void StateMachine::run()
{
    while(true)
    {
        int event;
        if(this->requestQueue.receive(&event, sizeof(int)) == sizeof(int))
        {
            // Get the event processed
            int nextState = this->user->doTransition(this, this->state, event);
            // Do the trace
            if(this->tracer != NULL)
            {
                this->tracer->printf("%s: %s[%d] --%s[%d]--> %s[%d]\n",
                        this->name.c_str(), this->stateNames[this->state],
                        this->state, this->eventNames[event], event,
                        this->stateNames[nextState], nextState);
                //*this->tracer << this->name << ": " << this->stateNames[this->state] <<
                //        "[" << this->state << "] --" << this->eventNames[event] <<
                //        "[" << event << "]--> " << this->stateNames[nextState] <<
                //        "[" << nextState << "]" << std::endl;
            }
            // Update the state record
            this->portDriver->lock();
            char stateRecord[maxStateRecordLength+1];
            this->portDriver->getStringParam(this->handleRecord, maxStateRecordLength,
                    stateRecord);
            stateRecord[maxStateRecordLength] = '\0';
            size_t stateRecordLen = strlen(stateRecord);
            if(stateRecordLen < maxStateRecordLength)
            {
                stateRecord[stateRecordLen] += ('0'+(char)nextState);
                stateRecord[stateRecordLen+1] = '\0';
                this->portDriver->setStringParam(this->handleRecord, stateRecord);
            }
            this->portDriver->unlock();
            // Change the state
            this->state = nextState;
        }
    }
}

/**
 * Return the number of events on the queue.
 */
 int StateMachine::pending()
 {
     return this->requestQueue.pending();
 }

