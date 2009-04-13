/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Ankur Kapoor, Peter Kazanzides, Anton Deguet, Min Yang Jung
  Created on: 2004-04-30

  (C) Copyright 2004-2009 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Defines a periodic task.
*/

#ifndef _mtsTask_h
#define _mtsTask_h

#include <cisstCommon/cmnPortability.h>
#include <cisstOSAbstraction/osaThread.h>
#include <cisstOSAbstraction/osaMutex.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstMultiTask/mtsMailBox.h>
#include <cisstMultiTask/mtsCommandVoid.h>
#include <cisstMultiTask/mtsCommandRead.h>
#include <cisstMultiTask/mtsCommandWrite.h>
#include <cisstMultiTask/mtsCommandQueuedVoid.h>
#include <cisstMultiTask/mtsCommandQueuedWrite.h>
#include <cisstMultiTask/mtsDevice.h>
#include <cisstMultiTask/mtsRequiredInterface.h>
#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsHistory.h>
#include <cisstMultiTask/mtsFunctionVoid.h>

#include <set>
#include <map>

// Always include last
#include <cisstMultiTask/mtsExport.h>

/*!
  \ingroup cisstMultiTask

  This class provides the base for implementing tasks that have
  a thread, a state table to store the state at each 'tick' (increment)
  of the task, and queues to receive messages (commands) from other tasks.
  It is derived from mtsDevice, so it also contains the provided and required
  interfaces, with their lists of commands.
*/

class mtsCollectorBase;

class CISST_EXPORT mtsTask: public mtsDevice
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, 5);

    friend class mtsTaskManager;
    friend class mtsCollectorDump;

    friend class mtsCollectorBaseTest;

public:
    typedef mtsDevice BaseType;

    /*! The task states:

        CONSTRUCTED  -- Initial state set by mtsTask constructor.
        INITIALIZING -- Set by mtsTask::Create.  The task stays in this state until the
                        thread calls mtsTask::RunInternal, which calls mtsTask::StartupInternal
                        and the user-supplied mtsTask::Startup. If a new thread is created,
                        the call to mtsTask::RunInternal happens some time after the call
                        to mtsTask::Create.
        READY        -- Set by mtsTask::StartupInternal. This means that the task is ready
                        to be used (i.e., all interfaces have been initialized). Also,
                        a task can return to the READY state (from the ACTIVE state) in
                        response to a call to mtsTask::Suspend.
        ACTIVE       -- Set by mtsTask::Start.  This is the normal running state, where
                        the task is calling the user-supplied mtsTask::Run method.
        FINISHING    -- Set by mtsTask::Kill. If the mtsTask::Run method is currently
                        executing, it will finish, but no further calls will be made.
                        The task will then call mtsTask::CleanupInternal, which calls
                        the user-supplied mtsTask::Cleanup. The state will then be set
                        to FINISHED.
        FINISHED     -- The task has finished.
    */

    enum TaskStateType { CONSTRUCTED, INITIALIZING, READY, ACTIVE, FINISHING, FINISHED };

protected:
    /************************ Protected task data  *********************/

	/*! The OS independent thread object. */
	osaThread Thread;

    /*! The task state. */
    TaskStateType TaskState;

    /*! Mutex used when changing task states. */
    osaMutex StateChange;

	/*! The state data table object to store the states of the task. */
	mtsStateTable StateTable;

	/*! True if the task took more time to do computation than allocated time.
	  */
    bool OverranPeriod;

    /*! The data passed to the thread. */
    void *ThreadStartData;

    /*! The return value for RunInternal. */
    void *retValue;

    /*! Typedef for a map of connected interfaces (receiving commands). */
    typedef mtsMap<mtsRequiredInterface> RequiredInterfacesMapType;
    
    RequiredInterfacesMapType RequiredInterfaces; // Interfaces we can send commands to

    //-----------------------------
    //  Data collection related 
    //-----------------------------    
    class dataCollectionInfo {
    public:
        /*! True if collectData flag is set at the constructor. */
        bool CollectData;

        /*! Flag for enabling or disabling trigger. */
        bool TriggerEnabled;

        /*! Function bound to the command used to send the data collection event. */
        //mtsFunctionWrite TriggerEvent;
        mtsFunctionVoid TriggerEvent;

        /*! Number of data that are newly generated and are to be fetched by the 
        data collection tool. */
        unsigned int NewDataCount;

        /*! If the ratio of NewDataCount to HistoryLength is greater than this value,
            an event is triggered so that the data collector fetches all data.*/
        double EventTriggeringRatio;

        /*! If NewDataCount becomes greater than this vaule, an event for data collection
            is generated. Though this value is redundant in some respect (because
            EventTriggeringRatio is already defined), this value is kept for the purpose 
            of efficiency. */
        unsigned int EventTriggeringLimit;

        dataCollectionInfo() : CollectData(false), TriggerEnabled(false), NewDataCount(0),
            EventTriggeringRatio(0.3), EventTriggeringLimit(0) {}

        ~dataCollectionInfo() {}
    };

    dataCollectionInfo DataCollectionInfo;

    /********************* Methods to connect interfaces  *****************/

    bool ConnectRequiredInterface(const std::string & requiredInterfaceName,
                                   mtsDeviceInterface * providedInterface);


    /********************* Methods that call user methods *****************/

	/*! The member function that is passed as 'start routine' argument for
	  thread creation. */
	virtual void *RunInternal(void* argument) = 0;

    /*! The main part of the Run loop that is the same for all derived classes.
        This should not be overloaded. */
    void DoRunInternal(void);

	/*! The member funtion that is executed as soon as the thread gets created.
	  It does some housekeeping before the user code can be executed. */
	virtual void StartupInternal(void);

	/*! The member function that is executed once the task terminates. This
	  does some cleanup work. */
	virtual void CleanupInternal(void);

    /********************* Methods to process queues  *********************/

    /*! Process all messages in mailboxes. Returns number of commands processed. */
    virtual unsigned int ProcessMailBoxes(ProvidedInterfacesMapType & interfaces);

    /*! Process all queued commands. Returns number of events processed.
        These are the commands provided by all interfaces of the task. */
    inline unsigned int ProcessQueuedCommands(void) 
        { return ProcessMailBoxes(ProvidedInterfaces); }

    
    /*! Process all queued events. Returns number of events processed.
        These are the commands queued following events currently observed
        via the required interfaces. */
    unsigned int ProcessQueuedEvents(void);

    /**************** Methods for managing task timing ********************/

    /*! Delay the task by the specified amount. This is a protected member
        function because it should only be called from within the task.
        Otherwise, use osaSleep. */
    void Sleep(double timeInSeconds);

    /*********** Methods for thread start data and return values **********/

    /*! Save any 'user data' that was passed to the thread start routine. */
    virtual void SaveThreadStartData(void *data) { ThreadStartData = data; }

    virtual void SetThreadReturnValue(void *ret) { retValue = ret; }

public:
    /********************* Task constructor and destructor *****************/

	/*! Create a task with name 'name' and set the state table size (see mtsStateTable).
        This is the task base class. Tasks should be derived from one of the
        existing derived classes:  mtsTaskContinuous, mtsTaskPeriodic, and
        mtsTaskFromCallback.

        \param name  The name for the task
        \param sizeStateTable The history size of the state table

        \note The full string name is maintained in the class member data
              (in mtsDevice base class).  But, be aware that when a thread and/or
              thread buddy is created, only the first 6 characters of this name
              are used with the thread or thread buddy.  This is an artifact
              of the 6 character limit imposed by RTAI/Linux.

        \sa mtsDevice, mtsTaskContinuous, mtsTaskPeriodic, mtsTaskFromCallback
	 */
	mtsTask(const std::string & name, 
            unsigned int sizeStateTable = 256);

	/*! Default Destructor. */
	virtual ~mtsTask();

    /********************* Methods to be defined by user *****************/
    /* The Run, Startup, and Cleanup methods could be made protected.    */

	/*! Virtual method that gets overloaded to run the actual task.
	  */
    virtual void Run(void) = 0;

	/*! Virtual method that gets overloaded, and is run before the
	    task is started.
	  */
    virtual void Startup(void) {}

	/*! Virtual method that gets overloaded, and is run after the
	    task gets killed using Kill() method.
	  */
	virtual void Cleanup(void) {}

	/*! Virtual method that gets called when the task/interface needs
	  to be configured. Should it take XML info?? */
	virtual void Configure(const std::string & CMN_UNUSED(filename) = "") {}

    /********************* Methods to change task state ******************/
    /* Maybe some or all of these should be pure virtual functions.      */

    /* Create a new thread (if needed). */
    virtual void Create(void *data) = 0;
    void Create(void) { Create(0); }

	/*! Start or resume execution of the task. */
	virtual void Start(void) = 0;

	/*! Suspend the execution of the task. */
    virtual void Suspend(void) = 0;

	/*! End the task */
	virtual void Kill(void);

    /********************* Methods to query the task state ****************/
	
	/*! Return true if task is active. */
    inline bool Running(void) const { return (TaskState == ACTIVE); }

	/*! Return true if task was started. */
	inline bool IsStarted(void) const { return (TaskState >= READY); }

    /*! Return true if task is terminated. */
	inline bool IsTerminated(void) const { return (TaskState == FINISHED); }

	/*! Return true if task is marked for killing. */
	inline bool IsEndTask(void) const { return (TaskState >= FINISHING); }

    /*! Return task state. */
    inline TaskStateType GetTaskState(void) const { return TaskState; }

    /*! Convert tasks state to string representation. */
    const char *TaskStateName(TaskStateType state) const;

    /*! Return task state as a string. */
    inline const char *GetTaskStateName(void) const { return TaskStateName(TaskState); }

    /*! Return the average period. */
    double GetAveragePeriod(void) const { return StateTable.GetAveragePeriod(); }

    /********************* Methods to manage interfaces *******************/
	
    /* documented in base class */
    mtsDeviceInterface * AddProvidedInterface(const std::string & newInterfaceName);

    /*! Add a required interface.  This interface will later on be
      connected to another task and use the provided interface of the
      other task.  The required interface created also contains a list
      of event handlers to be used as observers.
      PK: should move this to base class (mtsDevice). */
    mtsRequiredInterface * AddRequiredInterface(const std::string & requiredInterfaceName, mtsRequiredInterface * requiredInterface);
    mtsRequiredInterface * AddRequiredInterface(const std::string & requiredInterfaceName);

    /*! Provide a list of existing required interfaces (by names) */ 
    std::vector<std::string> GetNamesOfRequiredInterfaces(void) const;
    
    /*! Associate an event (defined by its name) to a command object
      (i.e. handler defined by name) for a given required interface.
      This method can only work if the required interface has been
      connected to a provided interface from another task. */
    bool CISST_DEPRECATED AddObserverToRequiredInterface(const std::string & requiredInterfaceName,
                                                         const std::string & eventName,
                                                         const std::string & handlerName);

    /*! Get a pointer on the provided interface that has been
      connected to a given required interface (defined by its name).
      This method will return a null pointer if the required interface
      has not been connected.  See mtsTaskManager::Connect. */
    mtsDeviceInterface * GetProvidedInterfaceFor(const std::string & requiredInterfaceName) {
        mtsRequiredInterface *requiredInterface = RequiredInterfaces.GetItem(requiredInterfaceName, 3);
        return requiredInterface ? requiredInterface->GetConnectedInterface() : 0;
    }

    /*! Get the required interface */
    mtsRequiredInterface * GetRequiredInterface(const std::string & requiredInterfaceName) {
        return RequiredInterfaces.GetItem(requiredInterfaceName);
    }

    /********************* Methods to manage event handlers *******************/
	
    /*! Add a write command to an event handler interface associated
      to a required interface. */
    template <class __classType, class __argumentType>
    CISST_DEPRECATED
    mtsCommandWriteBase * AddEventHandlerWrite(void (__classType::*action)(const __argumentType &),
                                               __classType * classInstantiation,
                                               const std::string & requiredInterfaceName,
                                               const std::string & commandName,
                                               const __argumentType & argumentModel = __argumentType(),
                                               bool queued = true);
    
    /*! Get the command defined as user handler based on the required
      interface name and the command name. */
    CISST_DEPRECATED
    mtsCommandWriteBase * GetEventHandlerWrite(const std::string & requiredInterfaceName,
                                               const std::string & commandName);

    /*! Add a void command to an event handler interface associated
      to a required interface. */
    template <class __classType>
    CISST_DEPRECATED
    mtsCommandVoidBase * AddEventHandlerVoid(void (__classType::*action)(void),
                                             __classType * classInstantiation,
                                             const std::string & requiredInterfaceName,
                                             const std::string & commandName,
                                             bool queued = true);
    
    /*! Get the command defined as user handler based on the required
      interface name and the command name. */
    CISST_DEPRECATED
    mtsCommandVoidBase * GetEventHandlerVoid(const std::string & requiredInterfaceName,
                                             const std::string & commandName);

    
    /********************* Methods for task synchronization ***************/

	/*! Wait for task to start.
        \param timeout The timeout in seconds
        \returns true if task has started; false if timeout occurred before task started.
     */
    virtual bool WaitToStart(double timeout);

	/*! Wait for task to finish (after issuing a task Kill).
	  \param timeout  The timeout in seconds
	  \returns  true if the task terminated without timeout happening;
                false if timeout occured and task did not finish
	  */
    virtual bool WaitToTerminate(double timeout);

    /*! Suspend this task until the Wakeup method is called. */
    virtual void WaitForWakeup() { Thread.WaitForWakeup(); }

    /*! Wakeup the task. */
    virtual void Wakeup() { Thread.Wakeup(); }

    /********************* Methods for task period and overrun ************/
	
    /*! Return true if thread is periodic. */
    virtual bool IsPeriodic(void) const { return false; }

	/*! Return true if task overran allocated period. Note that this is not
        restricted to mtsTaskPeriodic.  For example, an mtsTaskFromCallback
        can overrun if a second callback occurs before the first is finished. */
    virtual bool IsOverranPeriod(void) const { return OverranPeriod; }

	/*! Reset overran period flag. */
    virtual void ResetOverranPeriod(void) { OverranPeriod = false; }

    /********************* Methods for data collection ********************/

	/*! Check if the signal has been registered. */
	int GetStateVectorID(const std::string & dataName) const;

    /*! Fetch data from state table using accessor (mtsStateTable::AccessorBase). */
    void GetStateTableHistory(mtsHistoryBase * history, 
                              const unsigned int signalIndex,
                              const unsigned int lastFetchIndex);

    /*! Enable trigger. */
    void ResetDataCollectionTrigger(void);

    /*! Activate or deactivate data collector. */
    void ActivateDataCollection(const bool activate) { 
        DataCollectionInfo.CollectData = activate;
    }

    inline const std::string GetDataCollectorProvidedInterfaceName() const { 
        return std::string("DCEventGenerator"); 
    }
};


CMN_DECLARE_SERVICES_INSTANTIATION(mtsTask)


#include <cisstMultiTask/mtsTaskInterface.h>

// these two methods are defined in mtsDevice.h but implemented here
// as they require the definition of mtsTaskInterface
template <class __classType>
inline mtsCommandVoidBase * mtsDevice::AddCommandVoid(void (__classType::*action)(void),
                                                      __classType * classInstantiation,
                                                      const std::string & providedInterfaceName,
                                                      const std::string & commandName) {
    mtsDeviceInterface * deviceInterface = this->GetProvidedInterface(providedInterfaceName);
    if (deviceInterface) {
        mtsTaskInterface * taskInterface = dynamic_cast<mtsTaskInterface *>(deviceInterface);
        if (taskInterface) {
            return taskInterface->AddCommandVoid(action, classInstantiation, commandName);
        } else {
            return deviceInterface->AddCommandVoid(action, classInstantiation, commandName);
        }
    }
    CMN_LOG_CLASS(1) << "AddCommandVoid cannot find an interface named " << providedInterfaceName << std::endl;
    return 0;
}

inline mtsCommandVoidBase * mtsDevice::AddCommandVoid(void (*action)(void),
                                                      const std::string & providedInterfaceName,
                                                      const std::string & commandName) {
    mtsDeviceInterface * deviceInterface = this->GetProvidedInterface(providedInterfaceName);
    if (deviceInterface) {
        mtsTaskInterface * taskInterface = dynamic_cast<mtsTaskInterface *>(deviceInterface);
        if (taskInterface) {
            return taskInterface->AddCommandVoid(action, commandName);
        } else {
            return deviceInterface->AddCommandVoid(action, commandName);
        }
    }
    CMN_LOG_CLASS(1) << "AddCommandVoid cannot find an interface named " << providedInterfaceName << std::endl;
    return 0;
}

template <class __classType, class __argumentType>
inline mtsCommandWriteBase * mtsDevice::AddCommandWrite(void (__classType::*action)(const __argumentType &),
                                                        __classType * classInstantiation,
                                                        const std::string & providedInterfaceName,
                                                        const std::string & commandName,
                                                        const __argumentType & argumentModel) {
    mtsDeviceInterface * deviceInterface = this->GetProvidedInterface(providedInterfaceName);
    if (deviceInterface) {
        mtsTaskInterface * taskInterface = dynamic_cast<mtsTaskInterface *>(deviceInterface);
        if (taskInterface) {
            return taskInterface->AddCommandWrite(action, classInstantiation, commandName, argumentModel);
        } else {
            return deviceInterface->AddCommandWrite(action, classInstantiation, commandName, argumentModel);
        }
    }
    CMN_LOG_CLASS(1) << "AddCommandWrite cannot find an interface named " << providedInterfaceName << std::endl;
    return 0;
}

// this method is defined in this header file
template <class __classType, class __argumentType>
inline mtsCommandWriteBase * mtsTask::AddEventHandlerWrite(void (__classType::*action)(const __argumentType &),
                                                           __classType * classInstantiation,
                                                           const std::string & requiredInterfaceName,
                                                           const std::string & commandName,
                                                           const __argumentType & argumentModel,
                                                           bool queued)  {
    mtsRequiredInterface * requiredInterface = GetRequiredInterface(requiredInterfaceName);
    if (requiredInterface) {
        return requiredInterface->AddEventHandlerWrite(action, classInstantiation, commandName, argumentModel, queued);
    }
    CMN_LOG_CLASS(1) << "AddEventHandlerWrite: cannot find an interface named " << requiredInterfaceName << std::endl;
    return 0;
}

// this method is defined in this header file
template <class __classType>
inline mtsCommandVoidBase * mtsTask::AddEventHandlerVoid(void (__classType::*action)(void),
                                                         __classType * classInstantiation,
                                                         const std::string & requiredInterfaceName,
                                                         const std::string & commandName,
                                                         bool queued)  {
    mtsRequiredInterface * requiredInterface = GetRequiredInterface(requiredInterfaceName);
    if (requiredInterface) {
        return requiredInterface->AddEventHandlerVoid(action, classInstantiation, commandName, queued);
    }
    CMN_LOG_CLASS(1) << "AddEventHandlerVoid: cannot find an interface named " << requiredInterfaceName << std::endl;
    return 0;
}


#endif // _mtsTask_h

