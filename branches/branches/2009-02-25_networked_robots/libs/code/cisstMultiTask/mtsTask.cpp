/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Ankur Kapoor, Peter Kazanzides
  Created on: 2004-04-30

  (C) Copyright 2004-2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnExport.h>
#include <cisstCommon/cmnPortability.h>
#include <cisstOSAbstraction/osaThread.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsTask.h>
#include <cisstMultiTask/mtsTaskInterface.h>
#include <cisstMultiTask/mtsTaskManager.h>

#include <cisstMultiTask/mtsTaskInterfaceProxyServer.h>
#include <cisstMultiTask/mtsTaskInterfaceProxyClient.h>

#include <iostream>
#include <string>


CMN_IMPLEMENT_SERVICES(mtsTask)

/********************* Methods to connect interfaces  *****************/

bool mtsTask::ConnectRequiredInterface(const std::string & requiredInterfaceName, mtsDeviceInterface * providedInterface)
{
    mtsRequiredInterface *requiredInterface = RequiredInterfaces.GetItem(requiredInterfaceName, 1);
    if (requiredInterface) {
        requiredInterface->ConnectTo(providedInterface);
        CMN_LOG_CLASS(3) << "ConnectRequiredInterface: required interface " << requiredInterfaceName
                         << " successfuly connected to provided interface " << providedInterface->GetName() << std::endl;
        return true;
    }
    return false;            
}


/********************* Methods that call user methods *****************/

void mtsTask::DoRunInternal(void)
{
	StateTable.Start();
	this->Run();
    StateTable.Advance();
}
  
void mtsTask::StartupInternal(void) {
    CMN_LOG_CLASS(3) << "Starting StartupInternal for " << Name << std::endl;

    bool success = true;
    // Loop through the required interfaces and bind all commands and events
    RequiredInterfacesMapType::MapType::const_iterator requiredIterator = RequiredInterfaces.GetMap().begin();
    mtsDeviceInterface * connectedInterface;
    for (;
         requiredIterator != RequiredInterfaces.GetMap().end();
         requiredIterator++) {
        connectedInterface = requiredIterator->second->GetConnectedInterface();
        if (connectedInterface) {
            CMN_LOG_CLASS(3) << "StartupInternal: ask " << connectedInterface->GetName() 
                             << " to allocate resources for " << this->GetName() << std::endl;
            connectedInterface->AllocateResourcesForCurrentThread();
            CMN_LOG_CLASS(3) << "StartupInternal: binding commands and events" << std::endl;
            success &= requiredIterator->second->BindCommandsAndEvents();
        } else {
            CMN_LOG_CLASS(2) << "StartupInternal: void pointer to required interface (required not connected to provided)" << std::endl;
            success = false;
        }
    }
    // Call user-supplied startup function
    this->Startup();
    // StateChange should already be locked
    if (success)
       TaskState = READY;
    else
        CMN_LOG_CLASS(1) << "ERROR: Task " << GetName() << " cannot be started." << std::endl;
    StateChange.Unlock();
    CMN_LOG_CLASS(3) << "Ending StartupInternal for " << Name << std::endl;
}

void mtsTask::CleanupInternal() {
    // Call user-supplied cleanup function
	this->Cleanup();
    // Perform Cleanup on all interfaces provided
    ProvidedInterfaces.Cleanup();
    // StateChange should be locked by Kill().
	TaskState = FINISHED;
    StateChange.Unlock();
	CMN_LOG_CLASS(3) << "Done base class CleanupInternal " << Name << std::endl;
}


/********************* Methods to process queues  *********************/

// Execute all commands in the mailbox.  This is just a temporary implementation, where
// all commands in a mailbox are executed before moving on the next mailbox.  The final
// implementation will probably look at timestamps.  We may also want to pass in a
// parameter (enum) to set the mailbox processing policy.
unsigned int mtsTask::ProcessMailBoxes(ProvidedInterfacesMapType & interfaces)
{
    unsigned int numberOfCommands = 0;
    ProvidedInterfacesMapType::MapType::iterator iterator = interfaces.GetMap().begin();
    const ProvidedInterfacesMapType::MapType::iterator end = interfaces.GetMap().end();
    for (;
         iterator != end;
         ++iterator) {
        numberOfCommands += iterator->second->ProcessMailBoxes();
    }
    return numberOfCommands;
}


unsigned int mtsTask::ProcessQueuedEvents(void) {
    RequiredInterfacesMapType::MapType::iterator iterator = RequiredInterfaces.GetMap().begin();
    const RequiredInterfacesMapType::MapType::iterator end = RequiredInterfaces.GetMap().end();
    unsigned int numberOfEvents = 0;
    for (;
         iterator != end;
         iterator++) {
        numberOfEvents += iterator->second->ProcessMailBoxes();
    }
    return numberOfEvents;
}


/**************** Methods for managing task timing ********************/

void mtsTask::Sleep(double timeInSeconds)
{
    if (Thread.IsValid())
        Thread.Sleep(timeInSeconds);
    else
        osaSleep(timeInSeconds);
}


/********************* Task constructor and destructor *****************/

mtsTask::mtsTask(const std::string & name, unsigned int sizeStateTable) :
    mtsDevice(name),
    Thread(),
    TaskState(CONSTRUCTED),
    StateChange(),
	StateTable(sizeStateTable),
    OverranPeriod(false),
    ThreadStartData(0),
    retValue(0),
    RequiredInterfaces("RequiredInterfaces"),
    TaskInterfaceCommunicatorID("TaskInterfaceServerSender")
{
}

mtsTask::~mtsTask()
{
    CMN_LOG_CLASS(5) << "mtsTask destructor: deleting task " << Name << std::endl;
    if (!IsTerminated()) {
        //It is safe to call CleanupInternal() more than once.
        //Should we call the user-supplied Cleanup()?
        CleanupInternal();
    }
}


/********************* Methods to change task state ******************/

void mtsTask::Kill(void)
{
    CMN_LOG_CLASS(7) << "Kill " << Name << std::endl;
    StateChange.Lock();
    // If we get here, we cannot be in the INITIALIZING or FINISHING
    // states because we are holding the StateChange Mutex. 
    if (TaskState == FINISHED)
        StateChange.Unlock();
    else if (TaskState == CONSTRUCTED) {
        TaskState = FINISHED;
        StateChange.Unlock();
    }
    else {
        TaskState = FINISHING;
        // Unlock StateChange in RunInternal
    }
}


/********************* Methods to query the task state ****************/

const char *mtsTask::TaskStateName(TaskStateType state) const
{
    static char *taskStateNames[] = { "constructed", "initializing", "ready", "active", "finishing", "finished" };
    if ((state < CONSTRUCTED) || (state > FINISHED))
        return "unknown";
    else
        return taskStateNames[state];
}

/********************* Methods to manage interfaces *******************/
	
bool mtsTask::AddProvidedInterface(const std::string & newInterfaceName) 
{
    return ProvidedInterfaces.AddItem(newInterfaceName, new mtsTaskInterface(newInterfaceName, this));
}


mtsRequiredInterface *mtsTask::AddRequiredInterface(const std::string & requiredInterfaceName,
                                                    mtsRequiredInterface *requiredInterface)
{
    return RequiredInterfaces.AddItem(requiredInterfaceName, requiredInterface)?requiredInterface:0;    
}

mtsRequiredInterface *mtsTask::AddRequiredInterface(const std::string & requiredInterfaceName) {
    // PK: move DEFAULT_EVENT_QUEUE_LEN somewhere else (not in mtsTaskInterface)
    mtsMailBox *mbox = new mtsMailBox(requiredInterfaceName+"Events", mtsTaskInterface::DEFAULT_EVENT_QUEUE_LEN);
    mtsRequiredInterface *required = new mtsRequiredInterface(requiredInterfaceName, mbox);
    return AddRequiredInterface(requiredInterfaceName, required)?required:0;
}


std::vector<std::string> mtsTask::GetNamesOfRequiredInterfaces(void) const {
    return RequiredInterfaces.GetNames();
}


bool mtsTask::AddObserverToRequiredInterface(const std::string & requiredInterfaceName,
                                             const std::string & eventName,
                                             const std::string & handlerName)
{
    CMN_LOG_CLASS(1) << "AddObserverToRequiredInterface now obsolete" << std::endl;
    return false;
}

	
/********************* Methods to manage event handlers *******************/

mtsCommandWriteBase * mtsTask::GetEventHandlerWrite(const std::string & requiredInterfaceName,
                                                    const std::string & commandName)
{
    mtsRequiredInterface * requiredInterface = GetRequiredInterface(requiredInterfaceName);
    if (requiredInterface) {
        return requiredInterface->GetEventHandlerWrite(commandName);
    }
    return 0;
}


mtsCommandVoidBase * mtsTask::GetEventHandlerVoid(const std::string & requiredInterfaceName,
                                                  const std::string & commandName)
{
    mtsRequiredInterface * requiredInterface = GetRequiredInterface(requiredInterfaceName);
    if (requiredInterface) {
        return requiredInterface->GetEventHandlerVoid(commandName);
    }
    return 0;
}

/********************* Methods for task synchronization ***************/

bool mtsTask::WaitToStart(double timeout)
{
    if (TaskState == INITIALIZING) {
        CMN_LOG_CLASS(5) << "Waiting for task " << Name << " to start." << std::endl;
        // PK: Following doesn't work because WaitToStart is generally called from same thread
        // as Create, which is where the Lock was done.
        //StateChange.Lock();  // Should use TryLock with timeout
        // For now, we just use a Sleep and hope it is long enough
        osaSleep(timeout);
        if (TaskState != READY)
            CMN_LOG_CLASS(1) << "Task " << Name << " did not start properly, state = " << TaskStateName(TaskState) << std::endl;
        StateChange.Unlock();
    }
    return (TaskState >= READY);
}

bool mtsTask::WaitToTerminate(double timeout)
{
	CMN_LOG_CLASS(5) << "WaitToTerminate " << Name << std::endl;
    if (TaskState < FINISHING)
        return false;
    if (TaskState == FINISHING) {
        CMN_LOG_CLASS(5) << "Waiting for task " << Name << " to finish." << std::endl;
        StateChange.Lock();  // Should use TryLock with timeout
        if (TaskState != FINISHED)
            CMN_LOG_CLASS(1) << "Task " << Name << " did not finish properly, state = " << GetTaskStateName() << std::endl;
        StateChange.Unlock();
    }
    // If task state is finished, we wait for the thread to be destroyed
    if ((TaskState == FINISHED) && Thread.IsValid()) {
        CMN_LOG_CLASS(5) << "Waiting for task " << Name << " thread to exit." << std::endl;
        Thread.Wait();
    }
	return true;
}

//
//
//  MJUNG: TODO: Multiple [provided, required] interface => USE dictionary (SLICE)!!!
//
//
//
void mtsTask::StartProxyServer()
{
    mtsTaskManager * TaskManager = mtsTaskManager::GetInstance();
    if (TaskManager->GetTaskManagerType() == mtsTaskManager::TASK_MANAGER_LOCAL) {
        return;
    }

    if (ProvidedInterfaces.GetCount() <= 0) {
        CMN_LOG_CLASS(5) << "No provided interface added. Proxy server wasn't created." << std::endl;
        return;
    }

    // Start a provided interface proxy (proxy server, mtsTaskInterfaceProxyServer)    
    //
    // TODO: I assume there is only one provided interface and one required interface
    //
    ProvidedInterfacesMapType::MapType::iterator iterator = 
        ProvidedInterfaces.GetMap().begin();
    //const ProvidedInterfacesMapType::MapType::iterator end = interfaces.GetMap().end();
    //for (;
    //     iterator != end;
    //     ++iterator) {
    //    numberOfCommands += iterator->second->ProcessMailBoxes();
    //}

    const std::string adapterName = "TaskInterfaceServerAdapter";
    const std::string endpointInfo = "tcp -p 11705";
    const std::string endpointInfoForClient = ":default -p 11705";
    const std::string communicatorID = TaskInterfaceCommunicatorID;

    Proxy = new mtsTaskInterfaceProxyServer(adapterName, endpointInfo, communicatorID);
    Proxy->Start(this);
    Proxy->GetLogger()->trace("mtsTask", "Provided interface proxy starts.");

    // Inform the global task manager of the existence on a newly created 
    // provided interface with the access information.
    if (!TaskManager->AddProvidedInterface(
        iterator->first, adapterName, endpointInfoForClient, communicatorID, Name)) 
    {
        Proxy->GetLogger()->error("Failed to add provided interface: " + iterator->first);
        return;
    } else {
        Proxy->GetLogger()->trace("mtsTask", "Registered provided interface: " + iterator->first);
    }
}

void mtsTask::StartProxyClient(const std::string & endpointInfo, 
                               const std::string & communicatorID)
{
    mtsTaskManager * TaskManager = mtsTaskManager::GetInstance();
    if (TaskManager->GetTaskManagerType() == mtsTaskManager::TASK_MANAGER_LOCAL) {
        return;
    }

    if (RequiredInterfaces.GetCount() <= 0) {
        CMN_LOG_CLASS(5) << "No required interface added. Proxy client wasn't created." << std::endl;
        return;
    }

    // Start a required interface proxy (proxy client, mtsTaskInterfaceProxyClient)
    //
    // TODO: I assume there is only one provided interface and one required interface
    //
    RequiredInterfacesMapType::MapType::iterator iterator = 
        RequiredInterfaces.GetMap().begin();

    //const std::string communicatorID = TaskInterfaceCommunicatorID;

    Proxy = new mtsTaskInterfaceProxyClient(endpointInfo, communicatorID);
    Proxy->Start(this);
    Proxy->GetLogger()->trace("mtsTask", "Required interface proxy starts.");

    // Inform the global task manager of the existence on a newly created 
    // required interface.
    if (!TaskManager->AddRequiredInterface(iterator->first, Name)) 
    {
        Proxy->GetLogger()->error("Failed to add required interface: " + iterator->first);
        return;
    } else {
        Proxy->GetLogger()->trace("mtsTask", "Registered required interface: " + iterator->first);
    }
}