/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: ManagerComponent.h 

  Author(s):  Min Yang Jung
  Created on: 2010-09-01

  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnConstants.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstOSAbstraction/osaSleep.h>

#include "ManagerComponent.h"

const std::string PeerProcessName          = "ProcessCounter";
const std::string CounterOddComponentType  = "CounterOddComponent";
const std::string CounterOddComponentName  = "CounterOddComponentObject";
const std::string CounterEvenComponentType = "CounterEvenComponent";
const std::string CounterEvenComponentName = "CounterEvenComponentObject";

const std::string NameCounterOddInterfaceProvided = "CounterOddInterfaceProvided";
const std::string NameCounterOddInterfaceRequired = "CounterOddInterfaceRequired";
const std::string NameCounterEvenInterfaceProvided = "CounterEvenInterfaceProvided";
const std::string NameCounterEvenInterfaceRequired = "CounterEvenInterfaceRequired";

CMN_IMPLEMENT_SERVICES(ManagerComponent);

ManagerComponent::ManagerComponent(const std::string & componentName, double period):
    mtsTaskPeriodic(componentName, period, false, 1000)
{
}

void ManagerComponent::Run(void) 
{
    ProcessQueuedCommands();

    static double lastTick = 0;
    static int count = 0;

#if 0
    std::vector<std::string> processes, components, interfaces, connections;
    if (osaGetTime() - lastTick > 5.0) {
        std::cout << "==================================== Processes" << std::endl;
        if (RequestGetNamesOfProcesses(processes)) {
            for (size_t i = 0; i < processes.size(); ++i) {
                std::cout << processes[i] << std::endl;
            }
        }
        
        std::cout << "==================================== Components" << std::endl;
        for (size_t i = 0; i < processes.size(); ++i) {
            if (RequestGetNamesOfComponents(processes[i], components)) {
                for (size_t j = 0; j < components.size(); ++j) {
                    std::cout << processes[i] << " - " << components[j] << std::endl;
                }
            }
        }

        std::cout << "==================================== Interfaces" << std::endl;
        for (size_t i = 0; i < processes.size(); ++i) {
            if (RequestGetNamesOfInterfaces(processes[i], interfaces)) {
                for (size_t j = 0; j < interfaces.size(); ++j) {
                    std::cout << interfaces[j] << std::endl;
                }
            }
        }

        std::cout << "==================================== Connections" << std::endl;
        if (RequestGetListOfConnections(connections)) {
            for (size_t i = 0; i < connections.size(); ++i) {
                std::cout << connections[i] << std::endl;
            }
        }

        std::cout << std::endl << std::endl;
        std::flush(std::cout);

        lastTick = osaGetTime();
    }
#endif

#if 1
    std::cout << "....... " << count++ << std::endl;

    if (count == 5) {
        //
        // Create the two components: odd counter and even counter
        //
        std::cout << std::endl << "Creating counter components across network....." << std::endl;

        std::cout << "> " << PeerProcessName << ", " << CounterOddComponentType << ", " << CounterOddComponentName << ": ";
        if (!RequestComponentCreate(PeerProcessName, CounterOddComponentType, CounterOddComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << "> " << PeerProcessName << ", " << CounterEvenComponentType << ", " << CounterEvenComponentName << ": ";
        if (!RequestComponentCreate(PeerProcessName, CounterEvenComponentType, CounterEvenComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        // MJ: needs to be replaced with blocking command with return value
        std::cout << std::endl << "Wait for 5 seconds for \"Component Connect\"...." << std::endl;
        osaSleep(5.0);

        //
        // Connect the two components
        //
        std::cout << std::endl << "Connecting counter components across network....." << std::endl;
        std::cout << "> Connection 1: ";
        if (!RequestComponentConnect(PeerProcessName, CounterOddComponentName, NameCounterOddInterfaceRequired, 
                                     PeerProcessName, CounterEvenComponentName, NameCounterEvenInterfaceProvided))
        {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << "> Connection 2: ";
        if (!RequestComponentConnect(PeerProcessName, CounterEvenComponentName, NameCounterEvenInterfaceRequired,
                                     PeerProcessName, CounterOddComponentName, NameCounterOddInterfaceProvided))
        {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        // MJ: needs to be replaced with blocking command with return value
        std::cout << std::endl << "Wait for 5 seconds for \"Component Start\"...." << std::endl;
        osaSleep(5.0);

        //
        // Start the two components
        //
        std::cout << std::endl << "Starting counter components across network....." << std::endl;
        std::cout << "> " << PeerProcessName << "." << CounterOddComponentName << ": ";
        if (!RequestComponentStart(PeerProcessName, CounterOddComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << "> " << PeerProcessName << "." << CounterEvenComponentName << ": ";
        if (!RequestComponentStart(PeerProcessName, CounterEvenComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << std::endl << "Wait for 5 seconds for \"Component Stop\"...." << std::endl;
        osaSleep(5.0);

        //
        // Stop the two components
        //
        std::cout << std::endl << "Stopping counter components across network....." << std::endl;
        std::cout << "> " << PeerProcessName << "." << CounterOddComponentName << ": ";
        if (!RequestComponentStop(PeerProcessName, CounterOddComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << "> " << PeerProcessName << "." << CounterEvenComponentName << ": ";
        if (!RequestComponentStop(PeerProcessName, CounterEvenComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << std::endl << "Wait for 5 seconds for \"Component Resume\"...." << std::endl;
        osaSleep(5.0);

        //
        // Resume the two components
        //
        std::cout << std::endl << "Resuming counter components across network....." << std::endl;
        std::cout << "> " << PeerProcessName << "." << CounterOddComponentName << ": ";
        if (!RequestComponentResume(PeerProcessName, CounterOddComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }

        std::cout << "> " << PeerProcessName << "." << CounterEvenComponentName << ": ";
        if (!RequestComponentResume(PeerProcessName, CounterEvenComponentName)) {
            std::cout << "failure" << std::endl;
        } else {
            std::cout << "success" << std::endl;
        }
    }
#endif
}

