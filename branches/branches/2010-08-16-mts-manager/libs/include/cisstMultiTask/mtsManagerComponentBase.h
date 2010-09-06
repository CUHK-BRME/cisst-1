/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsManagerComponentBase.h 1726 2010-08-30 05:07:54Z mjung5 $

  Author(s):  Anton Deguet, Min Yang Jung
  Created on: 2010-08-29

  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsManagerComponentBase_h
#define _mtsManagerComponentBase_h

#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsParameterTypes.h>

/*!
  \file mtsManagerComponentBase.h
  \brief Declaration of Base Class for Manager Components
  \ingroup cisstMultiTask

  In the networked configuration, the communication between the global component
  manager (GCM) and local component managers (LCMs) is currently done by SLICE.
  If the cisstMultiTask's command pattern is used instead, users can have more
  flexible way to interact with the system, such as creating and starting 
  components dynamically.  To replace SLICE with the cisstMultiTask's command 
  pattern, we introduce a special type of component, manager component.
  
  There are two different typs of it, manager component server and client. The
  manager component server (MCS) runs in LCM that runs with GCM in the same 
  process and only one instance of MCS exists in the whole system.  On the 
  contrary, each LCM has manager component client (MCC) that connects to MCS
  and thus more than one MCC can exist in a system.  However, LCM can have only
  one MCC, i.e., one MCC per LCM.

  The cisstMultiTask's command pattern is based on a pair of interfaces that 
  are connected to each other.  The following diagram shows how interfaces are
  defined and how interfaces are connected to each other.

  (INTFC = one provided interface + one required interface)

              GCM - LCM - MCS (of type mtsManagerComponentServer)
                           |
                         INTFC ("InterfaceGCM")
                           
                           :
                           :
                         
                         INTFC ("InterfaceLCM")
                           |
                    LCM - MCC (of type mtsManagerComponentClient)
                           |
                         INTFC ("InterfaceComponent")

                           :
                           :

                         INTFC ("InterfaceInternal")
                           |
                     User Component
                with internal interfaces
  

  There are four internal connections between components in a system.

  1) InterfaceInternal.Required - InterfaceComponent.Provided
     : Established when mtsManagerLocal::CreateAll() gets called
       (See mtsManagerLocal::ConnectAllToManagerComponentClient())

  2) InterfaceLCM.Required - InterfaceGCM.Provided
     : Established when mtsManagerLocal::CreateAll() gets called
       (See mtsManagerLocal::ConnectManagerComponentClientToServer())

  3) InterfaceGCM.Required - InterfaceLCM.Provided
     : When MCC connects to MCS
       (See mtsManagerComponentServer::AddNewClientProcess())

  4) InterfaceComponent.Required - InterfaceInternal.Provided
     : When user component with internal interfaces connects to MCC
       (See mtsManagerComponentClient::AddNewClientComponent())

  \note Related classes: mtsManagerLocalInterface, mtsManagerGlobalInterface, 
  mtsManagerGlobal, mtsManagerProxyServer
*/

//
// MJ: When we support mts blocking command with return value mechanism, all 
// the commands and functions have to be updated such that they return result
// values in a blocking way.
//

class mtsManagerComponentBase : public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
    /*! Command name definitions */
    class CommandNames {
    public:
        // Dynamic component management
        static std::string ComponentCreate;
        static std::string ComponentConnect;
        static std::string ComponentStart;
        static std::string ComponentStop;
        static std::string ComponentResume;
        // Getters
        static std::string GetNamesOfProcesses;
        static std::string GetNamesOfComponents;
        static std::string GetNamesOfInterfaces;
        static std::string GetListOfConnections;
    };

    mtsManagerComponentBase(const std::string & componentName);
    virtual ~mtsManagerComponentBase();

    virtual void Startup(void) = 0;
    virtual void Run(void);
    virtual void Cleanup(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsManagerComponentBase);

#endif // _mtsManagerComponentBase_h
