/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Min Yang Jung
  Created on: 2010-08-29

  (C) Copyright 2010-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \brief Declaration of Manager Component Client
  \ingroup cisstMultiTask

  This class defines the manager component client which is managed by all local 
  component managers (LCMs).  An instance of this class is automatically created 
  and gets connected to the manager component server which runs on LCM that runs
  with the global component manager (GCM).
  
  This component has two sets of interfaces, one for communication with the 
  manager component server and the other one for command exchange between other
  manager component clients.
  
  \note Related classes: mtsManagerComponentBase, mtsManagerComponentServer
*/

#ifndef _mtsManagerComponentClient_h
#define _mtsManagerComponentClient_h

#include <cisstMultiTask/mtsManagerComponentBase.h>

class mtsManagerComponentClient : public mtsManagerComponentBase
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    friend class mtsManagerLocal;

protected:
    /*! Get a list of all processes running in the system */
    mtsFunctionRead GetNamesOfProcesses;

    /*! Functions for InterfaceComponent's required interface.  Since one 
        manager component client needs to be able to handle multiple user 
        components, we keep a list of function objects using named map with 
        (key = component name, value = function object set instance) */
    typedef struct {
        mtsFunctionVoid ComponentStop;
        mtsFunctionVoid ComponentResume;
        mtsFunctionRead ComponentGetState;
        mtsFunctionWriteReturn GetEndUserInterface;
        mtsFunctionWriteReturn AddObserverList;
        mtsFunctionWriteReturn RemoveEndUserInterface;
        mtsFunctionWriteReturn RemoveObserverList;
    } InterfaceComponentFunctionType;

    typedef cmnNamedMap<InterfaceComponentFunctionType> InterfaceComponentFunctionMapType;
    InterfaceComponentFunctionMapType InterfaceComponentFunctionMap;

    /*! Functions for InterfaceLCM's required interface */
    typedef struct {
        // Dynamic component management
        mtsFunctionWrite ComponentCreate;
        mtsFunctionWrite ComponentConnect;
        mtsFunctionWrite ComponentDisconnect;
        mtsFunctionWrite ComponentStart;
        mtsFunctionWrite ComponentStop;
        mtsFunctionWrite ComponentResume;
        mtsFunctionQualifiedRead ComponentGetState;
        // Getters
        mtsFunctionRead          GetNamesOfProcesses;
        mtsFunctionQualifiedRead GetNamesOfComponents; // in: process name, out: components' names
        mtsFunctionQualifiedRead GetNamesOfInterfaces; // in: process name, out: interfaces' names
        mtsFunctionRead          GetListOfConnections;
        mtsFunctionQualifiedRead GetInterfaceProvidedDescription;
        mtsFunctionQualifiedRead GetInterfaceRequiredDescription;
    } InterfaceLCMFunctionType;

    InterfaceLCMFunctionType InterfaceLCMFunction;

    // Event handlers for InterfaceLCM's required interface (handle events from MCS)
    void HandleAddComponentEvent(const mtsDescriptionComponent &component);
    void HandleChangeStateEvent(const mtsComponentStateChange &componentStateChange);
    void HandleAddConnectionEvent(const mtsDescriptionConnection &connection);
    void HandleRemoveConnectionEvent(const mtsDescriptionConnection &connection);

    // Event handlers for InterfaceComponent's required interface (handle events from Component)
    void HandleChangeStateFromComponent(const mtsComponentStateChange & componentStateChange);

    // General-purpose interface. These are used to allow a class method to be invoked from
    // any thread, but still allow that method to queue commands for execution by the MCC.
    // Because any thread can call these methods, thread-safety is obtained by using a mutex.
    struct GeneralInterfaceStruct {
        osaMutex Mutex;        
        mtsFunctionWrite ComponentConnect;
    } GeneralInterface;

    /*! Create new component and add it to LCM */
    bool CreateAndAddNewComponent(const std::string & className, const std::string & componentName);

    /*! \brief Connect two local interfaces.
        \param clientComponentName Name of client component
        \param clientInterfaceRequiredName Name of required interface
        \param serverComponentName Name of server component
        \param serverInterfaceProvidedName Name of provided interface
        \param clientProcessName Name of client process (ignored in standalone
               configuration, used in networked configuration)
        \return true if successful, false otherwise
        \note  It is assumed that the two components are in the same process. */
    bool ConnectLocally(const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
                        const std::string & serverComponentName, const std::string & serverInterfaceProvidedName,
                        const std::string & clientProcessName = "");

    bool DisconnectLocally(const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
                           const std::string & serverComponentName, const std::string & serverInterfaceProvidedName);

    // If connection between InterfaceComponent.required - InterfaceInternal.provided is
    // disconnected, required interface instance of InterfaceComponent that corresponds
    // to the connection should be removed.
    bool DisconnectCleanup(const std::string & componentName);

public:
    mtsManagerComponentClient(const std::string & componentName);
    ~mtsManagerComponentClient();

    void Startup(void);
    void Run(void);
    void Cleanup(void);

    bool AddInterfaceLCM(void);
    bool AddInterfaceComponent(void);

    /*! Create a new set of function objects, add a new instance of 
        InterfaceComponent's required interface to this component, and connect 
        it to InterfaceInternal's provided interface */
    bool AddNewClientComponent(const std::string & clientComponentName);

    // Called from LCM
    bool Connect(const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
                 const std::string & serverComponentName, const std::string & serverInterfaceProvidedName);

    /*! Commands for InterfaceLCM's provided interface */
    void InterfaceLCMCommands_ComponentCreate(const mtsDescriptionComponent & arg);
    void InterfaceLCMCommands_ComponentConnect(const mtsDescriptionConnection & arg);
    void InterfaceLCMCommands_ComponentDisconnect(const mtsDescriptionConnection & arg);
    void InterfaceLCMCommands_ComponentStart(const mtsComponentStatusControl & arg);
    void InterfaceLCMCommands_ComponentStop(const mtsComponentStatusControl & arg);
    void InterfaceLCMCommands_ComponentResume(const mtsComponentStatusControl & arg);
    void InterfaceLCMCommands_ComponentGetState(const mtsDescriptionComponent &component,
                                                mtsComponentState &state) const;
    void InterfaceLCMCommands_GetInterfaceProvidedDescription(const mtsDescriptionInterface &intfc,
                                                InterfaceProvidedDescription & description) const;
    void InterfaceLCMCommands_GetInterfaceRequiredDescription(const mtsDescriptionInterface &intfc,
                                                InterfaceRequiredDescription & description) const;

    /*! Event generators for InterfaceLCM's provided interface */
    mtsFunctionWrite InterfaceLCMEvents_ChangeState;

    /*! Commands for InterfaceComponent's provided interface */
    void InterfaceComponentCommands_ComponentCreate(const mtsDescriptionComponent & arg);
    void InterfaceComponentCommands_ComponentConnect(const mtsDescriptionConnection & arg);
    void InterfaceComponentCommands_ComponentDisconnect(const mtsDescriptionConnection & arg);
    void InterfaceComponentCommands_ComponentStart(const mtsComponentStatusControl & arg);
    void InterfaceComponentCommands_ComponentStop(const mtsComponentStatusControl & arg);
    void InterfaceComponentCommands_ComponentResume(const mtsComponentStatusControl & arg);
    void InterfaceComponentCommands_ComponentGetState(const mtsDescriptionComponent &component,
                                                      mtsComponentState &state) const;

    void InterfaceComponentCommands_GetNamesOfProcesses(std::vector<std::string> & names) const;
    void InterfaceComponentCommands_GetNamesOfComponents(const std::string & processName,
                                                         std::vector<std::string> & names) const;
    void InterfaceComponentCommands_GetNamesOfInterfaces(const mtsDescriptionComponent & component, mtsDescriptionInterface & interfaces) const;
    void InterfaceComponentCommands_GetListOfConnections(std::vector <mtsDescriptionConnection> & listOfConnections) const;
    void InterfaceComponentCommands_GetInterfaceProvidedDescription(const mtsDescriptionInterface & intfc, 
                                                                    InterfaceProvidedDescription & description) const;
    void InterfaceComponentCommands_GetInterfaceRequiredDescription(const mtsDescriptionInterface & intfc, 
                                                                    InterfaceRequiredDescription & description) const;

    /*! Event generators for InterfaceComponent's provided interface */
    mtsFunctionWrite InterfaceComponentEvents_AddComponent;
    mtsFunctionWrite InterfaceComponentEvents_ChangeState;
    mtsFunctionWrite InterfaceComponentEvents_AddConnection;
    mtsFunctionWrite InterfaceComponentEvents_RemoveConnection;

    /*! Returns name of manager component client */
    static std::string GetNameOfManagerComponentClient(const std::string & processName);
    
    /*! Returns name of InterfaceComponent's required interface */
    static std::string GetNameOfInterfaceComponentRequired(const std::string & userComponentName);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsManagerComponentClient);

#endif // _mtsManagerComponentClient_h
