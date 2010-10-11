/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsManagerComponentServer.cpp 1726 2010-08-30 05:07:54Z mjung5 $

  Author(s):  Min Yang Jung
  Created on: 2010-08-29

  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsManagerComponentServer.h>
#include <cisstMultiTask/mtsManagerComponentClient.h>
#include <cisstMultiTask/mtsManagerGlobal.h>

CMN_IMPLEMENT_SERVICES(mtsManagerComponentServer);

mtsManagerComponentServer::mtsManagerComponentServer(mtsManagerGlobal * gcm)
    : mtsManagerComponentBase(mtsManagerComponentBase::ComponentNames::ManagerComponentServer),
      GCM(gcm)
{
    // Prevent this component from being created more than once
    // MJ: singleton can be implemented instead.
    static int instanceCount = 0;
    if (instanceCount != 0) {
        cmnThrow(std::runtime_error("Error in creating manager component server: it's already created"));
    }
    gcm->SetMCS(this);
}

mtsManagerComponentServer::~mtsManagerComponentServer()
{
    InterfaceGCMFunctionMapType::iterator it = InterfaceGCMFunctionMap.begin();
    const InterfaceGCMFunctionMapType::iterator itEnd = InterfaceGCMFunctionMap.end();
    for (; it != itEnd; ++it) {
        delete it->second;
    }
}

void mtsManagerComponentServer::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Manager component SERVER starts" << std::endl;
}

void mtsManagerComponentServer::Run(void)
{
    mtsManagerComponentBase::Run();
}

void mtsManagerComponentServer::Cleanup(void)
{
}

void mtsManagerComponentServer::GetNamesOfProcesses(mtsStdStringVec & stdStringVec) const
{
    std::vector<std::string> namesOfProcesses;
    GCM->GetNamesOfProcesses(namesOfProcesses);

    const size_t n = namesOfProcesses.size();
    stdStringVec.SetSize(n);
    for (unsigned int i = 0; i < n; ++i) {
        stdStringVec(i) = namesOfProcesses[i];
    }
}

bool mtsManagerComponentServer::AddInterfaceGCM(void)
{
    // InterfaceGCM's required interface is not create here but is created
    // when a manager component client connects to the manager component
    // server.  
    // See mtsManagerComponentServer::AddNewClientProcess()
    // for the creation of required interfaces.

    // Add provided interface to which InterfaceLCM's required interface connects.
    const std::string interfaceName = mtsManagerComponentBase::InterfaceNames::InterfaceGCMProvided;
    mtsInterfaceProvided * provided = AddInterfaceProvided(interfaceName);
    if (!provided) {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceGCM: failed to add \"GCM\" provided interface: " << interfaceName << std::endl;
        return false;
    }

    provided->AddCommandWrite(&mtsManagerComponentServer::InterfaceGCMCommands_ComponentCreate,
                              this, mtsManagerComponentBase::CommandNames::ComponentCreate);
    provided->AddCommandWrite(&mtsManagerComponentServer::InterfaceGCMCommands_ComponentConnect,
                              this, mtsManagerComponentBase::CommandNames::ComponentConnect);
    provided->AddCommandWrite(&mtsManagerComponentServer::InterfaceGCMCommands_ComponentStart,
                              this, mtsManagerComponentBase::CommandNames::ComponentStart);
    provided->AddCommandWrite(&mtsManagerComponentServer::InterfaceGCMCommands_ComponentStop,
                              this, mtsManagerComponentBase::CommandNames::ComponentStop);
    provided->AddCommandWrite(&mtsManagerComponentServer::InterfaceGCMCommands_ComponentResume,
                              this, mtsManagerComponentBase::CommandNames::ComponentResume);
    provided->AddCommandRead(&mtsManagerComponentServer::InterfaceGCMCommands_GetNamesOfProcesses,
                              this, mtsManagerComponentBase::CommandNames::GetNamesOfProcesses);
    provided->AddCommandQualifiedRead(&mtsManagerComponentServer::InterfaceGCMCommands_GetNamesOfComponents,
                              this, mtsManagerComponentBase::CommandNames::GetNamesOfComponents);
    provided->AddCommandQualifiedRead(&mtsManagerComponentServer::InterfaceGCMCommands_GetNamesOfInterfaces,
                              this, mtsManagerComponentBase::CommandNames::GetNamesOfInterfaces);
    provided->AddCommandRead(&mtsManagerComponentServer::InterfaceGCMCommands_GetListOfConnections,
                              this, mtsManagerComponentBase::CommandNames::GetListOfConnections);

    provided->AddEventWrite(this->InterfaceGCMEvents_AddComponent,
                            mtsManagerComponentBase::EventNames::AddComponent, mtsDescriptionComponent());
    provided->AddEventWrite(this->InterfaceGCMEvents_AddConnection,
                            mtsManagerComponentBase::EventNames::AddConnection, mtsDescriptionConnection());

    CMN_LOG_CLASS_INIT_VERBOSE << "AddInterfaceGCM: successfully added \"GCM\" interfaces" << std::endl;

    return true;
}

bool mtsManagerComponentServer::AddNewClientProcess(const std::string & clientProcessName)
{
    if (InterfaceGCMFunctionMap.FindItem(clientProcessName)) {
        CMN_LOG_CLASS_INIT_VERBOSE << "AddNewClientProcess: process is already known" << std::endl;
        return true;
    }

    // Create a new set of function objects
    InterfaceGCMFunctionType * newFunctionSet = new InterfaceGCMFunctionType;

    std::string interfaceName = mtsManagerComponentBase::InterfaceNames::InterfaceGCMRequired;
    interfaceName += "For";
    interfaceName += clientProcessName;
    mtsInterfaceRequired * required = AddInterfaceRequired(interfaceName);
    if (!required) {
        CMN_LOG_CLASS_INIT_ERROR << "AddNewClientProcess: failed to create \"GCM\" required interface: " << interfaceName << std::endl;
        return false;
    }
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentCreate,
                          newFunctionSet->ComponentCreate);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentConnect,
                          newFunctionSet->ComponentConnect);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStart,
                          newFunctionSet->ComponentStart);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStop,
                          newFunctionSet->ComponentStop);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentResume,
                          newFunctionSet->ComponentResume);

    // Remember a required interface (InterfaceGCM's required interface) to 
    // connect it to the provided interface (InterfaceLCM's provided interface).
    if (!InterfaceGCMFunctionMap.AddItem(clientProcessName, newFunctionSet)) {
        CMN_LOG_CLASS_INIT_ERROR << "AddNewClientProcess: failed to add \"GCM\" required interface: " 
            << "\"" << clientProcessName << "\", " << interfaceName << std::endl;
        return false;
    }

    // Connect InterfaceGCM's required interface to InterfaceLCM's provided interface
    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
#if CISST_MTS_HAS_ICE
    if (!LCM->Connect(LCM->GetProcessName(), this->GetName(), interfaceName,
                     clientProcessName, 
                     mtsManagerComponentClient::GetNameOfManagerComponentClient(clientProcessName),
                     mtsManagerComponentBase::InterfaceNames::InterfaceLCMProvided))
    {
        CMN_LOG_CLASS_INIT_ERROR << "AddNewClientProcess: failed to connect: " 
            << mtsManagerGlobal::GetInterfaceUID(LCM->GetProcessName(), this->GetName(), interfaceName)
            << " - "
            << mtsManagerGlobal::GetInterfaceUID(clientProcessName,
                    mtsManagerComponentClient::GetNameOfManagerComponentClient(clientProcessName),
                    mtsManagerComponentBase::InterfaceNames::InterfaceLCMProvided)
            << std::endl;
        return false;
    }
#else
    if (!LCM->Connect(this->GetName(), interfaceName,
                      mtsManagerComponentClient::GetNameOfManagerComponentClient(clientProcessName),
                      mtsManagerComponentClient::NameOfInterfaceLCMProvided))
    {
        CMN_LOG_CLASS_INIT_ERROR << "AddNewClientProcess: failed to connect: " 
            << this->GetName() << ":" << interfaceName
            << " - "
            << mtsManagerComponentClient::GetNameOfManagerComponentClient(clientProcessName) << ":"
            << mtsManagerComponentClient::NameOfInterfaceLCMProvided
            << std::endl;
        return false;
    }
#endif

    CMN_LOG_CLASS_INIT_VERBOSE << "AddNewClientProcess: creation and connection success" << std::endl;

    return true;
}

void mtsManagerComponentServer::InterfaceGCMCommands_ComponentCreate(const mtsDescriptionComponent & arg)
{
    // Check if a new component with the name specified can be created
    if (GCM->FindComponent(arg.ProcessName, arg.ComponentName)) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentCreate: failed to create component: " << arg << std::endl
                                << "InterfaceGCMCommands_ComponentCreate: component already exists" << std::endl;
        return;
    }

    // Get a set of function objects that are bound to the InterfaceLCM's provided
    // interface.
    InterfaceGCMFunctionType * functionSet = InterfaceGCMFunctionMap.GetItem(arg.ProcessName);
    if (!functionSet) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentCreate: failed to execute \"Component Create\": " << arg << std::endl;
        return;
    }

    functionSet->ComponentCreate.ExecuteBlocking(arg);
}

void mtsManagerComponentServer::InterfaceGCMCommands_ComponentConnect(const mtsDescriptionConnection & arg)
{
    // We don't check argument validity with the GCM at this stage and rely on 
    // the current normal connection procedure (GCM allows connection at the 
    // request of LCM) because the GCM guarantees that arguments are valid.
    // The Connect request is then passed to the manager component client which
    // calls local component manager's Connect() method.

    // Get a set of function objects that are bound to the InterfaceLCM's provided
    // interface.
    InterfaceGCMFunctionType * functionSet = InterfaceGCMFunctionMap.GetItem(arg.Client.ProcessName);
    if (!functionSet) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentConnect: failed to execute \"Component Connect\": " << arg << std::endl;
        return;
    }

    functionSet->ComponentConnect.ExecuteBlocking(arg);
}

void mtsManagerComponentServer::InterfaceGCMCommands_ComponentStart(const mtsComponentStatusControl & arg)
{
    // Check if a new component with the name specified can be created
    if (!GCM->FindComponent(arg.ProcessName, arg.ComponentName)) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentStart: failed to start component - no component found: " << arg << std::endl;
        return;;
    }

    // Get a set of function objects that are bound to the InterfaceLCM's provided
    // interface.
    InterfaceGCMFunctionType * functionSet = InterfaceGCMFunctionMap.GetItem(arg.ProcessName);
    if (!functionSet) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentStart: failed to get function set: " << arg << std::endl;
        return;
    }
    if (!functionSet->ComponentStart.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentStart: failed to execute \"Component Start\": " << arg << std::endl;
        return;
    }

    functionSet->ComponentStart.ExecuteBlocking(arg);
}

void mtsManagerComponentServer::InterfaceGCMCommands_ComponentStop(const mtsComponentStatusControl & arg)
{
    // Check if a new component with the name specified can be created
    if (!GCM->FindComponent(arg.ProcessName, arg.ComponentName)) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentStop: failed to Stop component - no component found: " << arg << std::endl;
        return;
    }

    // Get a set of function objects that are bound to the InterfaceLCM's provided
    // interface.
    InterfaceGCMFunctionType * functionSet = InterfaceGCMFunctionMap.GetItem(arg.ProcessName);
    if (!functionSet) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentStop: failed to get function set: " << arg << std::endl;
        return;
    }
    if (!functionSet->ComponentStop.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentStop: failed to execute \"Component Stop\"" << std::endl;
        return;
    }

    functionSet->ComponentStop.ExecuteBlocking(arg);
}

void mtsManagerComponentServer::InterfaceGCMCommands_ComponentResume(const mtsComponentStatusControl & arg)
{
    // Check if a new component with the name specified can be created
    if (!GCM->FindComponent(arg.ProcessName, arg.ComponentName)) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentResume: failed to Resume component - no component found: " << arg << std::endl;
        return;
    }

    // Get a set of function objects that are bound to the InterfaceLCM's provided
    // interface.
    InterfaceGCMFunctionType * functionSet = InterfaceGCMFunctionMap.GetItem(arg.ProcessName);
    if (!functionSet) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentResume: failed to get function set: " << arg << std::endl;
        return;
    }
    if (!functionSet->ComponentResume.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceGCMCommands_ComponentResume: failed to execute \"Component Resume\"" << std::endl;
        return;
    }

    functionSet->ComponentResume.ExecuteBlocking(arg);
}

void mtsManagerComponentServer::InterfaceGCMCommands_GetNamesOfProcesses(mtsStdStringVec & names) const
{
    std::vector<std::string> _names;
    GCM->GetNamesOfProcesses(_names);

    names.SetSize(_names.size());
    for (size_t i = 0; i < names.size(); ++i) {
        names(i) = _names[i];
    }
}

void mtsManagerComponentServer::InterfaceGCMCommands_GetNamesOfComponents(const mtsStdString & processName, mtsStdStringVec & names) const
{
    std::vector<std::string> _names;
    GCM->GetNamesOfComponents(processName, _names);

    names.SetSize(_names.size());
    for (size_t i = 0; i < names.size(); ++i) {
        names(i) = _names[i];
    }
}

void mtsManagerComponentServer::InterfaceGCMCommands_GetNamesOfInterfaces(const mtsDescriptionComponent & component, mtsDescriptionInterface & interfaces) const
{
    std::vector<std::string> interfaceNames;

    // Get a list of required interfaces
    GCM->GetNamesOfInterfacesRequiredOrInput(component.ProcessName, component.ComponentName, interfaceNames);
    mtsParameterTypes::ConvertVectorStringType(interfaceNames, interfaces.InterfaceRequiredNames);

    // Get a list of provided interfaces
    GCM->GetNamesOfInterfacesProvidedOrOutput(component.ProcessName, component.ComponentName, interfaceNames);
    mtsParameterTypes::ConvertVectorStringType(interfaceNames, interfaces.InterfaceProvidedNames);
}

void mtsManagerComponentServer::InterfaceGCMCommands_GetListOfConnections(std::vector <mtsDescriptionConnection> & listOfConnections) const
{
    GCM->GetListOfConnections(listOfConnections);
}

void mtsManagerComponentServer::AddComponentEvent(const mtsDescriptionComponent &component)
{
    InterfaceGCMEvents_AddComponent.ExecuteBlocking(component);
}

void mtsManagerComponentServer::AddConnectionEvent(const mtsDescriptionConnection &connection)
{
    InterfaceGCMEvents_AddConnection.ExecuteBlocking(connection);
}

