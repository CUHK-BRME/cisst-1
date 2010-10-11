/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsManagerComponentClient.cpp 1726 2010-08-30 05:07:54Z mjung5 $

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

#include <cisstMultiTask/mtsManagerComponentClient.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstCommon/cmnUnits.h>

CMN_IMPLEMENT_SERVICES(mtsManagerComponentClient);

const std::string SuffixManagerComponentClient = "_MNGR-COMP-CLIENT";

mtsManagerComponentClient::mtsManagerComponentClient(const std::string & componentName)
    : mtsManagerComponentBase(componentName)
{
}

mtsManagerComponentClient::~mtsManagerComponentClient()
{
}

std::string mtsManagerComponentClient::GetNameOfManagerComponentClient(const std::string & processName)
{
    std::string name(processName);
    name += SuffixManagerComponentClient;

    return name;
}

std::string mtsManagerComponentClient::GetNameOfInterfaceComponentRequired(const std::string & userComponentName)
{
    std::string interfaceName = mtsManagerComponentBase::InterfaceNames::InterfaceComponentRequired;
    interfaceName += "For";
    interfaceName += userComponentName;

    return interfaceName;
}

void mtsManagerComponentClient::Startup(void)
{
   CMN_LOG_CLASS_INIT_VERBOSE << "Manager component CLIENT starts" << std::endl;
}

void mtsManagerComponentClient::Run(void)
{
    mtsManagerComponentBase::Run();
}

void mtsManagerComponentClient::Cleanup(void)
{
}

bool mtsManagerComponentClient::CreateAndAddNewComponent(const std::string & className, const std::string & componentName)
{
    // Try to create component as requested
    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();

    mtsComponent * newComponent = LCM->CreateComponentDynamically(className, componentName);
    if (!newComponent) {
        CMN_LOG_CLASS_RUN_ERROR << "CreateAndAddNewComponent: failed to create component: " 
            << "\"" << componentName << "\" of type \"" << className << "\"" << std::endl;
        return false;
    }

    // Add new component with supoprt for the dynamic component control
    if (!LCM->AddComponent(newComponent)) {
        CMN_LOG_CLASS_RUN_ERROR << "CreateAndAddNewComponent: failed to add component: "
            << "\"" << componentName << "\" of type \"" << className << "\"" << std::endl;
        return false;
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "CreateAndAddNewComponent: successfully added component: "
        << "\"" << componentName << "\" of type \"" << className << "\"" << std::endl;

    return true;
}

bool mtsManagerComponentClient::AddInterfaceComponent(void)
{
    // InterfaceComponent's required interface is not created here but is created
    // when a user component with internal interfaces connects to the manager 
    // component client.  
    // See mtsManagerComponentClient::AddNewClientComponent() for the dynamic 
    // creation of required interfaces.

    // Add provided interface to which InterfaceInternal's required interface connects.
    std::string interfaceName = mtsManagerComponentBase::InterfaceNames::InterfaceComponentProvided;
    mtsInterfaceProvided * provided = AddInterfaceProvided(interfaceName);
    if (!provided) {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceComponent: failed to add \"Component\" provided interface: " << interfaceName << std::endl;
        return false;
    }

    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceComponentCommands_ComponentCreate,
                              this, mtsManagerComponentBase::CommandNames::ComponentCreate);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceComponentCommands_ComponentConnect,
                              this, mtsManagerComponentBase::CommandNames::ComponentConnect);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceComponentCommands_ComponentStart,
                              this, mtsManagerComponentBase::CommandNames::ComponentStart);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceComponentCommands_ComponentStop,
                              this, mtsManagerComponentBase::CommandNames::ComponentStop);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceComponentCommands_ComponentResume,
                              this, mtsManagerComponentBase::CommandNames::ComponentResume);
    provided->AddCommandRead(&mtsManagerComponentClient::InterfaceComponentCommands_GetNamesOfProcesses,
                              this, mtsManagerComponentBase::CommandNames::GetNamesOfProcesses);
    provided->AddCommandQualifiedRead(&mtsManagerComponentClient::InterfaceComponentCommands_GetNamesOfComponents,
                              this, mtsManagerComponentBase::CommandNames::GetNamesOfComponents);
    provided->AddCommandQualifiedRead(&mtsManagerComponentClient::InterfaceComponentCommands_GetNamesOfInterfaces,
                              this, mtsManagerComponentBase::CommandNames::GetNamesOfInterfaces);
    provided->AddCommandRead(&mtsManagerComponentClient::InterfaceComponentCommands_GetListOfConnections,
                              this, mtsManagerComponentBase::CommandNames::GetListOfConnections);
    provided->AddEventWrite(this->InterfaceComponentEvents_AddComponent, 
                            mtsManagerComponentBase::EventNames::AddComponent, mtsDescriptionComponent());
    provided->AddEventWrite(this->InterfaceComponentEvents_AddConnection,
                            mtsManagerComponentBase::EventNames::AddConnection, mtsDescriptionConnection());
    
    CMN_LOG_CLASS_INIT_VERBOSE << "AddInterfaceComponent: successfully added \"Component\" interfaces" << std::endl;

    return true;
}

bool mtsManagerComponentClient::AddInterfaceLCM(void)
{
    // Add required interface
    std::string interfaceName = mtsManagerComponentBase::InterfaceNames::InterfaceLCMRequired;
    mtsInterfaceRequired * required = AddInterfaceRequired(interfaceName);
    if (!required) {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceLCM: failed to add \"LCM\" required interface: " << interfaceName << std::endl;
        return false;
    }
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentCreate,
                          InterfaceLCMFunction.ComponentCreate);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentConnect,
                          InterfaceLCMFunction.ComponentConnect);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStart,
                          InterfaceLCMFunction.ComponentStart);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStop,
                          InterfaceLCMFunction.ComponentStop);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentResume,
                          InterfaceLCMFunction.ComponentResume);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetNamesOfProcesses,
                          InterfaceLCMFunction.GetNamesOfProcesses);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetNamesOfComponents,
                          InterfaceLCMFunction.GetNamesOfComponents);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetNamesOfInterfaces,
                          InterfaceLCMFunction.GetNamesOfInterfaces);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetListOfConnections,
                          InterfaceLCMFunction.GetListOfConnections);
    // It is not necessary to queue the events because we are just passing them along (it would not
    // hurt to queue them, either).
    required->AddEventHandlerWrite(&mtsManagerComponentClient::HandleAddComponentEvent, this, 
                                   mtsManagerComponentBase::EventNames::AddComponent, MTS_EVENT_NOT_QUEUED);
    required->AddEventHandlerWrite(&mtsManagerComponentClient::HandleAddConnectionEvent, this, 
                                   mtsManagerComponentBase::EventNames::AddConnection, MTS_EVENT_NOT_QUEUED);

    // Add provided interface
    interfaceName = mtsManagerComponentBase::InterfaceNames::InterfaceLCMProvided;
    mtsInterfaceProvided * provided = AddInterfaceProvided(interfaceName);
    if (!provided) {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceLCM: failed to add \"LCM\" required interface: " << interfaceName << std::endl;
        return false;
    }
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceLCMCommands_ComponentCreate,
                             this, mtsManagerComponentBase::CommandNames::ComponentCreate);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceLCMCommands_ComponentConnect,
                             this, mtsManagerComponentBase::CommandNames::ComponentConnect);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceLCMCommands_ComponentStart,
                             this, mtsManagerComponentBase::CommandNames::ComponentStart);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceLCMCommands_ComponentStop,
                             this, mtsManagerComponentBase::CommandNames::ComponentStop);
    provided->AddCommandWrite(&mtsManagerComponentClient::InterfaceLCMCommands_ComponentResume,
                             this, mtsManagerComponentBase::CommandNames::ComponentResume);
    CMN_LOG_CLASS_INIT_VERBOSE << "AddInterfaceLCM: successfully added \"LCM\" interfaces" << std::endl;

    return true;
}

bool mtsManagerComponentClient::AddNewClientComponent(const std::string & clientComponentName)
{
    if (InterfaceComponentFunctionMap.FindItem(clientComponentName)) {
        CMN_LOG_CLASS_INIT_VERBOSE << "AddNewClientComponent: component is already known: " << clientComponentName << std::endl;
        return true;
    }

    // Create a new set of function objects
    InterfaceComponentFunctionType * newFunctionSet = new InterfaceComponentFunctionType;

    const std::string interfaceName = GetNameOfInterfaceComponentRequired(clientComponentName);
    mtsInterfaceRequired * required = AddInterfaceRequired(interfaceName);
    if (!required) {
        CMN_LOG_CLASS_INIT_ERROR << "AddNewClientComponent: failed to create \"Component\" required interface: " << interfaceName << std::endl;
        return false;
    }
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStop,
                          newFunctionSet->ComponentStop);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentResume,
                          newFunctionSet->ComponentResume);
    required->AddEventHandlerWrite(&mtsManagerComponentClient::HandleChangeState, this, 
                                   mtsManagerComponentBase::EventNames::ChangeState, MTS_EVENT_NOT_QUEUED);

    // Remember a required interface (InterfaceComponent's required interface) 
    // to connect it to the provided interface (InterfaceInternals's provided 
    // interface).
    if (!InterfaceComponentFunctionMap.AddItem(clientComponentName, newFunctionSet)) {
        CMN_LOG_CLASS_INIT_ERROR << "AddNewClientComponent: failed to add \"Component\" required interface: " 
            << "\"" << clientComponentName << "\", " << interfaceName << std::endl;
        return false;
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "AddNewClientComponent: successfully added new client component: " 
        << clientComponentName << std::endl;

    return true;
}

void mtsManagerComponentClient::InterfaceComponentCommands_ComponentCreate(const mtsDescriptionComponent & arg)
{
    if (!InterfaceLCMFunction.ComponentCreate.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentCreate: failed to execute \"Component Create\"" << std::endl;
        return;
    }

    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    const std::string nameOfThisLCM = LCM->GetProcessName();
    if (LCM->GetConfiguration() == mtsManagerLocal::LCM_CONFIG_STANDALONE || 
        nameOfThisLCM == arg.ProcessName) 
    {
        InterfaceLCMCommands_ComponentCreate(arg);
        return;
    } else {
        //InterfaceLCMFunction.ComponentCreate.ExecuteBlocking(arg);
        InterfaceLCMFunction.ComponentCreate(arg);
    }
}

void mtsManagerComponentClient::InterfaceComponentCommands_ComponentConnect(const mtsDescriptionConnection & arg)
{
    if (!InterfaceLCMFunction.ComponentConnect.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentConnect: failed to execute \"Component Connect\"" << std::endl;
        return;
    }

    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    const std::string nameOfThisLCM = LCM->GetProcessName();
    if (LCM->GetConfiguration() == mtsManagerLocal::LCM_CONFIG_STANDALONE ||
        (nameOfThisLCM == arg.Client.ProcessName && nameOfThisLCM == arg.Server.ProcessName))
    {
        InterfaceLCMCommands_ComponentConnect(arg);
        return;
    } else {
        //InterfaceLCMFunction.ComponentConnect.ExecuteBlocking(arg);
        InterfaceLCMFunction.ComponentConnect(arg);
    }
}

void mtsManagerComponentClient::InterfaceComponentCommands_ComponentStart(const mtsComponentStatusControl & arg)
{
    if (!InterfaceLCMFunction.ComponentStart.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentStart: failed to execute \"Component Start\"" << std::endl;
        return;
    }

    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    const std::string nameOfThisLCM = LCM->GetProcessName();
    if (LCM->GetConfiguration() == mtsManagerLocal::LCM_CONFIG_STANDALONE ||
        nameOfThisLCM == arg.ProcessName) 
    {
        // Check if the component specified exists
        if (!LCM->GetComponent(arg.ComponentName)) {
            CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentStart: failed to execute \"Component Start\"" << std::endl;
            return;
        }

        InterfaceLCMCommands_ComponentStart(arg);
        return;
    } else {
        //InterfaceLCMFunction.ComponentStart.ExecuteBlocking(arg);
        InterfaceLCMFunction.ComponentStart(arg);
    }
}

void mtsManagerComponentClient::InterfaceComponentCommands_ComponentStop(const mtsComponentStatusControl & arg)
{
    if (!InterfaceLCMFunction.ComponentStop.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentStop: failed to execute \"Component Stop\"" << std::endl;
        return;
    }

    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    const std::string nameOfThisLCM = LCM->GetProcessName();
    if (LCM->GetConfiguration() == mtsManagerLocal::LCM_CONFIG_STANDALONE ||
        LCM->GetProcessName() == arg.ProcessName) 
    {
        // Check if the component specified exists
        if (!LCM->GetComponent(arg.ComponentName)) {
            CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentStop: failed to execute \"Component Stop\"" << std::endl;
            return;
        }

        InterfaceLCMCommands_ComponentStop(arg);
        return;
    } else {
        //InterfaceLCMFunction.ComponentStop.ExecuteBlocking(arg);
        InterfaceLCMFunction.ComponentStop(arg);
    }
}

void mtsManagerComponentClient::InterfaceComponentCommands_ComponentResume(const mtsComponentStatusControl & arg)
{
    if (!InterfaceLCMFunction.ComponentResume.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentResume: failed to execute \"Component Resume\"" << std::endl;
        return;
    }

    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    const std::string nameOfThisLCM = LCM->GetProcessName();
    if (LCM->GetConfiguration() == mtsManagerLocal::LCM_CONFIG_STANDALONE ||
        LCM->GetProcessName() == arg.ProcessName)
    {
        // Check if the component specified exists
        if (!LCM->GetComponent(arg.ComponentName)) {
            CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_ComponentResume: failed to execute \"Component Resume\"" << std::endl;
            return;
        }

        InterfaceLCMCommands_ComponentResume(arg);
        return;
    } else {
        //InterfaceLCMFunction.ComponentResume.ExecuteBlocking(arg);
        InterfaceLCMFunction.ComponentResume(arg);
    }
}

void mtsManagerComponentClient::InterfaceComponentCommands_GetNamesOfProcesses(mtsStdStringVec & names) const
{
    if (!InterfaceLCMFunction.GetNamesOfProcesses.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_GetNamesOfProcesses: failed to execute \"GetNamesOfProcesses\"" << std::endl;
        return;
    }

    InterfaceLCMFunction.GetNamesOfProcesses(names);
}

void mtsManagerComponentClient::InterfaceComponentCommands_GetNamesOfComponents(const mtsStdString & processName, mtsStdStringVec & names) const
{
    if (!InterfaceLCMFunction.GetNamesOfComponents.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_GetNamesOfComponents: failed to execute \"GetNamesOfComponents\"" << std::endl;
        return;
    }

    InterfaceLCMFunction.GetNamesOfComponents(processName, names);
}

void mtsManagerComponentClient::InterfaceComponentCommands_GetNamesOfInterfaces(
    const mtsDescriptionComponent & component, mtsDescriptionInterface & interfaces) const
{
    if (!InterfaceLCMFunction.GetNamesOfInterfaces.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_GetNamesOfInterfaces: failed to execute \"GetNamesOfInterfaces\"" << std::endl;
        return;
    }

    InterfaceLCMFunction.GetNamesOfInterfaces(component, interfaces);
}

void mtsManagerComponentClient::InterfaceComponentCommands_GetListOfConnections(std::vector <mtsDescriptionConnection> & listOfConnections) const
{
    if (!InterfaceLCMFunction.GetListOfConnections.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceComponentCommands_GetListOfConnections: failed to execute \"GetListOfConnections\"" << std::endl;
        return;
    }

    InterfaceLCMFunction.GetListOfConnections(listOfConnections);
}

void mtsManagerComponentClient::InterfaceLCMCommands_ComponentCreate(const mtsDescriptionComponent & arg)
{
    // Steps to create a component dynamically :
    // 1. Create a component
    // 2. Add the created component to the local component manager
    // 3. Add internal interfaces to the component (InterfaceInternal).
    if (!CreateAndAddNewComponent(arg.ClassName, arg.ComponentName)) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentCreate: failed to execute \"ComponentCreate\": " << arg << std::endl;
        return;
    }

    // 4. Create InterfaceComponent's required interface which connects to 
    //    InterfaceInternal's provided interface.
    if (!AddNewClientComponent(arg.ComponentName)) {
        CMN_LOG_CLASS_INIT_ERROR << "InterfaceLCMCommands_ComponentCreate: "
            << "failed to add InterfaceComponent's required interface to manager component client: "
            << "\"" << arg.ComponentName << "\"" << std::endl;
        return;
    }

    // 5. Connect InterfaceInternal's interfaces to InterfaceComponent's
    //    interfaces.
    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    if (!LCM->ConnectToManagerComponentClient(arg.ComponentName)) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentCreate: failed to connect component to manager component client: " << arg << std::endl;
        return;
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "InterfaceLCMCommands_ComponentCreate: successfully created new component: " << arg << std::endl;
}

void mtsManagerComponentClient::InterfaceLCMCommands_ComponentConnect(const mtsDescriptionConnection & arg)
{
    // Try to connect interfaces as requested
    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();

#if CISST_MTS_HAS_ICE
    if (!LCM->Connect(arg.Client.ProcessName, arg.Client.ComponentName, arg.Client.InterfaceName,
                      arg.Server.ProcessName, arg.Server.ComponentName, arg.Server.InterfaceName))
#else
    if (!LCM->Connect(arg.Client.ComponentName, arg.Client.InterfaceName,
                      arg.Server.ComponentName, arg.Server.InterfaceName))
#endif
    {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentConnect: failed to execute \"Component Connect\": " << arg << std::endl;
        return;
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "InterfaceLCMCommands_ComponentConnect: successfully connected: " << arg << std::endl;
}

void mtsManagerComponentClient::InterfaceLCMCommands_ComponentStart(const mtsComponentStatusControl & arg)
{
    // Create internal thread (if needed)
    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    mtsComponent * component = LCM->GetComponent(arg.ComponentName);
    if (!component) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentStart - no component found: "
            << arg.ComponentName << std::endl;
        return;
    }
    
    // Wait for internal thread to be created
    osaSleep(arg.DelayInSecond);

    // Start an internal thread (if needed)
    component->Create();

    // Start the component
    component->Start();
}

void mtsManagerComponentClient::InterfaceLCMCommands_ComponentStop(const mtsComponentStatusControl & arg)
{
    // Get a set of function objects that are bound to the InterfaceLCM's provided
    // interface.
    InterfaceComponentFunctionType * functionSet = InterfaceComponentFunctionMap.GetItem(arg.ComponentName);
    if (!functionSet) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentStop: failed to execute \"Component Stop\"" << std::endl;
        return;
    }
    if (!functionSet->ComponentStop.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentStop: failed to execute \"Component Stop\"" << arg << std::endl;
    }

    // MJ: This Component Stop command could be executed through local component 
    // manager but it is not thread safe.  For thread-safe stop/resume, we
    // use the cisstMultiTask's thread-safe command pattern instead.
    //functionSet->ComponentStop.ExecuteBlocking(arg);
    functionSet->ComponentStop(arg);
}

void mtsManagerComponentClient::InterfaceLCMCommands_ComponentResume(const mtsComponentStatusControl & arg)
{
    // Create internal thread (if needed)
    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();
    mtsComponent * component = LCM->GetComponent(arg.ComponentName);
    if (!component) {
        CMN_LOG_CLASS_RUN_ERROR << "InterfaceLCMCommands_ComponentStart - no component found: "
            << arg.ComponentName << std::endl;
        return;
    }
    
    // Wait for internal thread to be created
    osaSleep(arg.DelayInSecond);

    // Resume (Start) the component
    component->Start();
}

void mtsManagerComponentClient::HandleAddComponentEvent(const mtsDescriptionComponent &component)
{
    CMN_LOG_INIT_VERBOSE << "MCC AddComponent event, component = " << component << std::endl;
    // Generate event to connected components
    InterfaceComponentEvents_AddComponent(component);
}

void mtsManagerComponentClient::HandleAddConnectionEvent(const mtsDescriptionConnection &connection)
{
    CMN_LOG_INIT_VERBOSE << "MCC AddConnection event, connection = " << connection << std::endl;
    // Generate event to connected components
    InterfaceComponentEvents_AddConnection(connection);
}

void mtsManagerComponentClient::HandleChangeState(const mtsComponentStateChange & componentStateChange)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "MCC ChangeState event: " << componentStateChange << std::endl;

    // MJ: TODO: propagate this event to MCS (Manager Component Server)?
}