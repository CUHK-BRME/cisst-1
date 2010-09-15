/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Ankur Kapoor, Peter Kazanzides, Anton Deguet, Min Yang Jung
  Created on: 2004-04-30

  (C) Copyright 2004-2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceOutput.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceInput.h>
#include <cisstMultiTask/mtsManagerComponentBase.h>

#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstOSAbstraction/osaSleep.h>

const std::string mtsComponent::NameOfInterfaceInternalProvided = "InterfaceInternalProvided";
const std::string mtsComponent::NameOfInterfaceInternalRequired = "InterfaceInternalRequired";

const std::string mtsComponent::EventNames::AddComponent = "AddComponentEvent"; // mtsManagerComponentBase::EventNames::AddComponent;
const std::string mtsComponent::EventNames::AddConnection = "AddConnectionEvent"; // mtsManagerComponentBase::EventNames::AddConnection;

mtsComponent::mtsComponent(const std::string & componentName):
    Name(componentName),
    State(mtsComponentState::CONSTRUCTED),
    UseSeparateLogFileFlag(false),
    LoDMultiplexerStreambuf(0),
    LogFile(0),
    InterfacesProvidedOrOutput("InterfacesProvided"),
    InterfacesRequiredOrInput("InterfacesRequiredOrInput")
{
    InterfacesProvidedOrOutput.SetOwner(*this);
    InterfacesRequiredOrInput.SetOwner(*this);
}


mtsComponent::mtsComponent(void):
    UseSeparateLogFileFlag(false),
    LoDMultiplexerStreambuf(0),
    LogFile(0),
    InterfacesProvidedOrOutput("InterfacesProvided"),
    InterfacesRequiredOrInput("InterfacesRequiredOrInput")
{
    InterfacesProvidedOrOutput.SetOwner(*this);
    InterfacesRequiredOrInput.SetOwner(*this);
}


mtsComponent::mtsComponent(const mtsComponent & other):
    cmnGenericObject(other)
{
    cmnThrow("Class mtsComponent: copy constructor for mtsComponent should never be called");
}


mtsComponent::~mtsComponent()
{
    if (this->LoDMultiplexerStreambuf) {
        this->LoDMultiplexerStreambuf->RemoveAllChannels();
        delete this->LoDMultiplexerStreambuf;
        this->LoDMultiplexerStreambuf = 0;
    }
    if (this->LogFile) {
        this->LogFile->close();
        delete this->LogFile;
    }
}


const std::string & mtsComponent::GetName(void) const
{
    return this->Name;
}


void mtsComponent::SetName(const std::string & componentName)
{
    this->Name = componentName;
}


void mtsComponent::Start(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Start: default start method for component \""
                               << this->GetName() << "\"" << std::endl;
}


void mtsComponent::Configure(const std::string & CMN_UNUSED(filename))
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: default start method for component \""
                               << this->GetName() << "\"" << std::endl;
}


std::vector<std::string> mtsComponent::GetNamesOfInterfacesProvidedOrOutput(void) const
{
    return InterfacesProvidedOrOutput.GetNames();
}


std::vector<std::string> mtsComponent::GetNamesOfInterfacesProvided(void) const
{
    std::vector<std::string> names;
    InterfacesProvidedListType::const_iterator iterator = InterfacesProvided.begin();
    const InterfacesProvidedListType::const_iterator end = InterfacesProvided.end();
    for (;
         iterator != end;
         ++iterator) {
        names.push_back((*iterator)->GetName());
    }
    return names;
}


std::vector<std::string> mtsComponent::GetNamesOfInterfacesOutput(void) const
{
    std::vector<std::string> names;
    InterfacesOutputListType::const_iterator iterator = InterfacesOutput.begin();
    const InterfacesOutputListType::const_iterator end = InterfacesOutput.end();
    for (;
         iterator != end;
         ++iterator) {
        names.push_back((*iterator)->GetName());
    }
    return names;
}


mtsInterfaceProvided * mtsComponent::AddInterfaceProvided(const std::string & interfaceProvidedName,
                                                          mtsInterfaceQueuingPolicy queuingPolicy)
{
    mtsInterfaceProvided * interfaceProvided;
    if ((queuingPolicy == MTS_COMPONENT_POLICY)
        || (queuingPolicy == MTS_COMMANDS_SHOULD_NOT_BE_QUEUED)) {
        interfaceProvided = new mtsInterfaceProvided(interfaceProvidedName, this, MTS_COMMANDS_SHOULD_NOT_BE_QUEUED);
    } else {
        CMN_LOG_CLASS_INIT_WARNING << "AddInterfaceProvided: adding provided interface \"" << interfaceProvidedName
                                   << "\" with policy MTS_COMMANDS_SHOULD_BE_QUEUED to component \""
                                   << this->GetName() << "\", make sure you call ProcessQueuedCommands to empty the queues" << std::endl;
        interfaceProvided = new mtsInterfaceProvided(interfaceProvidedName, this, MTS_COMMANDS_SHOULD_BE_QUEUED);
    }
    if (interfaceProvided) {
        if (InterfacesProvidedOrOutput.AddItem(interfaceProvidedName, interfaceProvided, CMN_LOG_LOD_INIT_ERROR)) {
            InterfacesProvided.push_back(interfaceProvided);
            return interfaceProvided;
        }
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceProvided: unable to add interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete interfaceProvided;
        return 0;
    }
    CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceProvided: unable to create interface \""
                             << interfaceProvidedName << "\"" << std::endl;
    return 0;
}


mtsInterfaceOutput * mtsComponent::AddInterfaceOutput(const std::string & interfaceOutputName)
{
    CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceOutput: adding output interface \"" << interfaceOutputName
                             << "\" to component \"" << this->GetName()
                             << "\" can't be performed from mtsComponent, this method must be derived." << std::endl;
    return 0;
}


mtsInterfaceProvidedOrOutput *
mtsComponent::GetInterfaceProvidedOrOutput(const std::string & interfaceProvidedOrOutputName)
{
    return InterfacesProvidedOrOutput.GetItem(interfaceProvidedOrOutputName,
                                              CMN_LOG_LOD_INIT_ERROR);
}


mtsInterfaceProvided *
mtsComponent::GetInterfaceProvided(const std::string & interfaceProvidedName) const
{
    return dynamic_cast<mtsInterfaceProvided *>(InterfacesProvidedOrOutput.GetItem(interfaceProvidedName,
                                                                                   CMN_LOG_LOD_INIT_ERROR));
}


mtsInterfaceOutput *
mtsComponent::GetInterfaceOutput(const std::string & interfaceOutputName) const
{
    return dynamic_cast<mtsInterfaceOutput *>(InterfacesProvidedOrOutput.GetItem(interfaceOutputName,
                                                                                 CMN_LOG_LOD_INIT_ERROR));
}


size_t mtsComponent::GetNumberOfInterfacesProvided(void) const
{
    return InterfacesProvided.size();
}


size_t mtsComponent::GetNumberOfInterfacesOutput(void) const
{
    return InterfacesOutput.size();
}


bool mtsComponent::RemoveInterfaceProvided(const std::string & interfaceProvidedName)
{
    mtsInterfaceProvided * interfaceProvided = GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceProvided: no provided interface found: \""
                                << interfaceProvidedName << "\"" << std::endl;
        return false;
    }

    // MJUNG: right now we don't consider the "safe disconnection" issue as
    // follows (not complete) but have to deal with them in the future.
    //
    // \todo Need to remove (clean up as well) all provided interface
    // instances which were created by this interface.
    //
    // \todo Need to add clean-up codes before deleting interface object
    // itself through "safe disconnection" mechanism. Otherwise, a connected
    // required interface might try to use provided interface's resource which
    // are already deallocated and invalidated.
    // The "safe disconection" mechanism should include (at least)
    // - To disable required interface so that users cannot use it any more
    // - To disable provided interface and instances that it generated
    // - (something more...)
    // - To disconnect the connection that the provided interface was related to
    // - To deallocate provided interface and instances that it generated
    // Things to consider:
    // - Thread-safety issue between caller thread and main thread (main thread
    //   uses component's inner structure(s) to process commands and events)
    // - Safe clean-up to avoid run-time crash

    // MJUNG: this code is NOT thread safe.
    if (!InterfacesProvidedOrOutput.RemoveItem(interfaceProvidedName)) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceProvided: failed to remove provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        return false;
    }

    bool removed = false;
    InterfacesProvidedListType::iterator it = InterfacesProvided.begin();
    const InterfacesProvidedListType::const_iterator itEnd = InterfacesProvided.end();
    for (; it != itEnd; ++it) {
        // MJUNG: this code is NOT thread safe.
        if (*it == interfaceProvided) {
            InterfacesProvided.erase(it);
            removed = true;
            break;
        }
    }

    if (!removed) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceProvided: failed to remove provided interface \""
                                 << interfaceProvidedName << "\" from list" << std::endl;
        return false;
    }

    delete interfaceProvided;

    CMN_LOG_CLASS_RUN_VERBOSE << "RemoveInterfaceProvided: removed provided interface \""
                              << interfaceProvidedName << "\"" << std::endl;
    return true;
}


bool mtsComponent::RemoveInterfaceOutput(const std::string & interfaceOutputName)
{
    mtsInterfaceOutput * interfaceOutput = GetInterfaceOutput(interfaceOutputName);
    if (!interfaceOutput) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceOutput: no output interface found: \""
                                << interfaceOutputName << "\"" << std::endl;
        return false;
    }

    if (!InterfacesProvidedOrOutput.RemoveItem(interfaceOutputName)) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceOutput: failed to remove output interface \""
                                << interfaceOutputName << "\"" << std::endl;
        return false;
    }

    bool removed = false;
    InterfacesOutputListType::iterator it = InterfacesOutput.begin();
    const InterfacesOutputListType::const_iterator itEnd = InterfacesOutput.end();
    for (; it != itEnd; ++it) {
        if (*it == interfaceOutput) {
            InterfacesOutput.erase(it);
            removed = true;
            break;
        }
    }

    if (!removed) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceOutput: failed to remove output interface \""
                                 << interfaceOutputName << "\" from list" << std::endl;
        return false;
    }

    delete interfaceOutput;
    CMN_LOG_CLASS_RUN_VERBOSE << "RemoveInterfaceOutput: removed output interface \""
                              << interfaceOutputName << "\"" << std::endl;
    return true;
}


mtsInterfaceRequiredOrInput *
mtsComponent::GetInterfaceRequiredOrInput(const std::string & interfaceRequiredOrInputName)
{
    return InterfacesRequiredOrInput.GetItem(interfaceRequiredOrInputName,
                                             CMN_LOG_LOD_INIT_ERROR);
}


mtsInterfaceRequired *
mtsComponent::GetInterfaceRequired(const std::string & interfaceRequiredName)
{
    return dynamic_cast<mtsInterfaceRequired *>(InterfacesRequiredOrInput.GetItem(interfaceRequiredName,
                                                                                  CMN_LOG_LOD_INIT_ERROR));
}


mtsInterfaceInput *
mtsComponent::GetInterfaceInput(const std::string & interfaceInputName) const
{
    return dynamic_cast<mtsInterfaceInput *>(InterfacesRequiredOrInput.GetItem(interfaceInputName,
                                                                               CMN_LOG_LOD_INIT_ERROR));
}


size_t mtsComponent::GetNumberOfInterfacesRequired(void) const
{
    return InterfacesRequired.size();
}


size_t mtsComponent::GetNumberOfInterfacesInput(void) const
{
    return InterfacesInput.size();
}


bool mtsComponent::RemoveInterfaceRequired(const std::string & interfaceRequiredName)
{
    mtsInterfaceRequired * interfaceRequired = GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceRequired: no required interface found: \""
                                << interfaceRequiredName << "\"" << std::endl;
        return false;
    }

    // MJUNG: see comments in mtsComponent::RemoveInterfaceProvided() for
    // potential thread-safety issues

    // MJUNG: this code is NOT thread safe.
    if (!InterfacesRequiredOrInput.RemoveItem(interfaceRequiredName)) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceRequired: failed to remove required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }

    bool removed = false;
    InterfacesRequiredListType::iterator it = InterfacesRequired.begin();
    const InterfacesRequiredListType::const_iterator itEnd = InterfacesRequired.end();
    for (; it != itEnd; ++it) {
        if (*it == interfaceRequired) {
            // MJUNG: this code is NOT thread safe.
            InterfacesRequired.erase(it);
            removed = true;
            break;
        }
    }

    if (!removed) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceRequired: failed to remove required interface \""
                                 << interfaceRequiredName << "\" from list" << std::endl;
        return false;
    }

    delete interfaceRequired;

    CMN_LOG_CLASS_RUN_VERBOSE << "RemoveInterfaceRequired: removed required interface \""
                              << interfaceRequiredName << "\"" << std::endl;
    return true;
}


bool mtsComponent::RemoveInterfaceInput(const std::string & interfaceInputName)
{
    mtsInterfaceInput * interfaceInput = GetInterfaceInput(interfaceInputName);
    if (!interfaceInput) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceInput: no required interface found: \""
                                << interfaceInputName << "\"" << std::endl;
        return false;
    }

    if (!InterfacesRequiredOrInput.RemoveItem(interfaceInputName)) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceInput: failed to remove required interface \""
                                << interfaceInputName << "\"" << std::endl;
        return false;
    }

    bool removed = false;
    InterfacesInputListType::iterator it = InterfacesInput.begin();
    const InterfacesInputListType::const_iterator itEnd = InterfacesInput.end();
    for (; it != itEnd; ++it) {
        if (*it == interfaceInput) {
            InterfacesInput.erase(it);
            removed = true;
            break;
        }
    }

    if (!removed) {
        CMN_LOG_CLASS_RUN_ERROR << "RemoveInterfaceInput: failed to remove required interface \""
                                 << interfaceInputName << "\" from list" << std::endl;
        return false;
    }

    delete interfaceInput;
    CMN_LOG_CLASS_RUN_VERBOSE << "RemoveInterfaceInput: removed required interface \""
                              << interfaceInputName << "\"" << std::endl;
    return true;
}


mtsInterfaceRequired * mtsComponent::AddInterfaceRequiredExisting(const std::string & interfaceRequiredName,
                                                                  mtsInterfaceRequired * interfaceRequired) {
    if (InterfacesRequiredOrInput.AddItem(interfaceRequiredName, interfaceRequired)) {
        InterfacesRequired.push_back(interfaceRequired);
        return interfaceRequired;
    }
    return 0;
}


mtsInterfaceRequired * mtsComponent::AddInterfaceRequiredUsingMailbox(const std::string & interfaceRequiredName,
                                                                      mtsMailBox * mailBox,
                                                                      mtsRequiredType required)
{
    mtsInterfaceRequired * interfaceRequired = new mtsInterfaceRequired(interfaceRequiredName, mailBox, required);
    if (interfaceRequired) {
        if (InterfacesRequiredOrInput.AddItem(interfaceRequiredName, interfaceRequired)) {
            InterfacesRequired.push_back(interfaceRequired);
            return interfaceRequired;
        }
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceRequired: unable to add interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        if (interfaceRequired) {
            delete interfaceRequired;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceRequired: unable to create interface for \""
                                 << interfaceRequiredName << "\"" << std::endl;
    }
    return 0;
}


mtsInterfaceRequired * mtsComponent::AddInterfaceRequired(const std::string & interfaceRequiredName,
                                                          mtsRequiredType required) {
    // by default, no mailbox for base component, events are not queued
    return this->AddInterfaceRequiredUsingMailbox(interfaceRequiredName, 0, required);
}


mtsInterfaceInput * mtsComponent::AddInterfaceInput(const std::string & interfaceInputName)
{
    CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceInput: adding input interface \"" << interfaceInputName
                             << "\" to component \"" << this->GetName()
                             << "\" can't be performed from mtsComponent, this method must be derived." << std::endl;
    return 0;
}


mtsInterfaceInput * mtsComponent::AddInterfaceInputExisting(const std::string & interfaceInputName,
                                                            mtsInterfaceInput * interfaceInput) {
    if (InterfacesRequiredOrInput.AddItem(interfaceInputName, interfaceInput)) {
        InterfacesInput.push_back(interfaceInput);
        return interfaceInput;
    }
    CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceInputExisting: component \"" << this->GetName()
                             << "\" already has a required or input interface \"" << interfaceInputName
                             << "\"" << std::endl;
    return 0;
}


mtsInterfaceOutput * mtsComponent::AddInterfaceOutputExisting(const std::string & interfaceOutputName,
                                                              mtsInterfaceOutput * interfaceOutput) {
    if (InterfacesProvidedOrOutput.AddItem(interfaceOutputName, interfaceOutput)) {
        InterfacesOutput.push_back(interfaceOutput);
        return interfaceOutput;
    }
    CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceOutputExisting: component \"" << this->GetName()
                             << "\" already has a provided or output interface \"" << interfaceOutputName
                             << "\"" << std::endl;
    return 0;
}


std::vector<std::string> mtsComponent::GetNamesOfInterfacesRequiredOrInput(void) const {
    return InterfacesRequiredOrInput.GetNames();
}


std::vector<std::string> mtsComponent::GetNamesOfInterfacesRequired(void) const
{
    std::vector<std::string> names;
    InterfacesRequiredListType::const_iterator iterator = InterfacesRequired.begin();
    const InterfacesRequiredListType::const_iterator end = InterfacesRequired.end();
    for (;
         iterator != end;
         ++iterator) {
        names.push_back((*iterator)->GetName());
    }
    return names;
}


std::vector<std::string> mtsComponent::GetNamesOfInterfacesInput(void) const
{
    std::vector<std::string> names;
    InterfacesInputListType::const_iterator iterator = InterfacesInput.begin();
    const InterfacesInputListType::const_iterator end = InterfacesInput.end();
    for (;
         iterator != end;
         ++iterator) {
        names.push_back((*iterator)->GetName());
    }
    return names;
}


const mtsInterfaceProvidedOrOutput * mtsComponent::GetInterfaceProvidedOrOutputFor(const std::string & interfaceRequiredOrInputName) {
    mtsInterfaceRequiredOrInput * interfaceRequiredOrInput =
        InterfacesRequiredOrInput.GetItem(interfaceRequiredOrInputName, CMN_LOG_LOD_INIT_WARNING);
    return interfaceRequiredOrInput ? interfaceRequiredOrInput->GetConnectedInterface() : 0;
}


bool mtsComponent::ConnectInterfaceRequiredOrInput(const std::string & interfaceRequiredOrInputName,
                                                   mtsInterfaceProvidedOrOutput * interfaceProvidedOrOutput)
{
    mtsInterfaceRequiredOrInput * interfaceRequiredOrInput =
        InterfacesRequiredOrInput.GetItem(interfaceRequiredOrInputName, CMN_LOG_LOD_INIT_ERROR);
    if (interfaceRequiredOrInput) {
        if (interfaceRequiredOrInput->ConnectTo(interfaceProvidedOrOutput)) {
            CMN_LOG_CLASS_INIT_VERBOSE << "ConnectInterfaceRequiredOrInput: component \""
                                       << this->GetName()
                                       << "\" required/input interface \"" << interfaceRequiredOrInputName
                                       << "\" successfully connected to provided/output interface \""
                                       << interfaceProvidedOrOutput->GetName() << "\"" << std::endl;
            return true;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConnectInterfaceRequiredOrInput: component \""
                                     << this->GetName()
                                     << "\" required/input interface \"" << interfaceRequiredOrInputName
                                     << "\" failed to connect to provided/output interface \""
                                     << interfaceProvidedOrOutput->GetName() << "\"" << std::endl;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConnectInterfaceRequiredOrInput: component \""
                                 << this->GetName()
                                 << "\" doesn't have required/input interface \""
                                 << interfaceRequiredOrInputName << "\"" << std::endl;
    }
    return false;
}


// Execute all commands in the mailbox.  This is just a temporary implementation, where
// all commands in a mailbox are executed before moving on the next mailbox.  The final
// implementation will probably look at timestamps.  We may also want to pass in a
// parameter (enum) to set the mailbox processing policy.
size_t mtsComponent::ProcessMailBoxes(InterfacesProvidedListType & interfaces)
{
    size_t numberOfCommands = 0;
    InterfacesProvidedListType::iterator iterator = interfaces.begin();
    const InterfacesProvidedListType::iterator end = interfaces.end();
    for (;
         iterator != end;
         ++iterator) {
        numberOfCommands += (*iterator)->ProcessMailBoxes();
    }
    return numberOfCommands;
}


size_t mtsComponent::ProcessQueuedEvents(void) {
    InterfacesRequiredListType::iterator iterator = InterfacesRequired.begin();
    const InterfacesRequiredListType::iterator end = InterfacesRequired.end();
    size_t numberOfEvents = 0;
    for (;
         iterator != end;
         iterator++) {
        numberOfEvents += (*iterator)->ProcessMailBoxes();
    }
    return numberOfEvents;
}


void mtsComponent::ToStream(std::ostream & outputStream) const
{
    outputStream << "Component name: " << Name << std::endl;
    InterfacesProvidedOrOutput.ToStream(outputStream);
    InterfacesRequiredOrInput.ToStream(outputStream);
}


void mtsComponent::UseSeparateLogFileDefault(bool forwardToLogger)
{
    std::string currentDateTime;
    osaGetDateTimeString(currentDateTime);
    std::string filename = this->GetName() + "-" + currentDateTime + "-log.txt";
    this->UseSeparateLogFile(filename, forwardToLogger);
}


void mtsComponent::UseSeparateLogFile(const std::string & filename, bool forwardToLogger)
{
    if (this->UseSeparateLogFileFlag) {
        CMN_LOG_CLASS_INIT_ERROR << "UseSeparateLogFile: flag already set for component \""
                                 << this->GetName() << "\", no-op" << std::endl;
        return;
    }
    CMN_LOG_CLASS_INIT_DEBUG << "UseSeparateLogFile: called for component \""
                             << this->GetName() << "\"" << std::endl;
    // create the multiplexer and log file if needed
    if (!this->LoDMultiplexerStreambuf) {
        this->LoDMultiplexerStreambuf = new cmnLogger::StreamBufType();
    }
    // if there is already a log file, remove it from multiplexer, close it and delete it
    if (this->LogFile) {
        this->LoDMultiplexerStreambuf->RemoveChannel(*(this->LogFile));
        this->LogFile->close();
        delete this->LogFile;
        CMN_LOG_CLASS_INIT_DEBUG << "UseSeparateLogFile: closed and removed previous log file for component \""
                                 << this->GetName() << "\"" << std::endl;
    }
    // create a new log file and add it to the multiplexer
    this->LogFile = new std::ofstream();
    this->LogFile->open(filename.c_str());
    if (this->LogFile->is_open()) {
        // set the multiplexer and change flag et the end!
        CMN_LOG_CLASS_INIT_DEBUG << "UseSeparateLogFile: opened log file \"" << filename
                                 << "\" for component \"" << this->GetName() << "\"" << std::endl;
        this->LoDMultiplexerStreambuf->AddChannel(*(this->LogFile), CMN_LOG_LOD_VERY_VERBOSE);
        if (forwardToLogger) {
            // note that if the component multiplexer already existed
            // and the cmnLogger multiplexer was already added, this
            // line has no effect
            this->LoDMultiplexerStreambuf->AddChannel(cmnLogger::GetMultiplexer(), CMN_LOG_LOD_VERY_VERBOSE);
        }
        this->UseSeparateLogFileFlag = true;
    } else {
        // can't open file, do not switch flag
        CMN_LOG_CLASS_INIT_ERROR << "UseSeparateLogFile: can't open log file \"" << filename
                                 << "\" for component \"" << this->GetName() << "\"" << std::endl;
    }
}


cmnLogger::StreamBufType * mtsComponent::GetLogMultiplexer(void) const
{
    if (this->UseSeparateLogFileFlag) {
        ThisType * nonConstThis = const_cast<ThisType *>(this);
        return nonConstThis->LoDMultiplexerStreambuf;
    }
    return cmnGenericObject::GetLogMultiplexer();
}


bool mtsComponent::IsRunning(void) const
{
    return (this->State == mtsComponentState::ACTIVE);
 }


bool mtsComponent::IsStarted(void) const
{
    return (this->State >= mtsComponentState::READY);
}


bool mtsComponent::IsTerminated(void) const
{
    return (this->State == mtsComponentState::FINISHED);
}


bool mtsComponent::IsEndTask(void) const
{
    return (this->State >= mtsComponentState::FINISHING);
}


const mtsComponentState & mtsComponent::GetState(void) const
{
    return this->State;
}


bool mtsComponent::WaitForState(mtsComponentState::Enum desiredState, double timeout)
{
    if (timeout != 0.0) {
        CMN_LOG_CLASS_INIT_VERBOSE << "WaitForState: called for component \"" << this->GetName()
                                   << "\" has no effect for mtsComponent" << std::endl;
    }
    return true;
}


bool mtsComponent::AddInterfaceInternal(void)
{
    // Add required interface
    std::string interfaceName = mtsComponent::NameOfInterfaceInternalRequired;
    mtsInterfaceRequired * required = AddInterfaceRequired(interfaceName);
    if (!required) {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceInternal: failed to add internal required interface: " << interfaceName << std::endl;
        return false;
    }
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentCreate,
                          InternalInterfaceFunctions.ComponentCreate);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentConnect,
                          InternalInterfaceFunctions.ComponentConnect);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStart,
                          InternalInterfaceFunctions.ComponentStart);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentStop,
                          InternalInterfaceFunctions.ComponentStop);
    required->AddFunction(mtsManagerComponentBase::CommandNames::ComponentResume,
                          InternalInterfaceFunctions.ComponentResume);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetNamesOfProcesses,
                          InternalInterfaceFunctions.GetNamesOfProcesses);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetNamesOfComponents,
                          InternalInterfaceFunctions.GetNamesOfComponents);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetNamesOfInterfaces,
                          InternalInterfaceFunctions.GetNamesOfInterfaces);
    required->AddFunction(mtsManagerComponentBase::CommandNames::GetListOfConnections,
                          InternalInterfaceFunctions.GetListOfConnections);

    // Add provided interface
    interfaceName = mtsComponent::NameOfInterfaceInternalProvided;
    mtsInterfaceProvided * provided = AddInterfaceProvided(interfaceName);
    if (!provided) {
        CMN_LOG_CLASS_INIT_ERROR << "AddInterfaceInternal: failed to add internal provided interface: " << interfaceName << std::endl;
        return false;
    }
    provided->AddCommandWrite(&mtsComponent::InterfaceInternalCommands_ComponentStop,
                             this, mtsManagerComponentBase::CommandNames::ComponentStop);
    provided->AddCommandWrite(&mtsComponent::InterfaceInternalCommands_ComponentResume,
                             this, mtsManagerComponentBase::CommandNames::ComponentResume);

    CMN_LOG_CLASS_INIT_VERBOSE << "AddInterfaceInternal: successfully added internal interfaces" << std::endl;

    return true;
}

void mtsComponent::InterfaceInternalCommands_ComponentStop(const mtsComponentStatusControl & arg)
{
    // MJ: How to implement "Stopping device-type component which might be already running"?
    // 
    // Possible solutions might be:
    // - For device-type component: disable all commands and functions in all 
    //   interfaces of this component.
    // - For task-type component: the above + call Suspend()
    //
    // The current implementation only can handle task-type component through mtsTask::Suspend()

    CMN_LOG_CLASS_RUN_VERBOSE << "InterfaceInternalCommands_ComponentStop: stopping component: " << GetName() << std::endl;

    // Stop device-type component
    mtsTask * task = dynamic_cast<mtsTask *>(this);
    if (!task) {
        // TODO: implement stopping a device-type component
        cmnThrow("InterfaceInternalCommands_ComponentStop: TODO - implement this function");
        return;
    } 
    // Stop task-type component
    else {
        task->Suspend();
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "InterfaceInternalCommands_ComponentStart: stopped component:  " << GetName() << std::endl;
}

void mtsComponent::InterfaceInternalCommands_ComponentResume(const mtsComponentStatusControl & arg)
{
    // MJ: How to implement "Resuming device-type component which might be already running"?
    // 
    // Possible solutions might be:
    // - For device-type component: enable all commands and functions in all 
    //   interfaces of this component.
    // - For task-type component: the above + call Start()

    CMN_LOG_CLASS_RUN_VERBOSE << "InterfaceInternalCommands_ComponentResume: resuming component: " << GetName() << std::endl;

    // Stop device-type component
    mtsTask * task = dynamic_cast<mtsTask *>(this);
    if (!task) {
        // TODO: implement stopping a device-type component
        cmnThrow("InterfaceInternalCommands_ComponentResume: TODO - implement this function");
        return;
    } 
    // Stop task-type component
    else {
        task->Start();
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "InterfaceInternalCommands_ComponentResume: resumed component:  " << GetName() << std::endl;
}

bool mtsComponent::RequestComponentCreate(const std::string & className, const std::string & componentName) const
{
    if (!InternalInterfaceFunctions.ComponentCreate.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentCreate: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsDescriptionComponent arg;
    arg.ProcessName   = mtsManagerLocal::GetInstance()->GetProcessName();
    arg.ClassName     = className;
    arg.ComponentName = componentName;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentCreate(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentCreate: requested component creation: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentCreate(
    const std::string& processName, const std::string & className, const std::string & componentName) const
{
    if (!InternalInterfaceFunctions.ComponentCreate.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentCreate: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsDescriptionComponent arg;
    arg.ProcessName   = processName;
    arg.ClassName     = className;
    arg.ComponentName = componentName;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentCreate(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentCreate: requested component creation: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentConnect(
    const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
    const std::string & serverComponentName, const std::string & serverInterfaceProvidedName) const
{
    if (!InternalInterfaceFunctions.ComponentConnect.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentConnect: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsDescriptionConnection arg;
    const std::string thisProcessName = mtsManagerLocal::GetInstance()->GetProcessName();
    arg.Client.ProcessName   = thisProcessName;
    arg.Client.ComponentName = clientComponentName;
    arg.Client.InterfaceName = clientInterfaceRequiredName;
    arg.Server.ProcessName   = thisProcessName;
    arg.Server.ComponentName = serverComponentName;
    arg.Server.InterfaceName = serverInterfaceProvidedName;
    arg.ConnectionID = -1;  // not yet assigned

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentConnect(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentConnect: requested component connection: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentConnect(
    const std::string & clientProcessName, 
    const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
    const std::string & serverProcessName, 
    const std::string & serverComponentName, const std::string & serverInterfaceProvidedName) const
{
    if (!InternalInterfaceFunctions.ComponentConnect.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentConnect: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsDescriptionConnection arg;
    arg.Client.ProcessName   = clientProcessName;
    arg.Client.ComponentName = clientComponentName;
    arg.Client.InterfaceName = clientInterfaceRequiredName;
    arg.Server.ProcessName   = serverProcessName;
    arg.Server.ComponentName = serverComponentName;
    arg.Server.InterfaceName = serverInterfaceProvidedName;
    arg.ConnectionID = -1;  // not yet assigned

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentConnect(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentConnect: requested component connection: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentStart(const std::string & componentName, const double delayInSecond) const
{
    if (!InternalInterfaceFunctions.ComponentStart.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentStart: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsComponentStatusControl arg;
    arg.ProcessName   = mtsManagerLocal::GetInstance()->GetProcessName();
    arg.ComponentName = componentName;
    arg.DelayInSecond = delayInSecond;
    arg.Command       = mtsComponentStatusControl::COMPONENT_START;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentStart(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentStart: requested component start: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentStart(const std::string& processName, const std::string & componentName,
                                         const double delayInSecond) const
{
    if (!InternalInterfaceFunctions.ComponentStart.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentStart: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsComponentStatusControl arg;
    arg.ProcessName   = processName;
    arg.ComponentName = componentName;
    arg.DelayInSecond = delayInSecond;
    arg.Command       = mtsComponentStatusControl::COMPONENT_START;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentStart(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentStart: requested component start: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentStop(const std::string & componentName, const double delayInSecond) const
{
    if (!InternalInterfaceFunctions.ComponentStop.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentStop: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsComponentStatusControl arg;
    arg.ProcessName   = mtsManagerLocal::GetInstance()->GetProcessName();
    arg.ComponentName = componentName;
    arg.DelayInSecond = delayInSecond;
    arg.Command       = mtsComponentStatusControl::COMPONENT_STOP;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentStop(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentStop: requested component stop: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentStop(const std::string& processName, const std::string & componentName,
                                        const double delayInSecond) const
{
    if (!InternalInterfaceFunctions.ComponentStop.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentStop: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsComponentStatusControl arg;
    arg.ProcessName   = processName;
    arg.ComponentName = componentName;
    arg.DelayInSecond = delayInSecond;
    arg.Command       = mtsComponentStatusControl::COMPONENT_STOP;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentStop(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentStop: requested component stop: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentResume(const std::string & componentName, const double delayInSecond) const
{
    if (!InternalInterfaceFunctions.ComponentResume.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentResume: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsComponentStatusControl arg;
    arg.ProcessName   = mtsManagerLocal::GetInstance()->GetProcessName();
    arg.ComponentName = componentName;
    arg.DelayInSecond = delayInSecond;
    arg.Command       = mtsComponentStatusControl::COMPONENT_RESUME;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentResume(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentResume: requested component resume: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestComponentResume(const std::string& processName, const std::string & componentName,
                                          const double delayInSecond) const
{
    if (!InternalInterfaceFunctions.ComponentResume.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestComponentResume: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsComponentStatusControl arg;
    arg.ProcessName   = processName;
    arg.ComponentName = componentName;
    arg.DelayInSecond = delayInSecond;
    arg.Command       = mtsComponentStatusControl::COMPONENT_RESUME;

    // MJ: TODO: change this with blocking command
    InternalInterfaceFunctions.ComponentResume(arg);

    CMN_LOG_CLASS_RUN_VERBOSE << "RequestComponentResume: requested component resume: " << arg << std::endl;

    return true;
}

bool mtsComponent::RequestGetNamesOfProcesses(std::vector<std::string> & namesOfProcesses) const
{
    if (!InternalInterfaceFunctions.GetNamesOfProcesses.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestGetNamesOfProcesses: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsStdStringVec names;
    InternalInterfaceFunctions.GetNamesOfProcesses(names);

    mtsParameterTypes::ConvertVectorStringType(names, namesOfProcesses);

    return true;
}

bool mtsComponent::RequestGetNamesOfComponents(const std::string & processName, std::vector<std::string> & namesOfComponents) const
{
    if (!InternalInterfaceFunctions.GetNamesOfComponents.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestGetNamesOfComponents: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    mtsStdStringVec names;
    InternalInterfaceFunctions.GetNamesOfComponents(mtsStdString(processName), names);

    mtsParameterTypes::ConvertVectorStringType(names, namesOfComponents);

    return true;
}

bool mtsComponent::RequestGetNamesOfInterfaces(const std::string & processName, 
                                               const std::string & componentName,
                                               std::vector<std::string> & namesOfInterfacesRequired,
                                               std::vector<std::string> & namesOfInterfacesProvided) const
{
    if (!InternalInterfaceFunctions.GetNamesOfInterfaces.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestGetNamesOfInterfaces: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    // input arg
    mtsDescriptionComponent argIn;
    argIn.ProcessName   = processName;
    argIn.ComponentName = componentName;

    // output arg
    mtsDescriptionInterface argOut;

    InternalInterfaceFunctions.GetNamesOfInterfaces(argIn, argOut);

    mtsParameterTypes::ConvertVectorStringType(argOut.InterfaceRequiredNames, namesOfInterfacesRequired);
    mtsParameterTypes::ConvertVectorStringType(argOut.InterfaceProvidedNames, namesOfInterfacesProvided);

    return true;
}

bool mtsComponent::RequestGetListOfConnections(std::vector<mtsDescriptionConnection> & listOfConnections) const
{
    if (!InternalInterfaceFunctions.GetListOfConnections.IsValid()) {
        CMN_LOG_CLASS_RUN_ERROR << "RequestGetListOfConnections: invalid function - has not been bound to command" << std::endl;
        return false;
    }

    InternalInterfaceFunctions.GetListOfConnections(listOfConnections);

    return true;
}
