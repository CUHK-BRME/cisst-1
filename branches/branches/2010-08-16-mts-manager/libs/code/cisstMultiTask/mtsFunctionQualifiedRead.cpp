/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Peter Kazanzides, Anton Deguet

  (C) Copyright 2007-2010 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <cisstMultiTask/mtsFunctionQualifiedRead.h>
#include <cisstMultiTask/mtsCommandQualifiedReadBase.h>


mtsFunctionQualifiedRead::~mtsFunctionQualifiedRead()
{}


// documented in base class
bool mtsFunctionQualifiedRead::Detach(void) {
    if (this->IsValid()) {
        Command = 0;
        return true;
    }
    return false;
}


bool mtsFunctionQualifiedRead::IsValid(void) const {
    return (this->Command != 0);
}


bool mtsFunctionQualifiedRead::Bind(CommandType * command) {
    Command = command;
    return (command != 0);
}


mtsCommandBase::ReturnType mtsFunctionQualifiedRead::operator()(const mtsGenericObject & qualifier,
                                                                mtsGenericObject & argument) const
{
    return Command ? Command->Execute(qualifier, argument) : mtsCommandBase::NO_INTERFACE;
}


mtsFunctionQualifiedRead::CommandType * mtsFunctionQualifiedRead::GetCommand(void) const {
    return Command;
}


const mtsGenericObject * mtsFunctionQualifiedRead::GetArgument1Prototype(void) const
{
    if (this->Command) {
        return this->Command->GetArgument1Prototype();
    }
    return 0;
}


const mtsGenericObject * mtsFunctionQualifiedRead::GetArgument2Prototype(void) const
{
    if (this->Command) {
        return this->Command->GetArgument2Prototype();
    }
    return 0;
}


void mtsFunctionQualifiedRead::ToStream(std::ostream & outputStream) const {
    if (this->Command != 0) {
        outputStream << "mtsFunctionQualifiedRead for " << *Command;
    } else {
        outputStream << "mtsFunctionQualifiedRead not initialized";
    }
}
