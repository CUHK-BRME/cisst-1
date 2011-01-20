/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Ankur Kapoor, Peter Kazanzides, Anton Deguet
  Created on: 2004-04-30

  (C) Copyright 2004-2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnGenericObjectProxy.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsFunctionVoid.h>
#include <cisstMultiTask/mtsComponent.h>


mtsInterfaceProvidedOrOutput::mtsInterfaceProvidedOrOutput(const std::string & interfaceName,
                                                           mtsComponent * component):
    Name(interfaceName),
    Component(component)
{
}


const std::string & mtsInterfaceProvidedOrOutput::GetName(void) const
{
    return this->Name;
}


const std::string & mtsInterfaceProvidedOrOutput::GetComponentName(void) const
{
    if (!Component) {
        return "Unnamed";
    } else {
        return this->Component->GetName();
    }
}


void mtsInterfaceProvidedOrOutput::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_DEBUG << "Cleanup: default implementation from mtsInterfaceProvidedOrOutput called" << std::endl;
}
