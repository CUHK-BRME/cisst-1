/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Peter Kazanzides, Anton Deguet
  Created on: 2008-11-13

  (C) Copyright 2008-2010 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsInterfaceRequiredOrInput_h
#define _mtsInterfaceRequiredOrInput_h

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassRegisterMacros.h>

#include <cisstMultiTask/mtsForwardDeclarations.h>

// Always include last
#include <cisstMultiTask/mtsExport.h>

/*!
  \file
  \brief Declaration of mtsInterfaceRequiredOrInput
 */


/*!
  \ingroup cisstMultiTask

  Base class for mtsInterfaceRequired and mtsInterfaceInput.  This
  class contains pure virtual methods and can not be instantiated.
 */

class CISST_EXPORT mtsInterfaceRequiredOrInput: public cmnGenericObject
{
    friend class mtsComponentProxy;
    friend class mtsComponentInterfaceProxyClient;
    friend class mtsManagerLocal;
    friend class mtsManagerLocalTest;

protected:

    /*! A string identifying the 'Name' of the required interface. */
    std::string Name;

    /*! Pointer to provided interface that we are connected to. */
    mtsDeviceInterface * ProvidedInterface;

    /*! Set as true when this interface is registered to the globalcomponent manager */
    bool Registered;

    /*! Default constructor. Does nothing, should not be used. */
    mtsInterfaceRequiredOrInput(void) {}

 public:
    /*! Constructor. Sets the name.
        \param interfaceName Name of required interface
    */
    mtsInterfaceRequiredOrInput(const std::string & interfaceName);

    /*! Default destructor. */
    virtual ~mtsInterfaceRequiredOrInput();

    /*! Returns the name of the interface. */
    const std::string & GetName(void) const;

    const mtsDeviceInterface * GetConnectedInterface(void) const;

    virtual bool CouldConnectTo(mtsDeviceInterface * interfaceProvidedOrOutput) = 0;
    virtual bool ConnectTo(mtsDeviceInterface * interfaceProvidedOrOutput) = 0;
    virtual bool Disconnect(void) = 0;

    /*! Bind command and events.  This method needs to provide a user
      Id so that GetCommandVoid and GetCommandWrite (queued
      commands) know which mailbox to use.  The user Id is provided
      by the provided interface when calling AllocateResources. */
    // bool BindCommandsAndEvents(unsigned int userId);

    /*! Getter and setter for 'Registered' flag */
    //@{
    void SetRegistered(void);
    bool GetRegistered(void) const;
    //@}

};


#endif // _mtsInterfaceRequiredOrInput_h

