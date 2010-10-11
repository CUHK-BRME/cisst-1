/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsManagerComponentServices.h 1726 2010-08-30 05:07:54Z mjung5 $

  Author(s):  Min Yang Jung, Peter Kazanzides
  Created on: 2010-08-29

  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsManagerComponentServices_h
#define _mtsManagerComponentServices_h

#include <cisstMultiTask/mtsManagerComponentBase.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsParameterTypes.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionQualifiedRead.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
//#include <cisstMultiTask/mtsParameterTypes.h>
//#include <cisstMultiTask/mtsForwardDeclarations.h>

class CISST_EXPORT mtsManagerComponentServices : public cmnGenericObject {

    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR)

protected:
    mtsInterfaceRequired * InternalInterfaceRequired;

    /*! Internal functions to use services provided by manager component client */
    // Dynamic component management
    struct {
        mtsFunctionWrite Create;
        mtsFunctionWrite Connect;
        mtsFunctionWrite Start;
        mtsFunctionWrite Stop;
        mtsFunctionWrite Resume;
    } ServiceComponentManagement;

    // Getters
    struct {
        mtsFunctionRead          GetNamesOfProcesses;
        mtsFunctionQualifiedRead GetNamesOfComponents; // in: process name, out: components' names
        mtsFunctionQualifiedRead GetNamesOfInterfaces; // in: process name, out: interfaces' names
        mtsFunctionRead          GetListOfConnections;
    } ServiceGetters;

public:
    mtsManagerComponentServices(mtsInterfaceRequired * internalInterfaceRequired);
    ~mtsManagerComponentServices() {}

    bool InitializeInterfaceInternalRequired(void);

    /*! Add event handlers to the internal required interface */
    //@{
    template <class __classType>
    bool AddComponentEventHandler(void (__classType::*method)(const mtsDescriptionComponent &),
                                  __classType * classInstantiation,
                                  mtsEventQueueingPolicy queuingPolicy = MTS_INTERFACE_EVENT_POLICY)
    {
        if (InternalInterfaceRequired) {
            InternalInterfaceRequired->AddEventHandlerWrite(method, classInstantiation, 
                mtsManagerComponentBase::EventNames::AddComponent, queuingPolicy);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Required interface not set for AddComponentEventHandler" << std::endl;
        }

        return (InternalInterfaceRequired != 0);
    }

    template <class __classType>
    bool AddConnectionEventHandler(void (__classType::*method)(const mtsDescriptionConnection &),
                                   __classType * classInstantiation,
                                   mtsEventQueueingPolicy queuingPolicy = MTS_INTERFACE_EVENT_POLICY)
    {
        if (InternalInterfaceRequired) {
            InternalInterfaceRequired->AddEventHandlerWrite(method, classInstantiation, 
                mtsManagerComponentBase::EventNames::AddConnection, queuingPolicy);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Required interface not set for AddConnectionEventHandler" << std::endl;
        }

        return (InternalInterfaceRequired != 0);
    }

    template <class __classType>
    bool ChangeStateEventHandler(void (__classType::*method)(const mtsComponentStateChange &),
                                 __classType * classInstantiation,
                                 mtsEventQueueingPolicy queuingPolicy = MTS_INTERFACE_EVENT_POLICY)
    {
        if (InternalInterfaceRequired) {
            InternalInterfaceRequired->AddEventHandlerWrite(method, classInstantiation, 
                mtsManagerComponentBase::EventNames::ChangeState, queueingPolicy);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Required interface not set for ChangeStateEventHandler" << std::endl;
        }

        return (InternalInterfaceRequired != 0);
    }
    //@}

    /*! Wrappers for internal function object */
    //@{
    bool RequestComponentCreate(const std::string & className, const std::string & componentName) const;
    bool RequestComponentCreate(
        const std::string& processName, const std::string & className, const std::string & componentName) const;

    bool RequestComponentConnect(
        const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
        const std::string & serverComponentName, const std::string & serverInterfaceProvidedName) const;
    bool RequestComponentConnect(
        const std::string & clientProcessName,
        const std::string & clientComponentName, const std::string & clientInterfaceRequiredName,
        const std::string & serverProcessName,
        const std::string & serverComponentName, const std::string & serverInterfaceProvidedName) const;

    bool RequestComponentStart(const std::string & componentName, const double delayInSecond = 0.0) const;
    bool RequestComponentStart(const std::string& processName, const std::string & componentName,
                               const double delayInSecond = 0.0) const;

    bool RequestComponentStop(const std::string & componentName, const double delayInSecond = 0.0) const;
    bool RequestComponentStop(const std::string& processName, const std::string & componentName,
                              const double delayInSecond = 0.0) const;

    bool RequestComponentResume(const std::string & componentName, const double delayInSecond = 0.0) const;
    bool RequestComponentResume(const std::string& processName, const std::string & componentName,
                                const double delayInSecond = 0.0) const;

    bool RequestGetNamesOfProcesses(std::vector<std::string> & namesOfProcesses) const;
    bool RequestGetNamesOfComponents(const std::string & processName, std::vector<std::string> & namesOfComponents) const;
    bool RequestGetNamesOfInterfaces(const std::string & processName,
                                     const std::string & componentName,
                                     std::vector<std::string> & namesOfInterfacesRequired,
                                     std::vector<std::string> & namesOfInterfacesProvided) const;
    bool RequestGetListOfConnections(std::vector<mtsDescriptionConnection> & listOfConnections) const;
    //@}

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsManagerComponentServices)

#endif // _mtsManagerComponentBase_h
