/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsDeviceInterfaceProxyClient.h 142 2009-03-11 23:02:34Z mjung5 $

  Author(s):  Min Yang Jung
  Created on: 2009-04-24

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDeviceInterfaceProxyClient_h
#define _mtsDeviceInterfaceProxyClient_h

#include <cisstMultiTask/mtsDeviceInterface.h>
#include <cisstMultiTask/mtsProxyBaseClient.h>
#include <cisstMultiTask/mtsDeviceInterfaceProxy.h>

#include <cisstMultiTask/mtsExport.h>

/*!
  \ingroup cisstMultiTask

  TODO: add class summary here
*/

class CISST_EXPORT mtsDeviceInterfaceProxyClient : public mtsProxyBaseClient<mtsTask> {
    
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
    mtsDeviceInterfaceProxyClient(const std::string & propertyFileName, 
                                  const std::string & propertyName);
    ~mtsDeviceInterfaceProxyClient();

    /*! Set a client task connected to this proxy client. Currently, this client task 
        can have the same number of the provided interfaces provided by the server task,
        which means only 1:1 connection between a provided interface and a required
        interface is allowed, right now. */
    void SetConnectedTask(mtsTask * clientTask) { ConnectedTask = clientTask; }

    /*! Entry point to run a proxy. */
    void Start(mtsTask * callingTask);

    /*! Stop the proxy. */
    void Stop();

protected:
    /*! Typedef for base type. */
    typedef mtsProxyBaseClient<mtsTask> BaseType;

    /*! Typedef for server proxy. */
    typedef mtsDeviceInterfaceProxy::DeviceInterfaceServerPrx DeviceInterfaceServerProxyType;

    /*! Pointer to the task connected. */
    mtsTask * ConnectedTask;

    /*! Connected server object */
    DeviceInterfaceServerProxyType DeviceInterfaceServerProxy;
    
    //-------------------------------------------------------------------------
    //  Proxy Implementation
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    //  Processing Methods
    //-------------------------------------------------------------------------
    /*! Send thread set up. */
    class DeviceInterfaceClientI;
    typedef IceUtil::Handle<DeviceInterfaceClientI> DeviceInterfaceClientIPtr;
    DeviceInterfaceClientIPtr Sender;

    /*! Create a proxy object and a send thread. */
    void CreateProxy() {
        DeviceInterfaceServerProxy = 
            mtsDeviceInterfaceProxy::DeviceInterfaceServerPrx::checkedCast(ProxyObject);
        if (!DeviceInterfaceServerProxy) {
            throw "Invalid proxy";
        }

        Sender = new DeviceInterfaceClientI(IceCommunicator, Logger, DeviceInterfaceServerProxy, this);
    }

    /*! Start a send thread and wait for shutdown (blocking call). */
    void StartClient();

    /*! Thread runner */
    static void Runner(ThreadArguments<mtsTask> * arguments);

    /*! Clean up thread-related resources. */
    void OnThreadEnd();

public:
    //-------------------------------------------------------------------------
    //  Methods to Receive and Process Events (Server -> Client)
    //-------------------------------------------------------------------------
    void ReceiveExecuteEventVoid(const int commandId);
    void ReceiveExecuteEventWriteSerialized(const int commandId, const std::string argument);
    bool ReceiveGetListsOfEventGeneratorsRegistered(
        const std::string & serverTaskProxyName, 
        const std::string & requiredInterfaceName,
        mtsDeviceInterfaceProxy::ListsOfEventGeneratorsRegistered & eventGeneratorProxies) const;

    //-------------------------------------------------------------------------
    //  Methods to Send Events (Client -> Server)
    //-------------------------------------------------------------------------
    const bool SendGetProvidedInterfaceInfo(
        const std::string & providedInterfaceName,
        mtsDeviceInterfaceProxy::ProvidedInterfaceInfo & providedInterfaceInfo);

    bool SendConnectServerSide(
        const std::string & userTaskName, const std::string & requiredInterfaceName,
        const std::string & resourceTaskName, const std::string & providedInterfaceName);

    void SendGetCommandId(
        const std::string & clientTaskProxyName,
        mtsDeviceInterfaceProxy::FunctionProxySet & functionProxies);

    void SendExecuteCommandVoid(const int commandId) const;
    void SendExecuteCommandWriteSerialized(const int commandId, const cmnGenericObject & argument);
    void SendExecuteCommandReadSerialized(const int commandId, cmnGenericObject & argument);
    void SendExecuteCommandQualifiedReadSerialized(
        const int commandId, const cmnGenericObject & argument1, cmnGenericObject & argument2);

    //-------------------------------------------------------------------------
    //  Definition by mtsDeviceInterfaceProxy.ice
    //-------------------------------------------------------------------------
protected:
    class DeviceInterfaceClientI : public mtsDeviceInterfaceProxy::DeviceInterfaceClient,
                                   public IceUtil::Monitor<IceUtil::Mutex>
    {
    private:
        Ice::CommunicatorPtr Communicator;
        bool Runnable;
        
        IceUtil::ThreadPtr Sender;
        Ice::LoggerPtr Logger;
        mtsDeviceInterfaceProxy::DeviceInterfaceServerPrx Server;
        mtsDeviceInterfaceProxyClient * DeviceInterfaceClient;

    public:
        DeviceInterfaceClientI(const Ice::CommunicatorPtr& communicator,                           
                               const Ice::LoggerPtr& logger,
                               const mtsDeviceInterfaceProxy::DeviceInterfaceServerPrx& server,
                               mtsDeviceInterfaceProxyClient * DeviceInterfaceClient);

        void Start();
        void Run();
        void Stop();

        // Server -> Client
        void ExecuteEventVoid(::Ice::Int, const ::Ice::Current&);
        void ExecuteEventWriteSerialized(::Ice::Int, const ::std::string&, const ::Ice::Current&);
        bool GetListsOfEventGeneratorsRegistered(
            const std::string & serverTaskProxyName,
            const std::string & requiredInterfaceName,
            mtsDeviceInterfaceProxy::ListsOfEventGeneratorsRegistered &, const ::Ice::Current&) const;
    };
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDeviceInterfaceProxyClient)

#endif // _mtsDeviceInterfaceProxyClient_h
