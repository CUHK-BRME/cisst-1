/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsManagerProxyClient.h 142 2009-03-11 23:02:34Z mjung5 $

  Author(s):  Min Yang Jung
  Created on: 2010-01-20

  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsManagerProxyClient_h
#define _mtsManagerProxyClient_h

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsManagerProxy.h>
#include <cisstMultiTask/mtsProxyBaseClient.h>
#include <cisstMultiTask/mtsManagerGlobalInterface.h>

#include <cisstMultiTask/mtsExport.h>

class CISST_EXPORT mtsManagerProxyClient : 
    public mtsProxyBaseClient<mtsManagerLocal>, public mtsManagerGlobalInterface
{
    
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

    /*! Typedef for base type. */
    typedef mtsProxyBaseClient<mtsManagerLocal> BaseClientType;

    /*! Typedef for connected server proxy. */
    typedef mtsManagerProxy::ManagerServerPrx ManagerServerProxyType;
    ManagerServerProxyType ManagerServerProxy;

protected:
    /*! Definitions for send thread */
    class ManagerClientI;
    typedef IceUtil::Handle<ManagerClientI> ManagerClientIPtr;
    ManagerClientIPtr Sender;

    /*! Instance counter used to set a short name of this thread */
    static unsigned int InstanceCounter;

    //-------------------------------------------------------------------------
    //  Proxy Implementation
    //-------------------------------------------------------------------------
    /*! Create a proxy object and a send thread. */
    void CreateProxy() {
        ManagerServerProxy = 
            mtsManagerProxy::ManagerServerPrx::checkedCast(ProxyObject);
        if (!ManagerServerProxy) {
            throw "mtsManagerProxyClient: Invalid proxy";
        }

        Sender = new ManagerClientI(IceCommunicator, IceLogger, ManagerServerProxy, this);
    }
    
    /*! Start a send thread and wait for shutdown (blocking call). */
    void StartClient();

    /*! Resource clean-up when a client disconnects or is disconnected.
        TODO: add session
        TODO: add resource clean up
        TODO: review/add safe termination  */
    void OnClose();

    /*! Thread runner */
    static void Runner(ThreadArguments<mtsManagerLocal> * arguments);

    /*! Miscellaneous methods */
    void GetConnectionStringSet(mtsManagerProxy::ConnectionStringSet & connectionStringSet,
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName,
        const std::string & requestProcessName = "");

    //-------------------------------------------------------------------------
    //  Event Handlers : Server -> Client
    //-------------------------------------------------------------------------
    void ReceiveTestMessageFromServerToClient(const std::string & str) const;

    // Proxy object control (creation and removal)
    bool ReceiveCreateComponentProxy(const std::string & componentProxyName);
    bool ReceiveRemoveComponentProxy(const std::string & componentProxyName);
    bool ReceiveCreateProvidedInterfaceProxy(const std::string & serverComponentProxyName, const ::mtsManagerProxy::ProvidedInterfaceDescription & providedInterfaceDescription);
    bool ReceiveCreateRequiredInterfaceProxy(const std::string & clientComponentProxyName, const ::mtsManagerProxy::RequiredInterfaceDescription & requiredInterfaceDescription);
    bool ReceiveRemoveProvidedInterfaceProxy(const std::string & clientComponentProxyName, const std::string & providedInterfaceProxyName);
    bool ReceiveRemoveRequiredInterfaceProxy(const std::string & serverComponentProxyName, const std::string & requiredInterfaceProxyName);

    // Connection management
    bool ReceiveConnectServerSideInterface(::Ice::Int providedInterfaceProxyInstanceID, const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet);
    bool ReceiveConnectClientSideInterface(::Ice::Int connectionID, const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet);

    // Getters
    bool ReceiveGetProvidedInterfaceDescription(const std::string & componentName, const std::string & providedInterfaceName, ::mtsManagerProxy::ProvidedInterfaceDescription & providedInterfaceDescription);
    bool ReceiveGetRequiredInterfaceDescription(const std::string & componentName, const std::string & requiredInterfaceName, ::mtsManagerProxy::RequiredInterfaceDescription & requiredInterfaceDescription);
    std::string ReceiveGetProcessName();
    ::Ice::Int ReceiveGetCurrentInterfaceCount(const std::string & componentName);

public:
    /*! Constructor and destructor */
    mtsManagerProxyClient(const std::string & serverEndpointInfo, const std::string & communicatorID)
        : BaseClientType(serverEndpointInfo, communicatorID)
    {}

    ~mtsManagerProxyClient()
    {}

    /*! Entry point to run a proxy. */
    bool Start(mtsManagerLocal * proxyOwner);

    /*! Stop the proxy (clean up thread-related resources) */
    void Stop(void);

    //-------------------------------------------------------------------------
    //  Implementation of mtsManagerGlobalInterface
    //  (See mtsManagerGlobalInterface.h for details)
    //-------------------------------------------------------------------------
    //  Process Management
    bool AddProcess(const std::string & processName);
    bool FindProcess(const std::string & processName) const;
    bool RemoveProcess(const std::string & processName);

    //  Component Management
    bool AddComponent(const std::string & processName, const std::string & componentName);
    bool FindComponent(const std::string & processName, const std::string & componentName) const;
    bool RemoveComponent(const std::string & processName, const std::string & componentName);

    //  Interface Management
    bool AddProvidedInterface(
        const std::string & processName, const std::string & componentName, const std::string & interfaceName, const bool isProxyInterface);
    bool AddRequiredInterface(
        const std::string & processName, const std::string & componentName, const std::string & interfaceName, const bool isProxyInterface);
    bool FindProvidedInterface(
        const std::string & processName, const std::string & componentName, const std::string & interfaceName) const;
    bool FindRequiredInterface(
        const std::string & processName, const std::string & componentName, const std::string & interfaceName) const;
    bool RemoveProvidedInterface(
        const std::string & processName, const std::string & componentName, const std::string & interfaceName);
    bool RemoveRequiredInterface(
        const std::string & processName, const std::string & componentName, const std::string & interfaceName);

    //  Connection Management
    unsigned int Connect(const std::string & requestProcessName,
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName);
    bool ConnectConfirm(unsigned int connectionSessionID);
    bool Disconnect(
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName);

    //  Networking
    bool SetProvidedInterfaceProxyAccessInfo(
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName,
        const std::string & endpointInfo, const std::string & communicatorID);
    bool GetProvidedInterfaceProxyAccessInfo(
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName,
        std::string & endpointInfo, std::string & communicatorID);
    bool InitiateConnect(const unsigned int connectionID,
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName);
    bool ConnectServerSideInterface(const unsigned int providedInterfaceProxyInstanceID,
        const std::string & clientProcessName, const std::string & clientComponentName, const std::string & clientRequiredInterfaceName,
        const std::string & serverProcessName, const std::string & serverComponentName, const std::string & serverProvidedInterfaceName);

    //-------------------------------------------------------------------------
    //  Event Generators (Event Sender) : Client -> Server
    //-------------------------------------------------------------------------
    void SendTestMessageFromClientToServer(const std::string & str) const;

    // Process Management
    bool SendAddProcess(const std::string & processName);
    bool SendFindProcess(const std::string & processName) const;
    bool SendRemoveProcess(const std::string & processName);

    // Component Management
    bool SendAddComponent(const std::string & processName, const std::string & componentName);
    bool SendFindComponent(const std::string & processName, const std::string & componentName) const;
    bool SendRemoveComponent(const std::string & processName, const std::string & componentName);

    bool SendAddProvidedInterface(const std::string & processName, const std::string & componentName, const std::string & interfaceName, bool isProxyInterface);
    bool SendFindProvidedInterface(const std::string & processName, const std::string & componentName, const std::string & interfaceName) const;
    bool SendRemoveProvidedInterface(const std::string & processName, const std::string & componentName, const std::string & interfaceName);

    bool SendAddRequiredInterface(const std::string & processName, const std::string & componentName, const std::string & interfaceName, bool isProxyInterface);
    bool SendFindRequiredInterface(const std::string & processName, const std::string & componentName, const std::string & interfaceName) const;
    bool SendRemoveRequiredInterface(const std::string & processName, const std::string & componentName, const std::string & interfaceName);

    // Connection Management
    ::Ice::Int SendConnect(const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet);
    bool SendConnectConfirm(::Ice::Int connectionSessionID);
    bool SendDisconnect(const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet);

    // Networking
    bool SendSetProvidedInterfaceProxyAccessInfo(const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet, const std::string & endpointInfo, const std::string & communicatorID);
    bool SendGetProvidedInterfaceProxyAccessInfo(const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet, std::string & endpointInfo, std::string & communicatorID);
    bool SendInitiateConnect(::Ice::Int connectionID, const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet);
    bool SendConnectServerSideInterface(::Ice::Int providedInterfaceProxyInstanceID, const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet);

    //-------------------------------------------------------------------------
    //  Definition by mtsDeviceInterfaceProxy.ice
    //-------------------------------------------------------------------------
protected:
    class ManagerClientI : 
        public mtsManagerProxy::ManagerClient, public IceUtil::Monitor<IceUtil::Mutex>
    {
    private:
        /*! Ice objects */
        Ice::CommunicatorPtr Communicator;
        IceUtil::ThreadPtr SenderThreadPtr;
        Ice::LoggerPtr IceLogger;

        /*! Network event processor */
        mtsManagerProxyClient * ManagerProxyClient;

        /*! Connected server proxy */
        mtsManagerProxy::ManagerServerPrx Server;

    public:
        ManagerClientI(
            const Ice::CommunicatorPtr& communicator,                           
            const Ice::LoggerPtr& logger,
            const mtsManagerProxy::ManagerServerPrx& server,
            mtsManagerProxyClient * ManagerClient);

        /*! Proxy management */
        void Start();
        void Run();
        void Stop();

        /*! Getter */
        bool IsActiveProxy() const {
            return ManagerProxyClient->IsActiveProxy();
        }

        //-------------------------------------------------
        //  Event handlers (Server -> Client)
        //-------------------------------------------------
        /*! Test method */
        void TestMessageFromServerToClient(const std::string & str, const ::Ice::Current & current);

        /*! Proxy object control (creation and removal) */
        bool CreateComponentProxy(const std::string & componentProxyName, const ::Ice::Current & current);
        bool RemoveComponentProxy(const std::string & componentProxyName, const ::Ice::Current & current);
        bool CreateProvidedInterfaceProxy(const std::string & serverComponentProxyName, const ::mtsManagerProxy::ProvidedInterfaceDescription & providedInterfaceDescription, const ::Ice::Current & current);
        bool CreateRequiredInterfaceProxy(const std::string & clientComponentProxyName, const ::mtsManagerProxy::RequiredInterfaceDescription & requiredInterfaceDescription, const ::Ice::Current & current);
        bool RemoveProvidedInterfaceProxy(const std::string & clientComponentProxyName, const std::string & providedInterfaceProxyName, const ::Ice::Current & current);
        bool RemoveRequiredInterfaceProxy(const std::string & serverComponentProxyName, const std::string & requiredInterfaceProxyName, const ::Ice::Current & current);

        /*! Connection management */
        bool ConnectServerSideInterface(::Ice::Int providedInterfaceProxyInstanceID, const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet, const ::Ice::Current & current);
        bool ConnectClientSideInterface(::Ice::Int connectionID, const ::mtsManagerProxy::ConnectionStringSet & connectionStringSet, const ::Ice::Current & current);

        /*! Getters */
        bool GetProvidedInterfaceDescription(const std::string & componentName, const std::string & providedInterfaceName, ::mtsManagerProxy::ProvidedInterfaceDescription & providedInterfaceDescription, const ::Ice::Current &) const;
        bool GetRequiredInterfaceDescription(const std::string & componentName, const std::string & requiredInterfaceName, ::mtsManagerProxy::RequiredInterfaceDescription & requiredInterfaceDescription, const ::Ice::Current &) const;        
        std::string GetProcessName(const ::Ice::Current &) const;
        ::Ice::Int GetCurrentInterfaceCount(const std::string & componentName, const ::Ice::Current &) const;
    };
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsManagerProxyClient)

#endif // _mtsManagerProxyClient_h
