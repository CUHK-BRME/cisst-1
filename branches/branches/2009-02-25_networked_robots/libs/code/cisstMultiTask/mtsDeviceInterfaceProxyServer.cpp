/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsDeviceInterfaceProxyServer.cpp 145 2009-03-18 23:32:40Z mjung5 $

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

#include <cisstCommon/cmnAssert.h>
#include <cisstMultiTask/mtsDeviceInterfaceProxyServer.h>
#include <cisstMultiTask/mtsDeviceInterface.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsDeviceProxy.h>
#include <cisstMultiTask/mtsTask.h>

CMN_IMPLEMENT_SERVICES(mtsDeviceInterfaceProxyServer);

#define DeviceInterfaceProxyServerLogger(_log) Logger->trace("mtsDeviceInterfaceProxyServer", _log)
#define DeviceInterfaceProxyServerLoggerError(_log1, _log2) \
        std::stringstream s;\
        s << "mtsDeviceInterfaceProxyServer: " << _log1 << ": " << _log2;\
        Logger->error(s.str());

mtsDeviceInterfaceProxyServer::mtsDeviceInterfaceProxyServer(
    const std::string& adapterName, const std::string& endpointInfo,
    const std::string& communicatorID) 
    : BaseType(adapterName, endpointInfo, communicatorID), ConnectedTask(0)
{
    Serializer = new cmnSerializer(SerializationBuffer);
    DeSerializer = new cmnDeSerializer(DeSerializationBuffer);
}

mtsDeviceInterfaceProxyServer::~mtsDeviceInterfaceProxyServer()
{
    OnClose();
}

void mtsDeviceInterfaceProxyServer::OnClose()
{
    delete Serializer;
    delete DeSerializer;
}

void mtsDeviceInterfaceProxyServer::Start(mtsTask * callingTask)
{
    // Initialize Ice object.
    // Notice that a worker thread is not created right now.
    Init();
    
    if (InitSuccessFlag) {
        // Create a worker thread here and returns immediately.
        ThreadArgumentsInfo.argument = callingTask;
        ThreadArgumentsInfo.proxy = this;
        ThreadArgumentsInfo.Runner = mtsDeviceInterfaceProxyServer::Runner;

        WorkerThread.Create<ProxyWorker<mtsTask>, ThreadArguments<mtsTask>*>(
            &ProxyWorkerInfo, &ProxyWorker<mtsTask>::Run, &ThreadArgumentsInfo, "C-PRX");
    }
}

void mtsDeviceInterfaceProxyServer::StartServer()
{
    Sender->Start();

    // This is a blocking call that should run in a different thread.
    IceCommunicator->waitForShutdown();
}

void mtsDeviceInterfaceProxyServer::Runner(ThreadArguments<mtsTask> * arguments)
{
    mtsDeviceInterfaceProxyServer * ProxyServer = 
        dynamic_cast<mtsDeviceInterfaceProxyServer*>(arguments->proxy);
    
    ProxyServer->SetConnectedTask(arguments->argument);
    
    // for test
    //mtsDeviceInterfaceProxy::ProvidedInterfaceSequence providedInterfaces;
    //ProxyServer->GetProvidedInterface(providedInterfaces);

    ProxyServer->GetLogger()->trace("mtsDeviceInterfaceProxyServer", "Proxy server starts.");

    try {
        ProxyServer->StartServer();
    } catch (const Ice::Exception& e) {
        ProxyServer->GetLogger()->trace("mtsDeviceInterfaceProxyServer error: ", e.what());
    } catch (const char * msg) {
        ProxyServer->GetLogger()->trace("mtsDeviceInterfaceProxyServer error: ", msg);
    }

    ProxyServer->OnThreadEnd();
}

void mtsDeviceInterfaceProxyServer::Stop()
{
    OnThreadEnd();
}

void mtsDeviceInterfaceProxyServer::OnThreadEnd()
{
    DeviceInterfaceProxyServerLogger("Proxy server ends.");

    BaseType::OnThreadEnd();

    Sender->Stop();
}

mtsProvidedInterface * mtsDeviceInterfaceProxyServer::GetProvidedInterface(
    const std::string resourceDeviceName, const std::string providedInterfaceName) const
{
    mtsTaskManager * taskManager = mtsTaskManager::GetInstance();
    
    mtsProvidedInterface * providedInterface = NULL;
    mtsTask * resourceTask = NULL;
    mtsDevice * resourceDevice = NULL;

    // Get an original resource device or task
    resourceDevice = taskManager->GetDevice(resourceDeviceName);
    if (!resourceDevice) {
        resourceTask = taskManager->GetTask(resourceDeviceName);
        if (!resourceTask) {
            DeviceInterfaceProxyServerLoggerError("GetProvidedInterface", 
                "Cannot find an original resource device or task at server side: " + resourceDeviceName);
            return 0;
        }
        DeviceInterfaceProxyServerLogger("GetProvidedInterface: found an original resource TASK at server side: " + resourceDeviceName);
    } else {
        // Look for the task in the task map (i.e., if a resource task is of mtsTask type)
        DeviceInterfaceProxyServerLogger("GetProvidedInterface: found an original resource DEVICE at server side: " + resourceDeviceName);
    }

    // Get the provided interface
    if (resourceDevice) {
        providedInterface = resourceDevice->GetProvidedInterface(providedInterfaceName);        
    } else {
        CMN_ASSERT(resourceTask);
        providedInterface = resourceTask->GetProvidedInterface(providedInterfaceName);
    }

    CMN_ASSERT(providedInterface);

    return providedInterface;
}

//-------------------------------------------------------------------------
//  Methods to Receive and Process Events
//-------------------------------------------------------------------------
// MJUNG: Currently, only one server task is connected to only one client task.
// Thus, we don't need to manage a client object of which type is 
// DeviceInterfaceClientProxyType while we have to manage them in case of 
// the global task manager (see mtsTaskmanagerProxyServer::ReceiveAddClient()).
void mtsDeviceInterfaceProxyServer::ReceiveAddClient(const DeviceInterfaceClientProxyType & clientProxy)
{
    ConnectedClient = clientProxy;
}

const bool mtsDeviceInterfaceProxyServer::ReceiveGetProvidedInterfaceInfo(
    const std::string & providedInterfaceName,
    ::mtsDeviceInterfaceProxy::ProvidedInterfaceInfo & providedInterfaceInfo)
{
    CMN_ASSERT(ConnectedTask);

    // 1. Get the provided interface by its name.
    mtsDeviceInterface * providedInterface = 
        ConnectedTask->GetProvidedInterface(providedInterfaceName);
    CMN_ASSERT(providedInterface);

    // 2. Get the name of the provided interface.
    providedInterfaceInfo.InterfaceName = providedInterface->GetName();
            
    // 3. Extract all the information on the command objects, events, and so on.
#define ITERATE_COMMAND_BEGIN(_commandType) \
    mtsDeviceInterface::Command##_commandType##MapType::MapType::const_iterator iterator##_commandType = \
        providedInterface->Commands##_commandType.GetMap().begin();\
    mtsDeviceInterface::Command##_commandType##MapType::MapType::const_iterator iterator##_commandType##End = \
        providedInterface->Commands##_commandType.GetMap().end();\
    for (; iterator##_commandType != iterator##_commandType##End; ++iterator##_commandType) {\
        mtsDeviceInterfaceProxy::Command##_commandType##Info info;\
        info.Name = iterator##_commandType->second->GetName();
#define ITERATE_COMMAND_END(_commandType) \
        providedInterfaceInfo.Commands##_commandType.push_back(info);\
    }

    // 3-1. Command: Void
    ITERATE_COMMAND_BEGIN(Void);            
    ITERATE_COMMAND_END(Void);

    // 3-2. Command: Write
    ITERATE_COMMAND_BEGIN(Write);
        info.ArgumentTypeName = iteratorWrite->second->GetArgumentClassServices()->GetName();
    ITERATE_COMMAND_END(Write);

    // 3-3. Command: Read
    ITERATE_COMMAND_BEGIN(Read);
        info.ArgumentTypeName = iteratorRead->second->GetArgumentClassServices()->GetName();
    ITERATE_COMMAND_END(Read);

    // 3-4. Command: QualifiedRead
    ITERATE_COMMAND_BEGIN(QualifiedRead);
        info.Argument1TypeName = iteratorQualifiedRead->second->GetArgument1Prototype()->Services()->GetName();
        info.Argument2TypeName = iteratorQualifiedRead->second->GetArgument2Prototype()->Services()->GetName();
    ITERATE_COMMAND_END(QualifiedRead);

    // for debug
    //mtsDeviceInterface::EventVoidMapType::MapType::const_iterator iteratorEventVoid = 
    //    providedInterface->EventVoidGenerators.GetMap().begin();
    //mtsDeviceInterface::EventVoidMapType::MapType::const_iterator iteratorEventVoidEnd = 
    //    providedInterface->EventVoidGenerators.GetMap().end();
    //for (; iteratorEventVoid != iteratorEventVoidEnd; ++iteratorEventVoid) {
    //    mtsDeviceInterfaceProxy::EventVoidInfo info;
    //    info.Name = iteratorEventVoid->second->GetName();            
    //}
#define ITERATE_EVENT_BEGIN(_eventType)\
    mtsDeviceInterface::Event##_eventType##MapType::MapType::const_iterator iteratorEvent##_eventType = \
        providedInterface->Event##_eventType##Generators.GetMap().begin();\
    mtsDeviceInterface::Event##_eventType##MapType::MapType::const_iterator iteratorEvent##_eventType##End = \
        providedInterface->Event##_eventType##Generators.GetMap().end();\
    for (; iteratorEvent##_eventType != iteratorEvent##_eventType##End; ++iteratorEvent##_eventType) {\
        mtsDeviceInterfaceProxy::Event##_eventType##Info info;\
        info.Name = iteratorEvent##_eventType->second->GetName();
#define ITERATE_EVENT_END(_eventType)\
        providedInterfaceInfo.Events##_eventType.push_back(info);\
    }

    // 3-5) Event: Void
    ITERATE_EVENT_BEGIN(Void);
    ITERATE_EVENT_END(Void);

    // 3-6) Event: Write
    ITERATE_EVENT_BEGIN(Write);
    ITERATE_EVENT_END(Write);

#undef ITERATE_COMMAND_BEGIN
#undef ITERATE_COMMAND_END
#undef ITERATE_EVENT_BEGIN
#undef ITERATE_EVENT_END

    return true;
}

bool mtsDeviceInterfaceProxyServer::ReceiveConnectServerSide(
    const std::string & userTaskName, const std::string & requiredInterfaceName,
    const std::string & resourceTaskName, const std::string & providedInterfaceName)
{
    mtsTaskManager * taskManager = mtsTaskManager::GetInstance();

    const std::string clientDeviceProxyName = mtsDeviceProxy::GetClientTaskProxyName(
        resourceTaskName, providedInterfaceName, userTaskName, requiredInterfaceName);

    // Get an original provided interface.
    mtsProvidedInterface * providedInterface = GetProvidedInterface(
        resourceTaskName, providedInterfaceName);
    if (!providedInterface) {
        DeviceInterfaceProxyServerLoggerError("ReceiveConnectServerSide",
            "Failed looking up a provided interface: " + providedInterfaceName);
        return false;
    }

    // Create a client task proxy (mtsDeviceProxy).
    mtsDeviceProxy * clientTaskProxy = new mtsDeviceProxy(clientDeviceProxyName);
    if (!taskManager->AddDevice(clientTaskProxy)) {
        DeviceInterfaceProxyServerLoggerError("ReceiveConnectServerSide",
            "Failed adding a device proxy: " + clientDeviceProxyName);
        return false;
    }

    // Create and populate a required interface proxy (mtsRequiredInterface)
    if (!clientTaskProxy->CreateRequiredInterfaceProxy(
            providedInterface, requiredInterfaceName, this)) {
        DeviceInterfaceProxyServerLoggerError("ReceiveConnectServerSide",
            "Failed creating a required interface proxy: " + 
            requiredInterfaceName + " @ " + clientTaskProxy->GetName());
        return false;
    }

    // Connect() locally    
    if (!taskManager->Connect(
        clientDeviceProxyName, requiredInterfaceName, resourceTaskName, providedInterfaceName)) 
    {
        DeviceInterfaceProxyServerLoggerError("ReceiveConnectServerSide",
            "Failed to connect: " + 
            userTaskName + " : " + requiredInterfaceName + " - " + 
            resourceTaskName + " : " +  providedInterfaceName);
        return false;
    }

    /*
    // After Connect() is executed successfully at server side, update the command id of 
    // command proxies at client side.
    std::string serverTaskProxyName = mtsDeviceProxy::GetServerTaskProxyName(
            resourceTaskName, providedInterfaceName, userTaskName, requiredInterfaceName);
    if (!GetFunctionPointers(serverTaskProxyName, providedInterfaceName)) {
        DeviceInterfaceProxyServerLoggerError("ReceiveConnectServerSide",
            "Failed to get function pointers: " + serverTaskProxyName);
        return false;
    }
    */

    DeviceInterfaceProxyServerLogger("Connect() at server side succeeded: " +
        userTaskName + " : " + requiredInterfaceName + " - " + 
        resourceTaskName + " : " +  providedInterfaceName);

    return true;
}

void mtsDeviceInterfaceProxyServer::ReceiveGetCommandId(
    const std::string & clientTaskProxyName,
    mtsDeviceInterfaceProxy::FunctionProxySet & functionProxies)
{
    mtsTaskManager * taskManager = mtsTaskManager::GetInstance();

    mtsDeviceProxy * serverTaskProxy = dynamic_cast<mtsDeviceProxy*>(
        taskManager->GetDevice(clientTaskProxyName));
    CMN_ASSERT(serverTaskProxy);

    serverTaskProxy->GetFunctionPointers(functionProxies);
}

void mtsDeviceInterfaceProxyServer::ReceiveExecuteCommandVoid(const int commandId) const
{
    mtsFunctionVoid * functionVoid = reinterpret_cast<mtsFunctionVoid *>(commandId);
    CMN_ASSERT(functionVoid);

    (*functionVoid)();
}

void mtsDeviceInterfaceProxyServer::ReceiveExecuteCommandWriteSerialized(
    const int commandId, const std::string argument)
{
    //mtsCommandWriteBase * commandWrite = reinterpret_cast<mtsCommandWriteBase *>(commandId);
    //CMN_ASSERT(commandWrite);
    mtsFunctionWrite * functionWrite = reinterpret_cast<mtsFunctionWrite*>(commandId);
    CMN_ASSERT(functionWrite);

    static char buf[100];
    sprintf(buf, "ExecuteCommandWriteSerialized: %d bytes received", argument.size());
    Logger->trace("TIServer", buf);

    // Deserialization
    DeSerializationBuffer.str("");
    DeSerializationBuffer << argument;
    
    const mtsGenericObject * obj = dynamic_cast<mtsGenericObject *>(DeSerializer->DeSerialize());
    CMN_ASSERT(obj);
    //commandWrite->Execute(*obj);
    (*functionWrite)(*obj);
}

void mtsDeviceInterfaceProxyServer::ReceiveExecuteCommandReadSerialized(
    const int commandId, std::string & argument)
{
    //mtsCommandReadBase * commandRead = reinterpret_cast<mtsCommandReadBase *>(commandId);    
    //CMN_ASSERT(commandRead);
    mtsFunctionRead * functionRead = reinterpret_cast<mtsFunctionRead*>(commandId);
    CMN_ASSERT(functionRead);

    // Create a placeholder
    mtsGenericObject * placeHolder = dynamic_cast<mtsGenericObject *>(
        //commandRead->GetArgumentClassServices()->Create());
        functionRead->GetCommand()->GetArgumentClassServices()->Create());
    CMN_ASSERT(placeHolder);
    {
        //commandRead->Execute(*placeHolder);
        (*functionRead)(*placeHolder);

        // Serialization
        SerializationBuffer.str("");
        Serializer->Serialize(*placeHolder);
        std::string s = SerializationBuffer.str();

        argument = s;
    }
    delete placeHolder;    
}

void mtsDeviceInterfaceProxyServer::ReceiveExecuteCommandQualifiedReadSerialized(
    const int commandId, const std::string argument1, std::string & argument2)
{
    //
    // TODO: implement here
    //
}

//-------------------------------------------------------------------------
//  Methods to Send Events
//-------------------------------------------------------------------------
//void mtsDeviceInterfaceProxyServer::SendUpdateCommandId(
//    const mtsDeviceInterfaceProxy::FunctionProxySet & functionProxySet)
//{
//    GetLogger()->trace("TIServer", ">>>>> SEND: SendUpdateCommandId");
//
//    ConnectedClient->UpdateCommandId(functionProxySet);
//}

void mtsDeviceInterfaceProxyServer::SendExecuteEventVoid(const int commandId) const
{
    Logger->trace("TIServer", ">>>>> SEND: SendExecuteEventVoid");

    ConnectedClient->ExecuteEventVoid(commandId);
}

void mtsDeviceInterfaceProxyServer::SendExecuteEventWriteSerialized(
    const int commandId, const cmnGenericObject & argument)
{
    Logger->trace("TIServer", ">>>>> SEND: SendExecuteEventWriteSerialized");

    // Serialization
    std::string serializedData;
    //Serialize(argument, serializedData);
    //
    //DeviceInterfaceServerProxy->ExecuteCommandWriteSerialized(commandId, serializedData);
    ConnectedClient->ExecuteEventWriteSerialized(commandId, serializedData);
}

//-------------------------------------------------------------------------
//  Definition by mtsTaskManagerProxy.ice
//-------------------------------------------------------------------------
mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::DeviceInterfaceServerI(
    const Ice::CommunicatorPtr& communicator,
    const Ice::LoggerPtr& logger,
    mtsDeviceInterfaceProxyServer * DeviceInterfaceServer) 
    : Communicator(communicator), Logger(logger),
      DeviceInterfaceServer(DeviceInterfaceServer),
      Runnable(true),
      Sender(new SendThread<DeviceInterfaceServerIPtr>(this))
{
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::Start()
{
    DeviceInterfaceProxyServerLogger("Send thread starts");

    Sender->start();
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::Run()
{
    int num = 0;
    while(true)
    {
        //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
        timedWait(IceUtil::Time::milliSeconds(10));

        if(!Runnable) break;

#ifdef _COMMUNICATION_TEST_
        if(!clients.empty())
        {
            ++num;
            for(std::set<mtsDeviceInterfaceProxy::DeviceInterfaceClientPrx>::iterator p 
                = clients.begin(); p != clients.end(); ++p)
            {
                try
                {
                    std::cout << "server sends: " << num << std::endl;
                    (*p)->ReceiveData(num);
                }
                catch(const IceUtil::Exception& ex)
                {
                    std::cerr << "removing client `" << Communicator->identityToString((*p)->ice_getIdentity()) << "':\n"
                        << ex << std::endl;

                    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
                    _clients.erase(*p);
                }
            }
        }
#endif
    }
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::Stop()
{
    DeviceInterfaceProxyServerLogger("Send thread is terminating.");

    IceUtil::ThreadPtr callbackSenderThread;
    {
        //IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);

        DeviceInterfaceProxyServerLogger("Destroying sender.");
        Runnable = false;

        notify();

        callbackSenderThread = Sender;
        Sender = 0; // Resolve cyclic dependency.
    }
    callbackSenderThread->getThreadControl().join();
}

//-----------------------------------------------------------------------------
//  Device Interface Proxy Server Implementation
//-----------------------------------------------------------------------------
void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::AddClient(
    const ::Ice::Identity& ident, const ::Ice::Current& current)
{
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);

    Logger->trace("TIServer", "<<<<< RECV: AddClient: " + Communicator->identityToString(ident));

    mtsDeviceInterfaceProxy::DeviceInterfaceClientPrx clientProxy = 
        mtsDeviceInterfaceProxy::DeviceInterfaceClientPrx::uncheckedCast(current.con->createProxy(ident));
    
    DeviceInterfaceServer->ReceiveAddClient(clientProxy);
}

bool mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::GetProvidedInterfaceInfo(
    const std::string & providedInterfaceName,
    ::mtsDeviceInterfaceProxy::ProvidedInterfaceInfo & providedInterfaceInfo,
    const ::Ice::Current& current) const
{
    Logger->trace("TIServer", "<<<<< RECV: GetProvidedInterfaceInfo");

    return DeviceInterfaceServer->ReceiveGetProvidedInterfaceInfo(
        providedInterfaceName, providedInterfaceInfo);
}

bool mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::ConnectServerSide(
    const std::string & userTaskName, const std::string & requiredInterfaceName,
    const std::string & resourceTaskName, const std::string & providedInterfaceName,
    const ::Ice::Current&)
{
    Logger->trace("TIServer", "<<<<< RECV: ConnectServerSide");
    
    return DeviceInterfaceServer->ReceiveConnectServerSide(
        userTaskName, requiredInterfaceName, resourceTaskName, providedInterfaceName);
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::GetCommandId(
    const std::string & clientTaskProxyName,
    ::mtsDeviceInterfaceProxy::FunctionProxySet & functionProxies, const ::Ice::Current&) const
{
    Logger->trace("TIServer", "<<<<< RECV: GetCommandId");

    DeviceInterfaceServer->ReceiveGetCommandId(clientTaskProxyName, functionProxies);
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::ExecuteCommandVoid(
    ::Ice::Int commandID, const ::Ice::Current&)
{
    //Logger->trace("TIServer", "<<<<< RECV: ExecuteCommandVoid");

    DeviceInterfaceServer->ReceiveExecuteCommandVoid(commandID);
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::ExecuteCommandWriteSerialized(
    ::Ice::Int commandID, const ::std::string& argument, const ::Ice::Current&)
{
    //Logger->trace("TIServer", "<<<<< RECV: ExecuteCommandWriteSerialized");

    DeviceInterfaceServer->ReceiveExecuteCommandWriteSerialized(commandID, argument);
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::ExecuteCommandReadSerialized(
    ::Ice::Int commandID, ::std::string& argument, const ::Ice::Current&)
{
    //Logger->trace("TIServer", "<<<<< RECV: ExecuteCommandReadSerialized");

    DeviceInterfaceServer->ReceiveExecuteCommandReadSerialized(commandID, argument);
}

void mtsDeviceInterfaceProxyServer::DeviceInterfaceServerI::ExecuteCommandQualifiedReadSerialized(
    ::Ice::Int commandID, const ::std::string& argument1, ::std::string& argument2, const ::Ice::Current&)
{
    //Logger->trace("TIServer", "<<<<< RECV: ExecuteCommandQualifiedReadSerialized");

    DeviceInterfaceServer->ReceiveExecuteCommandQualifiedReadSerialized(commandID, argument1, argument2);
}
