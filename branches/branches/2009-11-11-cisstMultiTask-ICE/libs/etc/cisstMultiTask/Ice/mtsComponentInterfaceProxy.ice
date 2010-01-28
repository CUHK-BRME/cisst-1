/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsComponentInterfaceProxy.ice 2009-03-16 mjung5 $
  
  Author(s):  Min Yang Jung
  Created on: 2010-01-12
  
  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

//
// This Slice file defines the communication between a provided interface
// and a required interfaces. 
// A provided interfaces act as a server while a required interface does 
// as a client.
//

#ifndef _mtsComponentInterfaceProxy_ICE_h
#define _mtsComponentInterfaceProxy_ICE_h

#include <Ice/Identity.ice>

module mtsComponentInterfaceProxy
{
	//-----------------------------------------------------------------------------
	//	Command and Event Object Definition
	//-----------------------------------------------------------------------------
	struct CommandVoidInfo {
		string Name;
	};
	
	struct CommandWriteInfo { 
		string Name;
        string ArgumentPrototypeSerialized;
	};
	
	struct CommandReadInfo { 
		string Name;
        string ArgumentPrototypeSerialized;
	};
	
	struct CommandQualifiedReadInfo { 
		string Name;
        string Argument1PrototypeSerialized;
        string Argument2PrototypeSerialized;
	};
	
	struct EventVoidInfo { 
		string Name;
	};
	
	struct EventWriteInfo {
		string Name;
        string ArgumentPrototypeSerialized;
	};

	sequence<CommandVoidInfo>          CommandVoidSequence;
	sequence<CommandWriteInfo>         CommandWriteSequence;
	sequence<CommandReadInfo>          CommandReadSequence;
	sequence<CommandQualifiedReadInfo> CommandQualifiedReadSequence;
    sequence<EventVoidInfo>            EventVoidSequence;
    sequence<EventWriteInfo>           EventWriteSequence;

    //-----------------------------------------------------------------------------
	//	Provided Interface Related Definition
	//-----------------------------------------------------------------------------	
	// Data structure definition
	struct ProvidedInterfaceInfo {
		// Interface name
		string InterfaceName;
		
		// Commands
		CommandVoidSequence          CommandsVoid;
		CommandWriteSequence         CommandsWrite;
		CommandReadSequence          CommandsRead;
		CommandQualifiedReadSequence CommandsQualifiedRead;
        
        // Events
		EventVoidSequence            EventsVoid;
		EventWriteSequence           EventsWrite;
	};

    //-----------------------------------------------------------------------------
	//	Function Proxy Related Definition
	//-----------------------------------------------------------------------------	
    
    //
    // TODO: Change the name of FunctionProxyInfo => ProxyElementInfo (?)
    //
    // The information about the function proxies.
    struct FunctionProxyInfo {
        string Name;
        // This id is set as a pointer to a function proxy at server side.
        // Note that type 'long' in slice is converted to ::Ice::Long which can
        // handle 64-bit numbers.
        long FunctionProxyId;
    };

    sequence<FunctionProxyInfo> FunctionProxySequence;

    struct FunctionProxyPointerSet {
        // A name of the server task proxy. This is used as a key to find a server 
        // task proxy at client side.
        string ServerTaskProxyName;
        
        // A name of the provided interface proxy that has command proxies at client side.
        string ProvidedInterfaceProxyName;

        // Set of pointers to the function proxies.
        FunctionProxySequence FunctionVoidProxies;
        FunctionProxySequence FunctionWriteProxies;
        FunctionProxySequence FunctionReadProxies;
        FunctionProxySequence FunctionQualifiedReadProxies;
    };

    //
    //  Definitions for Event Generator Proxy Pointers
    //
    struct EventGeneratorProxyElement {
        string Name;
        // This ID is set as a pointer to a event generator proxy poninter at client side.
        // Note that type 'long' in slice is converted to ::Ice::Long which can
        // handle 64-bit numbers.
        long EventGeneratorProxyId;
    };
    sequence<EventGeneratorProxyElement> EventGeneratorProxySequence;

    struct EventGeneratorProxyPointerSet {
        EventGeneratorProxySequence EventGeneratorVoidProxies;
        EventGeneratorProxySequence EventGeneratorWriteProxies;
    };

	//-----------------------------------------------------------------------------
	// Interface for Required Interface (Proxy Client)
	//-----------------------------------------------------------------------------
	interface ComponentInterfaceClient
	{
        /*! Methods for testing and unit tests */
        void TestMessageFromServerToClient(string str);

        /*! Fetch function proxy pointers from a required interface proxy at 
            server side. */
        ["cpp:const"] idempotent
        bool FetchFunctionProxyPointers(string requiredInterfaceName, out FunctionProxyPointerSet functionProxyPointers);

        /*! Execute command objects across a network. In these APIs, Slice's 
            'long' type is used instead of 'unsigned int' because Slice does 
            not support unsigned type.
		    See http://zeroc.com/doc/Ice-3.3.1/manual/Slice.5.8.html for details.
            Also see http://www.zeroc.com/doc/Ice-3.3.1/manual/Cpp.7.6.html for
		    mapping for simple built-in types. */
		void ExecuteCommandVoid(long commandID);
        void ExecuteCommandWriteSerialized(long commandID, string argument);
        void ExecuteCommandReadSerialized(long commandID, out string argument);
        void ExecuteCommandQualifiedReadSerialized(long commandID, string argumentIn, out string argumentOut);
	};

	//-----------------------------------------------------------------------------
	// Interface for Provided Interface (Proxy Server)
	//-----------------------------------------------------------------------------
	interface ComponentInterfaceServer
	{
        /*! Methods for testing and unit tests */
        void TestMessageFromClientToServer(string str);

        //
        //  Connection Management
        //
		/*! Called by a proxy client when it connects to a proxy server */
		bool AddClient(string connectingProxyName, int providedInterfaceProxyInstanceId, Ice::Identity ident);

        /*! This is called by a client when it terminates. This allows a server to
            shutdown (or close) safely and cleanly. */
        void Shutdown();

        //
        //  Interface Interaction
        //
        /*! Fetch pointers of event generator proxies from a provided interface
            proxy at server side. */
        ["cpp:const"] idempotent
        bool FetchEventGeneratorProxyPointers(
            string clientComponentName, string requiredInterfaceName,
            out EventGeneratorProxyPointerSet eventGeneratorProxyPointers);

        void ExecuteEventVoid(long CommandID);
        void ExecuteEventWriteSerialized(long CommandID, string argument);
	};
};

#endif // _mtsComponentInterfaceProxy_ICE_h
