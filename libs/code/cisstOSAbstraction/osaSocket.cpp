/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
$Id$

Author(s):  Peter Kazanzides
Created on: 2009

(C) Copyright 2007-2009 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstOSAbstraction/osaSocket.h>

#if (CISST_OS == CISST_WINDOWS)
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#define WINSOCKVERSION MAKEWORD(2,2)
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>  // for memset
#if (CISST_OS == CISST_QNX)
#include <sys/select.h>
#endif
#endif


struct osaSocketInternals {
    struct sockaddr_in ServerAddr;
};

#define SERVER_ADDR (reinterpret_cast<struct osaSocketInternals *>(Internals)->ServerAddr)


unsigned int osaSocket::SizeOfInternals(void)
{
    return sizeof(osaSocketInternals);
}


osaSocket::osaSocket(SocketTypes type)
:
Connected(false)
#ifdef OSA_SOCKET_WITH_STREAM
,std::iostream(&Streambuf),
Streambuf(this)
#endif // OSA_SOCKET_WITH_STREAM
{
    CMN_ASSERT(sizeof(Internals) >= SizeOfInternals());
    memset(&SERVER_ADDR, 0, sizeof(SERVER_ADDR));

#if (CISST_OS == CISST_WINDOWS)
    WSADATA wsaData;
    int retval = WSAStartup(WINSOCKVERSION, &wsaData);
    if (retval != 0) {
        CMN_LOG_CLASS_RUN_ERROR << "osaSocket: WSAStartup() failed with error code " << retval << std::endl;
        return;
    }
#endif

    SocketType = type;
    SocketFD = socket(PF_INET, (type == UDP) ? SOCK_DGRAM : SOCK_STREAM, 0);
    if (SocketFD == INVALID_SOCKET) {
        CMN_LOG_CLASS_RUN_ERROR << "osaSocket: failed to create a socket" << std::endl;
    }
    CMN_LOG_CLASS_RUN_VERBOSE << "osaSocket: created socket " << SocketFD << std::endl;
}


osaSocket::osaSocket(void * socketFDPtr)
:
Connected(false)
#ifdef OSA_SOCKET_WITH_STREAM
,std::iostream(&Streambuf),
Streambuf(this)
#endif // OSA_SOCKET_WITH_STREAM
{


#if (CISST_OS == CISST_WINDOWS)
    WSADATA wsaData;
    int retval = WSAStartup(WINSOCKVERSION, &wsaData);
    if (retval != 0) {
        CMN_LOG_CLASS_RUN_ERROR << "osaSocket: WSAStartup() failed with error code " << retval << std::endl;
        return;
    }
#endif

    SocketType = TCP;
    SocketFD = *(reinterpret_cast<SOCKET *> (socketFDPtr));
    if (SocketFD == INVALID_SOCKET) {
        CMN_LOG_CLASS_RUN_ERROR << "osaSocket: failed to create a socket" << std::endl;
    }
    CMN_LOG_CLASS_RUN_VERBOSE << "osaSocket: created socket " << SocketFD << std::endl;
}


osaSocket::~osaSocket(void)
{
    Close();
#if (CISST_OS == CISST_WINDOWS)
    WSACleanup();
#endif
}


std::string osaSocket::GetLocalhostIP(void)
{
#if (CISST_OS == CISST_WINDOWS)
    WSADATA wsaData;
    int retval = WSAStartup(WINSOCKVERSION, &wsaData);
    if (retval != 0) {
        CMN_LOG_RUN_ERROR << "osaSocket: WSAStartup() failed with error code " << retval << std::endl;
        return 0;
    }
#endif

    char hostname[256] = { 0 };
    gethostname(hostname, 255);
    CMN_LOG_RUN_VERBOSE << "GetLocalhostIP: hostname is " << hostname << std::endl;

    struct hostent * he = gethostbyname(hostname);
    if (!he) {
        CMN_LOG_RUN_ERROR << "GetLocalhostIP: invalid host" << std::endl;
#if (CISST_OS == CISST_WINDOWS)
        WSACleanup();
#endif
        return "";
    }
    struct in_addr localAddr;
    memcpy(&localAddr, he->h_addr_list[0], sizeof(struct in_addr));

#if (CISST_OS == CISST_WINDOWS)
    WSACleanup();
#endif
    return inet_ntoa(localAddr);
}

int osaSocket::GetLocalhostIP(std::vector<std::string> & IPaddress)
{    
#if (CISST_OS == CISST_WINDOWS)
    WSADATA wsaData;
    int retval = WSAStartup(WINSOCKVERSION, &wsaData);
    if (retval != 0) {
        CMN_LOG_RUN_ERROR << "osaSocket: WSAStartup() failed with error code " << retval << std::endl;
        return 0;
    }
#endif

    char hostname[256] = { 0 };
    if (gethostname(hostname, 255) != 0) {
        CMN_LOG_RUN_ERROR << "osaSocket: failed to get host name" << std::endl;
#if (CISST_OS == CISST_WINDOWS)
        WSACleanup();
#endif
        return 0;
    }

    CMN_LOG_RUN_VERBOSE << "GetLocalhostIP: hostname is " << hostname << std::endl;

    struct hostent * he = gethostbyname(hostname);
    if (!he) {
        CMN_LOG_RUN_ERROR << "GetLocalhostIP: invalid host" << std::endl;
#if (CISST_OS == CISST_WINDOWS)
        WSACleanup();
#endif
        return 0;
    }

    int i;
    struct in_addr localAddr;
    std::string s;
    for (i = 0; he->h_addr_list[i] != 0; ++i) {
        memcpy(&localAddr, he->h_addr_list[i], sizeof(struct in_addr));
        s = inet_ntoa(localAddr);        
        IPaddress.push_back(s);
        CMN_LOG_RUN_VERBOSE << "Localhost IP (" << i << ") : " << s << std::endl;
    }

#if (CISST_OS == CISST_WINDOWS)
    WSACleanup();
#endif

    return i;
}


bool osaSocket::AssignPort(unsigned short port)
{
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    int retval = bind(SocketFD, reinterpret_cast<struct sockaddr *>(&serverAddr), sizeof(serverAddr));
    if (retval == SOCKET_ERROR) {
        CMN_LOG_CLASS_RUN_ERROR << "AssignPort: failed to bind socket" << std::endl;
        Close();
        return false;
    }
    return true;
}


void osaSocket::SetDestination(const std::string & host, unsigned short port)
{
    memset(&SERVER_ADDR, 0, sizeof(SERVER_ADDR));
    SERVER_ADDR.sin_family = AF_INET;
    SERVER_ADDR.sin_port = htons(port);
    SERVER_ADDR.sin_addr.s_addr = GetIP(host);

    CMN_LOG_CLASS_INIT_VERBOSE << "SetDestination: destination set to "
        << host << ":" << port << std::endl;
}


bool osaSocket::Connect(void)
{
    if (SocketType == UDP) {
        CMN_LOG_CLASS_RUN_ERROR << "osaSocket: Connect is not allowed with UDP type sockets"<< std::endl;
        return false;      
    }

    if (Connected){
        Close();
    }

    //create a new one just in case we closed it.
    if (SocketFD == INVALID_SOCKET ) {   
        SocketFD = socket(PF_INET, SOCK_STREAM, 0);
        if (SocketFD == INVALID_SOCKET) 
            CMN_LOG_CLASS_RUN_ERROR << "osaSocket: failed to create a socket" << std::endl;
        else 
            CMN_LOG_CLASS_RUN_VERBOSE << "osaSocket: created socket " << SocketFD << std::endl;
    }

    int retval = connect(SocketFD, reinterpret_cast<struct sockaddr *>(&SERVER_ADDR), sizeof(SERVER_ADDR));
    if (retval == SOCKET_ERROR) {
        Connected=false;
        CMN_LOG_CLASS_RUN_ERROR << "Connect: failed to connect" << std::endl;
        return false;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Connect: connection established" << std::endl;
    Connected = true;
    return true;
}


bool osaSocket::Connect(const std::string & host, unsigned short port)
{
    SetDestination(host, port);
    return Connect();
}


int osaSocket::Send(const char * bufsend, unsigned int msglen, const double timeoutSec )
{
   
    //TCP Socket
    if (SocketType == TCP && !Connected) {
        CMN_LOG_CLASS_RUN_WARNING << "Send: Not Connected " << std::endl;
        return -1;
    }

    int retval = 0;
    int err    = 0;

    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(SocketFD, &writefds);

    long sec = static_cast<long>(floor(timeoutSec));
    long usec = static_cast<long>((timeoutSec - sec) * 1e6);
    timeval timeout = { sec, usec };

    //see if the socket is available for writing
    //timeout is useful here if lots of data is to be sent.
    retval = select(SocketFD + 1, &writefds, NULL, NULL, &timeout);

    if (retval == SOCKET_ERROR) {

#if (CISST_OS == CISST_WINDOWS)
        err = WSAGetLastError();
#else
        err = errno;
#endif
        //! \Todo : some of the errors here might be soft errors, EAGAIN/EWOULDBLOCK and we might be able to recover from those.
        CMN_LOG_CLASS_RUN_ERROR << "Send: failed to send because socket is not ready " << SocketFD << " Error: " <<err<<std::endl;

        if (SocketType == TCP)
            Close();
        return -1;
    }

    //UDP
    if (SocketType == UDP) {
        socklen_t length = sizeof(SERVER_ADDR);
        retval = sendto(SocketFD, bufsend, msglen, 0, reinterpret_cast<struct sockaddr *>(&SERVER_ADDR), length);

        if (retval == SOCKET_ERROR) {
            CMN_LOG_CLASS_RUN_ERROR << "Send: failed to send" << std::endl;
            return -1;
        } 
        else if (retval != static_cast<int>(msglen)) {
            CMN_LOG_CLASS_RUN_WARNING << "Send: failed to send the whole message" << std::endl;
        }   

        CMN_LOG_CLASS_RUN_DEBUG << "Send: sent " << retval << " bytes" << std::endl;

        return retval;
    }

    //TCP
    if (!Connected) {
        CMN_LOG_CLASS_RUN_DEBUG << "Send: Not Connected " << std::endl;
        return -1;
    }

    //no errors so send 
    retval = send(SocketFD, bufsend, msglen, 0);

    if (retval == SOCKET_ERROR) {
        
        //! \Todo : some of the errors here might be soft errors, EAGAIN/EWOULDBLOCK and we might be able to recover from those. This was an attempt to test it.
        //This error is returned from operations on nonblocking sockets that cannot be completed immediately,
        //for example recv when no data is queued to be read from the socket. It is a nonfatal error, and the 
        //operation should be retried later. It is normal for WSAEWOULDBLOCK to be reported as the result from
        //calling connect on a nonblocking SOCK_STREAM socket, since some time must elapse for the connection to be established.

#if (CISST_OS == CISST_WINDOWS)
        err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK) {
            CMN_LOG_CLASS_RUN_WARNING << "Send: failed to send the whole message, missing " <<msglen<< " bytes"<<std::endl;
            return 0;
        }
#else
        err = errno;
        if (err == EWOULDBLOCK) {
            CMN_LOG_CLASS_RUN_WARNING << "Send: failed to send the whole message, missing " <<msglen<< " bytes"<<std::endl;
            return 0;
        }
#endif
        //other error:
        CMN_LOG_CLASS_RUN_ERROR << "Send: failed to send with Error: " <<err<<std::endl;
        Close();
        return -1;
    } 
    else if (retval != static_cast<int>(msglen)) {
        CMN_LOG_CLASS_RUN_WARNING << "Send: failed to send the whole message, missing " <<msglen - retval<< " bytes"<<std::endl;
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Send: sent " << retval << " bytes" << std::endl;
    return retval;
}


int osaSocket::Receive(char * bufrecv, unsigned int maxlen, const double timeoutSec )
{
    
    //TCP Socket
    if (SocketType == TCP && !Connected) {
        CMN_LOG_CLASS_RUN_WARNING << "Receive: Not Connected " << std::endl;
        return -1;
    }

    int retval = 0;
    int err    = 0;
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(SocketFD, &readfds);

    long second = static_cast<long>(floor (timeoutSec));
    long usec = static_cast<long>(floor ( (timeoutSec - second) *1e6));
    timeval timeout = { second , usec };

    /* Notes for QNX from the QNX library reference (Min)
    *
    * The select() function is thread safe as long as the fd sets
    * used by each thread point to memory that is specific to that thread.
    *
    * In Neutrino, if multiple threads block in select() on the same
    * fd for the same condition, all threads may unblock when the
    * condition is satisfied. This may differ from other implementations
    * where only one thread may unblock.
    */

    retval = select(SocketFD + 1, &readfds, NULL, NULL, &timeout);

    if (retval == SOCKET_ERROR) {

#if (CISST_OS == CISST_WINDOWS)
        err = WSAGetLastError();
#else
        err = errno;
#endif
        //! \Todo : some of the errors here might be soft errors, EAGAIN/EWOULDBLOCK and we might be able to recover from those.
        CMN_LOG_CLASS_RUN_ERROR << "Receive: failed to receive because socket is not ready " << SocketFD << " Error: " <<err<<std::endl;

        if (SocketType == TCP)
            Close();
        return -1;
    }


    //Manage UDP Sockets first:
    if (SocketType == UDP) {

        if (retval > 0) {
            struct sockaddr_in fromAddr;

            socklen_t length = sizeof(fromAddr);
            retval = recvfrom(SocketFD, bufrecv, maxlen, 0, reinterpret_cast<struct sockaddr *>(&fromAddr), &length);

            if (retval > 0) {
                if (static_cast<unsigned int>(retval) < maxlen - 1) {
                    bufrecv[retval] = 0;  // NULL terminate the string for convenience if there is room
                    CMN_LOG_CLASS_RUN_DEBUG << "Receive: received " << retval << " bytes: " << std::endl;
                } 
                else { 
                    CMN_LOG_CLASS_RUN_DEBUG << "Receive: received " << retval << " bytes: " << std::endl;
                }
                if (SERVER_ADDR.sin_addr.s_addr == 0) {
                    CMN_LOG_CLASS_RUN_VERBOSE << "Receive: setting destination address to "
                        << inet_ntoa(fromAddr.sin_addr) << ":" << ntohs(fromAddr.sin_port) << std::endl;
                    SERVER_ADDR = fromAddr;
                } else if ((SERVER_ADDR.sin_addr.s_addr != fromAddr.sin_addr.s_addr) ||
                    (SERVER_ADDR.sin_port != fromAddr.sin_port)) {
                        CMN_LOG_CLASS_RUN_VERBOSE << "Receive: updating destination from "
                            << inet_ntoa(SERVER_ADDR.sin_addr) << ":" << ntohs(SERVER_ADDR.sin_port)
                            << " to "
                            << inet_ntoa(fromAddr.sin_addr) << ":" << ntohs(fromAddr.sin_port) << std::endl;
                        SERVER_ADDR = fromAddr;
                }
            }
        }
        return retval;
    }

 

    if (retval > 0) {
        retval = recv(SocketFD, bufrecv, maxlen, 0);
        if (retval > 0) {
            if (static_cast<unsigned int>(retval) < maxlen - 1) {
                bufrecv[retval] = 0;  // NULL terminate the string for convenience if there is room
                CMN_LOG_CLASS_RUN_DEBUG << "Receive: received " << retval << " bytes: " << bufrecv << std::endl;
            } 
            else { 
                // Should not print the whole msg, because it might not be null terminated.
                CMN_LOG_CLASS_RUN_DEBUG << "Receive: received " << retval << " bytes " << std::endl;
            }
        }
    } else if (retval == SOCKET_ERROR) {
        Close();
        CMN_LOG_CLASS_RUN_ERROR << "Receive: failed to receive" << std::endl;
    }
    return retval;
}

//! This could be static or external to the osaSocket class
unsigned long osaSocket::GetIP(const std::string & host) const
{
    hostent * he = gethostbyname(host.c_str());
    if (he) {
        if (he->h_addr_list && he->h_addr_list[0]) {
            return *reinterpret_cast<unsigned long *>(he->h_addr_list[0]);
        }
    }
    CMN_LOG_CLASS_RUN_ERROR << "GetIP: invalid host " << host << std::endl;
    return 0;
}

bool osaSocket::Close(void)
{
    if (SocketFD != INVALID_SOCKET) {
        int retval = 0;

#if (CISST_OS == CISST_WINDOWS)
        retval = closesocket(SocketFD);
        if (retval != 0) {
            CMN_LOG_CLASS_RUN_ERROR << "Close: failed to close socket " << SocketFD << " Error: " <<WSAGetLastError()<<std::endl;
            return false;
        }
        WSACleanup();
#else
        retval = close(SocketFD);
        if (retval != 0) {
            CMN_LOG_CLASS_RUN_ERROR << "Close: failed to close socket " << SocketFD << "Error: " <<errno<< std::endl;
            return false;
        }
#endif
        CMN_LOG_CLASS_RUN_ERROR<< "Close: closed socket " << SocketFD << std::endl;
        SocketFD = INVALID_SOCKET;
    }
    Connected = false;
    return true;
}


/*! \ brief Connection state (only works for TCP)
\return Returns true if the socket thinks it is connected */
bool osaSocket::IsConnected(void) { 

    if (SocketType == UDP) {
        CMN_LOG_CLASS_RUN_WARNING<< "IsConnected: Not implemented for UDP packets"<< std::endl;
        return false;
    }

    if (!Connected){
        return false;
    }

    //see if socket is ready to be used?
    int retval = 0;
    int err    = 0;

    fd_set readfds;
    fd_set writefds;
    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    FD_SET(SocketFD, &readfds);
    FD_SET(SocketFD, &writefds);

    long second = 0;
    long usec = 0;
    timeval timeout = { second , usec };

    retval = select(SocketFD + 1, &readfds, &writefds, NULL, &timeout);

    if (retval == SOCKET_ERROR) {

#if (CISST_OS == CISST_WINDOWS)
        err = WSAGetLastError();
        //! \Todo : some of the errors here might be soft errors, EAGAIN/EWOULDBLOCK and we might be able to recover from those.
#else
        err = errno;
#endif
        CMN_LOG_CLASS_RUN_DEBUG << "IsConnected: check failed on socket " << SocketFD << " Error: " <<err<<std::endl;
        Close();
        return false;
    }

    //Send a dummy message to see if the socket is alive?
    retval = send(SocketFD, 0, 0, 0);
    
    if (retval == SOCKET_ERROR) {
#if (CISST_OS == CISST_WINDOWS)
        err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK) {
            return true;
        }
        //! \Todo : some of the errors here might be soft errors, EAGAIN/EWOULDBLOCK and we might be able to recover from those.
#else
        err = errno;
        if (err == EWOULDBLOCK) {
            return true;
        }

#endif
        CMN_LOG_CLASS_RUN_DEBUG << "IsConnected: check failed on socket " << SocketFD << " Error: " <<err<<std::endl;
        Close();
        return false;
    }
    return true;
}
