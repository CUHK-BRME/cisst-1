/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Anton Deguet, Balazs Vagvolgyi
  Created on: 2009-03-26

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/


#include <cisstCommon/cmnPortability.h>
#include <cisstCommon/cmnAssert.h>
#include <cisstCommon/cmnGetChar.h>

#if (CISST_OS == CISST_LINUX) || (CISST_OS == CISST_DARWIN) || (CISST_OS == CISST_SOLARIS) || (CISST_OS == CISST_LINUX_RTAI)
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif // CISST_LINUX || CISST_DARWIN || CISST_SOLARIS || CISST_RTAI

#if (CISST_OS == CISST_WINDOWS)
#include <conio.h>
#endif // CISST_WINDOWS

struct cmnGetCharEnvironmentInternals {
#if (CISST_OS == CISST_LINUX) || (CISST_OS == CISST_DARWIN) || (CISST_OS == CISST_SOLARIS) || (CISST_OS == CISST_LINUX_RTAI)
    struct  termios OldSettings;
    struct  termios NewSettings;
    int     Keyboard;
#endif // CISST_LINUX || CISST_DARWIN || CISST_SOLARIS || CISST_RTAI
};


#define INTERNALS(A) (reinterpret_cast<cmnGetCharEnvironmentInternals*>(Internals)->A)


cmnGetCharEnvironment::cmnGetCharEnvironment(void):
    Activated(false)
{
    CMN_ASSERT(sizeof(Internals) >= SizeOfInternals());
}


cmnGetCharEnvironment::~cmnGetCharEnvironment(void)
{
    if (this->Activated) {
        this->DeActivate();
    }
}


unsigned int cmnGetCharEnvironment::SizeOfInternals(void) {
    return sizeof(cmnGetCharEnvironmentInternals);
}


#if (CISST_OS == CISST_LINUX) || (CISST_OS == CISST_SOLARIS) || (CISST_OS == CISST_LINUX_RTAI)
bool cmnGetCharEnvironment::Activate(void)
{
    if (!this->Activated) {
        INTERNALS(Keyboard) = open("/dev/tty",O_RDWR);
        ioctl(INTERNALS(Keyboard), TCGETS, &INTERNALS(OldSettings));
        INTERNALS(NewSettings) = INTERNALS(OldSettings);
        INTERNALS(NewSettings).c_lflag &= !ICANON;
        INTERNALS(NewSettings).c_lflag &= !ECHO;
        ioctl(INTERNALS(Keyboard), TCSETS, &INTERNALS(NewSettings));
        this->Activated = true;
        return true;
    }
    return false;
}
#endif // CISST_LINUX || CISST_SOLARIS || CISST_RTAI

#if (CISST_OS == CISST_DARWIN)
bool cmnGetCharEnvironment::Activate(void)
{
    if (!this->Activated) {
        INTERNALS(Keyboard) = open("/dev/tty",O_RDWR);
        ioctl(INTERNALS(Keyboard), TIOCGETA, &INTERNALS(OldSettings));
        INTERNALS(NewSettings) = INTERNALS(OldSettings);
        INTERNALS(NewSettings).c_lflag &= !ICANON;
        INTERNALS(NewSettings).c_lflag &= !ECHO;
        ioctl(INTERNALS(Keyboard), TIOCSETA, &INTERNALS(NewSettings));
        this->Activated = true;
        return true;
    }
    return false;
}
#endif // CISST_DARWIN

#if (CISST_OS == CISST_WINDOWS)
bool cmnGetCharEnvironment::Activate(void)
{
    if (!this->Activated) {
        this->Activated = true;
        return true;
    }
    return false;
}
#endif // CISST_WINDOWS


#if (CISST_OS == CISST_LINUX) || (CISST_OS == CISST_SOLARIS) || (CISST_OS == CISST_LINUX_RTAI)
bool cmnGetCharEnvironment::DeActivate(void)
{
    if (this->Activated) {
        ioctl(INTERNALS(Keyboard), TCSETS, &INTERNALS(OldSettings));
        close(INTERNALS(Keyboard));
        this->Activated = false;
        return true;
    }
    return false;
}
#endif // CISST_LINUX || CISST_SOLARIS || CISST_RTAI

#if (CISST_OS == CISST_DARWIN)
bool cmnGetCharEnvironment::DeActivate(void)
{
    if (this->Activated) {
        ioctl(INTERNALS(Keyboard), TIOCSETA, &INTERNALS(OldSettings));
        close(INTERNALS(Keyboard));
        this->Activated = false;
        return true;
    }
    return false;
}
#endif // CISST_DARWIN

#if (CISST_OS == CISST_WINDOWS)
bool cmnGetCharEnvironment::DeActivate(void)
{
    if (this->Activated) {
        this->Activated = false;
        return true;
    }
    return false;
}
#endif // CISST_WINDOWS


#if (CISST_OS == CISST_LINUX) || (CISST_OS == CISST_DARWIN) || (CISST_OS == CISST_SOLARIS) || (CISST_OS == CISST_LINUX_RTAI)
int cmnGetCharEnvironment::GetChar(void)
{
    if (this->Activated) {
        return getchar();
    }
    return 0;
}
#endif // CISST_LINUX || CISST_DARWIN ||CISST_SOLARIS || CISST_RTAI

#if (CISST_OS == CISST_WINDOWS)
int cmnGetCharEnvironment::GetChar(void)
{
    if (this->Activated) {
        return _getch();
    }
    return 0;
}
#endif // CISST_WINDOWS


int cmnGetChar(void)
{
    cmnGetCharEnvironment environment;
    environment.Activate();
    int result;
    result = environment.GetChar();
    environment.DeActivate();
    return result;
}