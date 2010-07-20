/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Balazs Vagvolgyi
  Created on: 2006

  (C) Copyright 2006-2007 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/


#ifndef _svlStreamManager_h
#define _svlStreamManager_h

#include <cisstVector/vctDynamicVector.h>
#include <cisstMultiTask/mtsComponent.h>

// Always include last!
#include <cisstStereoVision/svlExport.h>


// Forward declarations
class svlSyncPoint;
class svlFilterBase;
class svlFilterSourceBase;
class svlStreamProc;
class osaThread;
class osaCriticalSection;


class CISST_EXPORT svlStreamManager: public mtsComponent
{
friend class svlStreamProc;

public:
    svlStreamManager();
    svlStreamManager(unsigned int threadcount);
    ~svlStreamManager();

    int SetSourceFilter(svlFilterSourceBase* source);
    int Initialize(void);
    void Release(void);
    bool IsInitialized(void);
    inline void Start(void) {
        StartInternal();
    }
    int StartInternal(void);
    void Stop(void);
    bool IsRunning(void);
    int WaitForStop(double timeout = -1.0);
    int GetStreamStatus(void);

private:
    unsigned int ThreadCount;
    vctDynamicVector<svlStreamProc*> StreamProcInstance;
    vctDynamicVector<osaThread*> StreamProcThread;
    svlSyncPoint* SyncPoint;
    osaCriticalSection* CS;

    svlFilterSourceBase* StreamSource;
    bool Initialized;
    bool Running;
    bool StopThread;
    int StreamStatus;

    void InternalStop(unsigned int callingthreadID);
};

#endif // _svlStreamManager_h

