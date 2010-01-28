/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/* $Id: clientMain.cpp 1087 2010-01-28 00:52:13Z mjung5 $ */

#include <cisstCommon.h>
#include <cisstOSAbstraction.h>
#include <cisstMultiTask.h>

#include "C1Task.h"
#include "C2ClientTask.h"

int main(int argc, char * argv[])
{
    // TODO: uncomment this
    //if (argc != 2) {
    //    std::cerr << "Usage: " << argv[0] << "[global component manager IP]" << std::endl;
    //    exit(-1);
    //}

    // Set global component manager IP
    //const std::string globalComponentManagerIP(argv[1]);
    const std::string globalComponentManagerIP("127.0.0.1");
    std::cout << "Global component manager IP is set as " << globalComponentManagerIP << std::endl;

    // log configuration
    cmnLogger::SetLoD(CMN_LOG_LOD_VERY_VERBOSE);
    cmnLogger::GetMultiplexer()->AddChannel(std::cout, CMN_LOG_LOD_VERY_VERBOSE);
    // add a log per thread
    osaThreadedLogFile threadedLog("P1");
    cmnLogger::GetMultiplexer()->AddChannel(threadedLog, CMN_LOG_LOD_VERY_VERBOSE);

    // Get the local component manager
    mtsManagerLocal * localManager = mtsManagerLocal::GetInstance("P1", globalComponentManagerIP);

    // create our server task
    const double PeriodClient = 10 * cmn_ms; // in milliseconds
    C1Task * C1 = new C1Task("C1", PeriodClient);
    C2ClientTask * C2 = new C2ClientTask("C2", PeriodClient);
    localManager->AddComponent(C1);
    localManager->AddComponent(C2);

    // Connect the tasks across networks
    if (!localManager->Connect("P1", "C1", "r1", "P2", "C2", "p1")) {
        CMN_LOG_INIT_ERROR << "Connect failed: P1:C1:r1-P2:C2:p1" << std::endl;
        return 1;
    }
    if (!localManager->Connect("P1", "C1", "r2", "P2", "C2", "p2")) {
        CMN_LOG_INIT_ERROR << "Connect failed: P1:C2:r1-P2:C2:p2" << std::endl;
        return 1;
    }
    if (!localManager->Connect("P1", "C2", "r1", "P2", "C2", "p2")) {
        CMN_LOG_INIT_ERROR << "Connect failed: P1:C2:r1-P2:C2:p2" << std::endl;
        return 1;
    }

    // create the tasks, i.e. find the commands
    localManager->CreateAll();
    // start the periodic Run
    localManager->StartAll();

    while (1) {
        osaSleep(10 * cmn_ms);
    }
    
    // cleanup
    localManager->KillAll();
    localManager->Cleanup();
    return 0;
}

/*
  Author(s):  Ankur Kapoor, Peter Kazanzides, Anton Deguet, Min Yang Jung
  Created on: 2004-04-30

  (C) Copyright 2004-2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/
