/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/* $Id: sineTask.cpp 188 2009-03-20 17:07:32Z mjung5 $ */

#include <cisstCommon/cmnConstants.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include "sineTask.h"

// required to implement the class services, see cisstCommon
CMN_IMPLEMENT_SERVICES(sineTask);

sineTask::sineTask(const std::string & taskName, mtsCollectorBase * dataCollector, double period):
    // base constructor, same task name and period.  Set the length of
    // state table to 5000
    mtsTaskPeriodic(taskName, period, false, dataCollector, 5000)
{
    // add SineData to the StateTable defined in mtsTask
    StateTable.AddData(SineData, "SineData1");
    StateTable.AddData(SineData, "SineData2");
    StateTable.AddData(SineData, "SineData3");
    StateTable.AddData(SineData, "SineData4");
    StateTable.AddData(SineData, "SineData5");
    StateTable.AddData(SineData, "SineData6");
    StateTable.AddData(SineData, "SineData7");
    StateTable.AddData(SineData, "SineData8");
    StateTable.AddData(SineData, "SineData9");
    StateTable.AddData(SineData, "SineData10");

    // add one interface, this will create an mtsTaskInterface
    mtsProvidedInterface *prov = AddProvidedInterface("MainInterface");
    if (prov) {
        // add command to access state table values to the interface
        prov->AddCommandReadState(StateTable, SineData, "GetData");
        prov->AddCommandReadHistory(StateTable, SineData, "GetDataHistory");
        // following should be done automatically
        prov->AddCommandRead(&mtsStateTable::GetIndexReader, &StateTable, "GetStateIndex");
        // add command to modify the sine amplitude 
        prov->AddCommandWrite(&sineTask::SetAmplitude, this, "SetAmplitude");
    }

    std::string fileName = "DataCollectionSRC_" + taskName + "_";
    std::string timeStamp; 
    osaGetDateTimeString(timeStamp);
    fileName += timeStamp;
    
    logFile.open(fileName.c_str(), std::ios::out);
}

void sineTask::Startup(void) {
    SineAmplitude = 1.0; // set the initial amplitude
}

void sineTask::Run(void) {
    // the state table provides an index
    const mtsStateIndex now = StateTable.GetIndexWriter();
    // process the commands received, i.e. possible SetSineAmplitude
    ProcessQueuedCommands();
    // compute the new values based on the current time and amplitude
    SineData = SineAmplitude
        * sin(2 * cmnPI * static_cast<double>(now.Ticks()) * Period / 10.0);
    
    static int i = 0;
    logFile << i++ << " " << SineData.ToString() << " " << std::endl;
}

void sineTask::Cleanup(void)
{
    logFile.close();
}
/*
  Author(s):  Ankur Kapoor, Peter Kazanzides, Anton Deguet
  Created on: 2004-04-30

  (C) Copyright 2004-2008 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/
