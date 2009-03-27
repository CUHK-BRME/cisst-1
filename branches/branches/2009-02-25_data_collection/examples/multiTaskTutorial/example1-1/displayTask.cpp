/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/* $Id: displayTask.cpp 188 2009-03-20 17:07:32Z mjung5 $ */

#include <math.h>
#include "displayTask.h"
#include "displayUI.h"

CMN_IMPLEMENT_SERVICES(displayTask);

displayTask::displayTask(const std::string & taskName, double period):
    mtsTaskPeriodic(taskName, period, false, 5000),
    ExitFlag(false),
    DataVec(10)
{
    // to communicate with the interface of the resource
    mtsRequiredInterface *req = AddRequiredInterface("DataGenerator");
    if (req) {
       req->AddFunction("GetData", Generator.GetData);
       req->AddFunction("GetStateIndex", Generator.GetStateIndex);
       req->AddFunction("GetDataHistory", Generator.GetDataHistory);
       req->AddFunction("SetAmplitude", Generator.SetAmplitude);
    }
}

void displayTask::Configure(const std::string & CMN_UNUSED(filename))
{
    // define some values, ideally these come from a configuration
    // file and then configure the user interface
    double maxValue = 0.5; double minValue = 5.0;
    StartValue =  1.0;
    CMN_LOG_CLASS(3) << "Configure: setting bounds to: "
                     << minValue << ", " << maxValue << std::endl;
    CMN_LOG_CLASS(3) << "Configure: setting start value to: "
                     << StartValue << std::endl;
    UI.Amplitude->bounds(minValue, maxValue);
    UI.Amplitude->value(StartValue);
    AmplitudeData = StartValue;
}

void displayTask::Startup(void) 
{
    const cmnGenericObject *obj = Generator.GetDataHistory.GetCommand()->GetArgument2Prototype();
    CMN_LOG_CLASS(1) << "GetHistory prototype = " << obj->Services()->GetName() << std::endl;
#if 0
    // Future plans:  use mtsHistoryBase instead of mtsVector (equivalently, could use mtsVectorBase)
    cmnGenericObject *newObj = obj->Services()->Create();
    mtsHistoryBase *newObjDerived = dynamic_cast<mtsHistoryBase *>(newObj);
    newObjDerived->SetSize(10);
#endif

    // set the initial amplitude based on the configuration
    AmplitudeData = StartValue;

    // make the UI visible
    UI.show(0, NULL);
}

void displayTask::Run(void)
{
    // get the current time index to display it in the UI
    const mtsStateIndex now = StateTable.GetIndexWriter();
    // get the data from the sine wave generator task
    Generator.GetStateIndex(StateIndex);
    Generator.GetData(Data);
    Generator.GetDataHistory(StateIndex, DataVec);
    UI.Data->value(Data.Data);
    // check if the user has entered a new amplitude in UI
    if (UI.AmplitudeChanged) {
        // retrieve the new amplitude and send it to the sine task
        AmplitudeData.Data = UI.Amplitude->value();
        Generator.SetAmplitude(AmplitudeData);
        UI.AmplitudeChanged = false;
        CMN_LOG_CLASS(7) << "Run: " << now.Ticks()
                         << " - Amplitude: " << AmplitudeData.Data << std::endl;
    }
    // log some extra information
    CMN_LOG_CLASS(7) << "Run : " << now.Ticks()
                     << " - Data: " << Data << std::endl;
    // Following displays the history (last 10 values)
    //CMN_LOG_CLASS(7) << "Last 10: " << DataVec << std::endl;
    // update the UI, process UI events 
    if (Fl::check() == 0) {
        ExitFlag = true;
    }
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
