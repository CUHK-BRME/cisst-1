/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _vctForceTorque2DWidget_h
#define _vctForceTorque2DWidget_h

#include <cisstVector/vctQtForwardDeclarations.h>
#include <cisstVector/vctPlot2DBase.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>

#include <QWidget>

class QLabel;

// Always include last
#include <cisstVector/vctExportQt.h>

class CISST_EXPORT vctForceTorque2DWidget: public QWidget
{
    Q_OBJECT;

public:
    enum {
        Fx = 0,
        Fy = 1,
        Fz = 2,
        FNorm = 3,
        Fxyz = 4,
        Txyz = 5
    };
    vctForceTorque2DWidget(void);
    ~vctForceTorque2DWidget(){}

protected:
    virtual void closeEvent(QCloseEvent * event);

private:
    void setupUi(void);
    void SetupSensorPlot(void);

    vct3 Force;
    vct3 Torque;

    // vctQtWidgetDynamicVectorDoubleRead * QFTSensorValues;

    QLabel * UpperLimit;
    QLabel * LowerLimit;

//    QLineEdit * ErrorMsg;

    vctPlot2DOpenGLQtWidget * QFTPlot;
    vctPlot2DBase::Signal * ForceSignal[3];
    vctPlot2DBase::Signal * FNormSignal;
    vctPlot2DBase::Signal * TorqueSignal[3];

    vctPlot2DBase::Scale * ForceScale;
    vctPlot2DBase::Scale * TorqueScale;

    double InternalTime;
    int PlotIndex;

 public:
    inline void SetValue(const vct3 & force, const vct3 & torque) {
        this->InternalTime += 0.001;
        SetValue(InternalTime, force, torque);
    }
    void SetValue(const double & time, const vct3 & force, const vct3 & torque);

                   
private slots:
//    void SlotRebiasFTSensor(void);
     void SlotPlotIndex(int newAxis);
};

CMN_DECLARE_SERVICES_INSTANTIATION(vctForceTorque2DWidget);

#endif // _vctForceTorque2DWidget_h
