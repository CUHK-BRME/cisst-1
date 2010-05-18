/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$
  
  Author(s):  Balazs Vagvolgyi
  Created on: 2007 

  (C) Copyright 2006-2007 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <cisstStereoVision/svlFilterStereoImageJoiner.h>


/******************************************/
/*** svlFilterStereoImageJoiner class *****/
/******************************************/

CMN_IMPLEMENT_SERVICES(svlFilterStereoImageJoiner)

svlFilterStereoImageJoiner::svlFilterStereoImageJoiner() :
    svlFilterBase(),
    cmnGenericObject(),
    OutputImage(0),
    Layout(svlLayoutSideBySide)
{
    AddInput("input", true);
    AddInputType("input", svlTypeImageRGBStereo);
    AddInputType("input", svlTypeImageMono8Stereo);
    AddInputType("input", svlTypeImageMono16Stereo);

    AddOutput("output", true);
    SetAutomaticOutputType(false);
}

svlFilterStereoImageJoiner::~svlFilterStereoImageJoiner()
{
    Release();
}

int svlFilterStereoImageJoiner::UpdateTypes(svlFilterInput &input, svlStreamType type)
{
    // Check if type is on the supported list
    if (!input.IsTypeSupported(type)) return SVL_FAIL;

    if      (type == svlTypeImageRGBStereo)    GetOutput()->SetType(svlTypeImageRGB);
    else if (type == svlTypeImageMono8Stereo)  GetOutput()->SetType(svlTypeImageMono8);
    else if (type == svlTypeImageMono16Stereo) GetOutput()->SetType(svlTypeImageMono16);
    else return SVL_FAIL;

    return SVL_OK;
}

int svlFilterStereoImageJoiner::Initialize(svlSample* syncInput, svlSample* &syncOutput)
{
    svlSampleImage* input = dynamic_cast<svlSampleImage*>(syncInput);
    unsigned int width = input->GetWidth(SVL_LEFT);
    unsigned int height = input->GetHeight(SVL_LEFT);

    if (input->GetWidth(SVL_RIGHT)  != width ||
        input->GetHeight(SVL_RIGHT) != height) return SVL_FAIL;

    Release();

    switch (Layout) {
        case svlLayoutInterlaced:
        case svlLayoutInterlacedRL:
            height += input->GetHeight(SVL_RIGHT);
        break;

        case svlLayoutSideBySide:
        case svlLayoutSideBySideRL:
            width += input->GetWidth(SVL_RIGHT);
        break;

        default:
            return SVL_FAIL;
    }

    if      (GetInput()->GetType() == svlTypeImageRGBStereo)    OutputImage = new svlSampleImageRGB;
    else if (GetInput()->GetType() == svlTypeImageMono8Stereo)  OutputImage = new svlSampleImageMono8;
    else if (GetInput()->GetType() == svlTypeImageMono16Stereo) OutputImage = new svlSampleImageMono16;
    else return SVL_FAIL;
    OutputImage->SetSize(width, height);

    syncOutput = OutputImage;

    return SVL_OK;
}

int svlFilterStereoImageJoiner::Process(svlProcInfo* procInfo, svlSample* syncInput, svlSample* &syncOutput)
{
    syncOutput = OutputImage;
    _SkipIfAlreadyProcessed(syncInput, syncOutput);

    _OnSingleThread(procInfo)
    {
        svlSampleImage* id = dynamic_cast<svlSampleImage*>(syncInput);
        unsigned int stride = id->GetWidth(SVL_LEFT) * id->GetBPP();
        unsigned int height = id->GetHeight(SVL_LEFT);
        unsigned char *input1 = id->GetUCharPointer(SVL_LEFT);
        unsigned char *input2 = id->GetUCharPointer(SVL_RIGHT);
        unsigned char *output = OutputImage->GetUCharPointer();

        // copy data
        switch (Layout) {
            case svlLayoutInterlaced:
            case svlLayoutSideBySide:
                for (unsigned int j = 0; j < height; j ++) {
                    memcpy(output, input1, stride);
                    input1 += stride;
                    output += stride;
                    memcpy(output, input2, stride);
                    input2 += stride;
                    output += stride;
                }
            break;

            case svlLayoutInterlacedRL:
            case svlLayoutSideBySideRL:
                for (unsigned int j = 0; j < height; j ++) {
                    memcpy(output, input2, stride);
                    input2 += stride;
                    output += stride;
                    memcpy(output, input1, stride);
                    input1 += stride;
                    output += stride;
                }
            break;

            default:
                return SVL_FAIL;
        }
    }

    return SVL_OK;
}

int svlFilterStereoImageJoiner::Release()
{
    if (OutputImage) {
        delete OutputImage;
        OutputImage = 0;
    }
    return SVL_OK;
}

int svlFilterStereoImageJoiner::SetLayout(svlStereoLayout layout)
{
    if (IsInitialized()) return SVL_FAIL;
    Layout = layout;
    return SVL_OK;
}

svlStereoLayout svlFilterStereoImageJoiner::GetLayout()
{
    return Layout;
}

