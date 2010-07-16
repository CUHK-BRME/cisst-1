/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: $
  
  Author(s):  Balazs Vagvolgyi
  Created on: 2010

  (C) Copyright 2006-2010 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <cisstStereoVision/svlTypes.h>


/******************************/
/*** svlSample class **********/
/******************************/

svlSample::svlSample() :
    cmnGenericObject(),
    EncoderParameter(-1)
{
}

svlSample::~svlSample()
{
}

bool svlSample::IsInitialized() const
{
    return false;
}

void svlSample::SetTimestamp(double ts)
{
    Timestamp = ts;
}

double svlSample::GetTimestamp() const
{
    return Timestamp;
}

svlSample* svlSample::GetNewFromType(svlStreamType type)
{
    switch (type) {
        case svlTypeInvalid:           return 0;                              break;
        case svlTypeStreamSource:      return 0;                              break;
        case svlTypeStreamSink:        return 0;                              break;
        case svlTypeImageCustom:       return 0;                              break;
        case svlTypeImageRGB:          return new svlSampleImageRGB;          break;
        case svlTypeImageRGBA:         return new svlSampleImageRGBA;         break;
        case svlTypeImageRGBStereo:    return new svlSampleImageRGBStereo;    break;
        case svlTypeImageRGBAStereo:   return new svlSampleImageRGBAStereo;   break;
        case svlTypeImageMono8:        return new svlSampleImageMono8;        break;
        case svlTypeImageMono8Stereo:  return new svlSampleImageMono8Stereo;  break;
        case svlTypeImageMono16:       return new svlSampleImageMono16;       break;
        case svlTypeImageMono16Stereo: return new svlSampleImageMono16Stereo; break;
        case svlTypeImageMonoFloat:    return new svlSampleImageMonoFloat;    break;
        case svlTypeImage3DMap:        return new svlSampleImage3DMap;        break;
        case svlTypeTransform3D:       return new svlSampleTransform3D;       break;
        case svlTypeTargets:           return new svlSampleTargets;           break;
        case svlTypeText:              return new svlSampleText;              break;
    }
    return 0;
}

void svlSample::SetEncoder(const std::string & codec, const int parameter)
{
    Encoder = codec;
    EncoderParameter = parameter;
}

void svlSample::GetEncoder(std::string & codec, int & parameter) const
{
    codec = Encoder;
    parameter = EncoderParameter;
}

