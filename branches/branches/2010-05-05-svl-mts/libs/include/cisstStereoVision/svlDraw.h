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

#ifndef _svlDraw_h
#define _svlDraw_h

#include <cisstStereoVision/svlTypes.h>

// Always include last!
#include <cisstStereoVision/svlExport.h>


namespace svlDraw
{
    void CISST_EXPORT Pixel(svlSampleImage* image, unsigned int videoch, svlPoint2D pos, svlRGB color);
    void CISST_EXPORT Pixel(svlSampleImage* image, unsigned int videoch, int x, int y, unsigned char r, unsigned char g, unsigned char b);
    void CISST_EXPORT Rectangle(svlSampleImage* image, unsigned int videoch, svlRect rect, svlRGB color, bool fill = true);
    void CISST_EXPORT Rectangle(svlSampleImage* image, unsigned int videoch, int left, int top, int right, int bottom, unsigned char r, unsigned char g, unsigned char b, bool fill = true);
    void CISST_EXPORT Line(svlSampleImage* image, unsigned int videoch, svlPoint2D from, svlPoint2D to, svlRGB color);
    void CISST_EXPORT Line(svlSampleImage* image, unsigned int videoch, int from_x, int from_y, int to_x, int to_y, unsigned char r, unsigned char g, unsigned char b);
    void CISST_EXPORT Poly(svlSampleImage* image, unsigned int videoch, const vctDynamicVectorRef<svlPoint2D> points, svlRGB color, unsigned int start = 0);
    void CISST_EXPORT Ellipse(svlSampleImage* image, unsigned int videoch, svlPoint2D center, vctInt2 radii, svlRGB color, double from_angle = 0.0, double to_angle = 360.0, double rotation = 0.0, int thickness = 1);
    void CISST_EXPORT Crosshair(svlSampleImage* image, unsigned int videoch, svlPoint2D pos, svlRGB color, unsigned int radius = 5);
    void CISST_EXPORT Crosshair(svlSampleImage* image, unsigned int videoch, int x, int y, unsigned char r, unsigned char g, unsigned char b, unsigned int radius = 5);
    void CISST_EXPORT Text(svlSampleImage* image, unsigned int videoch, svlPoint2D pos, const std::string & text, double fontsize, svlRGB color);
    void CISST_EXPORT Text(svlSampleImage* image, unsigned int videoch, int x, int y, const std::string & text, double fontsize, unsigned char r, unsigned char g, unsigned char b);
};

#endif // _svlDraw_h

