/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id$
 
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

#include <iostream>

#include <cisstCommon/cmnPortability.h>
#include <cisstStereoVision/svlTypes.h>
#include "svlVideoCodecInitializer.h"

#if (CISST_OS == CISST_WINDOWS)
#include "svlVideoCodecVfW32.h"
#endif // CISST_WINDOWS

#if (CISST_SVL_HAS_ZLIB == ON)
#include "svlVideoCodecCVI.h"
#include "svlVideoCodecUDPStream.h"
#endif // CISST_SVL_HAS_ZLIB

#if (CISST_SVL_HAS_OPENCV == ON)
#include "svlVideoCodecOpenCV.h"
#endif // CISST_SVL_HAS_OPENCV


void svlInitializeVideoCodecs()
{
#ifdef _svlVideoCodecVfW32_h
    delete new svlVideoCodecVfW32;
#endif // _svlVideoCodecVfW32_h

#ifdef _svlVideoCodecCVI_h
    delete new svlVideoCodecCVI;
#endif // _svlVideoCodecCVI_h

#ifdef _svlVideoCodecUDPStream_h
    delete new svlVideoCodecUDPStream;
#endif // _svlVideoCodecUDPStream_h
    
#ifdef _svlVideoCodecOpenCV_h
    delete new svlVideoCodecOpenCV;
#endif // _svlVideoCodecOpenCV_h
}

