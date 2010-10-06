/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id:$

  Author(s):  Peter Kazanzides

  (C) Copyright 2005-2010 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Define an internal command for cisstMultiTask
*/


#ifndef _mtsCommandFilteredQueuedWrite_h
#define _mtsCommandFilteredQueuedWrite_h

#include <cisstMultiTask/mtsCommandQueuedWrite.h>


/*!
  \ingroup cisstMultiTask
 */

class mtsCommandFilteredQueuedWrite: public mtsCommandQueuedWriteGeneric
{
public:
    typedef mtsCommandQueuedWriteGeneric  BaseType;
    typedef mtsCommandFilteredQueuedWrite ThisType;

protected:
    mtsCommandQualifiedReadBase * ActualFilter;
    mtsGenericObject *filterOutput;

private:
    /*! Private copy constructor to prevent copies */
    inline mtsCommandFilteredQueuedWrite(const ThisType & CMN_UNUSED(other));

public:

    mtsCommandFilteredQueuedWrite(mtsCommandQualifiedReadBase * actualFilter,
                                  mtsCommandWriteBase * actualCommand);

    mtsCommandFilteredQueuedWrite(mtsMailBox * mailBox,
                                  mtsCommandQualifiedReadBase * actualFilter,
                                  mtsCommandWriteBase * actualCommand,
                                  size_t size);

    virtual ~mtsCommandFilteredQueuedWrite();

    mtsCommandFilteredQueuedWrite * Clone(mtsMailBox * mailBox, size_t size) const;

    /*! Return a pointer on the argument prototype */
    virtual const mtsGenericObject * GetArgumentPrototype(void) const;

    mtsExecutionResult Execute(const mtsGenericObject & argument, mtsBlockingType blocking);
};


#endif // _mtsCommandFilteredQueuedWrite_h
