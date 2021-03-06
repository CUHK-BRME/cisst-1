/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Anton Deguet, Min Yang Jung
  Created on: 2009-09-03

  (C) Copyright 2009-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Defines the function proxy objects.
*/

#ifndef _mtsFunctionWriteReturnProxy_h
#define _mtsFunctionWriteReturnProxy_h

#include <cisstMultiTask/mtsFunctionWriteReturn.h>
#include "mtsFunctionReturnProxyBase.h"
#include "mtsInterfaceRequiredProxy.h"
#include "mtsProxySerializer.h"

class mtsFunctionWriteReturnProxy: public mtsFunctionWriteReturn, public mtsFunctionReturnProxyBase
{
protected:
    mtsGenericObject * ArgumentPointer;

public:
    mtsFunctionWriteReturnProxy(mtsInterfaceRequired * interfaceRequired):
        mtsFunctionWriteReturn(true /* this is a proxy class */),
        mtsFunctionReturnProxyBase(interfaceRequired),
        ArgumentPointer(0)
    {
    }

    ~mtsFunctionWriteReturnProxy()
    {
        if (this->ArgumentPointer) {
            delete this->ArgumentPointer;
        }
    }

    inline mtsGenericObject * GetArgumentPointer(void) const {
        return this->ArgumentPointer;
    }

    inline void SetArgumentPointer(mtsGenericObject * genericObject) {
        this->ArgumentPointer = genericObject;
    }
};

#endif // _mtsFunctionWriteReturnProxy_h
