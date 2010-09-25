/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Ankur Kapoor, Anton Deguet
  Created on: 2004-04-30

  (C) Copyright 2004-2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Defines a base class for a command with no argument
 */

#ifndef _mtsCommandVoidBase_h
#define _mtsCommandVoidBase_h

#include <cisstMultiTask/mtsCommandBase.h>

// Always include last
#include <cisstMultiTask/mtsExport.h>

/*!
  \ingroup cisstMultiTask

  A base class command object with an execute method that takes no
  arguments.  To be used to contain 0*Methods. */
class mtsCommandVoidBase: public mtsCommandBase
{
public:
    typedef mtsCommandBase BaseType;

    /*! The constructor. Does nothing */
    mtsCommandVoidBase(void): BaseType() {}

    /*! Constructor with a name. */
    mtsCommandVoidBase(const std::string & name): BaseType(name) {}

    /*! The destructor. Does nothing */
    virtual ~mtsCommandVoidBase() {}

    /*! The execute method. Abstract method to be implemented by derived
      classes to run the actual operation on the receiver
      \result Boolean value, true if success, false otherwise */
    virtual BaseType::ReturnType Execute(mtsBlockingType blocking) = 0;

    /* documented in base class */
    virtual void ToStream(std::ostream & outputStream) const = 0;

    /* documented in base class */
    inline size_t NumberOfArguments(void) const {
        return 0;
    }

    /* documented in base class */
    inline bool Returns(void) const {
        return false;
    }
};

#endif // _mtsCommandVoidBase_h

