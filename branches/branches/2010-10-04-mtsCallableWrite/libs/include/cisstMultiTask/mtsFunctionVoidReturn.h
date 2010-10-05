/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s): Anton Deguet
  Created on: 2010-09-16

  (C) Copyright 2010 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/



/*!
  \file
  \brief Defines a function object to use a void command (mtsCommandVoidReturn)
*/

#ifndef _mtsFunctionVoidReturn_h
#define _mtsFunctionVoidReturn_h

#include <cisstMultiTask/mtsFunctionBase.h>
#include <cisstMultiTask/mtsCommandVoidReturnBase.h>
#include <cisstMultiTask/mtsForwardDeclarations.h>

// Always include last
#include <cisstMultiTask/mtsExport.h>

class CISST_EXPORT mtsFunctionVoidReturn: public mtsFunctionBase {
 protected:
    typedef mtsCommandVoidReturnBase CommandType;
    CommandType * Command;

 public:
    /*! Default constructor.  Does nothing, use Instantiate before
      using. */
    mtsFunctionVoidReturn(void): Command(0) {}

    /*! Destructor. */
    ~mtsFunctionVoidReturn();

    // documented in base class
    bool Detach(void);

    // documented in base class
    bool IsValid(void) const;

    /*! Bind using a command pointer.  This allows to avoid
      querying by name from an interface.
      \param command Pointer on an existing command
      \result Boolean value, true if the command pointer is not null.
    */
    bool Bind(CommandType * command);

    /*! Overloaded operator to enable more intuitive syntax
      e.g., Command() instead of Command->Execute(). */
    mtsExecutionResult operator()(mtsGenericObject & result) const;

    /*! Access to underlying command object. */
    CommandType * GetCommand(void) const;

    /*! Human readable output to stream. */
    void ToStream(std::ostream & outputStream) const;
};


#endif // _mtsFunctionVoidReturn_h

