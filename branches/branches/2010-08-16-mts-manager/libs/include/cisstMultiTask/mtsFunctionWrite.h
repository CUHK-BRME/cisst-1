/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Peter Kazanzides, Anton Deguet

  (C) Copyright 2007-2010 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Defines the command interfaces.
*/

#ifndef _mtsFunctionWrite_h
#define _mtsFunctionWrite_h


#include <cisstMultiTask/mtsFunctionBase.h>
#include <cisstMultiTask/mtsCommandWriteBase.h>
#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsGenericObjectProxy.h>

// Always include last
#include <cisstMultiTask/mtsExport.h>

class CISST_EXPORT mtsFunctionWrite: public mtsFunctionBase {
protected:
    typedef mtsCommandWriteBase CommandType;
    CommandType * Command;

    template <typename _userType, bool>
    class ConditionalWrap {
    public:
        static mtsCommandBase::ReturnType Call(mtsCommandWriteBase * command, const _userType & argument, mtsBlockingType blocking) {
            mtsGenericObjectProxyRef<_userType> argumentWrapped(argument);
            return command->Execute(argumentWrapped, blocking);
        }
    };
    template <typename _userType>
    class ConditionalWrap<_userType, true> {
    public:
        static mtsCommandBase::ReturnType Call(mtsCommandWriteBase * command, const _userType & argument, mtsBlockingType blocking) {
            return command->Execute(argument, blocking);
        }
    };

public:
    /*! Default constructor.  Does nothing, use Bind before
      using. */
    mtsFunctionWrite(void): Command(0) {}

    /*! Destructor. */
    virtual ~mtsFunctionWrite();

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
      e.g., Command(argument) instead of Command->Execute(argument). */
    mtsCommandBase::ReturnType operator()(const mtsGenericObject & argument) const;

    mtsCommandBase::ReturnType ExecuteBlocking(const mtsGenericObject & argument) const;

	/*! Overloaded operator that accepts different argument types. */
    template <class _userType>
    mtsCommandBase::ReturnType operator()(const _userType & argument) const {
        mtsCommandBase::ReturnType ret = Command ?
            ConditionalWrap<_userType, cmnIsDerivedFrom<_userType, mtsGenericObject>::YES>::Call(Command, argument, MTS_NOT_BLOCKING)
          : mtsCommandBase::NO_INTERFACE;
        return ret;
    }

    template <class _userType>
    mtsCommandBase::ReturnType ExecuteBlocking(const _userType & argument) const {
        mtsCommandBase::ReturnType ret = Command ?
            ConditionalWrap<_userType, cmnIsDerivedFrom<_userType, mtsGenericObject>::YES>::Call(Command, argument, MTS_BLOCKING)
          : mtsCommandBase::NO_INTERFACE;
        return ret;
    }

    /*! Access to underlying command object. */
    CommandType * GetCommand(void) const;

    /*! Access to the command argument prototype. */
    const mtsGenericObject * GetArgumentPrototype(void) const;

    /*! Human readable output to stream. */
    void ToStream(std::ostream & outputStream) const;

};

#endif // _mtsFunctionWrite_h

