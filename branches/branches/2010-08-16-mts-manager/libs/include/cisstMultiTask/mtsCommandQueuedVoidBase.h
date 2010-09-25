/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Ankur Kapoor, Peter Kazanzides, Anton Deguet
  Created on: 2005-05-02

  (C) Copyright 2005-2009 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Defines base class for a queued void command.
*/

#ifndef _mtsCommandQueuedVoidBase_h
#define _mtsCommandQueuedVoidBase_h

#include <cisstMultiTask/mtsCommandVoidBase.h>
#include <cisstMultiTask/mtsMailBox.h>

// Always include last
#include <cisstMultiTask/mtsExport.h>

class CISST_EXPORT mtsCommandQueuedVoidBase: public mtsCommandVoidBase
{
 public:
    /*! Base type */
    typedef mtsCommandVoidBase BaseType;

    /*! This type. */
    typedef mtsCommandQueuedVoidBase ThisType;

 protected:
    /*! Mailbox used to queue the commands */
    mtsMailBox * MailBox;
    /*! Actual command being queued. */
    mtsCommandVoidBase * ActualCommand;
    /*! Queue of flags to indicate if the command is blocking or
      not */
    mtsQueue<mtsBlockingType> BlockingFlagQueue;

 private:
    /*! Private copy constructor to prevent copies */
    inline mtsCommandQueuedVoidBase(const ThisType & CMN_UNUSED(other));

 public:
    mtsCommandQueuedVoidBase(void);

    mtsCommandQueuedVoidBase(mtsMailBox * mailBox,
                             mtsCommandVoidBase * actualCommand,
                             size_t size);

    inline virtual ~mtsCommandQueuedVoidBase() {}


    virtual mtsCommandQueuedVoidBase * Clone(mtsMailBox * mailBox, size_t size) const;

    inline virtual void Allocate(unsigned int CMN_UNUSED(size)) {}

    /*! For a queued command, Execute means queueing the command.
      This method will return mtsCommandBase::DEV_OK if the command
      has been queued, it doesn't mean that the actual has been
      executed yet.  If the command has been disabled (see
      mtsCommandBase::Disable()), Execute will return
      mtsCommandBase::DISABLED.  finally, if the mailbox is full,
      Execute() will return mtsCommandBase::MAILBOX_FULL.  This can
      happen if the task receiving the command doesn't process/empty
      its mailboxes fast enough. */
    mtsCommandBase::ReturnType Execute(mtsBlockingType blocking);

    mtsBlockingType BlockingFlagGet(void);

    virtual mtsCommandVoidBase * GetActualCommand(void);

    virtual const std::string GetMailBoxName(void) const;

    void ToStream(std::ostream & outputStream) const;
};


#endif // _mtsCommandQueuedVoidBase_h
