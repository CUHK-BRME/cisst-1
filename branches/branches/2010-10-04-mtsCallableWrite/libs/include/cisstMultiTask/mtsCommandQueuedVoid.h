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

#ifndef _mtsCommandQueuedVoid_h
#define _mtsCommandQueuedVoid_h

#include <cisstMultiTask/mtsCommandVoid.h>
#include <cisstMultiTask/mtsMailBox.h>

// Always include last
#include <cisstMultiTask/mtsExport.h>

class CISST_EXPORT mtsCommandQueuedVoid: public mtsCommandVoid
{
 public:
    /*! Base type */
    typedef mtsCommandVoid BaseType;

    /*! This type. */
    typedef mtsCommandQueuedVoid ThisType;

 protected:
    /*! Mailbox used to queue the commands */
    mtsMailBox * MailBox;
    /*! Queue of flags to indicate if the command is blocking or
      not */
    mtsQueue<mtsBlockingType> BlockingFlagQueue;

 private:
    /*! Private copy constructor to prevent copies */
    inline mtsCommandQueuedVoid(const ThisType & CMN_UNUSED(other));

 public:
    mtsCommandQueuedVoid(void);

    mtsCommandQueuedVoid(mtsCallableVoidBase * callable,
                         const std::string & name,
                         mtsMailBox * mailBox,
                         size_t size);

    inline virtual ~mtsCommandQueuedVoid() {}


    virtual mtsCommandQueuedVoid * Clone(mtsMailBox * mailBox, size_t size) const;

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

    virtual const std::string GetMailBoxName(void) const;

    void ToStream(std::ostream & outputStream) const;
};


#endif // _mtsCommandQueuedVoid_h
