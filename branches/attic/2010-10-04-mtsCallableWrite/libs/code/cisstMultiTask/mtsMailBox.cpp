/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Peter Kazanzides
  Created on: 2007-09-05

  (C) Copyright 2007 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnAssert.h>
#include <cisstMultiTask/mtsMailBox.h>
#include <cisstMultiTask/mtsCallableVoidBase.h>
#include <cisstMultiTask/mtsCallableWriteBase.h>
#include <cisstMultiTask/mtsCommandQueuedVoid.h>
#include <cisstMultiTask/mtsCommandQueuedWrite.h>
#include <cisstMultiTask/mtsCommandQueuedVoidReturn.h>


mtsMailBox::mtsMailBox(const std::string & name,
                       size_t size,
                       mtsCallableVoidBase * postCommandQueuedCallable):
    CommandQueue(size, 0),
    Name(name),
    PostCommandQueuedCallable(postCommandQueuedCallable)
{}


mtsMailBox::~mtsMailBox(void)
{}


const std::string & mtsMailBox::GetName(void) const
{
    return this->Name;
}


bool mtsMailBox::Write(mtsCommandBase * command)
{
    bool result;
    result = (CommandQueue.Put(command) != 0);
    if (this->PostCommandQueuedCallable) {
        this->PostCommandQueuedCallable->Execute();
    }
    return result;
}


void mtsMailBox::ThreadSignalWait(void)
{
    this->ThreadSignal.Wait();
}


// return false if nothing to execute; true otherwise.
// NOTE: this will break if exceptions are thrown by the command objects
// because the commands (and parameters) won't be removed from the queues.
bool mtsMailBox::ExecuteNext(void)
{
   mtsCommandBase** command = CommandQueue.Peek();

   // test for empty queue
   if (!command) {
       return false;
   }

   mtsCommandQueuedVoid * commandVoid;
   mtsCommandQueuedWrite * commandWrite;
   //   mtsCommandQueuedWriteGeneric * commandWriteGeneric;
   mtsCommandQueuedVoidReturnBase * commandVoidReturn;

   if (!(*command)->Returns()) {
       switch ((*command)->NumberOfArguments()) {
       case 0:
           commandVoid = dynamic_cast<mtsCommandQueuedVoid *>(*command);
           CMN_ASSERT(commandVoid);
           commandVoid->GetCallable()->Execute();
           if (commandVoid->BlockingFlagGet() == MTS_BLOCKING) {
               this->ThreadSignal.Raise();
           }
           break;
       case 1:
           commandWrite = dynamic_cast<mtsCommandQueuedWrite *>(*command);
           if (commandWrite) {
               commandWrite->GetCallable()->Execute(*(commandWrite->ArgumentPeek()));
               commandWrite->ArgumentGet();  // Remove from parameter queue
               if (commandWrite->BlockingFlagGet() == MTS_BLOCKING) {
                   this->ThreadSignal.Raise();
               }
           } else {
               #if 0
               commandWriteGeneric = dynamic_cast<mtsCommandQueuedWriteGeneric *>(*command);
               CMN_ASSERT(commandWriteGeneric);
               commandWriteGeneric->GetActualCommand()->Execute(*(commandWriteGeneric->ArgumentPeek()), MTS_NOT_BLOCKING);
               commandWriteGeneric->ArgumentGet();  // Remove from parameter queue
               if (commandWriteGeneric->BlockingFlagGet() == MTS_BLOCKING) {
                   this->ThreadSignal.Raise();
               }
               #endif
           }
           break;
       default:
           CMN_LOG_RUN_ERROR << "Class mtsMailBox: Invalid parameter in ExecuteNext" << std::endl;
           return false;
       }
   } else {
       switch ((*command)->NumberOfArguments()) {
       case 0:
           commandVoidReturn = dynamic_cast<mtsCommandQueuedVoidReturnBase *>(*command);
           CMN_ASSERT(commandVoidReturn);
           commandVoidReturn->GetActualCommand()->Execute( *(commandVoidReturn->GetResultPointer()) );
           this->ThreadSignal.Raise();
           break;
       default:
           CMN_LOG_RUN_ERROR << "Class mtsMailBox: Invalid parameter in ExecuteNext" << std::endl;
           return false;
       }
   }
   CommandQueue.Get();  // Remove command from mailbox queue
   return true;
}

