/*

  Author(s): Simon Leonard
  Created on: Dec 02 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _devGroup_h
#define _devGroup_h

#include <cisstDevices/robotcomponents/manipulators/Barrett/devPuck.h>
#include <vector>

//! A logical group of pucks
/**
   Groups are used to communicate with pucks simultaneously. This has the 
   benifit of saving bandwidth. For example, all the pucks belong to the 
   BROADCAST group. Thus, when a CAN frame is destined to the BROADCAST group
   all the pucks will process the frame.
*/
class devGroup {
public:

  //! The ID used to identify each group
  enum ID{

    //! The broadcast group
    /**
       The broadcast group contains all the pucks in a WAM (with the exception 
       of the safety module)
    */
    BROADCAST = 0x00,

    //! The upper arm group
    /**
       The upper arm group represents the 4 pucks of the upper arm 
       (shoulder+elbow)
    */
    UPPERARM = 0x01,

    //! The forearm group
    /**
       The forearm group represents the 3 pucks of the upper arm (wrist)
    */
    FOREARM = 0x02,

    //! The motor position group
    /**
       All pucks belong to the position group. Send a message to this group to 
       query all the motor positions. Each puck will reply to a message to this
       group with its motor position
    */
    POSITION = 0x03,

    //! Upper arm property group
    UPPERARM_POSITION = 0x04,

    //! Forearm property group
    FOREARM_POSITION = 0x05,

    //! Feedback property group
    PROPERTY = 0x06,

    HAND = 0x07,

    HAND_POSITION = 0x08,
    
    LASTGROUP = 0x09

  };

  //! Define the status of a group
  /**
     Barrett defines the following mode in which a puck can be
  */
  enum Status{ RESET=0, READY=2 };

  //! Error codes used by devGroup
  enum Errno{ ESUCCESS, EFAILURE };
  
private:

  //! The ID of the pucks in the group
  std::vector<devPuck::ID> pucksid;

  //! The CAN bus that is connected to the group
  devCAN* canbus; 
  
  //! The ID of the group
  devGroup::ID id;

  //! Is the data contain a set property command
  /**
     Pucks have properties that can be read/write. To read/write a property, 
     you must send a CAN frame with the proper data format. To write a property,
     the CAN data must have a "write" bit set while a read command must have the
     "write" bit cleared. This method returns true if the "write" bit is set.
     \param canframe A CAN frame with a read/write command
     \return true if the command is a write. false if the command is a read
  */
  static bool IsSetFrame( const devCAN::Frame& canframe );
  
  //! pack a CAN frame
  /**
     Build a CAN frame destined to the group of pucks with the data formated.
     \param canframe[out] The resulting CAN frame
     \param propid The property ID to set or get
     \param propval The property value if the property must be set
     \param set True of the property must be set. False for a query
     \return false if no error occurred. true otherwise
  */
  devGroup::Errno PackProperty( devCAN::Frame& canframe,
				devProperty::Command command,
				devProperty::ID propid,
				devProperty::Value propval = 0 );
  
  //! The bit of a CAN ID that identicates a group
  static const devCAN::Frame::ID GROUP_CODE = 0x0400;
  
public:

  //! Create a group with an ID and a CAN device
  /**
     Initialize the group to the given ID and give the CAN device connected to
     the pucks.
     \param groupid The ID of the puck
     \param can The CAN device used to communicate with the pucks
  */
  devGroup( devGroup::ID id, devCAN* canbus );
  
  //! Convert a group ID to a CAN id
  /**
     Convert the ID of a group to a CAN ID used in a CAN frame. This assumes 
     that the origin of the CAN ID will be the host (00000)
  */
  static devCAN::Frame::ID CANID( devGroup::ID groupid );
  
  //! Return the group ID
  devGroup::ID GetID() const;

  //! Return the origin ID of the CAN id
  static devGroup::ID OriginID( devCAN::Frame::ID canid );
  
  //! Return the origin ID of the CAN frame
  static devGroup::ID OriginID( const devCAN::Frame& canframe );
  
  //! Return the destination ID of the CAN id
  static devGroup::ID DestinationID( devCAN::Frame::ID canid );
  
  //! Return the destination ID of the CAN id
  static devGroup::ID DestinationID( const devCAN::Frame& canframe );
  
  //! Return true if the CAN frame id's destination is a group (any group)
  static bool IsDestinationAGroup( const devCAN::Frame canframe );
  
  //! Add the puck ID to the group
  void AddPuckIDToGroup( devPuck::ID puckid ) { pucksid.push_back(puckid); }

  bool Empty() const { return pucksid.empty(); }

  //! Return the puck ID of the first member
  devPuck::ID FirstMember() const { return pucksid.front(); }
  devPuck::ID LastMember()  const { return pucksid.back(); }

  bool IsGroupEmpty() const { return pucksid.empty(); }

  //! Querry the group. This is only valid for querying
  //  positions on group 3
  devGroup::Errno GetProperty( devProperty::ID id );
  
  //! Set the property of a group
  devGroup::Errno SetProperty( devProperty::ID id, 
			       devProperty::Value value,
			       bool verify );

};

// Increment operator for pucks id
devGroup::ID operator++( devGroup::ID& gid, int i );

#endif
