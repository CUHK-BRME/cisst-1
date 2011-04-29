# (C) Copyright 2005-2007 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
# 
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
# 
# --- end cisst license ---

if( UNIX )

  # set the search path
  set( XENOMAI_SEARCH_PATH /usr/local/xenomai /usr/xenomai )
  
  # find xeno-config.h
  find_path( XENOMAI_DIR include/xeno_config.h ${XENOMAI_SEARCH_PATH} )
  
  # did we find xeno_config.h?
  if( XENOMAI_DIR ) 
    
    # set the include directory
    set( XENOMAI_INCLUDE_DIR ${XENOMAI_DIR}/include )
    set( XENOMAI_INCLUDE_POSIX_DIR ${XENOMAI_DIR}/include/posix )

    # find the xenomai pthread library
    find_library( XENOMAI_LIBRARY_NATIVE  native  ${XENOMAI_DIR}/lib )
    find_library( XENOMAI_LIBRARY_XENOMAI xenomai ${XENOMAI_DIR}/lib )
    find_library( XENOMAI_LIBRARY_PTHREAD_RT pthread_rt rtdm ${XENOMAI_DIR}/lib )
    find_library( XENOMAI_LIBRARY_RTDM    rtdm    ${XENOMAI_DIR}/lib )

    # find the posix wrappers
    find_file(XENOMAI_POSIX_WRAPPERS lib/posix.wrappers ${XENOMAI_SEARCH_PATH} )

    # set the linker flags
    set( XENOMAI_EXE_LINKER_FLAGS "-Wl,@${XENOMAI_POSIX_WRAPPERS}" )

    # add compile/preprocess options
    set(XENOMAI_DEFINITIONS "-D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__")

    
  endif( XENOMAI_DIR )

endif( UNIX )

