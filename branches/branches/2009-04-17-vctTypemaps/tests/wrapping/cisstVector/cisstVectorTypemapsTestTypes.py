# -*- tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
# ex: set softtabstop=4 shiftwidth=4 tabstop=4 expandtab:

#
# $Id: $
#
# Author(s):  Daniel Li, Anton Deguet
# Created on: 2009-05-20
#
# (C) Copyright 2009-2010 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---
#

import numpy

# import all test classes
from cisstVectorPythonTestPython import vctDynamicVectorTypemapsTest_int
from cisstVectorPythonTestPython import vctDynamicVectorTypemapsTest_double

from cisstVectorPythonTestPython import vctFixedSizeVectorTypemapsTest_int_4
from cisstVectorPythonTestPython import vctFixedSizeVectorTypemapsTest_uint_4
from cisstVectorPythonTestPython import vctFixedSizeVectorTypemapsTest_double_4

from cisstVectorPythonTestPython import vctDynamicMatrixTypemapsTest_int
from cisstVectorPythonTestPython import vctDynamicMatrixTypemapsTest_double

from cisstVectorPythonTestPython import vctFixedSizeMatrixTypemapsTest_int_4_4
from cisstVectorPythonTestPython import vctFixedSizeMatrixTypemapsTest_uint_4_4
from cisstVectorPythonTestPython import vctFixedSizeMatrixTypemapsTest_double_4_4

from cisstVectorPythonTestPython import vctDynamicNArrayTypemapsTest_int_4
from cisstVectorPythonTestPython import vctDynamicNArrayTypemapsTest_double_4


# define which types to use and their numpy/C++ test classes correspondences
vctDynamicVectorTypemapsTest_types = {numpy.int32  : vctDynamicVectorTypemapsTest_int,
                                      numpy.double : vctDynamicVectorTypemapsTest_double}

vctFixedSizeVectorTypemapsTest_types = {numpy.int32  : vctFixedSizeVectorTypemapsTest_int_4,
                                        numpy.uint32 : vctFixedSizeVectorTypemapsTest_uint_4,
                                        numpy.double : vctFixedSizeVectorTypemapsTest_double_4}

vctDynamicMatrixTypemapsTest_types = {numpy.int32  : vctDynamicMatrixTypemapsTest_int,
                                      numpy.double : vctDynamicMatrixTypemapsTest_double}

vctFixedSizeMatrixTypemapsTest_types = {numpy.int32  : vctFixedSizeMatrixTypemapsTest_int_4_4,
                                        numpy.uint32 : vctFixedSizeMatrixTypemapsTest_uint_4_4,
                                        numpy.double : vctFixedSizeMatrixTypemapsTest_double_4_4}

vctDynamicNArrayTypemapsTest_types = {numpy.int32  : vctDynamicNArrayTypemapsTest_int_4,
                                      numpy.double : vctDynamicNArrayTypemapsTest_double_4}
