/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$
  
  Author(s):  Anton Deguet
  Created on: 2003-07-28
  
  (C) Copyright 2003-2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnClassRegister.h>
#include <cisstCommon/cmnPortability.h>
#include <cisstConfig.h>

/* The tests based on class registration with an external dynamic
  library can not be used if the cisst libraries are compiled
  statically (this case leads to two instantiations of the class
  register. */
#if !CISST_BUILD_SHARED_LIBS
  #define USE_DYNAMIC_LIB_CLASS_REGISTRATION_TESTS 0
#else
  #define USE_DYNAMIC_LIB_CLASS_REGISTRATION_TESTS 1
#endif



class myGenericObject: public cmnGenericObject {
 public:
    inline void Message(cmnLogger::LoDType lod) {
        CMN_LOG(lod) << "Function " << Services()->GetName() << ": " << lod << std::endl;
        CMN_LOG_CLASS(lod) << lod << std::endl;
    }
};

class TestA: public myGenericObject {
    /* register this class with a default LoD 5 */
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, 5);
public:
};
CMN_DECLARE_SERVICES_INSTANTIATION(TestA);

class TestB : public myGenericObject {
    /* register this class with a default LoD 7 */
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, 7);
public:
};
CMN_DECLARE_SERVICES_INSTANTIATION(TestB);

class TestC : public myGenericObject {
    /* register this class with a default LoD 3 */
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, 3);
public:
};
CMN_DECLARE_SERVICES_INSTANTIATION(TestC);

class TestC1: public TestC {
    /* this class is not registered */
public:
};

class TestC2: public TestC {
    /* register this class with a default LoD 1 */
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, 1);
public:
    unsigned int Size;
    double * Elements;
    // add constructors to test dynamic creation
    inline TestC2(void): Size(2) {
        this->Elements = new double[this->Size];
        this->Elements[0] = 1;
        this->Elements[1] = 2;
    }
    inline TestC2(const TestC2 & other) {
        this->Size = other.Size;
        if (this->Size != 0) {
            this->Elements = new double[this->Size];
            unsigned int index;
            for (index = 0; index < this->Size; index++) {
                this->Elements[index] = other.Elements[index];
            }
        } else {
            this->Elements = 0;
        }
    }
};
CMN_DECLARE_SERVICES_INSTANTIATION(TestC2);

template <int _x, int _y>
class TestD: public cmnGenericObject {
    /* register these classes with LoD 9 */
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, 9);
public:
    enum {X = _x};
    enum {Y = _y};
};

typedef TestD<1, 2> TestD12;
CMN_DECLARE_SERVICES_INSTANTIATION(TestD12);

typedef TestD<3, 4> TestD34;
CMN_DECLARE_SERVICES_INSTANTIATION(TestD34);


class cmnClassRegisterTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(cmnClassRegisterTest);
    CPPUNIT_TEST(TestRegistration);
    CPPUNIT_TEST(TestRegistrationStaticAllInline);
    CPPUNIT_TEST(TestRegistrationStaticNoInline);
#if USE_DYNAMIC_LIB_CLASS_REGISTRATION_TESTS
    CPPUNIT_TEST(TestRegistrationDynamicAllInline);
    CPPUNIT_TEST(TestRegistrationDynamicNoInline);
#endif
    CPPUNIT_TEST(TestLoD);
    CPPUNIT_TEST(TestLog);
    CPPUNIT_TEST(TestDynamicCreation);
    CPPUNIT_TEST(TestIterators);
    CPPUNIT_TEST_SUITE_END();
    
 public:
    void setUp(void) {
    }
    
    void tearDown(void) {
    }
    
    /*! Test the class registration. */
    void TestRegistration(void);

    /*! Test the class registration with an external static library. */
    void TestRegistrationStaticAllInline(void);
    void TestRegistrationStaticNoInline(void);

    /*! Test the class registration with an external dynamic library. */
#if USE_DYNAMIC_LIB_CLASS_REGISTRATION_TESTS
    void TestRegistrationDynamicAllInline(void);
    void TestRegistrationDynamicNoInline(void);
#endif

    /*! Test the different ways to set the class LoD. */
    void TestLoD(void);

    /*! Test the log filtering. */
    void TestLog(void);

    /*! Test dynamic creation of objects */
    void TestDynamicCreation(void);

    /*! Test iterators */
    void TestIterators(void);
};


CPPUNIT_TEST_SUITE_REGISTRATION(cmnClassRegisterTest);

