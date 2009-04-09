/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsTaskManagerTest.h 2009-03-05 mjung5 $
  
  Author(s):  Min Yang Jung
  Created on: 2009-03-05
  
  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

//#include <cisstMultiTask/mtsCollector.h>
#include <cisstMultiTask/mtsStateData.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsTaskManager.h>

#include <string>

class mtsTaskManagerTestTask : public mtsTaskPeriodic {
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, 5);

protected:    
	mtsStateData<cmnDouble> TestData;

public:
	mtsTaskManagerTestTask(const std::string & collectorName, 
                           mtsCollectorBase * dataCollector = NULL,
                           double period = 10 * cmn_ms);
	virtual ~mtsTaskManagerTestTask() {}

	// implementation of four methods that are pure virtual in mtsTask
    void Configure(const std::string) {}
	void Startup(void)	{}
	void Run(void)		{}
    void Cleanup(void)	{}
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTaskManagerTestTask);

// Tester class ---------------------------------------------------------------
class mtsTaskManagerTest: public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(mtsTaskManagerTest);
	{		
		// public variables and methods		
		CPPUNIT_TEST(TestAddTask);
		CPPUNIT_TEST(TestRemoveTask);
		
		// private variables and methods		

	}
    CPPUNIT_TEST_SUITE_END();
	
private:
	//mtsCollector * Collector;
    
public:
    void setUp(void) {
		//Collector = new mtsCollector("collector", 10 * cmn_ms);
    }
    
    void tearDown(void) {
		//delete Collector;
    }
    
	// public variables and methods
	void TestAddTask(void);
	void TestRemoveTask(void);
	
	// private variables and methods	
};
