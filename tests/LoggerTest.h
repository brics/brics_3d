/**
 * @file 
 * LoggerTest.h
 *
 * @date: Nov 4, 2010
 * @author: sblume
 */

#ifndef LOGGERTEST_H_
#define LOGGERTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/Logger.h"
#include <sstream>

using namespace BRICS_3D;




namespace unitTests {

class LoggerTestListener : public Logger::Listener {
public:
	LoggerTestListener();

	virtual void write(Logger::Loglevel level, std::string message);
	void setMessageBufferHandle(std::string* messageBufferHandle){this->messageBufferHandle = messageBufferHandle;};

private:
	std::string* messageBufferHandle;

};

class LoggerTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( LoggerTest );
	CPPUNIT_TEST( testSimpleLogging );
	CPPUNIT_TEST( testLoggerDebugWithLevels );
	CPPUNIT_TEST( testLoggerInfoWithLevels );
	CPPUNIT_TEST( testLoggerWarningWithLevels );
	CPPUNIT_TEST( testLoggerErrorWithLevels );
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testSimpleLogging();
	void testLoggerDebugWithLevels();
	void testLoggerInfoWithLevels();
	void testLoggerWarningWithLevels();
	void testLoggerErrorWithLevels();

private:

	LoggerTestListener* listener;
	std::string messageBuffer;
	Logger::Loglevel unitTestlogLevel; //the log level that the other unit tests use - this needs to be restored ofter the Logger has been tested.

};

}

#endif /* LOGGERTEST_H_ */

/* EOF */
