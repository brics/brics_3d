/**
 * @file 
 * LoggerTest.cpp
 *
 * @date: Nov 4, 2010
 * @author: sblume
 */

#include "LoggerTest.h"


namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( LoggerTest );

LoggerTestListener::LoggerTestListener(){
	messageBufferHandle = 0;
}

void LoggerTestListener::write(Logger::Loglevel level, std::string message) {
	CPPUNIT_ASSERT(messageBufferHandle != 0);
	(*messageBufferHandle) = message;
}

void LoggerTest::setUp() {
	messageBuffer.clear();
	listener = new LoggerTestListener();
	listener->setMessageBufferHandle(&messageBuffer);
	Logger::setListener(listener);

	unitTestlogLevel = Logger::getMinLoglevel();
}

void LoggerTest::tearDown() {
	if (listener) {
		delete listener;
		listener = 0;
	}

	Logger::setMinLoglevel(unitTestlogLevel); //restore the initial logger level
	Logger::setListener(0); //un set the listener
}

void LoggerTest::testSimpleLogging() {
	std::string testMessage = "This is some text to test the Logger module. Please ignore this.";
	std::string expectedLogMessage;

	Logger::setMinLoglevel(Logger::INFO);
	expectedLogMessage = "[INFO] ";
	expectedLogMessage.append(testMessage);
	expectedLogMessage.append("\n");

	LOG(INFO) << testMessage;
//	std::cout << "expectedLogMessage:" << expectedLogMessage;
//	std::cout << "messageBuffer:" << messageBuffer;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);
}

void LoggerTest::testLoggerDebugWithLevels() {
	std::string testMessage = "This is some text to test the Logger module. Please ignore this.";
	std::string expectedLogMessage;

	expectedLogMessage = "[DEBUG] ";
	expectedLogMessage.append(testMessage);
	expectedLogMessage.append("\n");

	/* Set level to DEBUG and check */
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	//take previous defined message

	messageBuffer.clear();
	LOG(DEBUG) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to INFO and check */
	Logger::setMinLoglevel(Logger::INFO);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(DEBUG) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to WARNING and check */
	Logger::setMinLoglevel(Logger::WARNING);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(DEBUG) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to LOGERROR and check */
	Logger::setMinLoglevel(Logger::LOGERROR);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(DEBUG) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to FATAL and check */
	Logger::setMinLoglevel(Logger::FATAL);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(DEBUG) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);
}

void LoggerTest::testLoggerInfoWithLevels() {
	std::string testMessage = "This is some text to test the Logger module. Please ignore this.";
	std::string expectedLogMessage;

	expectedLogMessage = "[INFO] ";
	expectedLogMessage.append(testMessage);
	expectedLogMessage.append("\n");

	/* Set level to DEBUG and check */
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	//take previous defined message

	messageBuffer.clear();
	LOG(INFO) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to INFO and check */
	Logger::setMinLoglevel(Logger::INFO);
	//take previous defined message

	messageBuffer.clear();
	LOG(INFO) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to WARNING and check */
	Logger::setMinLoglevel(Logger::WARNING);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(INFO) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to LOGERROR and check */
	Logger::setMinLoglevel(Logger::LOGERROR);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(INFO) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to FATAL and check */
	Logger::setMinLoglevel(Logger::FATAL);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(INFO) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);
}

void LoggerTest::testLoggerWarningWithLevels() {
	std::string testMessage = "This is some text to test the Logger module. Please ignore this.";
	std::string expectedLogMessage;

	expectedLogMessage = "[WARNING] ";
	expectedLogMessage.append(testMessage);
	expectedLogMessage.append("\n");

	/* Set level to DEBUG and check */
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	//take previous defined message

	messageBuffer.clear();
	LOG(WARNING) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to INFO and check */
	Logger::setMinLoglevel(Logger::INFO);
	//take previous defined message

	messageBuffer.clear();
	LOG(WARNING) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to WARNING and check */
	Logger::setMinLoglevel(Logger::WARNING);
	//take previous defined message

	messageBuffer.clear();
	LOG(WARNING) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to LOGERROR and check */
	Logger::setMinLoglevel(Logger::LOGERROR);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(WARNING) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to FATAL and check */
	Logger::setMinLoglevel(Logger::FATAL);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(WARNING) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);
}

void LoggerTest::testLoggerErrorWithLevels() {
	std::string testMessage = "This is some text to test the Logger module. Please ignore this.";
	std::string expectedLogMessage;

	expectedLogMessage = "[ERROR] ";
	expectedLogMessage.append(testMessage);
	expectedLogMessage.append("\n");

	/* Set level to DEBUG and check */
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	//take previous defined message

	messageBuffer.clear();
	LOG(ERROR) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to INFO and check */
	Logger::setMinLoglevel(Logger::INFO);
	//take previous defined message

	messageBuffer.clear();
	LOG(ERROR) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to WARNING and check */
	Logger::setMinLoglevel(Logger::WARNING);
	//take previous defined message

	messageBuffer.clear();
	LOG(ERROR) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to LOGERROR and check */
	Logger::setMinLoglevel(Logger::LOGERROR);
	//take previous defined message

	messageBuffer.clear();
	LOG(ERROR) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);


	/* Set level to FATAL and check */
	Logger::setMinLoglevel(Logger::FATAL);
	expectedLogMessage = "";

	messageBuffer.clear();
	LOG(ERROR) << testMessage;
	CPPUNIT_ASSERT( expectedLogMessage.compare(messageBuffer) == 0);
}

}  // namespace unitTests
/* EOF */
