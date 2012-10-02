/**
 * @file 
 * ECUTTestRunner.cpp
 *
 * @date: Oct 22, 2009
 * @author: sblume
 */


#include "ECUTTestRunner.h"

/*******************************************************************************
 * Copyright (c) 2008 Gerhard Leonhartsberger.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

#include "ECUTTestRunner.h"

#ifdef ECUT_MAIN

#include "cppunit/TestResultCollector.h"
#include "cppunit/XmlOutputter.h"
#include "cppunit/TestSuite.h"
#include "cppunit/TestResult.h"
#include "cppunit/TestFailure.h"
#include "cppunit/SourceLine.h"
#include "cppunit/Exception.h"
#include "cppunit/extensions/TestFactoryRegistry.h"
#include "cppunit/extensions/TestDecorator.h"
#include "cppunit/ui/text/TestRunner.h"

#include <iostream>
#include <sstream>
#include <typeinfo>
#include <vector>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define MAX_HOSTNAME_SIZE		255

namespace ECUT {

/*
 * CppUnitServer protocol constants
 */
static const std::string TRACE_START = "%TRACES ";
static const std::string TRACE_END = "%TRACEE ";
static const std::string TEST_RUN_START = "%TESTC  ";
static const std::string TEST_START = "%TESTS  ";
static const std::string TEST_END = "%TESTE  ";
static const std::string TEST_ERROR = "%ERROR  ";
static const std::string TEST_FAILED = "%FAILED ";
static const std::string TEST_RUN_END = "%RUNTIME";
static const std::string TEST_STOPPED = "%TSTSTP ";
static const std::string TEST_TREE = "%TSTTREE";


EcutClient::EcutClient() {

	fTestResult = 0;
	fClientSocket = -1;
	fPort = 0;
	fKeepAlive = 0;
	fDebugMode = 0;

	fHost = (char *) malloc(MAX_HOSTNAME_SIZE);
	strncpy(fHost, "\0", MAX_HOSTNAME_SIZE);

	fTestName = "";
}

EcutClient::~EcutClient() {

	if (fHost != NULL) {
		free(fHost);
	}
}

int EcutClient::RunTestSuite(std::string &testSuiteName) {

	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Running client with a single test case..." << std::endl;
	}

	InstallListeners();

	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry(testSuiteName);
	CppUnit::Test *suite = registry.makeTest();

	int count = suite->countTestCases();
	NotifyTestRunStarted(count);

	// Stop when no tests are available
	if (count == 0) {
		NotifyTestRunEnded(0);
	}

	long startTime = CurrentTimeMillis();
	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Start sending test case tree ..."
				<< std::endl;
	}

	// Sends the test case tree to the server.
	SendTestTree(suite);

	int elapsedTime = CurrentTimeMillis() - startTime;
	if (fDebugMode) {
		std::cerr
				<< "EcutClient debug message: Done sending test case tree. Elapsed time is " << elapsedTime << "ms." << std::endl;
	}

	long testStartTime = CurrentTimeMillis();
	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Test start time is " << testStartTime << "ms." << std::endl;
	}

	suite->run(fTestResult);

	if (fTestResult == NULL || fTestResult->shouldStop()) {
		NotifyTestRunStopped(CurrentTimeMillis() - testStartTime);
	}
	else {
		NotifyTestRunEnded(CurrentTimeMillis() - testStartTime);
	}

	UninstallListeners();

	if (fTestResult != NULL) {
		fTestResult->stop();
		fTestResult = NULL;
	}

	return 0;
}

int EcutClient::RunSingleTest(std::string &testName) {

	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Running client with a single test case..." << std::endl;
	}

	InstallListeners();

	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	CppUnit::Test *suite = registry.makeTest();

	CppUnit::Test* test = suite->findTest(testName);
	if (test == NULL) {
		return -1;
	}

	suite = test;

	int count = suite->countTestCases();
	NotifyTestRunStarted(count);

	// Stop when no tests are available
	if (count == 0) {
		NotifyTestRunEnded(0);
	}

	long startTime = CurrentTimeMillis();
	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Start sending test case tree ..."
				<< std::endl;
	}

	// Sends the test case tree to the server.
	SendTestTree(suite);

	int elapsedTime = CurrentTimeMillis() - startTime;
	if (fDebugMode) {
		std::cerr
				<< "EcutClient debug message: Done sending test case tree. Elapsed time is " << elapsedTime << "ms." << std::endl;
	}

	long testStartTime = CurrentTimeMillis();
	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Test start time is " << testStartTime
				<< "ms." << std::endl;
	}

	suite->run(fTestResult);

	if (fTestResult == NULL || fTestResult->shouldStop()) {
		NotifyTestRunStopped(CurrentTimeMillis() - testStartTime);
	}
	else {
		NotifyTestRunEnded(CurrentTimeMillis() - testStartTime);
	}


	UninstallListeners();

	if (fTestResult != NULL) {
		fTestResult->stop();
		fTestResult = NULL;
	}

	return 0;
}


void EcutClient::RunAllTests() {

	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Running client..." << std::endl;
	}

	InstallListeners();

	RunTests();

	UninstallListeners();

	if (fTestResult != NULL) {
		fTestResult->stop();
		fTestResult = NULL;
	}
}

void EcutClient::SetDebugMode(int mode) {
	fDebugMode = mode;
}

void EcutClient::Init() {
	DefineHostName();
}

void EcutClient::DefineHostName() {

	// set fHost to hostname or localhost
	int ret = gethostname(fHost, MAX_HOSTNAME_SIZE);
	if (ret == -1) {
		strcpy(fHost, "localhost");
	}
}

int EcutClient::Connect(std::string hostAddress, int port) {

	fPort = port;

	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Trying to connect to " << fHost << ":" << fPort << std::endl;
	}

	fClientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (fClientSocket == -1) {
		std::cerr << "EcutClient debug message: Socket creation failed! error code: "	<< fClientSocket << std::endl;
		return -1;
	}

	if (!hostAddress.empty()) {
		int size = hostAddress.length();
		if (size > MAX_HOSTNAME_SIZE) {
			size = MAX_HOSTNAME_SIZE;
		}
		memset(fHost, 0, MAX_HOSTNAME_SIZE);
		strncpy(fHost, hostAddress.c_str(), size);
	}

	struct hostent *host = gethostbyname(fHost);
	if (host == NULL) {
		std::cerr << "EcutClient debug message: Cannot find host address for " << fHost
				<< "." << std::endl;
		fClientSocket = -1;
		return -1;
	}

	struct sockaddr_in name;
	memset((void *) &name, 0, sizeof(struct sockaddr_in));
	name.sin_family = AF_INET;
	name.sin_port = htons(fPort);

	memcpy(&name.sin_addr, host->h_addr, host->h_length);

	if (fDebugMode) {
		std::cerr
				<< "EcutClient debug message: Waiting for the JVM to listen ... (trying 3 times)"
				<< std::endl;
	}

	int ret = -1;
	int j = 0;
	while ((j < 3) && (ret == -1)) {
		ret = ::connect(fClientSocket, (struct sockaddr *) &name,
				sizeof(struct sockaddr_in));
		if (ret == -1) {
			if (fDebugMode) {
				std::cerr
						<< "EcutClient debug message: Connection request, waiting 1 second. "
						<< ((j - 3) * -1) << " times left." << std::endl;
			}
			PrivateSleep(1000);
			j++;
		}
	}
	if (ret == -1) {
		std::cerr << "EcutClient debug message: No connection established. Error code: "
				<< errno << std::endl;
		fClientSocket = -1;
		return -1;
	}

	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Connection established." << std::endl;
	}
	return 0;
}

void EcutClient::InstallListeners() {
	fTestResult = new CppUnit::TestResult();
	fTestResult->addListener(this);
}

void EcutClient::UninstallListeners() {
	fTestResult->removeListener(this);
}

void EcutClient::RunTests() {

	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	CppUnit::Test *suite = registry.makeTest();

	int count = suite->countTestCases();
	NotifyTestRunStarted(count);

	// Stop when no tests are available
	if (count == 0) {
		NotifyTestRunEnded(0);
	}

	long startTime = CurrentTimeMillis();
	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Start sending test case tree ..."
				<< std::endl;
	}

	// Sends the test case tree to the server.
	SendTestTree(suite);

	int elapsedTime = CurrentTimeMillis() - startTime;
	if (fDebugMode) {
		std::cerr
				<< "EcutClient debug message: Done sending test case tree. Elapsed time is " << elapsedTime << "ms." << std::endl;
	}

	long testStartTime = CurrentTimeMillis();
	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Test start time is " << testStartTime
				<< "ms." << std::endl;
	}

	suite->run(fTestResult);

	if (fTestResult == NULL || fTestResult->shouldStop()) {
		NotifyTestRunStopped(CurrentTimeMillis() - testStartTime);
	}
	else {
		NotifyTestRunEnded(CurrentTimeMillis() - testStartTime);
	}
}

void EcutClient::ShutDown() {

	if (fClientSocket != -1) {
		if (fDebugMode) {
			std::cerr << "EcutClient debug message: Closing connection to CppUnit sever at " << fHost << ":" << fPort << std::endl;
		}

		int result = close(fClientSocket);
		if (result != 0) {
			std::cerr << "EcutClient debug message: Close connection error: " << errno << std::endl;
		}

		fClientSocket = -1;
	}
}

void EcutClient::Stop() {
	if (fTestResult != NULL) {
		fTestResult->stop();
	}
}

void EcutClient::SendTestTree(CppUnit::Test *test) {

	if (typeid(*test) == typeid(CppUnit::TestDecorator)) {

		class TmpClass: public CppUnit::TestDecorator {

		public:
			TmpClass(Test *t) :
				CppUnit::TestDecorator(t) {
				// nothing to do
			}

			~TmpClass()
			{
				// nothing to do
			}

			CppUnit::Test *getTest() {
				return m_test;
			}
		};

		TmpClass *t = (TmpClass *) test;
		SendTestTree(t->getTest());
	}
	else if (typeid(*test) == typeid(CppUnit::TestSuite)) {
			CppUnit::TestSuite *suite = (CppUnit::TestSuite *) test;
			const std::vector<CppUnit::Test *> &x =	suite->getTests();

			std::ostringstream os;
			os << suite->getName() << ",true," << x.size();
			NotifyTestTreeEntry(os.str());

			for (unsigned int i = 0; i < x.size(); i++) {
				SendTestTree(x[i]);
			}
		}
		else {
			std::ostringstream os;
			os << test->getName() << ",false," << test->countTestCases();
			NotifyTestTreeEntry(os.str());
		}
}

void EcutClient::SendEcutMessage(std::string msg) {

	if (fClientSocket == -1) {
		return;
	}

	write(fClientSocket, msg.c_str(), msg.length());
	write(fClientSocket, "\n", 1);

	if (fDebugMode) {
		std::cerr << "EcutClient debug message: Sent message \"" << msg << "\"" << std::endl;
	}
}


void EcutClient::NotifyTestRunStarted(int testCount) {
	std::ostringstream os;
	os << TEST_RUN_START << testCount;
	SendEcutMessage(os.str());
}

void EcutClient::NotifyTestRunEnded(long elapsedTime) {
	std::ostringstream os;
	os << TEST_RUN_END << elapsedTime;
	SendEcutMessage(os.str());
}

void EcutClient::NotifyTestRunStopped(long elapsedTime) {
	std::ostringstream os;
	os << TEST_STOPPED << elapsedTime;
	SendEcutMessage(os.str());
}

void EcutClient::NotifyTestTreeEntry(std::string treeEntry) {
	SendEcutMessage(TEST_TREE + treeEntry);
}

void EcutClient::NotifyTestStarted(std::string testName) {
	SendEcutMessage(TEST_START + testName);
}

void EcutClient::NotifyTestEnded(std::string testName) {
	SendEcutMessage(TEST_END + testName);
}

void EcutClient::NotifyTestFailed(std::string status,
		std::string testName, std::string trace) {
	SendEcutMessage(status + testName);
	SendEcutMessage(TRACE_START);
	SendEcutMessage(trace);
	SendEcutMessage(TRACE_END);
}

// From TestListener
void EcutClient::startTest(CppUnit::Test *test) {
	NotifyTestStarted(test->getName());
}

// From TestListener
void EcutClient::addFailure(const CppUnit::TestFailure &failure) {
	if (failure.isError()) {
		NotifyTestFailed(TEST_ERROR, failure.failedTestName(),
				GetTrace(failure));
	}
	else {
		NotifyTestFailed(TEST_FAILED, failure.failedTestName(),
				GetTrace(failure));
	}
}

// From TestListener
void EcutClient::endTest(CppUnit::Test *test) {
	NotifyTestEnded(test->getName());
}

std::string EcutClient::GetTrace(
		const CppUnit::TestFailure &failure) {

	std::ostringstream os;

	CppUnit::Exception *e = failure.thrownException();
	if (e->sourceLine().lineNumber() != -1) {
		os << "File " << e->sourceLine().fileName() << ":" << e->sourceLine().lineNumber() << "\n";
	}
	else {
		os << "File Unknown:1\n";
	}
	/* TODO: expected, actual value implementation
	 if(typeid(*e)==typeid(CppUnit::NotEqualException))
	 {
	 CppUnit::NotEqualException *ne=(CppUnit::NotEqualException *)e;

	 os << "Expected Value: " << ne->expectedValue() << "\n";
	 os << "Actual Value: " << ne->expectedValue() << "\n";
	 os << "Additional Message: " << ne->additionalMessage() << "\n";
	 }
	 else
	 {
	 End		*/
	os << "Message: " << std::string(e->what()) << "\n";
	/*		} */

	return (os.str());
}

long EcutClient::CurrentTimeMillis() {
	struct timeval tv;
	gettimeofday(&tv, NULL);

	return ((long) (tv.tv_sec * 1000) + (tv.tv_usec / 1000));
}

void EcutClient::PrivateSleep(int millisecs) {
	struct timeval delta;
	delta.tv_sec = (millisecs * 1000L) / 1000000L;
	delta.tv_usec = (millisecs * 1000L) % 1000000L;
	select(0, NULL, NULL, NULL, &delta);
}

/*--------------------------------------------------------------------
 * Class CommandLine Implementation
 *--------------------------------------------------------------------*/
CommandLine::CommandLine() {
	fDebugMode = 0;
	fHostName = "localhost";
	fPort = -1;
	fTestName = "";
	fTestSuiteName = "";
	fPrintApplicationInfo = false;
}

int CommandLine::GetDebugMode() {
	return fDebugMode;
}

int CommandLine::GetPort() {
	return fPort;
}

std::string CommandLine::GetTestName() {
	return fTestName;
}

std::string CommandLine::GetTestSuiteName() {
	return fTestSuiteName;
}

std::string CommandLine::GetHostName() {
	return fHostName;
}

bool CommandLine::PrintApplicationInfo() {
	return fPrintApplicationInfo;
}

void CommandLine::Parse(int n, char* args[]) {

	// parse all arguments passed by args
	for (int i = 0; i < n; i++) {
		std::string arg(args[i]);

		// port option
		std::string portOption("-port=");
		int pos = arg.find(portOption);
		if (pos > -1) {
			std::string v = arg.substr(pos + portOption.length(), arg.length());
			fPort = atoi(v.c_str());
		}

		std::string hostName("-ecuthost=");
		pos = arg.find(hostName);
		if (pos > -1) {
			fHostName = arg.substr(pos + hostName.length(), arg.length());
		}

		// debug option
		std::string debugOption("-debug");
		pos = arg.find(debugOption);
		if (pos > -1) {
			fDebugMode = 1;
		}

		// test case or test suite specified
		std::string testCaseName("-test=");
		pos = arg.find(testCaseName);
		if (pos > -1) {
			fTestName = arg.substr(pos + testCaseName.length(), arg.length());
		}

		std::string testSuiteName("-testSuite=");
		pos = arg.find(testSuiteName);
		if (pos > -1) {
			fTestSuiteName = arg.substr(pos + testSuiteName.length(), arg.length());
		}

		std::string help("-help");
		pos = arg.find(help);
		fPrintApplicationInfo = pos > -1;
	}
}

} /*namespace ECUT*/

void PrintApplicationInfo() {

	std::cout << "\n\nECUT client V0.3.0.0" << std::endl;
	std::cout << "(C) Copyright by Gerhard H. Leonhartsberger, 2008" << std::endl << std::endl;

	std::cout << "Usage:"
			  << std::endl;
	std::cout << "EcutClient [-port] [-ecuthost] [-test|testSuite] [-debug] [-help]"
			  << std::endl
			  << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "The ECUT client is used in order to bridge CppUnit test results with ECUT server."
			  << std::endl
			  << "If EcutClient is invoked without -test or -testSuite all registered test cases are sent to the ECUT server."
			  << std::endl
			  << std::endl;
	std::cout << "-port       Indicates a port number (address of an application). If a port number is not specified -1 is used."
			  << std::endl
			  << std::endl;
	std::cout << "-ecuthost   Indicates the official name, an alias, or the Internet address of a remote host."
			  << std::endl
			  << std::endl;
	std::cout << "-test       Indicates a specific test case."
			  << std::endl
			  << std::endl;
	std::cout << "-testSuite  Indicates a specific test suite."
			  << std::endl
			  << std::endl;

	std::cout << "-debug	  Prints debug info during execution."
		      << std::endl
		      << std::endl;

	std::cout << "-help       Prints this info."
			  << std::endl
			  << std::endl;
}

/*!
 * This is the main routine. The TestRunnerClient is initialized and run. The
 * CppUnit tests are created, executed, and sent to the CppUnitServer.
 * If no connection to the CppUnitServer was established the CppUnit tests are
 * displayed on the console.
 *
 * @param n The number of arguments
 * @param args The char array of arguments. The following command line arguments are available:
 * <ul>
 * <li>-port=6485 Specifies the port on which to connect to the ECUT server.</li>
 * <li>-debug Enables debug output</li>
 * <li>-ecuthost= Indicates the official name, an alias, or the Internet address of a remote host.</li>
 * <li>-test=testClassName Indicates a specific test class.</li>
 * <li>-testSuite=testSuiteName Indicates a specific test suite.</li>
 * <li>-help Prints the info.</li>
 * </ul>
 *
 * @return <code>0</code> if the results of the CppUnit tests were sent to the
 *         CppUnitServer successfully.
 *         <code>-1</code> if a connection could not be established to the
 *         CppUnitServer.
 */
int ECUT_MAIN(int n, char *args[]) {

	// Configure logging output
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);

	ECUT::CommandLine commandLine;
	commandLine.Parse(n, args);

	if (commandLine.PrintApplicationInfo()) {
		PrintApplicationInfo();
		exit(0);
	}

	ECUT::EcutClient client;
	client.Init();
	client.SetDebugMode(commandLine.GetDebugMode());
	int success = client.Connect(commandLine.GetHostName(), commandLine.GetPort());
	if (success == -1) {
		std::cout << std::endl
				  << "Error:"
				  << std::endl;
		std::cout << "A connection to the ECUT server \""
				  << commandLine.GetHostName()
				  << "\" at port "
				  << commandLine.GetPort()
				  << " could not be established."
				  << "\nIf you are running ECUT client within Eclipse ensure that you use ECUT launch configuration "
				  << "in order to launch ECUT unit tests."
				  << std::endl
				  << std::endl;

		PrintApplicationInfo();

		exit(success);
	}

	std::string testName = commandLine.GetTestName();
	std::string testSuiteName = commandLine.GetTestSuiteName();
	if (testName != "") {
		client.RunSingleTest(testName);
	}
	else if (testSuiteName != "") {
		client.RunTestSuite(testSuiteName);
	}
	else {
		client.RunAllTests();
	}

	client.ShutDown();

	exit(success);
}

#endif /*ECUT_MAIN*/


/* EOF */
