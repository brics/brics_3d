/**
 * @file 
 * ECUTTestRunner.h
 *
 * @date: Oct 22, 2009
 * @author: sblume
 */

#ifndef ECUTTESTRUNNER_H_
#define ECUTTESTRUNNER_H_

/*******************************************************************************
 * Copyright (c) 2008 Gerhard Leonhartsberger.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

/*!
 * /file EcutClient.h
 *
 * /brief The ECUT client is the C++ part communicating with the ECUT server.
 */
#ifdef ECUT_MAIN

#include "cppunit/TestListener.h"

#include <vector>
#include <string>

#include "brics_3d/core/Logger.h" // control the BRICS logger module

namespace ECUT {

/*!
 * Class <code>EcutClient</code> handles the network connection to the
 * <code>EcutServer</code> and executes all registered CppUnit tests.
 * <p>
 * The meta data of the CppUnit test cases and the test results are sent to the
 * <code>EcutServer</code> for displaying. The ECUT protocol is used.
 * </p>
 *
 * The following code snippet shows a typical usage scenario:
 *
 * /code
 * EcutClient client;
 * client.Init();
 * client.Connect("localhost", 1278);
 *
 * client.RunAllTests();
 * client.ShutDown();
 * /endcode
 *
 * <p>
 * For debugging purposes class <code>EcutClient</code> displays debug
 * messages on <code>std.cerr</code>. The debug modus is activated by specifying
 * <code>debug</code> command line option. The command line is parsed by method
 * <code>Init()</code>.
 * </p>
 * <p>
 * Note: This class is not intended to be subclassed by clients.
 * </p>
 *
 * @author gerhard@leonhartsberger.org
 * @version $Revision: 1.9 $
 */
class EcutClient: public CppUnit::TestListener {
private:
	CppUnit::TestResult *fTestResult;

	int fClientSocket;
	char *fHost;
	int fKeepAlive;

	// command line arguments values
	int fPort;
	int fDebugMode;
	std::string fTestName;

public:
	EcutClient();
	virtual ~EcutClient();

	/*! /brief Initializes the <code>EcutClient</code>.
	 */
	void Init();

	/*!
	 * A connection to the ECUT server is established.
	 *
	 * @param hostAddress The host address. A name or an Internet address.
	 * @param port The application address.
	 *
	 * @return The return value is <code>0</code> when the ECUT server was connected successfully
	 * otherwise the return value is <code>-1</code>.
	 */
	int Connect(std::string hostAddress, int port);

	/*!
	 * Runs the ECUT client.
	 * The test results are sent to the ECUT server.
	 */
	void RunAllTests();

	int RunSingleTest(std::string &testName);

	int RunTestSuite(std::string &testSuiteName);

	/*!
	 * Stops processing <code>CppUnit</code> from executing tests.
	 */
	void Stop();

	/*!
	 * Shuts down the ECUT client by closing connection to ECUT server and releasing system resources.
	 */
	void ShutDown();

	void SetDebugMode(int mode);

	// TODO: check if it is really necessary to make the following methods public?
	// Methods are not part of the public API
	/*!
	 * Method defined in <code>CppUnit::TestListener</code>.
	 */
	void startTest(CppUnit::Test *test);

	/*!
	 * Method defined in <code>CppUnit::TestListener</code>.
	 */
	void addFailure(const CppUnit::TestFailure &failure);

	/*!
	 * Method defined in <code>CppUnit::TestListener</code>.
	 */
	void endTest(CppUnit::Test *test);

private:
	void RunTests();

	// utility methods
	void ParseCommandLine(int n, char *args[]);
	void DefineHostName();
	void InstallListeners();
	void UninstallListeners();

	/*!
	 * Sends the given test to the ECUT result view.
	 * <p>
	 * In case of test is of type CppUnit::Test the following protocol is sent:
	 * <code>TSTTREE name, false, testCaseCount</code>
	 * </p>
	 * <p>
	 * In case of test is of type /code CppUnit::TestSuite /endcode the following protocol is sent:
	 * /code TSTTREE name, true, testCount /endcode
	 * </p>
	 * /param test the CppUnit test
	 */
	void SendTestTree(CppUnit::Test *test);
	void SendEcutMessage(std::string msg);

	// Notification methods
	void NotifyTestRunStarted(int testCount);
	void NotifyTestRunEnded(long elapsedTime);
	void NotifyTestRunStopped(long elapsedTime);
	void NotifyTestTreeEntry(std::string treeEntry);
	void NotifyTestStarted(std::string testName);
	void NotifyTestEnded(std::string testName);
	void NotifyTestFailed(std::string status, std::string testName,
			std::string trace);

	std::string GetTrace(const CppUnit::TestFailure &failure);
	long CurrentTimeMillis();
	void PrivateSleep(int millisecs);
};

class CommandLine {
private:
	int fPort;
	int fDebugMode;
	std::string fTestName;
	std::string fTestSuiteName;
	std::string fHostName;
	bool fPrintApplicationInfo;

public:
	CommandLine();

	int GetPort();
	int GetDebugMode();
	std::string GetTestName();
	std::string GetTestSuiteName();
	std::string GetHostName();
	bool PrintApplicationInfo();

	void Parse(int n, char* args[]);
};

} /*namespace ECUT*/

#endif /*ECUT_MAIN*/


#endif /* ECUTTESTRUNNER_H_ */

/* EOF */
