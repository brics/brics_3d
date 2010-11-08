/**
 * @file 
 * TestRunner.cpp
 *
 * @brief The main method in this file runs all CppUnit tests.
 *
 * @date: Oct 16, 2009
 * @author: sblume
 */

#include <fstream>
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/XmlOutputter.h>


int main(int argc, char* argv[]) {

	/*see also: http://cppunit.sourceforge.net/doc/lastest/money_example.html#sec_running_test */
	// Get the top level suite from the registry
	CppUnit::Test *suite = CppUnit::TestFactoryRegistry::getRegistry().makeTest();

	// Adds the test to the list of test to run
	CppUnit::TextUi::TestRunner runner;
	runner.addTest(suite);

	// Check the set parameters.
	bool useXmlOutput = false;
	if (argc == 2) {
		std::string parameter = argv[1];
		if (parameter.compare("--report-xml") == 0) {
			useXmlOutput = true;
		}
	}

	// Decide which output format to take.
	std::ofstream outputStream;
	if (useXmlOutput) {
		// Change the default outputter to a XML outputter
		outputStream.open("unit_test_results.xml");
		runner.setOutputter(new CppUnit::XmlOutputter(&runner.result(), outputStream, std::string("ISO-8859-1")));
	} else {
		// Change the default outputter to a compiler error format outputter
		runner.setOutputter(new CppUnit::CompilerOutputter(&runner.result(), std::cerr));
	}

	// Run the tests.
	bool wasSucessful = runner.run();

	// Return error code 1 if the one of test failed.
	return wasSucessful ? 0 : 1;
}

/* EOF */
