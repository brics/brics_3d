/**
 * @file 
 * TestRunner.cpp
 *
 * @brief The main method in this file runs all CppUnit tests.
 *
 * @date: Oct 16, 2009
 * @author: sblume
 */

#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>


int main(int argc, char* argv[]) {

	/*see also: http://cppunit.sourceforge.net/doc/lastest/money_example.html#sec_running_test */
	// Get the top level suite from the registry
	CppUnit::Test *suite = CppUnit::TestFactoryRegistry::getRegistry().makeTest();

	// Adds the test to the list of test to run
	CppUnit::TextUi::TestRunner runner;
	runner.addTest(suite);

	// Change the default outputter to a compiler error format outputter
	runner.setOutputter(new CppUnit::CompilerOutputter(&runner.result(), std::cerr));

	// Run the tests.
	bool wasSucessful = runner.run();

	// Return error code 1 if the one of test failed.
	return wasSucessful ? 0 : 1;
}

/* EOF */
