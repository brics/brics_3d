/**
 * @file 
 * ConfigurationFileHandler.h
 *
 * @date: Dec 11, 2009
 * @author: sblume
 */

#ifndef CONFIGURATIONFILEHANDLER_H_
#define CONFIGURATIONFILEHANDLER_H_

#define BRICS_XERCES_ENABLE //TOD remove this line (only for eclipse)


#include <string>


#ifdef BRICS_XERCES_ENABLE
#include <xercesc/util/PlatformUtils.hpp> //TODO check if all headers are necessary
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLString.hpp>
//#if defined(XERCES_NEW_IOSTREAMS)
//#else
//#include <iostream.h>
//#endif
XERCES_CPP_NAMESPACE_USE
#endif

namespace BRICS_3D {

class ConfigurationFileHandler {
public:

	ConfigurationFileHandler(const std::string file);

	virtual ~ConfigurationFileHandler();

	bool getAttribute(std::string algorithm, std::string attribute, std::string* result);
	bool getAttribute(std::string algorithm, std::string attribute, int* result);
	bool getAttribute(std::string algorithm, std::string attribute, double* result);
	bool getSubAlgorithm(std::string algorithm, std::string subalgorithm, std::string* result);

private:

	std::string filename;

	bool errorsOccured;

#ifdef BRICS_XERCES_ENABLE
	XercesDOMParser* parser;
#endif

};

}

#endif /* CONFIGURATIONFILEHANDLER_H_ */

/* EOF */
