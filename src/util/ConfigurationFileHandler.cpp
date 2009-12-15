/**
 * @file 
 * ConfigurationFileHandler.cpp
 *
 * @date: Dec 11, 2009
 * @author: sblume
 */

#include "ConfigurationFileHandler.h"

#include <iostream>
#include <cstdlib>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace BRICS_3D {

ConfigurationFileHandler::ConfigurationFileHandler(const std::string file) {
	this->filename = file;

#ifdef BRICS_XERCES_ENABLE
	cout << "INFO: opening configuration file " << filename << endl;

	// Initialize the XML4C2 system.
	try {
		XMLPlatformUtils::Initialize();
	}

	catch (const XMLException& e) {
		char *pMsg = XMLString::transcode(e.getMessage());
		cerr << "ERROR: An error occured during Xerces initialization.\n"
				<< "  Exception message:"
				<< pMsg;
		XMLString::release(&pMsg);
		return;
	}

//	const XMLCh* gEncodingName = 0;
//	const XMLFormatter* gFormatter = 0;

	//
	//  Create our parser, then attach an error handler to the parser.
	//  The parser will call back to methods of the ErrorHandler if it
	//  discovers errors during the course of parsing the XML document.
	//
	parser = new XercesDOMParser;
	parser->setValidationScheme(XercesDOMParser::Val_Auto);
	parser->setDoNamespaces(false);
	parser->setDoSchema(false);
	parser->setValidationSchemaFullChecking(false);
	parser->setCreateEntityReferenceNodes(false);

	//
	//  Parse the XML file, catching any XML exceptions that might propagate
	//  out of it.
	//
	errorsOccured = false;
	try {
		parser->parse(filename.c_str());
		int errorCount = parser->getErrorCount();
		if (errorCount > 0) {
			errorsOccured = true;
			cerr << "ERORR: XML document has " << errorsOccured << " errors." << endl;
		}

	} catch (const XMLException& e) {
		cerr << "ERROR: An error occured during parsing\n   Message: " << e.getMessage() << endl;
		errorsOccured = true;
	} catch (const DOMException& e) {
		cerr <<  "ERROR: A DOM error occured during parsing\n   DOMException code: " << e.code << endl;
		errorsOccured = true;
	} catch (...) {
		cerr <<  "ERROR: An error occured during parsing\n" << endl;
		errorsOccured = true;
	}

#else
	std::cout << "WARNING: Xerces is not enabled. No XML I/O support." << std::endl;
	errorsOccured = true;
#endif

}

ConfigurationFileHandler::~ConfigurationFileHandler() {
	delete parser;
	XMLPlatformUtils::Terminate();
}

bool ConfigurationFileHandler::getAttribute(std::string algorithm, std::string attribute, std::string* result) {
#ifdef BRICS_XERCES_ENABLE
	if (errorsOccured) {
		return false;
	}

    DOMNode* current = NULL;
    DOMNode* attributeNode = NULL;
    XMLCh* algorithmName = XMLString::transcode(algorithm.c_str());
    XMLCh* attributeName = XMLString::transcode(attribute.c_str());

    DOMDocument* doc = parser->getDocument(); //TODO replication
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
        cerr << "WARNING: More than one " << algorithm << " found, taking the first one" << endl;
    } else if(root->getLength() < 1) {
        cerr << "WARNING: No algorithm called " << algorithm << " found." << endl;
		return false;
    }

    current = root->item(0);
    DOMNamedNodeMap* attributesList =  current->getAttributes();
    attributeNode = attributesList->getNamedItem(attributeName);

    *result = XMLString::transcode(attributeNode->getNodeValue());

    XMLString::release(&algorithmName);
    XMLString::release(&attributeName);

    return true;
#else
    return false;
#endif
}





bool ConfigurationFileHandler::getAttribute(std::string algorithm, std::string attribute, double* result) {
#ifdef BRICS_XERCES_ENABLE
	if (errorsOccured) {
		return false;
	}

    DOMNode* current = NULL;
    DOMNode* attributeNode = NULL;
    XMLCh* algorithmName = XMLString::transcode(algorithm.c_str());
    XMLCh* attributeName = XMLString::transcode(attribute.c_str());
    string tmpResult;

    DOMDocument* doc = parser->getDocument(); //TODO replication
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
        cerr << "WARNING: More than one " << algorithm << " found, taking the first one" << endl;
    } else if(root->getLength() < 1) {
        cerr << "WARNING: No algorithm called " << algorithm << " found." << endl;
		return false;
    }

    current = root->item(0);
    DOMNamedNodeMap* attributesList =  current->getAttributes();
    attributeNode = attributesList->getNamedItem(attributeName);

    tmpResult = XMLString::transcode(attributeNode->getNodeValue());
    *result = atof(tmpResult.c_str());

    XMLString::release(&algorithmName);
    XMLString::release(&attributeName);

    return true;
#else
    return false;
#endif
}

bool ConfigurationFileHandler::getAttribute(std::string algorithm, std::string attribute, int* result) {
#ifdef BRICS_XERCES_ENABLE
	if (errorsOccured) {
		return false;
	}

    DOMNode* current = NULL;
    DOMNode* attributeNode = NULL;
    XMLCh* algorithmName = XMLString::transcode(algorithm.c_str());
    XMLCh* attributeName = XMLString::transcode(attribute.c_str());
    string tmpResult;

    DOMDocument* doc = parser->getDocument(); //TODO replication
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
        cerr << "WARNING: More than one " << algorithm << " found, taking the first one" << endl;
    } else if(root->getLength() < 1) {
        cerr << "WARNING: No algorithm called " << algorithm << " found." << endl;
		return false;
    }

    current = root->item(0);
    DOMNamedNodeMap* attributesList =  current->getAttributes();
    attributeNode = attributesList->getNamedItem(attributeName);

    tmpResult = XMLString::transcode(attributeNode->getNodeValue());
    *result = atoi(tmpResult.c_str());

    XMLString::release(&algorithmName);
    XMLString::release(&attributeName);

    return true;
#else
    return false;
#endif
}

bool ConfigurationFileHandler::getSubAlgorithm(std::string algorithm, std::string subalgorithm, std::string* result){
#ifdef BRICS_XERCES_ENABLE
	if (errorsOccured) {
		return false;
	}

    DOMNode* current = NULL;
    DOMNode* attributeNode = NULL;
    XMLCh* algorithmName = XMLString::transcode(algorithm.c_str());
    XMLCh* subAlgorithmName = XMLString::transcode(subalgorithm.c_str());
    XMLCh* implementationString = XMLString::transcode("implementation");
    bool subAlgorithmFound = false;

    DOMDocument* doc = parser->getDocument(); //TODO replication
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
        cerr << "WARNING: More than one " << algorithm << " found, taking the first one" << endl;
    } else if(root->getLength() < 1) {
        cerr << "WARNING: No algorithm called " << algorithm << " found." << endl;
		return false;
    }

    current = root->item(0);

    //search in children notes
	for (current = current->getFirstChild()->getNextSibling(); current!=NULL; current = current->getNextSibling()) {
		string nodeName = XMLString::transcode(current->getNodeName());
		if (nodeName.compare(subalgorithm) == 0) {
		    DOMNamedNodeMap* attributesList =  current->getAttributes();
		    attributeNode = attributesList->getNamedItem(implementationString);
		    *result = XMLString::transcode(attributeNode->getNodeValue());
			subAlgorithmFound = true;
			break; //take only first found
		}
	}

    XMLString::release(&algorithmName);
    XMLString::release(&subAlgorithmName);
    XMLString::release(&implementationString);

    return subAlgorithmFound;
#else
    return false;
#endif
}

}

/* EOF */
