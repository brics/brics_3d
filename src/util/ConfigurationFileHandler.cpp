/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#include "ConfigurationFileHandler.h"

#include <iostream>
#include <cstdlib>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace BRICS_3D {

ConfigurationFileHandler::ConfigurationFileHandler(const std::string filename) {
	this->filename = filename;

#ifdef BRICS_XERCES_ENABLE
	LOG(INFO) << "Opening configuration file " << filename;

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
			LOG(ERROR) << "XML document has " << errorsOccured << " error(s).";
		}

	} catch (const XMLException& e) {
		LOG(ERROR) << "An error occured during parsing\n   Message: " << e.getMessage();
		errorsOccured = true;
	} catch (const DOMException& e) {
		LOG(ERROR) << "A DOM error occured during parsing\n   DOMException code: " << e.code;
		errorsOccured = true;
	} catch (...) {
		LOG(ERROR) << "An error occured during parsing\n" << endl;
		errorsOccured = true;
	}

#else
	LOG(WARNING) <<  "Xerces is not enabled. No XML I/O support.";
	errorsOccured = true;
#endif

}

ConfigurationFileHandler::~ConfigurationFileHandler() {
#ifdef BRICS_XERCES_ENABLE
	delete parser;
	XMLPlatformUtils::Terminate();
#endif BRICS_XERCES_ENABLE
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
    bool attributeFound = false;

    DOMDocument* doc = parser->getDocument();
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
        LOG(WARNING) << "More than one " << algorithm << " found, taking the first one";
    } else if(root->getLength() < 1) {
    	LOG(WARNING) << "No algorithm called " << algorithm << " found.";
		return false;
    }

    current = root->item(0);
    DOMNamedNodeMap* attributesList =  current->getAttributes();
    attributeNode = attributesList->getNamedItem(attributeName);

    if (attributeNode != 0) {
    	*result = XMLString::transcode(attributeNode->getNodeValue());
        attributeFound = true;
    }

    XMLString::release(&algorithmName);
    XMLString::release(&attributeName);

    return attributeFound;
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
    bool attributeFound = false;

    DOMDocument* doc = parser->getDocument();
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
    	LOG(WARNING) << "More than one " << algorithm << " found, taking the first one";
    } else if(root->getLength() < 1) {
    	LOG(WARNING) << "No algorithm called " << algorithm << " found.";
		return false; //TODO release resources
    }

    current = root->item(0);
    DOMNamedNodeMap* attributesList =  current->getAttributes();
    attributeNode = attributesList->getNamedItem(attributeName);

    if (attributeNode != 0) {
    	tmpResult = XMLString::transcode(attributeNode->getNodeValue());
    	*result = atof(tmpResult.c_str());
        attributeFound = true;
    }

    XMLString::release(&algorithmName);
    XMLString::release(&attributeName);

    return attributeFound;
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
    bool attributeFound = false;

    DOMDocument* doc = parser->getDocument();
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
    	LOG(WARNING) << "More than one " << algorithm << " found, taking the first one";
    } else if(root->getLength() < 1) {
    	LOG(WARNING) << "No algorithm called " << algorithm << " found.";
		return false; //TODO release resouces
    }

    current = root->item(0);
    DOMNamedNodeMap* attributesList =  current->getAttributes();
    attributeNode = attributesList->getNamedItem(attributeName);

    if (attributeNode != 0) {
		    tmpResult = XMLString::transcode(attributeNode->getNodeValue());
		    *result = atoi(tmpResult.c_str());
		    attributeFound = true;
	}

    XMLString::release(&algorithmName);
    XMLString::release(&attributeName);

    return attributeFound;
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

    DOMDocument* doc = parser->getDocument();
    DOMNodeList* root = doc->getElementsByTagName(algorithmName);
    if (root->getLength() > 1) {
    	LOG(WARNING) << "More than one " << algorithm << " found, taking the first one";
    } else if(root->getLength() < 1) {
    	LOG(WARNING) << "No algorithm called " << algorithm << " found.";
		return false; //TODO release resouces
    }

    current = root->item(0);

    //search in children notes
	for (current = current->getFirstChild()->getNextSibling(); current!=NULL; current = current->getNextSibling()) {
		string nodeName = XMLString::transcode(current->getNodeName());
		if (nodeName.compare(subalgorithm) == 0) {
		    DOMNamedNodeMap* attributesList =  current->getAttributes();
		    attributeNode = attributesList->getNamedItem(implementationString);
		    if (attributeNode != 0) {
		    	*result = XMLString::transcode(attributeNode->getNodeValue());
		    	subAlgorithmFound = true;
		    	break; //take only first found
		    }
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

bool ConfigurationFileHandler::getErrorsOccured() {
	return errorsOccured;
}

}

/* EOF */
