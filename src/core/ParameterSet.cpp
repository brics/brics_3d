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

#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include "core/ParameterSet.h"
#include "core/Logger.h"

namespace BRICS_3D {

std::string skipBlank(std::string s) {
	std::string::size_type spacesLeading = s.find_first_not_of(" ");
	if (spacesLeading != (std::string::size_type) -1)
		s = s.erase(0, spacesLeading);

	return s;
}

std::vector<double> getDoubleValuesFromString(std::string aCSVParameterString, unsigned int numberOfParameters) {
	std::vector<double> paramsAsDouble;
	double aNumber;
	boost::char_separator<char> sep(", ");
	boost::tokenizer<boost::char_separator<char> > tokens(aCSVParameterString, sep);
	BOOST_FOREACH(std::string t, tokens) {
		try {
			aNumber = boost::lexical_cast<double>( t.data() ); // convert string to double
			paramsAsDouble.push_back( aNumber );
		} catch ( const boost::bad_lexical_cast & ) {
			//unable to convert
			LOG(WARNING) << "Invalid parameter: unable to convert a string into double vector.";
		}
	}

	if (numberOfParameters > 0 && numberOfParameters != paramsAsDouble.size()) {
		LOG(WARNING) << "Invalid number of parameters.";
	}

	return paramsAsDouble;
}

std::string getStringFromDoubleValues(std::vector<double> values) {
	std::stringstream s;
	for (unsigned int i=0; i<values.size(); i++) {
		s << values[i];
		if (i < values.size() - 1)
			s << ",";
	}
	return s.str();
}


std::vector<std::string> getNamesFromString(std::string aCSVParameterString, unsigned int numberOfParameters) {
	std::vector<std::string> names;
	boost::char_separator<char> sep(", ");
	boost::tokenizer<boost::char_separator<char> > tokens(aCSVParameterString, sep);
	BOOST_FOREACH(std::string t, tokens)
		names.push_back(t.data());

	if (numberOfParameters > 0 && numberOfParameters != names.size())
		LOG(WARNING) << "Invalid number of parameters.";

	return names;
}


ParameterData::ParameterData() {
}

ParameterData::~ParameterData() {
}


ParameterSet::ParameterSet() {
};

ParameterSet::ParameterSet(std::string s) {
	boost::char_separator<char> sep(";");
	boost::tokenizer<boost::char_separator<char> > tokens(s, sep);
	BOOST_FOREACH(std::string t, tokens) {
		std::string::size_type posAssign = t.find_first_of("=");
		if (posAssign != (std::string::size_type) -1) {
			insert(Parameter(skipBlank(t.substr(0, posAssign)), skipBlank(t.substr(posAssign+1))));
		} else {
			insert(Parameter("", skipBlank(t)));
		};
	}
};

ParameterSet::ParameterSet(std::vector<std::string> sParams) {
	for (unsigned int i=0; i<sParams.size(); i++) {
		std::string s = sParams[i];
		std::string::size_type posAssign = s.find_first_of("=");
		if (posAssign != (std::string::size_type) -1) {
			insert(Parameter(skipBlank(s.substr(0, posAssign)), skipBlank(s.substr(posAssign+1))));
		} else {
			insert(Parameter("", skipBlank(s)));
		};
	}	
}

ParameterSet::~ParameterSet() {
	// release all data entries
	for (std::map<std::string, ParameterData*>::iterator it = dataEntries.begin(); it!=dataEntries.end(); it++)
		delete it->second;
};

bool ParameterSet::hasParam(std::string name) const {
	return find(name) != end();
}

bool ParameterSet::hasDouble(std::string name, double& value) const {
	const_iterator it = find(name);
	if (it == end() || it->second.empty())
		return false;

	try {
		value = boost::lexical_cast<double>(it->second);
	} catch(boost::bad_lexical_cast &) {
		LOG(WARNING) << "Could not parse value as double: " << it->second;
		return false;
	}

	return true;
}

bool ParameterSet::hasInt(std::string name, int& value) const {
	const_iterator it = find(name);
	if (it == end() || it->second.empty())
		return false;

	try {
		value = boost::lexical_cast<int>(it->second);
	} catch(boost::bad_lexical_cast &) {
		LOG(WARNING) << "Could not parse value as int: " << it->second;
		return false;
	}

	return true;
}

bool ParameterSet::hasUnsignedInt(std::string name, unsigned int& value) const {
	const_iterator it = find(name);
	if (it == end() || it->second.empty())
		return false;

	try {
		value = boost::lexical_cast<unsigned int>(it->second);
	} catch(boost::bad_lexical_cast &) {
		LOG(WARNING) << "Could not parse value as unsigned int: " << it->second;
		return false;
	}

	return true;
}

std::string ParameterSet::pack() {
	std::stringstream param;
	if (hasParam(""))
		param << operator[]("");
	for (const_iterator it = begin(); it != end(); it++)
		if (it->first != "" && it->second != "")
			param << "<" << it->first << ">" << it->second << "</" << it->first << ">";
			
	return param.str();
}

bool ParameterSet::unpack(const std::string& param) {

	// use very simple state machine for counting paranthesis
	enum { NEW, PAR1, PAR2, PAR3, PAR_END } state = NEW;
	unsigned int offset = 0;
	std::string name = "";
	std::string value = "";

	while (offset < param.length() && offset != -1) {
		int next = param.find_first_of("<>/", offset);
		if (next == -1)
			break;
		std::string sub = param.substr(offset, next-offset);
		switch (param[next]) {
			case '<':
				if (state == NEW) {
					if (!sub.empty())
						insert(Parameter("", sub));
					state = PAR1;
				} else if (state == PAR2) {
					value = sub;
					state = PAR3;
				} else {
					return false;
				}
				break;
			case '>':
				if (state == PAR1) {
					name = sub;
					state = PAR2;
				} else if (state == PAR_END) {
					state = NEW;
					insert(Parameter(name, value));
				} else {
					return false;
				}
				break;
			case '/':
				if (state == PAR3) {
					state = PAR_END;
				}
				break;
			default: ;
		}
		offset = next+1;
	}
	// special case that only name is included
	if (offset == 0 && param.length() > 0)
		insert(Parameter("", param));

	return true;
}


void ParameterSet::addData(std::string name, ParameterData* data) {
	// delete old entry, if existent
	std::map<std::string, ParameterData*>::iterator it = dataEntries.find(name);
	if (it != dataEntries.end())
		delete it->second;

	dataEntries.insert(std::pair<std::string, ParameterData*>(name, data));
}

bool ParameterSet::hasData(std::string name) {
	return dataEntries.find(name) != dataEntries.end();
}

ParameterData* ParameterSet::getData(std::string name) {
	std::map<std::string, ParameterData*>::iterator it = dataEntries.find(name);
	if (it != dataEntries.end())
		return it->second;

	return 0;
}

ParameterSet& operator<<(ParameterSet& pl, Parameter param) {
	pl.insert(param);
	return pl;
}

/*ParameterSet& operator<<(ParameterSet& pl, const std::string& name) {
	pl.insert(Parameter(name, ""));
	return pl;
}*/

ParameterSet operator<<(ParameterSet pl, const std::string& name) {
	pl.insert(Parameter(name, ""));
	return pl;
}


} // namespace BRICS_3D

