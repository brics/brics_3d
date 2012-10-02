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

#ifndef BRICS_3D_PARAMETERSET_H_
#define BRICS_3D_PARAMETERSET_H_

#include <map>
#include <string>
#include <vector>

namespace brics_3d {

//! A parameter has a name and a value, represented by std::string.
typedef std::pair<std::string, std::string> Parameter;

//! Empty data class to enable typesafe casting
struct ParameterData {
	ParameterData();
	virtual ~ParameterData();
};


//! A ParameterSet holds a number of parameters, representing (name/value) pairs
class ParameterSet: public std::map<std::string, std::string> {
public:
	//! Default constructor
	ParameterSet();
	
	//! Constructor that tries to parse string
	ParameterSet(std::string s);

	//! Constructor that tries to pass vector of strings
	ParameterSet(std::vector<std::string> sParams);
		
	//! Destructor.
	~ParameterSet();

	//! Returns true if a parameter with given name exists.
	bool hasParam(std::string name) const;

	//! Returns true if a parameter with given name exists, that can be casted to a double value.
	//! The double value is returned in \a value.
	bool hasDouble(std::string name, double& value) const;

	//! Returns true if a parameter with given name exists, that can be casted to an int value.
	//! The int value is returned in \a value.
	bool hasInt(std::string name, int& value) const;

	//! Returns true if a parameter with given name exists, that can be casted to an unsigned int value.
	//! The unsigned int value is returned in \a value.
	bool hasUnsignedInt(std::string name, unsigned int& value) const;

	void addData(std::string name, ParameterData* data);
	bool hasData(std::string);
	ParameterData* getData(std::string name);
	
	//! Packs this ParameterSet into one string.
	//! An XML notation is used to separate multiple parameters that are to be packed.
	std::string pack();
	
	//! Unpacks the given string into a whole ParameterSet, with all unpacked parameters entered into this set.
	//! The ParameterSet must possess an XML based form.
	//! Returns true if param could be succesfully unpacked.
	bool unpack(const std::string& param);
		
protected:
	std::map<std::string, ParameterData*> dataEntries;
};


//! Convenient function to add parameters to a ParameterList.
ParameterSet& operator<<(ParameterSet& pl, Parameter param);

//! Convenient function to add empty parameter to a ParameterList.
/*ParameterSet& operator<<(ParameterSet& pl, const std::string& name); */

//! Convenient function to add empty parameter to a ParameterList.
ParameterSet operator<<(ParameterSet pl, const std::string& name);


//! Parameter: numberOfParameters  Number of parameters. When zero, tries to take as many parameters as possible
std::vector<double> getDoubleValuesFromString(std::string aCSVParameterString, unsigned int numberOfParameters = 0);

//! Puts a double array into a comma separated string
std::string getStringFromDoubleValues(std::vector<double> values);

//! Parameter: numberOfParameters  Number of parameters. When zero, tries to take as many parameters as possible
std::vector<std::string> getNamesFromString(std::string aCSVParameterString, unsigned int numberOfParameters = 0);

//! Skip spaces in front of string s.
std::string skipBlank(std::string s);

} // namespace brics_3d

#endif // BRICS_3D_PARAMETERSET_H_
