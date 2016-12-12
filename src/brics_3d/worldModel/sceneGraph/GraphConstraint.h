/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#ifndef GRAPHCONSTRAINT_H_
#define GRAPHCONSTRAINT_H_

#include <string>
#include <brics_3d/core/Units.h>
#include <brics_3d/worldModel/sceneGraph/Id.h>

namespace brics_3d {
namespace rsg {

/**
 * @brief Represents a single constraint on graph primitives e.g. for sending updates.
 *
 * Constraints are use to model a particular interest in subsets of the stored in data.
 * Typically the updated send from one World Model Agent to another one are constrained
 * by a a set of these Constraints.
 *
 */
class GraphConstraint {

public:

	/// optional
	enum Action {
		UNDEFINED_ACTION = 0,
		SEND = 1,
		RECEIVE = 2
	};

	/// required
	enum Qualifier {
		UNDEFINED_QUALIFIER = 0,
		NO = 1,
		ONLY = 2
	};

	/// required
	enum Type {
		UNDEFINED_TYPE = 0,
		Atom = 1,
		Node = 2,
		GeometricNode = 3,
		Sphere = 4,
		Cylinder = 5,
		Box = 6,
		PointCloud = 7,
		Mesh = 8,
		Transform = 9,
		Group = 10,
		Connection = 11,
		TransformUpdate = 12,
		TYPE_NR_ITEMS = 13
	};

	/// optional
	enum NodeConstraint {
		UNDEFINED_NODE_CONSTRAINT = 0,
		NONE = 1,
		FREQUENCY = 2,
		DISTANCE = 3,
		LOD = 4,
		CONTAINMENT = 5,
		CONTEXT =6
	};

	/// required for FREQUENCY, DISTANCE or LOD constraints
	enum ComparisionOperator {
		UNDEFINED_OPERATOR = 0,
		LT = 1,  /// <=
		EQ = 2,  /// ==
		GT = 3   /// >=
	};

	GraphConstraint();
	virtual ~GraphConstraint();

	Action action;						/// optional
	Qualifier qualifier; 				/// required
	Type type;							/// required
	NodeConstraint nodeConstraint;		/// optional
	ComparisionOperator comparision;	///	required for FREQUENCY, DISTANCE or LOD nodeConstraint
	double value;						///	required for FREQUENCY, DISTANCE or LOD nodeConstraint
	Units::FrequencyUnit freqUnit;		/// required for FREQUENCY nodeConstraint
	Units::DistanceUnit  distUnit;		/// required for DISTANCE nodeConstraint
	Id node;							/// required for DISTANCE and CONTAINMENT nodeConstraint
	bool isMe;							/// In case node is set the "me" alias. This is synonym with with getRootNode()
	std::string context;				/// required for CONTEXT nodeConstraint

	/**
	 * @brief Parse a model.
	 *
	 * E.g. "send no PointClouds with freq < 5 Hz". Please note, the delimiter is exactly one white space.
	 * Do not add trailing white spaces.
	 *
	 * @param constriaconstraintModel Constraint represented as model. It has to obey the below syntax:
	 * @code
	 * ^(send|receive) \
	 * (no|only) \
	 * (Atom|Node|GeometricNode|Sphere|Cylinder|Boxe|PointCloud|Meshe|Transform|Group|Connection)s\
	 * (( with freq (<|=|>) [0-9]+ (Hz|kHz))|\
	 *  ( with lod (<|=|>) [0-9]+)|\
	 *  ( with dist (<|=|>) [0-9]+ (mm|cm|m|km) from (me|([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))|\
	 *  ( from context [a-zA-Z0-9]+)|\
	 *  ( contained in ([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))
	 * ))?$
	 * @endcode
	 * @return True if parsing created a valid constraint
	 */
	bool parse(std::string constraintModel);

	/**
	 * @brief Checks if this constraint is valid.
	 * @return True if it is valid.
	 */
	bool validate();

private:
	void setDefaultValues();
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* GRAPHCONSTRAINT_H_ */

/* EOF */
