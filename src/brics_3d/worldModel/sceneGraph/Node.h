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

#ifndef RSG_NODE_H
#define RSG_NODE_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "Id.h"
#include "Attribute.h"
#include "INodeVisitor.h"

using std::vector;

namespace brics_3d {

/**
 * @brief RobotSceneGraph: A scene graph for robot 3D world modeling.
 * @ingroup sceneGraph
 */
namespace rsg {

class Group;


/**
 *  @brief A node in the robot scenegraph.
 */
class Node {

public:

	typedef boost::shared_ptr<Node> NodePtr;
	typedef boost::shared_ptr<Node const> NodeConstPtr;

	typedef boost::weak_ptr<Node> NodeWeakPtr;
	typedef boost::weak_ptr<Node const> NodeWeakConstPtr;

//	typedef vector< Node::NodePtr > NodePath;
	typedef vector< Node* > NodePath;
	typedef vector< NodePath > NodePathList;

	Node();

	virtual ~Node();

	vector<Attribute> getAttributes();
	Id getId() const;

	void setAttributes(vector<Attribute> attributes);
	void setId(Id id);


	vector<Node*> getParents(); //TODO dangerous as it reveals the pointers?!?

	Node* getParent(unsigned int index);

    unsigned int getNumberOfParents() const;

    virtual void accept(INodeVisitor* visitor);


private:

	void addParent(Node* node);
	void removeParent(Node* node);
	friend class brics_3d::rsg::Group; //only this one will be allowed to add parent-child relations

	/// Unique ID that will help to identify a certain node.
	Id id;

	/// List of attributes for each node. Can be used to attach (semantic) tags.
	vector<Attribute> attributes;

	/// List of pointers to the parent Nodes.
	vector<Node*> parents; //these are rather weak references to prevent cyclic strong pointers

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

