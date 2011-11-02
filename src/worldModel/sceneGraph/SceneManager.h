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

#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H

#include "ISceneGraphQuery.h"
#include "ISceneGraphUpdate.h"
#include "IIdGenerator.h"
#include "Group.h"
#include "Node.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "Shape.h"

#include <map>
#include <boost/weak_ptr.hpp>
using std::map;


//namespace BRICS_3D { namespace RSG { class Attribute; }  }
//namespace BRICS_3D { class WorldModel; }
//namespace BRICS_3D { namespace RSG { class IIdGenerator; }  }

namespace BRICS_3D {

namespace RSG {

/**
 * @brief The central handle to create and maintain a robot scenegraph. It holds the root node of the scene graph.
 *
 * The SceneManager takes care (maintains consistency) of mapping between IDs and internal pointers.
 * The implemented interfaces allow to create and maintain a scengraph bases on the node IDs only.
 *
 */
class SceneManager : public ISceneGraphQuery, public ISceneGraphUpdate {

  public:
	SceneManager();

    SceneManager(IIdGenerator* idGenerator);

    virtual ~SceneManager();

    unsigned int getRootId();

    /* Implemented query interfaces */
//    void getNodes(vector<Attribute> attributes, vector<unsigned int>* ids);
    bool getNodeAttributes(unsigned int id, vector<Attribute>& attributes);
    bool getNodeParents(unsigned int id, vector<unsigned int>& parentIds);
    bool getGroupChildren(unsigned int id, vector<unsigned int>& childIds);
    bool getTransform(unsigned int id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform);
//getGlobalTransformForNode
    bool getGeometry(unsigned int id, Shape::ShapePtr& shape, TimeStamp& timeStamp);

    /* Implemented update interfaces */
    bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes);
    bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes);
    bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
    bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp);
    bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes);
//    void setTransform(unsigned int id, BRICS_3D::IHomogeneousMatrix44* transformation, TimeStamp timeStamp);
//    void deleteNode(unsigned int id);
    bool addParent(unsigned int id, unsigned int parentId);

  private:

    void initialize();

    Node::NodeWeakPtr findNodeRecerence(unsigned int id);

    void findSceneNodes(const Attribute & attributes, Node & nodeReferences);


    Group::GroupPtr rootNode; // root of all evil...

    map<unsigned int, Node::NodeWeakPtr > idLookUpTable;
    map<unsigned int, Node::NodeWeakPtr >::const_iterator nodeIterator;

    boost::weak_ptr<Node> testPtr;
    IIdGenerator* idGenerator;




};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

