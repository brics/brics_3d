/******************************************************************************
* Copyright (c) 2011
* GPS GmbH
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of GPS GmbH nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
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

