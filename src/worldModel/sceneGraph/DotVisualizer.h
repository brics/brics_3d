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

#ifndef RSG_DOTVISUALIZER_H_
#define RSG_DOTVISUALIZER_H_

#include <iostream>
#include <fstream>

#include "ISceneGraphUpdateObserver.h"
#include "DotGraphGenerator.h"
#include "SceneGraphFacade.h"

namespace brics_3d {

namespace rsg {

/**
 * @brief Observer that will create a print out on every update.
 * @note You need to have the dot executable installed on your system.
 * @ingroup sceneGraph
 */
class DotVisualizer : public ISceneGraphUpdateObserver {
public:
	DotVisualizer(brics_3d::rsg::SceneGraphFacade* scene);
	virtual ~DotVisualizer();

	/* implemetntations of observer interface */
	bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes);
	bool setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
	bool deleteNode(unsigned int id);
	bool addParent(unsigned int id, unsigned int parentId);

    bool getKeepHistory() const
    {
        return keepHistory;
    }

    void setKeepHistory(bool keepHistory)
    {
        this->keepHistory = keepHistory;
    }

private:
	///Handle the to the scene to be observed
	brics_3d::rsg::SceneGraphFacade* scene;

	/// Traverser to produce a dot file
	brics_3d::rsg::DotGraphGenerator graphPrinter;

	std::ofstream output;

	/**
	 * @brief Print the current graph into a file.
	 */
	void printGraph();

	/// If true all printaout will be saved to disk with a running number as part of the file name.
	bool keepHistory;

	unsigned int counter;
};

}

}

#endif /* RSG_DOTVISUALIZER_H_ */

/* EOF */
