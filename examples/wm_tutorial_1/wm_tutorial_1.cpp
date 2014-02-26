/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
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

/* BRICS_3D includes */
#include <brics_3d/worldModel/WorldModel.h>

using namespace brics_3d::rsg;

/*
 * This tutorial covers basic creation and deletion of nodes for the robot scene graph.
 */
int main(int argc, char **argv) {

	/* Create a world model handle */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

	/* Graph structure:
	 *                 root
	 */

	/*
	 * We are going to add a new "group" as child node to the root node:
	 */
	Id rootId = wm->getRootNodeId(); // Get the "root" node of the world model.
	Id group1Id = 0; // This in an output parameter. The world model itself will
	                 // generated these IDs.
	std::vector<Attribute> attributes; // Attributes are essentially
                                                      // key value pair that can
                                                      // be attached to any node.
	attributes.push_back(Attribute("name","my_group1")); // Just an (arbitrary) example for an attribute.
	wm->scene.addGroup(rootId, group1Id, attributes); // Add a new group node with
	                                                  // rootId as parent. The ID of the
	                                                  // new group is stored in the
	                                                  // group1 variable for later use.

	/* Graph structure:
	 *                 root
	 *                   |
	 *        -----------+
	 *        |
	 *      group1
	 */

	/*
	 * We add a second group relative to the root node:
	 */
	Id group2Id;
	attributes.clear(); // We re-use the attributes vector, but we have to clean it
	                    // unless we want to previous attributes to the new group.
	attributes.push_back(Attribute("name","my_group2"));
	wm->scene.addGroup(wm->getRootNodeId(), group2Id, attributes);


	/* Graph structure:
	 *                 root
	 *                   |
	 *        -----------+
	 *        |          |
	 *      group1     group2
	 *
	 */

	/*
	 * We add a new node that is _relative_ to the previously generated group1 node.
	 */

	Id node1Id;
	attributes.clear();
	attributes.push_back(Attribute("name","my_node1"));
	wm->scene.addNode(group1Id, node1Id, attributes); // Note that group1Id is the parent.

	/* Graph structure:
	 *                 root
	 *                   |
	 *        -----------+
	 *        |          |
	 *      group1     group2
	 *        |
	 *      node1
	 */

	/*
	 * Finally we delete the group1 node. This will also case node1 to be deleted,
	 * because a node is deleted when it has no parents any more (except for the
	 * root node).
	 * The implementation actually makes use of (boost) shared pointers to realize
	 * the automatic deletion of nodes. Thus, managing nodes in the graph also means
	 * to manage memory.
	 */
	wm->scene.deleteNode(group2Id);


	/* Graph structure:
	 *                 root
	 *                   |
	 *                   +
	 *                   |
	 *                group2
	 *
	 *
	 */


	/* Clean up */

	/*
	 * Deletion of the world model handle will internally delete the root node.
	 * That means all node in the graph will be automatically deleted in a
	 * cascaded fashion and the resources are freed.
	 */
	delete wm;
}

/* EOF */
