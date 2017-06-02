/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2017, KU Leuven
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

#ifndef NODEHASHTRAVERSER_H_
#define NODEHASHTRAVERSER_H_

#include "INodeVisitor.h"
#include "Group.h"
#include "Connection.h"
#include "Node.h"
#include "Transform.h"
#include "UncertainTransform.h"
#include "GeometricNode.h"
#include "Shape.h"
#include <map>

namespace brics_3d {
namespace rsg {

/**
 * @brief A traverser to generate a hash for a sub graph.
 *
 * The traverser traverses along the containment parent-child graph (DAG) in order to generate a Merkle DAG.
 * The hash value is computed from (a) the ID of the node and (b) is Attributes and (c) the hash values of all child
 * nodes. If subgraphs are identical, the root node for each will have the same hash.
 *
 * The consideration of (a) a node ID can be skipped. This allows to calculate a hash for the the <b>general</b> structure,
 * rather than individuals with concrete UUIDs. This can be handy if an a priori template of a scene is loaded from file,
 * while new IDs will be automatically generated. Such loaded graphs will be only similar i.e. have the same hash value
 * for the root node when IDs are not taken into account.
 *
 * After the traversal every node can be queried to retrieve its hash value.
 */
class NodeHashTraverser : public INodeVisitor {
public:
	NodeHashTraverser();
	virtual ~NodeHashTraverser();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);
	virtual void visit(Connection* connection);

	/**
	 * @brief Resets the results of a traversal.
	 * @param useNodeIds @see useNodeIds
	 */
	void reset(bool useNodeIds = true);

	/**
	 * @brief Get hash for a Node identified by an ID.
	 * @note The traverser has to be executed first!
	 *       This function makes a cache look up - so it has to be filled by a traversal first.
	 * @param id Id if node.
	 * @return The hash.
	 */
	std::string getHashById(Id id);

	/// A null / NIL representation of a hash.
	/// Compare to this string in order to see if there was an error.
	static const string NIL;

	/**
	 * @brief Return all IDs and hashes in an JSON array.
	 * @note  The traverser has to be executed first!
	 *        This function makes a cache look up - so it has to be filled by a traversal first.
	 * @return A JSON array of the below format:
	 *
	 * @code
	 *
	 * {
	 *   "hashes": [
	 *     ["80f7b54d-0bd5-4371-a623-fea2c8d278fd", "d5a8d7bd45dd19b774693519011964d983414c1a87783c1d9e32ac403827d83d"],
	 *     ["2a0e89a8-e108-410c-90f4-09a7b4ff1f96", "4ca49a957cd3f5ff0666f37a77a88be61781fbe9a49e92558f49c1423caa6461"]
	 *   ]
	 * }
	 *
	 * @endcode
	 */
	std::string getJSON();

protected:

    /// Table that maps IDs to already generated hashes - so we do not have to recompute them.
    std::map<Id, std::string > hashLookUpTable;

    /// Iterator for idLookUpTable
    std::map<Id, std::string >::const_iterator hashIterator;

    /// If true, the node IDs will be considered for the hash. Default is true.
	/// Set it to false when you are interested in the general structure, rather than individuals with concrete UUIDs.
    bool useNodeIds;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* NODEHASHTRAVERSER_H_ */

/* EOF */
