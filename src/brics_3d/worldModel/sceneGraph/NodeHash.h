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

#ifndef NODEHASH_H_
#define NODEHASH_H_

#include "Node.h"
#include "Group.h"

namespace brics_3d {
namespace rsg {

/**
 * @brief Helper class for generation of a hash sum for a Node.
 */
class NodeHash {
public:
	NodeHash();
	virtual ~NodeHash();

	static std::string idToHash(Id id);
	static std::string attributesToHash(std::vector<Attribute> attributes);
	static std::string sortStringsAndHash(std::vector<std::string> strings);

	/**
	 * @brief Get a SHA256 hash from a Node.
	 * @param[in] node Pointer to node.
	 * @param[in] useNodeId If true, the node id will be considered for the hash. Default is true.
	 *            Set it to false when you are interested in the general structure, rather than individuals with concrete UUIDs.
	 * @return A string representing the hash.
	 */
	static std::string nodeToHash(Node* node, bool useNodeId = true);

};

} /* namespace rsg */

} /* namespace brics_3d */

#endif /* NODEHASH_H_ */

/* EOF */
