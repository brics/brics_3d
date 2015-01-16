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

#ifndef RSG_CONNECTION_H
#define RSG_CONNECTION_H

#include "Node.h"
#include "TimeStamp.h"

namespace brics_3d {

namespace rsg {

/**
 * @brief A non-hierarchical connection between nodes.
 *
 * @ingroup sceneGraph
 */
class Connection : public Node {

  public:

	typedef boost::shared_ptr<Connection> ConnectionPtr;
	typedef boost::shared_ptr<Connection const> ConnectionConstPtr;

    Connection();

    virtual ~Connection();


    TimeStamp getStart() const
    {
        return start;
    }


    void setStart(TimeStamp start)
    {
        this->start = start;
    }

    TimeStamp getEnd() const
    {
        return end;
    }

    void setEnd(TimeStamp end)
    {
        this->end = end;
    }
    virtual void accept(INodeVisitor* visitor);

    /* Manipulation of sources */
    void addSourceNode(Node* node);
	void removeSourceNode(Node* node);
	Node* getSourceNode(unsigned int index);
    unsigned int getNumberOfSourceNodes() const;

    /* Manipulation of targets */
    void addTargetNode(Node* node);
	void removeTargetNode(Node* node);
	Node* getTargetNode(unsigned int index);
    unsigned int getNumberOfTargetNodes() const;


private:


	/// List of pointers to the source Nodes.
	vector<Node*> sourceNodes; //these are rather weak references; strong references are part of the hierarchy

	/// List of pointers to the source Nodes.
	vector<Node*> targetNodes; //these are rather weak references; stron references are part of the hierarchy

    /// Life cycle start
    TimeStamp start;

    /// Life cycle end
    TimeStamp end;

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

