/**
 * @file 
 * AttributeFinder.h
 *
 * @date: Oct 27, 2011
 * @author: sblume
 */

#ifndef ATTRIBUTEFINDER_H_
#define ATTRIBUTEFINDER_H_

#include "INodeVisitor.h"
#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "Attribute.h"

using std::vector;

namespace BRICS_3D {

namespace RSG {

class AttributeFinder: public INodeVisitor {
public:
	AttributeFinder();
	virtual ~AttributeFinder();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

	virtual void reset();

    vector<Node*> getMatchingNodes() const
    {
        return matchingNodes;
    }

    vector<Attribute> getQueryAttributes() const
    {
        return queryAttributes;
    }

    /**
     * Logical concatenation of multiple attributes is (for now) is AND
     * @param queryAttributes
     */
    void setQueryAttributes(vector<Attribute> queryAttributes)
    {
        this->queryAttributes = queryAttributes;
    }

protected:
	vector<Attribute> queryAttributes;
	vector<Node*> matchingNodes;
};

}

}

#endif /* ATTRIBUTEFINDER_H_ */

/* EOF */
