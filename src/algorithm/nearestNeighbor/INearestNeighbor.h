/**
 * @file 
 * INearestNeighbor.h
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#ifndef INEARESTNEIGHBOR_H_
#define INEARESTNEIGHBOR_H_

#include "core/PointCloud3D.h"
#include <vector>

using std::vector;

namespace BRICS_3D {


class INearestNeighbor {
public:
	INearestNeighbor(){};
	virtual ~INearestNeighbor(){};

	virtual void setData(vector< vector<float> >* data) = 0;
	virtual void setData(vector< vector<double> >* data) = 0;
	virtual void setData(PointCloud3D* data)= 0;

	virtual int findNearestNeigbor(vector<float>* query) = 0;
	virtual int findNearestNeigbor(vector<double>* query)= 0;
	virtual int findNearestNeigbor(Point3D* query)= 0;

    int getDimension() const
    {
        return dimension;
    }

    double getMaxDistance() const
    {
        return maxDistance;
    }

    void setMaxDistance(double maxDistance)
    {
        this->maxDistance = maxDistance;
    }

protected:

	int dimension;

	double maxDistance;
};

}  // namespace BRICS_3D

#endif /* INEARESTNEIGHBOR_H_ */

/* EOF */
