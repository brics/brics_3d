/**
 * @file 
 * IOctreePartition.h
 *
 * @date: Mar 22, 2010
 * @author: sblume
 */

#ifndef IOCTREEPARTITION_H_
#define IOCTREEPARTITION_H_

#include "core/PointCloud3D.h"
#include <vector>

namespace BRICS_3D {

class IOctreePartition {
public:
	IOctreePartition(){};
	virtual ~IOctreePartition(){};

	void partitionPointCloud(PointCloud3D* pointCloud, std::vector <PointCloud3D*> pointCloudCells) = 0;
};

}

#endif /* IOCTREEPARTITION_H_ */

/* EOF */
