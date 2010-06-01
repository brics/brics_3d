/**
 * @file 
 * IOctreeSetup.h
 *
 * @date: Feb 18, 2010
 * @author: sblume
 */

#ifndef IOCTREESETUP_H_
#define IOCTREESETUP_H_

/*
 *
 */
namespace BRICS_3D {

class IOctreeSetup {
public:
	IOctreeSetup(){};
	virtual ~IOctreeSetup(){};

	/**
	 *
	 * NOTE: the current implementation interprets voxelSize as the from the center to one of the cube faces.
	 */
	virtual void setVoxelSize(double voxelSize) = 0;

	virtual double getVoxelSize() = 0;
};

}

#endif /* IOCTREESETUP_H_ */

/* EOF */
