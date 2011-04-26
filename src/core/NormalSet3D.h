/**
 * @file 
 * NormalSet3D.h
 *
 * @date: Apr 26, 2011
 * @author: sblume
 */

#ifndef NORMALSET3D_H_
#define NORMALSET3D_H_

#include "Normal3D.h"
#include <vector>

namespace BRICS_3D {

/**
 * @brief Class to represent a set of 3D normals.
 */
class NormalSet3D {

public:

	/**
	 * @brief Standard constructor
	 */
	NormalSet3D();

	/**
	 * @brief Standard destructor
	 */
	virtual ~NormalSet3D();

	/**
	 * @brief Add a normal to the the normal set
	 * @param point Normal that will be added
	 */
	void addNormal(Normal3D normal);

	/**
	 * @brief Get the pointer to the normals
	 * @return Pointer to the normals
	 */
    std::vector<Normal3D>* getNormals();

    /**
     * @brief Get the number of normals in the normal set
     * @return Size of normals set (= number of stored normals)
     */
    unsigned int getSize();

protected:

	std::vector<Normal3D>* normals;
};

}

#endif /* NORMALSET3D_H_ */

/* EOF */
