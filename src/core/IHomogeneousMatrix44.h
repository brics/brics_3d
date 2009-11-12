/**
 * @file 
 * IHomogeneousMatrix44.h
 *
 * @date: Oct 23, 2009
 * @author: sblume
 */

#ifndef IHOMOGENEOUSMATRIX44_H_
#define IHOMOGENEOUSMATRIX44_H_


namespace BRICS_3D {

/**
 * @brief An abstract class interface to represent a homogeneous 4x4 matrix.
 *
 * The intention of this class is that the Point3D representation
 * does not depend on the implementation of the transformation matrix.
 * It only depends on this interface.
 *
 */
class IHomogeneousMatrix44 {
public:

	/**
	 * @brief Default constructor
	 */
	IHomogeneousMatrix44(){};

	/**
	 * @brief Copy raw data to input buffer (array)
	 *
	 * <b> Important: </b> the 16 values are stored in column-row order!
	 * So the layout is: <br>
	 * 0 &nbsp;&nbsp;4 &nbsp;&nbsp;8 &nbsp;&nbsp;&nbsp;12 <br>
	 * 1 &nbsp;&nbsp;5 &nbsp;&nbsp;9 &nbsp;&nbsp;&nbsp;13 <br>
	 * 2 &nbsp;&nbsp;6 &nbsp;&nbsp;10 &nbsp;14 <br>
	 * 3 &nbsp;&nbsp;7 &nbsp;&nbsp;11 &nbsp;15 <br>
	 *
	 *
	 * @param[out] matrixBuffer Pointer to buffer where the matrix is stored.
	 * Buffer is assumed to have a size of at least 16 double values.
	 */
	virtual void getRawData(double *matrixBuffer) = 0;

	/**
	 * @brief Standard destructor
	 */
	virtual ~IHomogeneousMatrix44(){};
};

}

#endif /* IHOMOGENEOUSMATRIX44_H_ */

/* EOF */