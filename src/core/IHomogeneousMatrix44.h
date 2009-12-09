/**
 * @file 
 * IHomogeneousMatrix44.h
 *
 * @date: Oct 23, 2009
 * @author: sblume
 */

#ifndef IHOMOGENEOUSMATRIX44_H_
#define IHOMOGENEOUSMATRIX44_H_

#include <iostream>
using std::ostream;
using std::istream;

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
	 * @brief Standard destructor
	 */
	virtual ~IHomogeneousMatrix44(){};

	/**
	 * @brief Returns a pointer to a read-only buffer where the raw data of the matrix is stored.
	 *
	 * <b> Important: </b> the 16 values are stored in column-row order!
	 * So the layout is: <br>
	 * 0 &nbsp;&nbsp;4 &nbsp;&nbsp;8 &nbsp;&nbsp;&nbsp;12 <br>
	 * 1 &nbsp;&nbsp;5 &nbsp;&nbsp;9 &nbsp;&nbsp;&nbsp;13 <br>
	 * 2 &nbsp;&nbsp;6 &nbsp;&nbsp;10 &nbsp;14 <br>
	 * 3 &nbsp;&nbsp;7 &nbsp;&nbsp;11 &nbsp;15 <br>
	 *
	 *
	 * @return (Constant) pointer to buffer where the matrix is stored.
	 * Buffer has a size of 16 double values.
	 */
	virtual const double* getRawData() const = 0;

	/**
	 * @brief Returns a pointer to a writable buffer where the raw data of the matrix is stored.
	 */
	virtual double* setRawData() = 0;

	/**
	 * @brief Multiply a matrix by another matrix
	 * @param matrix Multiplicand matrix
	 * @return Result of matrix multiplication
	 */
	virtual IHomogeneousMatrix44* operator*(const IHomogeneousMatrix44 &matrix) = 0;

	/**
	 * @brief Assign a matrix to another matrix
	 * @param matrix R-value matrix
	 */
	virtual IHomogeneousMatrix44* operator=(const IHomogeneousMatrix44 &matrix) = 0;

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a homogeneous matrix to a stream e.g. std::cout << homMatObj;
	 *
	 * @param outStream The output stream
	 * @param matrix Pointer to homogeneous matrix which data is forwarded to the output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, const IHomogeneousMatrix44 &matrix);

};

}

#endif /* IHOMOGENEOUSMATRIX44_H_ */

/* EOF */
