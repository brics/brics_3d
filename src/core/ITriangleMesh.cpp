/**
 * @file 
 * ITriangleMesh.cpp
 *
 * @date: Feb 24, 2010
 * @author: sblume
 */

#include "ITriangleMesh.h"

namespace BRICS_3D {

istream& operator>>(istream &inStream, ITriangleMesh &mesh) {
	mesh.read(inStream);
	return inStream;
}

ostream& operator<<(ostream &outStream, ITriangleMesh &mesh) {
	mesh.write(outStream);
    return outStream;
}

}  // namespace BRICS_3D

/* EOF */
