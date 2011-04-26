/**
 * @file 
 * NormalSet3D.cpp
 *
 * @date: Apr 26, 2011
 * @author: sblume
 */

#include "NormalSet3D.h"

namespace BRICS_3D {

NormalSet3D::NormalSet3D() {
	normals = new std::vector<Normal3D> ();
	normals->clear();

}

NormalSet3D::~NormalSet3D() {
	if (normals != NULL) {
		normals->clear();
		delete normals;
	}
}

void NormalSet3D::addNormal(Normal3D normal) {
	normals->push_back(normal);
}


std::vector<Normal3D>* NormalSet3D::getNormals(){
	return normals;
}


unsigned int NormalSet3D::getSize(){
	return normals->size();
}


}

/* EOF */
