/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#include "LODCalculator.h"

#include <math.h>

#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/ColoredPoint3D.h"
#include "brics_3d/core/TriangleMeshImplicit.h"
#include "Sphere.h"
#include "Cylinder.h"
#include "Box.h"
#include "PointCloud.h"
#include "Mesh.h"

namespace brics_3d {
namespace rsg {


LODCalculator::LODCalculator() {

}

LODCalculator::~LODCalculator() {
}

bool LODCalculator::calculateLOD(Shape::ShapePtr shape, double& lod) {
	lod = 0.0;
	if(!shape) {
		return false;
	}

	rsg::Sphere::SpherePtr sphere(new rsg::Sphere());
	sphere =  boost::dynamic_pointer_cast<rsg::Sphere>(shape);
	rsg::Box::BoxPtr box(new rsg::Box());
	box =  boost::dynamic_pointer_cast<rsg::Box>(shape);
	rsg::Cylinder::CylinderPtr cylinder(new rsg::Cylinder());
	cylinder =  boost::dynamic_pointer_cast<rsg::Cylinder>(shape);
	rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr mesh(new rsg::Mesh<brics_3d::ITriangleMesh>());
	mesh = boost::dynamic_pointer_cast<rsg::Mesh<brics_3d::ITriangleMesh> >(shape);
	double volume = 1.0;


		if (sphere !=0) {
			LOG(DEBUG) << "                 -> Found a sphere.";

			double r = sphere->getRadius();
			volume = 4.0/3.0 * M_PI * r*r*r;
			if (volume == 0.0) {
				lod = 0.0;
			} else {
				lod = 1.0 /* samples */ / volume;
			}

		} else if (cylinder !=0) {
			LOG(DEBUG) << "                 -> Found a cylinder.";

			double r = cylinder->getRadius();
			volume =  M_PI * r*r * cylinder->getHeight();
			if (volume == 0.0) {
				lod = 0.0;
			} else {
				lod = 2.0 /* samples */ / volume;
			}


		} else if (box !=0) {
			LOG(DEBUG) << "                 -> Found a box.";

			volume = box->getSizeX() * box->getSizeY() * box->getSizeZ();
			if (volume == 0.0) {
				lod = 0.0;
			} else {
				lod = 3.0 /* samples */ / volume;
			}


		} else if (shape->getPointCloudIterator() != 0) {
			LOG(DEBUG) << "                 -> Found a point cloud.";

			Density density = pointCloudLod.computeDensity(shape->getPointCloudIterator());
			lod  = density.numberOfPoints / volume;

		} else if (mesh != 0) {
			LOG(DEBUG) << "                 -> Found a mesh.";

			// TODO: we need a point cloud iterator for the mesh

		}

	LOG(DEBUG) << "LODCalculator: lod  = " << lod;

	return true;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
