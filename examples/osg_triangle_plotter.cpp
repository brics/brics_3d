/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#include <iostream>
#include <core/TriangleMeshImplicit.h>
#include <core/TriangleMeshExplicit.h>
#include <util/OSGTriangleMeshVisualizer.h>



using namespace std;
using namespace BRICS_3D;



int main(int argc, char **argv) {

	/* check argument */
//	string filename;
//	if (argc == 1) {
//		cout << "Usage: " << argv[0] << " <filename>" << endl;
//
//		char defaultFilename[255] = { BRICS_IMGAGES_DIR };
//		strcat(defaultFilename, "/zcam_param1c.pgm\0");
//		filename = defaultFilename;
//
//		cout << "Trying to get default file: " << filename << endl;
//	} else if (argc == 2) {
//		filename = argv[1];
//		cout << filename << endl;
//	} else {
//		cerr << "Usage: " << argv[0] << " <filename>" << endl;
//		return -1;
//	}

	/* create triangle mesh */
	Point3D* vertex000 = new Point3D(0,0,0);
	Point3D* vertex100 = new Point3D(1,0,0);
	Point3D* vertex101 = new Point3D(1,0,1);
//	Point3D* vertex001 = new Point3D(0,0,1);
	Point3D* vertex001 = new Point3D(0,0.2,1);
	Point3D* vertex110 = new Point3D(1,1,0);
//	Point3D* vertex111 = new Point3D(1,1,1);
	Point3D* vertex111 = new Point3D(0.8,1,1);

	Triangle* testTriangle1 = new Triangle(*vertex000, *vertex100, *vertex101);
	Triangle* testTriangle2 = new Triangle(*vertex101, *vertex001, *vertex000);
	Triangle* testTriangle3 = new Triangle(*vertex100, *vertex110, *vertex101); //...
	Triangle* testTriangle4 = new Triangle(*vertex101, *vertex110, *vertex111);

	TriangleMeshExplicit* meshExplicit = new TriangleMeshExplicit();
	meshExplicit->addTriangle(testTriangle1);
	meshExplicit->addTriangle(testTriangle2);
	meshExplicit->addTriangle(testTriangle3);
	meshExplicit->addTriangle(testTriangle4);



	TriangleMeshImplicit* meshImplicit = new TriangleMeshImplicit();
	                                                     // index
	(*meshImplicit->getVertices()).push_back(vertex000); // 0
	(*meshImplicit->getVertices()).push_back(vertex100); // 1
	(*meshImplicit->getVertices()).push_back(vertex101); // 2
	(*meshImplicit->getVertices()).push_back(vertex001); // 3
	(*meshImplicit->getVertices()).push_back(vertex110); // 4
	(*meshImplicit->getVertices()).push_back(vertex111); // 5

	(*meshImplicit->getIndices()).push_back(0);
	(*meshImplicit->getIndices()).push_back(1);
	(*meshImplicit->getIndices()).push_back(2);

	(*meshImplicit->getIndices()).push_back(2);
	(*meshImplicit->getIndices()).push_back(3);
	(*meshImplicit->getIndices()).push_back(0);

	(*meshImplicit->getIndices()).push_back(2);
	(*meshImplicit->getIndices()).push_back(1);
	(*meshImplicit->getIndices()).push_back(4);

	(*meshImplicit->getIndices()).push_back(2);
	(*meshImplicit->getIndices()).push_back(4);
	(*meshImplicit->getIndices()).push_back(5);


	/* visualize triangle mesh */
	OSGTriangleMeshVisualizer* visualizer = new OSGTriangleMeshVisualizer();
//	visualizer->addTriangleMesh(meshImplicit);
	visualizer->addTriangleMesh(meshExplicit);
	visualizer->visualize();

	delete visualizer;

	delete meshImplicit;
	delete meshExplicit;

	delete vertex000;
	delete vertex100;
	delete vertex101;
	delete vertex001;
	delete vertex110;
	delete vertex111;

	return 0;
}

