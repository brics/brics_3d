/**
 * @file
 * alpha_shapes.cpp
 *
 * @brief Test file to demonstrate alpha shapes by CGAL 3.4
 * 
 * @author: Sebastian Blumenthal
 * @date: Aug 27, 2009
 * @version: 0.1
 */

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Alpha_shape_3.h>

#include <boost/progress.hpp>

#include <fstream>
#include <list>
#include <iostream>

using namespace std;

struct K: CGAL::Exact_predicates_inexact_constructions_kernel {
};

typedef CGAL::Alpha_shape_vertex_base_3<K> Vb;
typedef CGAL::Triangulation_hierarchy_vertex_base_3<Vb> Vbh;
typedef CGAL::Alpha_shape_cell_base_3<K> Fb;
typedef CGAL::Triangulation_data_structure_3<Vbh, Fb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Delaunay;
typedef CGAL::Triangulation_hierarchy_3<Delaunay> Delaunay_hierarchy;
typedef CGAL::Alpha_shape_3<Delaunay_hierarchy> Alpha_shape_3;

typedef K::Point_3 CGAL_Point;
typedef Alpha_shape_3::Alpha_iterator Alpha_iterator;
typedef Alpha_shape_3::NT NT;
typedef K::Segment_3 Segment;

void construct_alpha_shape(const list<CGAL_Point> &V_p, Alpha_shape_3::Mode mode,
		Alpha_shape_3& A)
// Generate Alpha Shape
{
	vector<Segment> V_seg;

	int n = A.make_alpha_shape(V_p.begin(), V_p.end());
	cout << "Inserted " << n << " points" << endl;

	A.set_mode(mode);
}

int main(int argc, char **argv) {

	/* check argument */
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		cerr << "Try for example: " << argv[0]
				<< " ../examples/test_data/3d_models/boxes.txt" << endl;
		return -1;
	}
	string filename = argv[1];
	cout << filename << endl;

	Delaunay_hierarchy dt;
	std::ifstream is(filename.c_str());
	int n;
	is >> n;
	CGAL_Point p;
	cout << n << " points read" << endl;

	boost::progress_display show_progress(n);
	for (; n > 0; n--) {
		is >> p;
		dt.insert(p);
		++show_progress;
	}
	cout << "Delaunay computed." << endl;

	// compute alpha shape
	Alpha_shape_3 as(dt);
	cout << "Alpha shape computed in REGULARIZED mode by defaut." << endl;

	// find optimal alpha values
	Alpha_shape_3::NT alpha_solid = as.find_alpha_solid();
	Alpha_iterator opt = as.find_optimal_alpha(1);
	cout << "Smallest alpha value to get a solid through data points is "
			<< alpha_solid << endl;
	cout << "Optimal alpha value to get one connected component is " << *opt
			<< endl;
	as.set_alpha(*opt);
	assert(as.number_of_solid_components() == 1);
#ifndef WIN32
	CGAL::Geomview_stream gv(CGAL::Bbox_3(-300, -300, -300, 300, 300, 300));
	gv.set_line_width(4);
	gv.set_trace(false);
	// gv.set_bg_color(CGAL::Color(0, 200, 200));
	for (;;) {
		gv.set_wired(true);
		gv << as;
		//gv << dt;
		cin >> n;
	}
#endif

	return 0;
}

/* EOF */
