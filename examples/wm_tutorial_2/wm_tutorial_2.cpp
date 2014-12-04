/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
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

/* BRICS_3D includes */
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>

using namespace brics_3d::rsg;

/*
 * This tutorial covers basic tools for visualization of the scene graph.
 * Essentially there are two independent views: (a) the 3D visualization
 * and (b) the visualization of the graph structure.
 *
 * The 3D visualization makes use of the OpenScenGraph library (OSG) thus
 * we will use the OSGVisualizer class for that.
 *
 * The graph visualizer makes us of the dot format that is part of the
 * graphviz package. For this we need the DotVizualizer class.
 *
 * Both visualizers have in common that they will be attached to the world model
 * as _observers_ i.e. whenever the world model gets updated the respective
 * observers will be updated as well. This idea complies to the Observer Software
 * Pattern.
 *
 */
int main(int argc, char **argv) {

	/* Create a world model handle */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

	/*
	 * Create a 3D vizualizer and attach it to the world model.
	 * To fine tune the vizualization you can optionally set a couple of boolean
	 * parameters to toggle to visual appearance e.g. weather to show the node
	 * IDs or not, etc.
	 *
	 * Further details on the available options including the default values be
	 * found in the VisualizationConfiguration class.
	 *
	 * NOTE: You will have to compile the BRICS_3D will -DUSE_OSG to enable the
	 * OpenSceneGraph library as external, but optional dependency.
	 *
	 */
	OSGVisualizer* geometryVizualizer = new OSGVisualizer(); // Create the visualizer.
	VisualizationConfiguration osgConfiguration; // _Optional_ configuration file.
	osgConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	osgConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	osgConfiguration.abbreviateIds = true;       // Vizualize only the lower 2 bytes of an Id iff true.
	geometryVizualizer->setConfig(osgConfiguration);
	wm->scene.attachUpdateObserver(geometryVizualizer); // Enable 3D visualization

	/*
	 * Create a graph vizualizer and attach it to the world model, similar as above.
	 *
	 * The DotVisualizer will generate on each update a complete graph of the current
	 * scene and save it as a dot file on the hard disk which is typically called
	 * current_graph.gv. In addition it will be converted to a svg file if the dot
	 * tool is installed on the system.
	 *
	 * Every update on the scene graph will override the file. In case you want to keep
	 * the history of changes invoke setKeepHistory(true) on the vizualizer instance
	 *
	 */
	DotVisualizer* graphVizualizer = new DotVisualizer(&wm->scene); // NOTE: the constructor
	                                                                // needs the world model handle.
	VisualizationConfiguration dotConfiguration; // Again, _optional_ configuration file.
	dotConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	dotConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	dotConfiguration.abbreviateIds = true;       // Vizualize only the lower 2 bytes of anId iff true.
	graphVizualizer->setConfig(dotConfiguration);
	graphVizualizer->setKeepHistory(true); // Do not overwrite the produced files. Default is false.
	wm->scene.attachUpdateObserver(graphVizualizer); // Enable graph visualization

	/*
	 * All observers have to be informed about the root ID of the scene they are observing!
	 * This is becaues every scene including tthe one potntial observers are using
	 * can have a diffrent ID as root node. Sucessive updes to the observers without knoing the
	 * root ID causes loss od data. So we have to advertise this.
	 *
	 * Don't forget to call this one whenever new observers have been attached!
	 */
	wm->scene.advertiseRootNode();

	/* Graph structure:
	 *                 root
	 */

	/*
	 * We are going to create the same graph structure as discussed in the
	 * previous tutorial.
	 */
	Id group1Id;
	std::vector<Attribute> attributes;
	attributes.push_back(Attribute("name","my_group1"));
	wm->scene.addGroup(wm->getRootNodeId(), group1Id, attributes);

	Id group2Id;
	attributes.clear();
	attributes.push_back(Attribute("name","my_group2"));
	wm->scene.addGroup(wm->getRootNodeId(), group2Id, attributes);

	Id node1Id;
	attributes.clear();
	attributes.push_back(Attribute("name","my_node1"));
	wm->scene.addNode(group1Id, node1Id, attributes);

	/* Graph structure:
	 *                 root
	 *                   |
	 *        -----------+
	 *        |          |
	 *      group1     group2
	 *        |
	 *      node1
	 */

	/*
	 * As nodes and groups do not have something interesting to present in a 3D
	 * viewer, we will add a cylinder to the scene graph.
	 *
	 * In general geometric data is stored via shared pointers as it potentially
	 * could be used by multiple nodes at the same time if desired. In addition,
	 * we need to pass a time stamp indicating the creation time; the method
	 * call wm->now() will return a current time stamp.
	 *
	 * Time stamps might be also generated from sensors or other external sources
	 * so we preserve a way here to pass this data to the API of the world model.
	 */
	Id cylinder1Id;
	attributes.clear();
	attributes.push_back(Attribute("name","my_cylinder1"));
	Cylinder::CylinderPtr cylinder1(new Cylinder(0.02, 0.1)); //radius 0.02 and height 0.1
	wm->scene.addGeometricNode(group2Id, cylinder1Id, attributes, cylinder1, wm->now());

	/* Graph structure:
	 *                 root
	 *                   |
	 *        -----------+
	 *        |          |
	 *      group1     group2
	 *        |          |
	 *      node1     cylinder1
	 */


	/*
	 * To prevent that the program exits before we can see something useful,
	 * we will wait for the users's input at the OSG window.
	 */
	while(!geometryVizualizer->done()) { // Wait until user closes the GUI-
		// Nothing to do here.
	}


	/* Clean up */
	delete geometryVizualizer;
	delete graphVizualizer;
	delete wm;
}

/* EOF */
