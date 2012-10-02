This file defines the default settings of filenames and file-paths used in for evaluation and 
comparison of 3D segmentation algorithms. 

1. Evaluation
	
	** Required Files
	
		** Ground truth files for an input scenario. By default these files could be found in:
			trunk/src/segmentation/evaluation/groundTruthData/
			The set of ground truth files will be in the corresponding directory which can 
			be identified by its name. 
			Typical filename should look like : "Scenario1_gt_1.txt"
			
		** Machine Segmented files. By default these files should be located in:
			trunk/src/algorithm/segmentation/evaluation/data/ with user defined directory name. The file naming should be
			<prefix>_<region no><file-ext>. For Example "Scenario1_RANSAC_1.txt"
		

		** For user defined paths and files, there are set options in an evaluator object for
			setting "gtBaseName", "msBaseName" and "fileExt". But every region file should have 
			following filename convention :
			<prefix>_<region no.>_<file-extention>
			The prefix can be user defined file name. For example : "Scenario1_RANSAC". The filename
			should be then, "Scenario1_RANSAC_1.txt"

			A base name is composed as below:
			<path to the file>_<prefix>
			For example "../src/segmentation/evaluation/groundTruthData/Scenario1/Scenario1_RANSAC_"
			
			
	** Results of Evaluation.
	
		Two file formats are supported, HTML and CSV. The <HTML-prefix> and
		<CSV-prefix> can be set by the user as desired. Following files will be written in each
		case in the folder, trunk/bin/data/.
			
			*** <HTML-prefix>_result.html :	will contain region based evaluation results in detail.
			
			*** <CSV-prefix>_result_MsMetrics: MS region based metrics in CSV format.
			*** <CSV-prefix>_result_GtMetrics: GT region based metrics in CSV format.
			*** <CSV-prefix>_overlapMatrix: Overlap of each GT region with every MS region in CSV format.
			
			The CSV files are intended to be used for comparison of the algorithms.
			
2. Comparison

	The comparison framework by default considers that the csv file produced during the evaluation
	are located in trunk/src/algorithm/segmentation/evaluation/data/. This default path can be changed by setting path variable
	of a comparator object.
	The filenames should follow this format:
	<CSV-prefix><metrics-base-name>
	
	By default the metrics base names should be set as:
	
	comparatorObject.setGtMetricsBaseName("_result_GtMetrics.csv");
	comparatorObject.setMsMetricsBaseName("_result_MsMetrics.csv");
	comparatorObject.setOverlapBaseName("_result_overlapMatrix.csv");

	For examples on how to intialize and do the evaluation and comparison please refer to
	demoComparison.cpp and demoEvaluation.cpp in trunk/example/.