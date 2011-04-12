/*
 * Evaluator.cpp
 *
 * @author: Pinaki Banerjee
 * @date: Apr 10, 2011
 * @version: 0.1
 */

#include "Evaluator.h"
#include "FileReader.h"
#include <vector>
namespace BRICS_3D{


Evaluator::Evaluator() {
	countGt=0;
	countMs=0;
	gtBaseName="";
	msBaseName="";
	fileExt="";
	regionCorrespondence="";
	imageSize=0;
	initializationStatus =false;
}

Evaluator::~Evaluator() {}

//Impelementation of getters and setters
void Evaluator::setCountGt(int count){
	countGt=count;
}
int Evaluator::getCountGt(){
	return countGt;
}

void Evaluator::setCountMs(int count){
	countMs=count;
}
int Evaluator::getCountMs(){
	return countMs;
}

void Evaluator::setFileExt(string ext){
fileExt = ext;
}
string Evaluator::getFileExt(){
	return fileExt;
}

void Evaluator::setGtBaseName(string baseName){
	gtBaseName=baseName;
}
string Evaluator::getGtBaseName(){
	return gtBaseName;
}

void Evaluator::setMsBaseName(string baseName){
	msBaseName=baseName;
}
string Evaluator::getMsBaseName(){
	return msBaseName;
}

void Evaluator::setRegionCorrespondence(string regCorres){
	regionCorrespondence=regCorres;
}
string Evaluator::getRegionCorrespondence(){
	return regionCorrespondence;
}

void Evaluator::setImageSize(int size){
	imageSize = size;
}

int Evaluator::getImageSize(){
	return imageSize;
}


void Evaluator::Initialize(){
	vectorGt = new vector<string>[getCountGt()];
	vectorMs = new vector<string>[getCountMs()];
	FileReader fReader;
	stringstream filename;
	string s;

	for (int i = 0; i < countGt; i++) {

		filename.str("");
		filename.clear();
		filename << gtBaseName << i+1 << fileExt;
		s=filename.str();
		fReader.toStringVector(s,vectorGt[i]);
	}
	//cout<<"[Check Point:] "<< "GT file reading done, size: "<< vectorGt[0].size() <<endl;
	for (int i = 0; i < countMs; i++) {

			filename.str("");
			filename.clear();
			filename << msBaseName << i+1 << fileExt;
			s=filename.str();
			fReader.toStringVector(s,vectorMs[i]);
	}
	//cout<<"[Check Point:] "<< "MS file reading done, size: "<< vectorMs[0].size() <<endl;
	initializationStatus=true;
}

bool Evaluator::IsInitialized(){
	return initializationStatus;
}
}
