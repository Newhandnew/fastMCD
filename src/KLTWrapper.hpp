
#pragma once

#include "opencv2/opencv.hpp" 
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;  
using namespace std;  

#define GRID_SIZE_W (32)
#define GRID_SIZE_H (24)

typedef unsigned char BYTE;

class KLTWrapper {
 private:

	Mat imgPrevGray;
	int win_size;
	int numGridCols, numGridRows;
	vector<Point2f> gridPoints, currentPoints;

	int count;

	// For Homography Matrix
	Mat matHomography;

	int maxLevel;
	vector <float> err;
	vector <uchar> status;

	char fTest;

 private:
	void InitFeatures();

 public:
	 KLTWrapper(void);
	~KLTWrapper(void);

	void Init(Mat imgGray);
	Mat GetHomography(Mat imgGra);
};
