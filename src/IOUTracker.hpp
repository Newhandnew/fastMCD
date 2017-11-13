#pragma once

#include "opencv2/opencv.hpp"
#include <stdio.h>

using namespace cv;  
using namespace std;  


class IOUTracker {
 private:
 	Rect bbox;
 	int frameCount;
 	Scalar color;

 public:
	 IOUTracker(Rect inputBbox);
	// ~IOUTracker(void);
	// void initial(Rect inputBbox);
	int getFrameCount(void);
	void setFrameCount(int value);
	float iou(Rect inputBbox);

};
