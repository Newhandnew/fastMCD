#pragma once

#include "opencv2/opencv.hpp"
#include <stdio.h>

using namespace cv;  
using namespace std;  


class IOUTracker {
 private:
 	Rect bbox;
 	int frameCount;
 	int maxFrameCount;
 	Scalar color;

 public:
	 IOUTracker(Rect inputBbox);
	int getFrameCount(void);
	void setFrameCount(int value);
	void addFrameCount(void);
	void subFrameCount(void);
	Scalar getColor(void);
	void setBbox(Rect inputBbox);
	float iou(Rect inputBbox);

};
