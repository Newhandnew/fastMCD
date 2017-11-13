#pragma once

#include "opencv2/opencv.hpp"
#include <stdio.h>

using namespace cv;  
using namespace std;  


class IOUTracker {
 private:
 	Rect bbox;
 	int frameCount;

 public:
	 KLTWrapper(void);
	~KLTWrapper(void);
	void IOUTracker::initial(Rect inputBbox);
	float iou(Rect bbox1, Rect bbox2);

};
