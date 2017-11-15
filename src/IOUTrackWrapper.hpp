#pragma once

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "ActiveTracker.hpp"

using namespace cv;  
using namespace std;  


class IOUTrackWrapper {
 private:
	vector<ActiveTracker> activeTracker;
	float iouThreshold;
	Mat curFrame;

 public:
	IOUTrackWrapper(void);
	void runTrack(Mat inputImage, vector<Rect> detectedObjects);

};
