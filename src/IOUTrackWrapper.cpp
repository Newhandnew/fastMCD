#include "IOUTrackWrapper.hpp"


IOUTrackWrapper::IOUTrackWrapper(void)
{
	iouThreshold = 0.2;
}

void IOUTrackWrapper::runTrack(Mat inputImage, vector<Rect> detectedObjects)
{
	vector<Rect> finalTracking;
	inputImage.copyTo(curFrame);
	cout << "activeTracker count: " << activeTracker.size() << endl;
	for(int i = 0; i < activeTracker.size(); i++)
	{	
		float maxIOU = 0;
		int maxIndex = 0;
		for(int j = 0; j < detectedObjects.size(); j++)
		{
			float iou = activeTracker[i].iou(detectedObjects[j]);
			if(iou > maxIOU)
			{
				maxIOU = iou;
				maxIndex = j;
			}
		}
		if(maxIOU > iouThreshold)
		{
			finalTracking.push_back(detectedObjects[maxIndex]);
			activeTracker[i].setBbox(detectedObjects[maxIndex]);
			rectangle( curFrame, Point(detectedObjects[maxIndex].x, detectedObjects[maxIndex].y), Point(detectedObjects[maxIndex].x + detectedObjects[maxIndex].width, detectedObjects[maxIndex].y + detectedObjects[maxIndex].height), activeTracker[i].getColor(), 2, 8, 0 );
			detectedObjects.erase(detectedObjects.begin() + maxIndex);
			activeTracker[i].addFrameCount();
		}
		else
		{
			if(activeTracker[i].getFrameCount() == 0)
			{
				activeTracker.erase(activeTracker.begin()+i);
				i--;
			}
			else
			{
				activeTracker[i].subFrameCount();
			}
		}
	}
	int detectedObjectsSize = detectedObjects.size();
	cout << "detectedObjects count: " << detectedObjectsSize << endl;
	for(int i = 0; i < detectedObjectsSize; i++)
	{
		ActiveTracker tracker(detectedObjects[i]);
		activeTracker.push_back(tracker);
	}

	cout << "final tracking: " << finalTracking.size() << endl;
	// imshow("tracking", curFrame );
}