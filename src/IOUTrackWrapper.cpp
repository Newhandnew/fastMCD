// input: image, detected rectangles
// output: tracking rectangles

#include "IOUTrackWrapper.hpp"


IOUTrackWrapper::IOUTrackWrapper(void)
{
	iouThreshold = 0.2;			// threshold for checking the same object
	activeThreshold = 16;		// threshold for active tracking list 
}

vector<Rect> IOUTrackWrapper::runTrack(Mat inputImage, vector<Rect> detectedObjects)
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
			activeTracker[i].setBbox(detectedObjects[maxIndex]);
			detectedObjects.erase(detectedObjects.begin() + maxIndex);
			activeTracker[i].addFrameCount();
			if(activeTracker[i].getFrameCount() > activeThreshold)
			{
				finalTracking.push_back(detectedObjects[maxIndex]);
			}
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
	return finalTracking;
}