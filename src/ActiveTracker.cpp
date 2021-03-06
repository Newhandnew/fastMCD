#include "ActiveTracker.hpp"


ActiveTracker::ActiveTracker(Rect inputBbox)
{
	bbox = inputBbox;
	maxFrameCount = 16;
	frameCount = 3;
	color = Scalar(rand() % 255, rand() % 255, rand() % 255);
}

int ActiveTracker::getFrameCount(void)
{
	return frameCount;
}

void ActiveTracker::setFrameCount(int value)
{
	frameCount = value;
}

void ActiveTracker::addFrameCount(void)
{
	if(frameCount < maxFrameCount)
	{
		frameCount += 1;
	}
}

void ActiveTracker::subFrameCount(void)
{
	if(frameCount > 0)
	{
		frameCount -= 1;
	}
}

Scalar ActiveTracker::getColor(void)
{
	return color;
}

void ActiveTracker::setBbox(Rect inputBbox)
{
	bbox = inputBbox;
}

float ActiveTracker::iou(Rect inputBbox)
{
	int box1_x1 = inputBbox.x;
	int box1_x2 = inputBbox.x + inputBbox.width;
	int box1_y1 = inputBbox.y;
	int box1_y2 = inputBbox.y + inputBbox.height;

	int box2_x1 = bbox.x;
	int box2_x2 = bbox.x + bbox.width;
	int box2_y1 = bbox.y;
	int box2_y2 = bbox.y + bbox.height;

	int overlap_x1 = max(box1_x1, box2_x1);
	int overlap_y1 = max(box1_y1, box2_y1);
	int overlap_x2 = min(box1_x2, box2_x2);
	int overlap_y2 = min(box1_y2, box2_y2);
   	// check if there is an overlap
	if((overlap_x2 - overlap_x1 <= 0) || (overlap_y2 - overlap_y1 <= 0))
	{
		return 0;
	}
	// if yes, calculate the ratio of the overlap to each ROI size and the unified size
	int size_intersection = (overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
	int size_union = inputBbox.area() + bbox.area() - size_intersection;

	return (float)size_intersection / (float)size_union;
}