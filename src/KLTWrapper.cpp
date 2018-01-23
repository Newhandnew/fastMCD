// input: sequence of gray image
// output: mat of homography


#include "KLTWrapper.hpp"

#include <stdio.h>

KLTWrapper::KLTWrapper(void)
{
	win_size = 10;
	count = 0;
	maxLevel = 3;
	fTest = 0;  // test program
}

KLTWrapper::~KLTWrapper(void)
{

}

void KLTWrapper::Init(Mat imgGray)
{
	numGridCols = imgGray.cols / GRID_SIZE_W;
	numGridRows = imgGray.rows / GRID_SIZE_H;

	InitFeatures();

	imgGray.copyTo(imgPrevGray);
	// Init homography
	matHomography = Mat::eye(3, 3, CV_64F);
}

void KLTWrapper::InitFeatures()
{
	/* initial grid points */
	count = numGridCols * numGridRows;

	int cnt = 0;
	Point2f gridPoint;
	for (int i = 0; i < numGridCols - 1; ++i) {
		for (int j = 0; j < numGridRows - 1; ++j) {
			gridPoint.x = i * GRID_SIZE_W + GRID_SIZE_W / 2;
			gridPoint.y = j * GRID_SIZE_H + GRID_SIZE_H / 2;
			gridPoints.push_back(gridPoint);
		}
	}
}

Mat KLTWrapper::GetHomography(Mat imgGray)
{
	int i, k;
	// int nMatch[MAX_COUNT];
	vector<Point2f> gridGoodPoints, currentGoodPoints;

	if (imgPrevGray.empty()) 
	{
		imgGray.copyTo(imgPrevGray);
	}

	if (count > 0) {
		calcOpticalFlowPyrLK(imgPrevGray, imgGray, gridPoints, currentPoints, status, err, Size(win_size, win_size), maxLevel);
		for (i = k = 0; i < status.size(); i++) {
			if (status[i]) 
			{	
	            gridGoodPoints.push_back(gridPoints[i]);
	            currentGoodPoints.push_back(currentPoints[i]);
				k++;
			}
		}
		count = k;
	}
	// === test program
	if (fTest == 1)
	{
		Mat copyCurrentGray;
		Mat copyPrevGray;
		imgPrevGray.copyTo(copyPrevGray);
		imgGray.copyTo(copyCurrentGray);

		for( size_t i = 0; i < gridGoodPoints.size(); i++ )
	    {
	        cv::circle( copyPrevGray, gridGoodPoints[i], 5, cv::Scalar( 255. ), -1 );
	    }
	    imshow("prev corners", copyPrevGray);

	    for( size_t i = 0; i < currentGoodPoints.size(); i++ )
	    {
	        cv::circle( copyCurrentGray, currentGoodPoints[i], 5, cv::Scalar( 255. ), -1 );
	    }
	    imshow("current corners", copyCurrentGray);
		cout << "count: " << count << endl;
		cout << "status size: " << status.size() << endl;
	}
	// === end of test
	
	if (count >= 10) 
	{
		matHomography = findHomography( currentGoodPoints, gridGoodPoints, CV_RANSAC );
	} 
	else 
	{
		matHomography = Mat::eye(3, 3, CV_64F);
	}
	imgGray.copyTo(imgPrevGray);
	return matHomography;
}

