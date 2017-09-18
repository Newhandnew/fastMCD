// Copyright (c) 2016 Kwang Moo Yi.
// All rights reserved.

// This  software  is  strictly   for  non-commercial  use  only.  For
// commercial       use,       please        contact       me       at
// kwang.m<dot>yi<AT>gmail<dot>com.   Also,  when  used  for  academic
// purposes, please cite  the paper "Detection of  Moving Objects with
// Non-stationary Cameras in 5.8ms:  Bringing Motion Detection to Your
// Mobile Device,"  Yi et  al, CVPRW 2013  Redistribution and  use for
// non-commercial purposes  in source  and binary forms  are permitted
// provided that  the above  copyright notice  and this  paragraph are
// duplicated  in   all  such   forms  and  that   any  documentation,
// advertising  materials,   and  other  materials  related   to  such
// distribution and use acknowledge that the software was developed by
// the  Perception and  Intelligence Lab,  Seoul National  University.
// The name of the Perception  and Intelligence Lab and Seoul National
// University may not  be used to endorse or  promote products derived
// from this software without specific prior written permission.  THIS
// SOFTWARE IS PROVIDED ``AS IS''  AND WITHOUT ANY WARRANTIES.  USE AT
// YOUR OWN RISK!

#include "KLTWrapper.hpp"

#include <stdio.h>

KLTWrapper::KLTWrapper(void)
{
	win_size = 10;
	count = 0;
	maxLevel = 3;
	fTest = 1;  // test program
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

void KLTWrapper::RunTrack(Mat imgGray)
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
}

Mat KLTWrapper::GetHomography()
{
	return matHomography;
}
