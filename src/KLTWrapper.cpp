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
	// For LK funciton in opencv
	win_size = 10;
	// points[0] = points[1] = 0;
	count = 0;
	flags = 0;

	maxLevel = 3;

	// temp = NULL;
	// maskimg = NULL;
}

KLTWrapper::~KLTWrapper(void)
{
	// cvReleaseImage(&temp);
	// cvReleaseImage(&maskimg);
}

void KLTWrapper::Init(Mat imgGray)
{
	int ni = imgGray.cols;
	int nj = imgGray.rows;

	// Allocate Maximum possible + some more for safety
	MAX_COUNT = (float (ni) / float (GRID_SIZE_W) + 1.0)*(float (nj) / float (GRID_SIZE_H) + 1.0);

	// Pre-allocate
	imgGray.copyTo(image);
	// imgPrevGray = cvCreateImage(cvGetSize(imgGray), 8, 1);
	// pyramid = cvCreateImage(cvGetSize(imgGray), 8, 1);
	// prev_pyramid = cvCreateImage(cvGetSize(imgGray), 8, 1);
	// points[0] = (CvPoint2D32f *) cvAlloc(MAX_COUNT * sizeof(points[0][0]));
	// points[1] = (CvPoint2D32f *) cvAlloc(MAX_COUNT * sizeof(points[0][0]));
	flags = 0;
	InitFeatures(imgGray);
	// temp = cvCreateImage(cvGetSize(imgGray), 32, 1);
	// maskimg = cvCreateImage(cvGetSize(imgGray), IPL_DEPTH_8U, 1);

	// Gen mask
	// BYTE *pMask = (BYTE *) maskimg->imageData;
	// int widthStep = maskimg->widthStep;
	// for (int j = 0; j < nj; ++j) {
	// 	for (int i = 0; i < ni; ++i) {
	// 		pMask[i + j * widthStep] = (i >= ni / 5) && (i <= ni * 4 / 5) && (j >= nj / 5) && (j <= nj * 4 / 5) ? (BYTE) 255 : (BYTE) 255;
	// 	}
	// }

	// Init homography
	for (int i = 0; i < 9; i++)
		matH[i] = i / 3 == i % 3 ? 1 : 0;
}

void KLTWrapper::InitFeatures(Mat imgGray)
{
	/* automatic initialization */
	double quality = 0.01;
	double min_distance = 10;

	int ni = imgGray.cols;
	int nj = imgGray.rows;

	count = ni / GRID_SIZE_W * nj / GRID_SIZE_H;

	int cnt = 0;
	Point2f gridPoint;
	for (int i = 0; i < ni / GRID_SIZE_W - 1; ++i) {
		for (int j = 0; j < nj / GRID_SIZE_H - 1; ++j) {
			gridPoint.x = i * GRID_SIZE_W + GRID_SIZE_W / 2;
			gridPoint.y = j * GRID_SIZE_H + GRID_SIZE_H / 2;
			prevPoints.push_back(gridPoint);
		}
	}
	imgGray.copyTo(imgPrevGray);
}

void KLTWrapper::RunTrack(Mat imgGray)
{
	int i, k;
	int nMatch[MAX_COUNT];

	// if (prevGray.empty()) {
		Mat prevGray(imgPrevGray);
	// } else {
	// 	flags = 0;
	// }
	// memset(image->imageData, 0, image->imageSize);

	if (count > 0) {
		calcOpticalFlowPyrLK(prevGray, imgGray, prevPoints, points, status, err, Size(win_size, win_size), maxLevel);
		flags |= CV_LKFLOW_PYR_A_READY;
		for (i = k = 0; i < status.size(); i++) {
			if (status[i]) 
			{	
				nMatch[k] = i;
				k++;
			}

		}
		count = k;
	}
	Mat copyCurrentGray;
	Mat copyPrevGray;
	prevGray.copyTo(copyPrevGray);
	imgGray.copyTo(copyCurrentGray);

	for( size_t i = 0; i < prevPoints.size(); i++ )
    {
        cv::circle( copyCurrentGray, prevPoints[i], 5, cv::Scalar( 255. ), -1 );
    }
    imshow("prev corners", copyCurrentGray);

    for( size_t i = 0; i < points.size(); i++ )
    {
        cv::circle( copyPrevGray, points[i], 5, cv::Scalar( 255. ), -1 );
    }
    imshow("current corners", copyPrevGray);
	cout << "count :" << count << endl;
	cout << "status size: " << status.size() << endl;
	waitKey();
	if (count >= 10) {
		// Make homography matrix with correspondences
		// Mat _h = findHomography( points, prevPoints, CV_RANSAC )
		MakeHomoGraphy(nMatch, count);
	} else {
		for (int ii = 0; ii < 9; ++ii) {
			matH[ii] = ii % 3 == ii / 3 ? 1.0f : 0.0f;
		}
	}
	SwapData(imgGray);
}

void KLTWrapper::SwapData(Mat imgGray)
{
	imgGray.copyTo(imgPrevGray);
	// CV_SWAP(prev_pyramid, pyramid, swap_temp);
	// swap(prevPoints, points);
}

void KLTWrapper::GetHomography(double *pmatH)
{
	memcpy(pmatH, matH, sizeof(matH));
}

void KLTWrapper::MakeHomoGraphy(int *pnMatch, int nCnt)
{
	double h[9];
	Mat _h = Mat(3, 3, CV_64F, h);
	// std::vector < Point2f > pt1, pt2;

	// int i;

	// pt1.resize(nCnt);
	// pt2.resize(nCnt);
	// for (i = 0; i < nCnt; i++) {
	// 	//REVERSE HOMOGRAPHY
	// 	pt1[i] = points[pnMatch[i]];
	// 	pt2[i] = prevPoints[pnMatch[i]];
	// }
	_h = findHomography( points, prevPoints, CV_RANSAC );
	// matH = (double)_h.data;
	int i, j;
	for (i = 0; i < _h.cols; i++)
	{
		for (j = 0; j < _h.rows; j++)
		{
			matH[i*3 + j] = _h.at<double>(i,j);
		}
	}
	
	// for (i = 0; i < 9; i++) {
	// 	matH[i] = h[i];
	// }
}
