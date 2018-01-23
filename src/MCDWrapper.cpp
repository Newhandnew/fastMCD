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

#ifndef	_MCDWRAPPER_CPP_
#define	_MCDWRAPPER_CPP_

#include "MCDWrapper.hpp"
#include "params.hpp"


MCDWrapper::MCDWrapper()
{
}

MCDWrapper::~MCDWrapper()
{
}

void
 MCDWrapper::Init(Mat imgInput)
{

	imgFrame = imgInput;
	// Smoothing using median filter
	cvtColor(imgFrame, imgGray, CV_RGB2GRAY);
	medianBlur(imgGray, imgGray, 5);

	m_LucasKanade.Init(imgGray);

	BGModel.init(imgGray);
}

void MCDWrapper::Run()
{
	float tic, tic_total;
	float rt_motionComp;	// motion Compensation time
	float rt_modelUpdate;	// model update time
	float rt_total;		// Background Subtraction time

	cvtColor(imgFrame, imgGray, CV_RGB2GRAY);
	// Smmothign using median filter
	medianBlur(imgGray, imgGray, 5);

	//--TIME START
	tic = (float)getTickCount();
	// Calculate Backward homography
	// Get H
	Mat matHomography = m_LucasKanade.GetHomography(imgGray);
	BGModel.motionCompensate(matHomography);

	//--TIME END
	tic_total = (float)getTickCount() - tic;
	rt_motionComp = tic_total / (float)getTickFrequency() * 1000;

	//--TIME START
	tic = (float)getTickCount();
	// Update BG Model and Detect
	detect_img = BGModel.update(imgGray);
	//--TIME END
	tic_total = (float)getTickCount() - tic;
	rt_modelUpdate = tic_total / (float)getTickFrequency() * 1000;

	rt_total = rt_motionComp + rt_modelUpdate;

	cout << " motion compensate: " << rt_motionComp << " model update: " << rt_modelUpdate << " total time: " << rt_total << endl;
}

Mat MCDWrapper::getDetectImage()
{
	return detect_img;
}

#endif				// _MCDWRAPPER_CPP_
