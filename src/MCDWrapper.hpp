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

#ifndef	_MCDWRAPPER_H_
#define	_MCDWRAPPER_H_

/************************************************************************/
/* Basic Includes                                                       */
/************************************************************************/
#include	<iostream>
/************************************************************************/
/* Includes for the OpenCV                                              */
#include "opencv2/opencv.hpp"
#include "opencv2/core/utility.hpp" 

// Inlcludes for this wrapper
#include "KLTWrapper.hpp"
#include "prob_model.hpp"

using namespace std;
using namespace cv;

class MCDWrapper {

/************************************************************************/
/*  Internal Variables					                                */
/************************************************************************/
 private:

	Mat detect_img;
	/* Note that the variable names are legacy */
	KLTWrapper m_LucasKanade;
	Mat imgFrame;
	Mat imgGray;

	Mat imgGaussLarge;
	Mat imgGaussSmall;
	Mat imgDOG;

	Mat debugCopy;
	Mat debugDisp;

	ProbModel BGModel;

/************************************************************************/
/*  Methods								                                */
/************************************************************************/
 public:

	MCDWrapper();
	~MCDWrapper();

	void Init(Mat imgInput);
	void Run();
	Mat getDetectImage();

};

#endif				//_MCDWRAPPER_H_
