#include "main.hpp"


const String keys =
    "{@video_path  |0 | video name            }"
    "{@flag_output_image  |0 | flag output image}"
;

string getFileName(string inputPath)
{
	int i = inputPath.find_last_of('/');
	if (i != string::npos)
	{
		return inputPath.substr(i+1); // f contains the result :)
	}
	else
	{
		return inputPath;
	}
}

Mat getSeperatedElement(Mat inputImage, bool showImage)
{
	int erosion_type = MORPH_RECT;
    int erosion_size = 1;
	Mat erosion_element = getStructuringElement(erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ),Point( erosion_size, erosion_size));
	int dilation_type = MORPH_RECT;
    int dilation_size = 7;
	Mat dilation_element = getStructuringElement(dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ),Point( dilation_size, dilation_size));


	Mat imgErosion, imgDilation;
    erode(inputImage, imgErosion, erosion_element);
    dilate(imgErosion, imgDilation, dilation_element);//Mat(), Point(-1,-1), dilationIteration);
    if(showImage)
    {
    	imshow("erosion", imgErosion);
    	imshow("dilation", imgDilation);
    }
    return imgDilation;
}

int main(int argc, char *argv[])
{
	CommandLineParser parser( argc, argv, keys );
    String video_path = parser.get<String>(0);
    string videoName = getFileName(video_path);
    cout << "video name: " << videoName << endl;
    int flag_output_image = parser.get<int>(1);

	struct stat st = {0};

	if (stat("results", &st) == -1) {
	    mkdir("results", 0700);
	}

    VideoCapture cap;
    if(video_path == "0")
    {
    	cap.open(0);	
   	}
   	else if(video_path == "1")
   	{
   		cap.open(1);
   	}
   	else
   	{
   		cap.open(video_path);
   	}

    if( !cap.isOpened() )
    {
        cout << "video didn't open!" << endl;
        return -1;
    }

	MCDWrapper *mcdwrapper = new MCDWrapper();
	IOUTrackWrapper *iouTrackWrapper = new IOUTrackWrapper();

	Mat curFrame, imgDetect, imgOutput, imgResizedFrame;
	Size imageSize = Size(500, 500);

	int frame_num = 1;

	float tic, tic_total;
	float tic2, tic_total2;
	float timeTracking, timeTotal;

				int frame_width = 500;
				int frame_height = 500;
				VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),20, Size(frame_width,frame_height),true);

	/************************************************************************/
	/*  The main process loop                                               */
	/************************************************************************/
	while (1) 
	{	// the main loop
		tic = (float)getTickCount();
        cap >> curFrame;
        if(curFrame.empty())
        {
            cout << "end of video" << endl;
            return -1;
        }

        if(video_path != "0" && video_path != "1")
     	{
     		stringstream ss;
        	ss << cap.get(CAP_PROP_POS_FRAMES);
        	ss >> frame_num;
     	}   

     	resize(curFrame, imgResizedFrame, imageSize);

		if (frame_num == 1) 
		{
			mcdwrapper->Init(imgResizedFrame);
			frame_num++;
		} 
		else {
			// Run detection
			mcdwrapper->Run();
			imgDetect = mcdwrapper->getDetectImage();
			// imshow("detect image", imgDetect);

			// Display detection results as overlay
			// Mat imgProposed;
			// Mat imgRed(imgResizedFrame.size(), imgResizedFrame.type(), Scalar(0, 0, 255));
	  		// imgRed.copyTo(imgProposed, imgDetect);
	  		// addWeighted(imgResizedFrame , 1, imgProposed, 0.6, 0, imgOutput, 0);

			// stringstream ss;
			// ss << frame_num;
			// putText(imgOutput, ss.str(), cv::Point(15, 15), FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,255,0));

			// Show image
			// imshow("Output", imgOutput);

			// tracking program
			Mat imgDilation = getSeperatedElement(imgDetect, false);
			Mat imgBoundingBox;
			vector<vector<Point> > contours;
	        vector<Vec4i> hierarchy;
	        findContours( imgDilation, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	        int numContours = contours.size();
	        cout << "bounding box number: " << numContours << endl;
	        imgResizedFrame.copyTo(imgBoundingBox);

	        int minBoundingArea = 300;
    		int numLimitBounding = 60;
    		vector<Rect> detectedObjects;

	        if (numContours < numLimitBounding)
	        {
	            for( int i = 0; i < numContours; i++ )
	            {
	                if (contourArea(contours[i]) > minBoundingArea)
	                {
	                    Rect boundRect = boundingRect(contours[i]);
	                    detectedObjects.push_back(boundRect);
	                    rectangle( imgBoundingBox, Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), Scalar(255, 0, 0), 2, 8, 0 );
	                }
	            }
	        }
        	imshow("Bounding Box", imgBoundingBox );
			tic2 = (float)getTickCount();
			vector<Rect> trackingRec;
			Mat imgTracking;
			imgResizedFrame.copyTo(imgTracking);
        	trackingRec = iouTrackWrapper->runTrack(imgResizedFrame, detectedObjects);
        	for(int i = 0; i < trackingRec.size(); i++)
        	{
        		rectangle(imgTracking, Point(trackingRec[i].x, trackingRec[i].y), Point(trackingRec[i].x + trackingRec[i].width, trackingRec[i].y + trackingRec[i].height), Scalar(255, 0, 0), 2, 8, 0 ); //activeTracker[i].getColor()
        	}
        	imshow("tracking", imgTracking);

	     	tic_total2 = (float)getTickCount() - tic2;
			timeTracking = tic_total2 / (float)getTickFrequency() * 1000;
			cout << "tracking time: " << timeTracking << endl;

			if (flag_output_image == 1) 
			{
				video.write(imgTracking);
			}
		}

		//KeyBoard Process
		switch (waitKey(1)) 
		{
			case 'q':	// press q to quit
			case 27:	// press ESC
				cout << "exit" << endl;
				return -1;
			case 'p':
				cout << "pause" << endl;
				waitKey();
		}
     	tic_total = (float)getTickCount() - tic;
		timeTotal = tic_total / (float)getTickFrequency() * 1000;
		cout << "total time: " << timeTotal << endl;
	}

	return 0;
}
