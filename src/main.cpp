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

	const char window_name[] = "OUTPUT";

	Mat curFrame, imgDetect, imgOutput;

	namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	int frame_num = 1;

	vector<IOUTracker> activeTracker;

	/************************************************************************/
	/*  The main process loop                                               */
	/************************************************************************/
	while (1) 
	{	// the main loop
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

		if (frame_num == 1) 
		{
			mcdwrapper->Init(curFrame);
			frame_num++;
		} 
		else {
			// Run detection
			mcdwrapper->Run();
			imgDetect = mcdwrapper->getDetectImage();
			imshow("detect image", imgDetect);

			// Display detection results as overlay
			Mat imgProposed;
			Mat imgRed(curFrame.size(), curFrame.type(), Scalar(0, 0, 255));
	        imgRed.copyTo(imgProposed, imgDetect);
	        addWeighted(curFrame , 1, imgProposed, 0.6, 0, imgOutput, 0);

			stringstream ss;
			ss << frame_num;
			putText(imgOutput, ss.str(), cv::Point(15, 15), FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,255,0));

			// Show image
			imshow(window_name, imgOutput);

			Mat imgDilation = getSeperatedElement(imgDetect, true);

			Mat imgBoundingBox;
			vector<vector<Point> > contours;
	        vector<Vec4i> hierarchy;
	        findContours( imgDilation, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	        int numContours = contours.size();
	        cout << "bounding box number: " << numContours << endl;
	        curFrame.copyTo(imgBoundingBox);

	        int minBoundingArea = 250;
    		int numLimitBounding = 10;
    		vector<Rect> detectedObjects;
	        if (numContours < numLimitBounding)
	        {
	            for( int i = 0; i < numContours; i++ )
	            {
	                if (contourArea(contours[i]) > minBoundingArea)
	                {
	                    // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	                    // cout << contourArea(contours[i]) << endl;
	                    Rect boundRect = boundingRect(contours[i]);
	                    detectedObjects.push_back(boundRect);
	                    rectangle( imgBoundingBox, Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), Scalar(255, 0, 0), 2, 8, 0 );
	                }
	            }
	        }
        	imshow("Bounding Box", imgBoundingBox );

        	for(int i = 0; i < activeTracker.size(); i++)
        	{	
        		float maxIOU = 0;
        		int maxIndex = 0;
        		for(int j = 0; j < detectedObjects.size(); j++)
        		{
        			if(activeTracker[i].iou(detectedObjects[j]) > maxIOU)
        			{
        				maxIOU = activeTracker[i].iou(detectedObjects[j]);
        				maxIndex = j;
        			}
        		}
        		cout << "iou: " << maxIOU << endl;
    			if(maxIOU > 0.3)
    			{
    				detectedObjects.erase(detectedObjects.begin() + maxIndex);
    				cout << "add to final" << endl;
    			}
    			else
    			{
    				if(activeTracker[i].getFrameCount() == 0)
    				{
    					activeTracker.erase(activeTracker.begin()+i);
    					cout << "activeTracker count" << activeTracker.size() << endl;
    				}
    				else
    				{
    					activeTracker[i].setFrameCount(0);
    				}
    			}
        	}
        	cout << "dtectedObjects count:" << detectedObjects.size() << endl;
        	for(int i = 0; i < detectedObjects.size(); i++)
        	{
        		IOUTracker tracker(detectedObjects[i]);
        		// tracker.initial(detectedObjects[i]);
        		activeTracker.push_back(tracker);
        	}

			if (flag_output_image == 1) 
			{
				stringstream ss;
				ss << "./results/" << videoName << "_frame" << frame_num << ".png";
								cout << "write: " <<ss.str() << endl;
				imwrite(ss.str(), imgDetect);
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
	}

	return 0;
}
