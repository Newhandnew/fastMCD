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
		return "";
	}
}

int main(int argc, char *argv[])
{
	CommandLineParser parser( argc, argv, keys );
    String video_path = parser.get<String>(0);
    string videoName = getFileName(video_path);
    int flag_output_image = parser.get<int>(1);

	struct stat st = {0};

	if (stat("results", &st) == -1) {
	    mkdir("results", 0700);
	}

    VideoCapture cap;
    cap.open(video_path);

    if( !cap.isOpened() )
    {
        cout << "video didn't open!" << endl;
        return -1;
    }

	MCDWrapper *mcdwrapper = new MCDWrapper();

	const char window_name[] = "OUTPUT";

	Mat curFrame, imgDetect, imgOutput;

	namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	int frame_num;

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
        stringstream ss;
        ss << cap.get(CAP_PROP_POS_FRAMES);
        ss >> frame_num;

		if (frame_num == 1) 
		{
			mcdwrapper->Init(curFrame);
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
	        addWeighted(curFrame , 1, imgProposed, 0.5, 0, imgOutput, 0);

			stringstream ss;
			ss << frame_num;
			putText(imgOutput, ss.str(), cv::Point(15, 15), FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,255,0));

			// Show image
			imshow(window_name, imgOutput);

			if (flag_output_image == 1) 
			{
				stringstream ss;
				ss << "./results/" << videoName << "_frame" << frame_num << ".png";
								cout << "write: " <<ss.str() << endl;
				imwrite(ss.str(), imgOutput);
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
		++frame_num;

	}

	return 0;
}
