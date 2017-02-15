// OpenCVWebcamTest.cpp

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <map>
#include<iostream>
#include<conio.h>           // may have to modify this line if not using Windows
using namespace cv;
using namespace std;
///////////////////////////////////////////////////////////////////////////////////////////////////
int main() {
	cv::VideoCapture capWebcam(0);            // declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

	if (capWebcam.isOpened() == false) {                                // check if VideoCapture object was associated to webcam successfully
		std::cout << "error: capWebcam not accessed successfully\n\n";      // if not, print error message to std out
		_getch();                                                           // may have to modify this line if not using Windows
		return(0);                                                          // and exit program
	}
	Mat cdst;
	cv::Mat imgOriginal;        // input image
	cv::Mat imgGrayscale;       // grayscale of input image
	cv::Mat imgBlurred;         // intermediate blured image
	cv::Mat imgCanny; 
	cv::Mat imgCanny2;// Canny edge image
	int state = 0;
	char charCheckForEscKey = 0;

	while (charCheckForEscKey != 27 && capWebcam.isOpened()) {            // until the Esc key is pressed or webcam connection is lost
		bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal);            // get next frame

		if (!blnFrameReadSuccessfully || imgOriginal.empty()) {                 // if frame not read successfully
			std::cout << "error: frame not read from webcam\n";                 // print error message to std out
			break;                                                              // and jump out of while loop
		}

		cv::cvtColor(imgOriginal, imgGrayscale, CV_BGR2GRAY);                   // convert to grayscale
		//
		cv::GaussianBlur(imgGrayscale,              // input image
			imgBlurred,                // output image
			cv::Size(5, 5),            // smoothing window width and height in pixels
			1.5);                      // sigma value, determines how much the image will be blurred

		cv::Canny(imgBlurred,                       // input image
			imgCanny,                         // output image
			95,                              // low threshold
			100);                             // high threshold
												  /// Detector parameters
		cvtColor(imgCanny, cdst, CV_GRAY2BGR);
		// detect lines
		vector<Vec4i> lines;
		map<Point, int> intersectionCount;
		int count = 0;
		
		HoughLinesP(imgCanny, lines, 1, CV_PI / 180, 40 , 10, 10) ;
		
		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4i l = lines[i];
			int x1 = (l[0] + l[2]+ l[3] - l[1]) / 2;
			int x2 = (l[1] + l[3] + l[2] - l[0]) / 2;
			int vec1 = (l[1] - l[3]);
			int vec2 = (l[0] - l[2]);
			int count = 0;

			
			
			for (int ii = -3; ii < 4; ii++) {
				for (int jj = -3; jj < 4; jj++) {


					int x = x1 + ii * vec1 - jj * vec2;
					int y = x2 + ii * vec2 + jj * vec1;
					if (x > 0 && y > 0 && x < imgGrayscale.size[0] && y < imgGrayscale.size[1]) {
						Vec3b vIntensity = imgOriginal.at<Vec3b>(x, y);

						int intensity = sqrt(vIntensity.val[0] * vIntensity.val[0] + vIntensity.val[1] * vIntensity.val[1] + vIntensity.val[2] * vIntensity.val[2]);
						count += ((intensity < 220) == ((ii + jj)% 3 == 0));
					}
					else {
						break;
					}
				}
			}

			if (count >= 40) {
				cout << count << endl;
				cout << "found blue!" << endl;

				line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, CV_AA);
			}

			else
			{
				line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 0), 3, CV_AA);
			}
			
			
		}
	
		
		// declare windows
		cv::namedWindow("imgOriginal", CV_WINDOW_NORMAL);       // note: you can use CV_WINDOW_NORMAL which allows resizing the window
		cv::namedWindow("imgCanny", CV_WINDOW_NORMAL);          // or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
																// CV_WINDOW_AUTOSIZE is the default
		cv::imshow("cdstl", cdst);                 // show windows
		cv::imshow("imgCanny", imgOriginal);                       //

		charCheckForEscKey = cv::waitKey(1);        // delay (in ms) and get key press, if any
	}   // end while

	return(0);
}









