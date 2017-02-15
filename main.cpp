#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <conio.h>
#include <iomanip>
#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>
#include <math.h>
#include <string.h>
#include <cxtypes.h>


using namespace std;
using namespace cv;

int main(int, char**)
{

	cvNamedWindow("Edges", CV_WINDOW_AUTOSIZE);
	CvCapture* capture = cvCaptureFromCAM(0);

	IplImage* frame;
	while (1) {
		frame = cvQueryFrame(capture);


		int depth_img = frame->depth;
		int height_img = frame->height;
		int width_img = frame->width;
		int size_img = frame->imageSize;
		int nchan_img = frame->nChannels;
		int nsize_img = frame->nSize;

		cout << setw(15) << "depth" << depth_img << endl;
		cout << setw(15) << "height" << height_img << endl;
		cout << setw(15) << "width" << width_img << endl;
		cout << setw(15) << "size" << size_img << endl;
		cout << setw(15) << "nchan" << nchan_img << endl;
		cout << setw(15) << "nsize" << nsize_img << endl;


		IplImage* out = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		cvSmooth(frame, out, CV_GAUSSIAN, 11, 11);
		cvCvtColor(out, out, CV_RGB2GRAY);
		cvCanny(out, out, 10, 10, 3);

		if (!frame) break;
		cvShowImage("Edge", out);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("Edge");
	return 0;
}