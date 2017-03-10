#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

void findTarget(Mat &img);

int main(int argc, char** argv){
    Mat src;

    VideoCapture cap(0); //video device 0
    if(!cap.isOpened()){
        cout << "unable to open video source" << endl;
        return -1;
    }

    while(1){
        cap >> src; 

        findTarget(src); 

        if(waitKey(10) == 27){ //quit when ESC is pressed
            break;
        }
    }

    return 0;
}

void findTarget(Mat &img){
    Mat dst, cdst;
    vector<Vec4i> lines;
    bool found;
    float threshold = 20.0;
    int numLines = 0;

    Canny(img, dst, 100, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);

    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 20);

    for(size_t i = 0; i < lines.size(); i++){
        Point pt1(lines[i][0], lines[i][1]);
        Point pt2(lines[i][2], lines[i][3]);

        float angle = fastAtan2(pt1.y - pt2.y, pt1.x - pt2.x);
        if(angle < 90 + threshold && angle > 90 - threshold){ 
            line(cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);//
            numLines++;
        }

    }

    found = numLines > 10;

    //draw text and bg
    rectangle(cdst, cvPoint(10, cdst.rows - 10), cvPoint(250, cdst.rows - 30), Scalar(0, 0, 0), -1, 8);
    if(found){
        putText(cdst, "target found", cvPoint(10, cdst.rows - 10), FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    } else {
        putText(cdst, "no target found", cvPoint(10, cdst.rows - 10), FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    }

    imshow("source", img);
    imshow("detected lines", cdst);
}
