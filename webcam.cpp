#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture cap(0);
    Mat img, imgHSV;

    while (true)
    {
        cap.read(img);

        cvtColor(img, imgHSV, COLOR_BGR2HSV);

        imshow("IMG", img);
        imshow("IMG HSV", imgHSV);
        waitKey(1);
    }
    return 0;
}