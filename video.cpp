#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

/////////////////////////// video /////////////////////////////
using namespace cv;
using namespace std;

int main()
{
    string path = "D:/Vscode_OpenCV/Resources/CWvideo.mp4";
    VideoCapture cap(path);
    Mat img;

    while(true){
        cap.read(img);
        imshow("Image", img);
        waitKey(10);
    }
    return 0;
}

// int main()
// {
//     VideoCapture capture;
//     Mat frame;
//     frame= capture.open("D:/Vscode_OpenCV/Resources/oceans.mp4");
//     if(!capture.isOpened())
//     {
//         printf("can not open ...\n");
//         return -1;
//     }
//     //namedWindow("output", CV_WINDOW_AUTOSIZE);

//     while (capture.read(frame))
//     {
//         imshow("output", frame);
//         waitKey(10);
//     }
//     capture.release();
//     return 0;
// }