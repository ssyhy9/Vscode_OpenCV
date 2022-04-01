#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

Mat imgHSV, mask;
int hmin = 0, smin = 0, vmin = 0;
int hmax = 179, smax = 132, vmax = 76; 

// black 0-32 0-255 0-255
// yellow 0-30 82-255 130-210
// pink 132-179 135-213 153-229
// blue 88-116 94-215 7-135
// red 158-179 139-250 92-194

// int main(){
//     Mat img = imread("D:/Vscode_OpenCV/Resources/colours.jpg");
//     Size dsize=Size(640,480);
//     resize(img,img,dsize,0,0,INTER_AREA);
//     cvtColor(img, imgHSV, COLOR_BGR2HSV);

//     namedWindow("Trackbars", (600, 200));
//     createTrackbar("Hue min", "Trackbars", &hmin, 179);
//     createTrackbar("Hue max", "Trackbars", &hmax, 179);
//     createTrackbar("Sat min", "Trackbars", &smin, 255);
//     createTrackbar("Sat max", "Trackbars", &smax, 255);
//     createTrackbar("Val min", "Trackbars", &vmin, 255);
//     createTrackbar("Val max", "Trackbars", &vmax, 255);

//     while(1){
//         Scalar lower(hmin, smin, vmin);
//         Scalar upper(hmax, smax, vmax);
//         inRange(imgHSV, lower, upper, mask);

//         imshow("IMG", img);
//         imshow("IMG_HSV", imgHSV);
//         imshow("MASK", mask);
//         waitKey(2);
//     }
// }

int main()
{
    string path = "D:/Vscode_OpenCV/Resources/Full_video.mp4";
    VideoCapture cap(path);
    Mat img;

    namedWindow("Trackbars", (600, 200));
        createTrackbar("Hue min", "Trackbars", &hmin, 179);
        createTrackbar("Hue max", "Trackbars", &hmax, 179);
        createTrackbar("Sat min", "Trackbars", &smin, 255);
        createTrackbar("Sat max", "Trackbars", &smax, 255);
        createTrackbar("Val min", "Trackbars", &vmin, 255);
        createTrackbar("Val max", "Trackbars", &vmax, 255);


    while(true){
        cap.read(img);
        Size dsize=Size(640,480);
        resize(img,img,dsize,0,0,INTER_AREA);
        cvtColor(img, imgHSV, COLOR_BGR2HSV);

        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(imgHSV, lower, upper, mask);

        imshow("Image", img);
        imshow("IMG_HSV", imgHSV);
        imshow("MASK", mask);
        waitKey(10);
    }
    return 0;
}