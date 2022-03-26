#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

//function prototypes
void findcolour(Mat imggray);
void getContours(Mat Mask);
void draw_centerPoints(Mat hsv, Mat img);

int hmin = 0, smin = 0, vmin = 102;
int hmax = 179, smax = 255, vma = 255; 
Mat img, imgHSV, imgGRAY, mask;


int main()
{
    string path = "D:/Vscode_OpenCV/Resources/CWvideo.mp4";
    VideoCapture cap(path);

    namedWindow("Trackbars", (600, 200));
    createTrackbar("Hue min", "Trackbars", &hmin, 179);
    createTrackbar("Hue max", "Trackbars", &hmax, 179);
    createTrackbar("Sat max", "Trackbars", &smin, 255);
    createTrackbar("Sat max", "Trackbars", &smax, 255);
    createTrackbar("Val min", "Trackbars", &vmin, 255);
    createTrackbar("Val ma", "Trackbars", &vma, 255);

    while(true){
        cap.read(img);
        cvtColor(img, imgGRAY, COLOR_BGR2GRAY);
        cvtColor(img, imgHSV, COLOR_BGR2HSV);

        findcolour(imgHSV);

     //   draw_centerPoints(imgHSV, img);

       // imshow("Image", img);
         //cout << img.size() << endl;  //640 * 368
         
        //imshow("IMG GRAY", imgGRAY);
        imshow("IMG HSV", imgHSV);
        imshow("IMG mask", mask);
        imshow("Image", img);
        waitKey(80);
    }
    return 0;
}

void findcolour(Mat imgHSV){
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vma);
        inRange(imgHSV, lower, upper, mask);
        getContours(mask);
}

void getContours(Mat Mask){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(Mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //cout << contours.size() << endl;
    for (int i = 0; i < contours.size(); i++){
        int area = contourArea(contours[i]);
        //cout << area << endl;
        if(area > 2000){
            drawContours(img, contours, i, Scalar(255, 0, 255), 2);
        }
    }
    //drawContours(img, contours, -1, Scalar(255, 0, 255), 2);
}

void draw_centerPoints(Mat HSV, Mat IMG){
    int column = HSV.cols;
    int row = 400;
    double H = 0.0, S = 0.0, V = 0.0;
    int cnt = 0;
    int start_flag = 0;
    int start_point = 0;

    for(int i = 0; i < column; i++){
        V = HSV.at<Vec3b>(300, i)[1];
        cout << V << endl;
        if(102 < V < 255 && start_flag == 0){
            start_point = i;
            start_flag = 1;
        }
        if(102 < V < 255 && start_flag == 1){
            cnt++;
        }
        waitKey(1);
    }

    putText(IMG, "For the greater good", Point(column, start_point + cnt/2), FONT_HERSHEY_PLAIN, 2, Scalar(0, 69, 255), 1);
}