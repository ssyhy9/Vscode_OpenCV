
#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include <sys/time.h>
#include <wiringSerial.h>
#include <wiringPi.h>           //WiringPi headers
#include <lcd.h>                //LCD headers from WiringPi
#include <stdio.h>              //Needed for the printf function below

#include <wiringPi.h>
#include <softPwm.h>

using namespace cv;
using namespace std;

#include "image_compare.h"
#include "main.h"
#include "tasks.h"

void task(void){

    correcting();

    delay(80);
    softPwmWrite(PWM_PIN,6);  //up
    delay(700);
    softPwmWrite(PWM_PIN, 0);
    cout<<"UP_UP_UP_UP_UP_UP"<<endl;
    delay(80);
    capture.open(0); //因为拍照开相机
    capture.set(CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CAP_PROP_FRAME_HEIGHT, 1024);
    capture.set(CAP_PROP_FPS, 30);

    int i = 0;

     while(i<10) {

      Mat img;
      int task_code = 0;

     capture.read(img);

    // imshow("camera",img);
     cout<<"START COMPARE"<<endl;
     task_code = compare_task(img);//match
     cout<<task_code<<endl;

    if(task_code!=0){
     cout<<"DO------TASK!"<<endl;
       active_task(task_code,img);
     cout<<"-----------------------DONE-----------------———"<<endl;
       break;
    } // do task
        i++;
        //waitKey(1);
   } //拍十次照片

    capture.release(); //做完任务 拍完了 重启一下
    delay(100);
    softPwmWrite(PWM_PIN, 12);  //down
    delay(700);
    softPwmWrite(PWM_PIN, 0);
    cout<<"DOWN-DOWN-DOWN-DOWN"<<endl;
     delay(100);
    capture.open(0); // 开摄像头接着巡线
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);
   // capture.set(CAP_PROP_FPS, 30);
}

void correcting(void)
{
    float Error = 0, output = 0;
    Mat img, imgHSV, mask;
    capture.open(0);
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);

    while( Error != 0 )
    {

        capture.read(img); //640 480

        Size dsize=Size(160,100);
        resize(img,img,dsize,0,0,INTER_AREA);
        //imshow("Ori", img);
        cvtColor(img, imgHSV, COLOR_BGR2HSV);

        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(imgHSV, lower, upper, mask);

        Error = back_center( mask );
        output = PID(Error, normal_pid);
        correct_move(output);
    }

    waitKey(1);
    capture.release();
}


float back_center( Mat mask ){

    int right_num = 0, left_num = 0;
    int num = 0;
    int colour = 0;
   // imshow("Img Bi", mask);

    int left = 0, right = 0, real_error = 0, x = 0, range = 0, grayValue = 0;

        for(int r = 80; r < 90; r++){
            for (int i = 0; i < mask.cols; i++ )//for loop to read the grayvalue of each pixel point on row 250
            {
                grayValue = (int)mask.at<uchar>(r, i);//read the grayvalue of point ( 250, c )
                if (grayValue == 255){
                    x += i;//gain the sum of c
                    range ++;//calculate the number of pixel that satisfy grayvalue = 0
                }
            }
        }

         if(range == 0){
          // x = 80;
            left_turn();
//          left_sharp_turn(turn_cnt);
            stopMotor();
         }
         else{
            x = x / range;
            turn_flag = 0;
         }

        for (int row = 80; row < 90; row++)
        {
            for (int m = 0; m < x ;m++){
                 int intensity1 = mask.at<uchar>(row, m);
                 if(intensity1 < 100) {
                     left++;
                 }
            }

            for (int n = x; n < 160; n++){
                 int intensity2 = mask.at<uchar>(row,n);
                 if(intensity2 < 100) {
                     right++;
                 }
            }
        }
        real_error = left - right;
      cout << "Real Error" << real_error << endl;
    return real_error;   //|200| - |1400|
}

void correct_move( float output )
{
    serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", (int)output, (int)output, (int)output, (int)output);
}

//********************************************************* Choose Task ***************************************************
float compareImages(Mat cameralmage, Mat librarySymbol){
    float matchPercent = 100 -(100/((float)librarySymbol.cols*(float)librarySymbol.rows) *(2*(float)countNonZero(librarySymbol^cameralmage)));
    return matchPercent;
}
Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints){
	Mat dst;
	Mat Trans = getPerspectiveTransform(scrPoints, dstPoints);
	warpPerspective(src, dst, Trans, Size(src.cols, src.rows));
	return dst;
}
int library(Mat detect) {

int num=0;

float MP,MP1,MP2,MP3,MP4,MP5,MP6,MP7,MP8,MP9,MP10,MP11,MP12;

Mat library1= imread("/home/pi/Desktop/CountShape1.bmp");
Mat library2= imread("/home/pi/Desktop/CountShape2.bmp");
Mat library3= imread("/home/pi/Desktop/CountShape3.bmp");
Mat library4= imread("/home/pi/Desktop/ShortCutBlue.bmp");
Mat library5= imread("/home/pi/Desktop/ShortcutGreen.bmp");
Mat library6= imread("/home/pi/Desktop/ShortcutRed.bmp");
Mat library7= imread("/home/pi/Desktop/ShortcutYellow.bmp");
Mat library8= imread("/home/pi/Desktop/PlayAudio.bmp");
Mat library9= imread("/home/pi/Desktop/Alarm.bmp");
Mat library10= imread("/home/pi/Desktop/MeasureDistance.bmp");
Mat library11= imread("/home/pi/Desktop/TrafficLight.bmp");
Mat library12= imread("/home/pi/Desktop/Football.bmp");

Mat library1_gray,library2_gray,library3_gray,library4_gray,library5_gray,library6_gray,library7_gray,library8_gray,library9_gray,library10_gray,library11_gray,library12_gray;
Mat library1_binary,library2_binary,library3_binary,library4_binary,library5_binary,library6_binary,library7_binary,library8_binary,library9_binary,library10_binary,library11_binary,library12_binary;

cvtColor(library1,library1_gray,COLOR_BGR2GRAY);
threshold(library1_gray,library1_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library1_binary=~library1_binary;

cvtColor(library2,library2_gray,COLOR_BGR2GRAY);
threshold(library2_gray,library2_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library2_binary=~library2_binary;

cvtColor(library3,library3_gray,COLOR_BGR2GRAY);
threshold(library3_gray,library3_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library3_binary=~library3_binary;

cvtColor(library4,library4_gray,COLOR_BGR2GRAY);
threshold(library4_gray,library4_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library4_binary=~library4_binary;

cvtColor(library5,library5_gray,COLOR_BGR2GRAY);
threshold(library5_gray,library5_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library5_binary=~library5_binary;

cvtColor(library6,library6_gray,COLOR_BGR2GRAY);
threshold(library6_gray,library6_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library6_binary=~library6_binary;

cvtColor(library7,library7_gray,COLOR_BGR2GRAY);
threshold(library7_gray,library7_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library7_binary=~library7_binary;

cvtColor(library8,library8_gray,COLOR_BGR2GRAY);
threshold(library8_gray,library8_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library8_binary=~library8_binary;

cvtColor(library9,library9_gray,COLOR_BGR2GRAY);
threshold(library9_gray,library9_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library9_binary=~library9_binary;

cvtColor(library10,library10_gray,COLOR_BGR2GRAY);
threshold(library10_gray,library10_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library10_binary=~library10_binary;

cvtColor(library11,library11_gray,COLOR_BGR2GRAY);
threshold(library11_gray,library11_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library11_binary=~library11_binary;

cvtColor(library12,library12_gray,COLOR_BGR2GRAY);
threshold(library12_gray,library12_binary,0,255,THRESH_BINARY|THRESH_OTSU);
library12_binary=~library12_binary;

while(1) {

     MP1=compareImages(detect,library1_binary);
      if( MP1 > 75 ) {
        num=1;
        MP=MP1;
        break;
      }
     MP2=compareImages(detect,library2_binary);
      if( MP2 > 75 ) {
        num=2;
        MP=MP2;
        break;
      }
       MP3=compareImages(detect,library3_binary);
      if( MP3 > 75) {
        num=3;
        MP=MP3;
        break;
      }
       MP4=compareImages(detect,library4_binary);
      if( MP4 > 86) {
        num=4;
        MP=MP4;
        break;
      }
       MP5=compareImages(detect,library5_binary);
      if( MP5 > 85) {
        num=5;
        MP=MP5;
        break;
      }
       MP6=compareImages(detect,library6_binary);
      if( MP6 > 85) {
        num=6;
        MP=MP6;
        break;
      }
       MP7=compareImages(detect,library7_binary);
      if( MP7 > 70) {
        num=7;
        MP=MP7;
        break;
      }
       MP8=compareImages(detect,library8_binary);
      if( MP8 > 70) {
        num=8;
        MP=MP8;
        break;
      }
       MP9=compareImages(detect,library9_binary);
      if( MP9 > 60) {
        num=9;
        MP=MP9;
        break;
      }
       MP10=compareImages(detect,library10_binary);
      if( MP10 > 70) {
        num=10;
        MP=MP10;
        break;
      }
       MP11=compareImages(detect,library11_binary);
      if( MP11 > 60) {
        num=11;
        MP=MP11;
        break;
      }
       MP12=compareImages(detect,library12_binary);
      if( MP12 > 50) {
        num=12;
        MP=MP12;
        break;
      }
      if (num==0) {
         MP=0;
        break;
      }
}
    cout<<"MP:"<<MP<<endl;
    return num;
}

int compare_task(Mat frame) {

     int num1,num2, tape, t;
     int area_max=0;
     Mat frame1,frame2;
     Mat image1,image2;
     Mat dstlmg(frame.size(),CV_8UC3,Scalar::all(0));
     int height=frame.rows;
     int width=frame.cols;

    cvtColor(frame, frame1, COLOR_BGR2HSV);
    inRange(frame1,Scalar(150,100,100),Scalar(170,255,255),frame2);
    //imshow("frame2",frame2); //二值化后图像

    vector<vector<Point>> contours;
    vector<Point> point;
    vector<Vec4i> hireachy;
    findContours(frame2, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    vector<vector<Point>> contours_poly(contours.size());
    cout<<"contours size:"<<contours.size()<<endl;//轮廓数目

     if (contours.size() >5) {
    for (int i=0;i<contours.size();i++)
    {
        int area = contourArea(contours[i]);

        if( contourArea(contours[i]) > area_max) {
            area_max=contourArea(contours[i]);
            t=i;
        }
        approxPolyDP(contours[i],contours_poly[i],30,true);
        drawContours(dstlmg,contours_poly,i,Scalar(0,255,255),2,8);
        // imshow("dst",dstlmg); //轮廓图片
    }

    Point2f AffinePoints1[4] = {  Point2f(contours_poly[t][1]),Point2f(contours_poly[t][0]),Point2f(contours_poly[t][2]),Point2f(contours_poly[t][3]) };
    Point2f AffinePoints2[4] = {  Point2f(contours_poly[t][0]),Point2f(contours_poly[t][3]),Point2f(contours_poly[t][1]),Point2f(contours_poly[t][2]) };
    Point2f AffinePoints0[4] = { Point2f(0, 0), Point2f(width,0), Point2f(0,height), Point2f(width, height) };
    Mat dst_perspective1 = PerspectiveTrans(frame2, AffinePoints1, AffinePoints0);
    Mat dst_perspective2 = PerspectiveTrans(frame2, AffinePoints2, AffinePoints0);
    resize(dst_perspective1,image1,Size(320,240),0,0,INTER_AREA);
    resize(dst_perspective2,image2,Size(320,240),0,0,INTER_AREA);

  // imshow("perspective1", image1); //转换图片1
  // imshow("perspective2", image2); //转换图片2

    num1=library(image1);
    num2=library(image2);

    if (num1 > 0) {tape = num1;}
    else if (num2 > 0){tape = num2;}
    else{ tape = 0;}
     } //if contours !=0

        else{
        tape = 0;
       } //if contours =0

    //cout<<"tape1:"<<tape<<endl;
    return tape;
}

