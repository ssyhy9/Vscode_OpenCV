
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>



using namespace std;
using namespace cv;

#include "image_compare.h"

//float compareImages(Mat cameralmage, Mat librarySymbol);
//Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints);
//int image(Mat frame); //$$$$$$$$$$
//int library(Mat detect);
//
//int shapes(int num);
//int light(Mat frame);




////*****************************************************************************************************
//int Main (void){
//     int num;
//
//     Mat frame=imread("stop(1).bmp"); //读图片
//
//
//    if (frame.empty()) {
//        printf("not open");
//        return -1;
//     } //检查图片能否打开
//
//     num = image(frame);   //对输入图片进行 边缘检测 转换 match
//
//
//     if(num==1) {
//        cout<<"Count Shape"<<endl;
//        shapes(num); //输出相应形状数量
//     }
//     if(num==2) {
//        cout<<"Count Shape"<<endl;
//        shapes(num); //输出相应形状数量
//     }
//       if(num==3) {
//        cout<<"Count Shape"<<endl;
//        shapes(num); //输出相应形状数量
//     }
//
//     if(num==4||num==5||num==6||num==7) {
//        cout<<"Short Cut"<<endl;
//     }
//     if(num==8) {
//        cout<<"Play Music"<<endl;
//     }
//     if(num==9) {
//        cout<<"Alarm Flash"<<endl;
//     }
//     if(num==10) {
//        cout<<"Approach and Stop"<<endl;
//     }
//     if(num==11) {
//        cout<<"Traffic Light"<<endl;
//        //light(frame1);
//     }
//     if(num==12) {
//        cout<<"Kick Football"<<endl;
//     }
//
//     waitKey(0);
//
//    }

//*******************************************************************************************************



//视角转换内部函数 ***************************************************************
Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints)
{
	Mat dst;
	Mat Trans = getPerspectiveTransform(scrPoints, dstPoints);
	warpPerspective(src, dst, Trans, Size(src.cols, src.rows));
	return dst;
}
//********************************************************************************




//in range // contours// transform // match ***********************************************************
int image(Mat frame)
{
     int num1,num2;
     int t;
     int area_max=0;
     Mat image1,image2;
     Mat dstlmg(frame.size(),CV_8UC3,Scalar::all(0)); //Õ¼ÄÚ´æ

     int height=frame.rows;
     int width=frame.cols;

     // *8**8**8**8*8**8**in range (HSV)
    cvtColor(frame, frame, COLOR_BGR2HSV);
    inRange(frame,Scalar(140,100,100),Scalar(250,255,255),frame);
    //imshow("frame1",frame);

     //**8**8***8**8 Contours
    vector<vector<Point>> contours;
    vector<Point> point;
    vector<Vec4i> hireachy;
    findContours(frame, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    //cout <<contours.size() <<endl;

    vector<vector<Point>> contours_poly(contours.size()); //定义轮廓坐标

    for (int i=0;i<contours.size();i++){
    int area = contourArea(contours[i]); //每个轮廓面积
    //cout<<"area"<<area<<endl;

    if( contourArea(contours[i]) > area_max)  //寻找最大面积轮廓
    {
            area_max=contourArea(contours[i]);
           // cout<<"area max"<<area_max<<endl;
             t=i; //最大面积轮廓对应在数组中的位置
    }
    approxPolyDP(contours[i],contours_poly[i],30,true);
    drawContours(dstlmg,contours_poly,i,Scalar(0,255,255),2,8);
      // imshow("dst",dstlmg);
    }

    // cout<<"t"<<t<<endl; //最大轮廓位置

   /* for (int i=0;i<4;i++){
     cout<<" final contour:"<<contours_poly[t][i]<<endl; //最大轮廓坐标
    } */

    //transform //分了两种情况讨论 两种输入 一种输出
    Point2f AffinePoints1[4] = {  Point2f(contours_poly[t][1]),Point2f(contours_poly[t][0]),Point2f(contours_poly[t][2]),Point2f(contours_poly[t][3]) };
    Point2f AffinePoints2[4] = {  Point2f(contours_poly[t][0]),Point2f(contours_poly[t][3]),Point2f(contours_poly[t][1]),Point2f(contours_poly[t][2]) };
    Point2f AffinePoints0[4] = { Point2f(0, 0), Point2f(width,0), Point2f(0,height), Point2f(width, height) };
    Mat dst_perspective1 = PerspectiveTrans(frame, AffinePoints1, AffinePoints0);
    Mat dst_perspective2 = PerspectiveTrans(frame, AffinePoints2, AffinePoints0);
    resize(dst_perspective1,image1,Size(320,240),0,0,INTER_AREA); //改变图片大小
    resize(dst_perspective2,image2,Size(320,240),0,0,INTER_AREA);
  // imshow("perspective1", image1);
  //imshow("perspective2", image2);

    num1=library(image1); //图像与图库中图片进行比对
    num2=library(image2);

//两种情况都进行了比对，有一种情况比对成功，则为识别成功，返回识别出的图片对于的数值
    if (num1>0) {
        return num1;
    }
    if (num2>0) {
       return num2;
    }

} //********************************************************************************************************************************************




//match率函数计算*******************************************************************************
float compareImages(Mat cameralmage, Mat librarySymbol){
    float matchPercent = 100 -(100/((float)librarySymbol.cols*(float)librarySymbol.rows) *(2*(float)countNonZero(librarySymbol^cameralmage)));
    return matchPercent;
} //****************************************************************************************************************




//图库 ********************************************************************
int library(Mat detect) {

int num=0;

float MP,MP1,MP2,MP3,MP4,MP5,MP6,MP7,MP8,MP9,MP10,MP11,MP12;

Mat library1= imread("CountShape1.bmp"); //Í¼¿â
Mat library2= imread("CountShape2.bmp"); //Í¼¿â
Mat library3= imread("CountShape3.bmp"); //Í¼¿â
Mat library4= imread("ShortCutBlue.bmp"); //Í¼¿â
Mat library5= imread("ShortCutGreen.bmp"); //Í¼¿â
Mat library6= imread("ShortCutRed.bmp"); //Í¼¿â
Mat library7= imread("ShortCutYellow.bmp"); //Í¼¿â
Mat library8= imread("PlayAudio.bmp"); //Í¼¿â
Mat library9= imread("Alarm.bmp"); //Í¼¿â
Mat library10= imread("MeasureDistance.bmp"); //Í¼¿â
Mat library11= imread("TrafficLight.bmp"); //Í¼¿â
Mat library12= imread("Football.bmp"); //Í¼¿â

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
      if( MP1 > 87) {
        num=1;
        MP=MP1;
        break;
      }
     MP2=compareImages(detect,library2_binary);
      if( MP2 > 85) {
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
      if( MP4 > 75) {
        num=4;
        MP=MP4;
        break;
      }
       MP5=compareImages(detect,library5_binary);
      if( MP5 > 75) {
        num=5;
        MP=MP5;
        break;
      }
       MP6=compareImages(detect,library6_binary);
      if( MP6 > 75) {
        num=6;
        MP=MP6;
        break;
      }
       MP7=compareImages(detect,library7_binary);
      if( MP7 > 75) {
        num=7;
        MP=MP7;
        break;
      }
       MP8=compareImages(detect,library8_binary);
      if( MP8 > 75) {
        num=8;
        MP=MP8;
        break;
      }
       MP9=compareImages(detect,library9_binary);
      if( MP9 > 75) {
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
      if( MP12 > 75) {
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
} //**************************************************************************





// count shapes*******************************************
int shapes(int num) {

    if (num ==1) {
    cout << "T:3" << endl;
    cout << "R:2" << endl;
    cout << "C:3" << endl;
    }

      if (num ==2) {
    cout << "T:2" << endl;
    cout << "R:2" << endl;
    cout << "C:2" << endl;
    }

  if (num ==3) {
    cout << "T:2" << endl;
    cout << "R:1" << endl;
    cout << "C:2" << endl;
    }

 } //*************************************************************


// traffic light*******************************************************
int light(Mat frame){

 double area;
 Rect roi;
  //Mat frame=imread("traffic2.bmp");
 Mat dst;
 Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
 Mat kernel_dilite = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));

  //筛选出绿色
  inRange(frame, Scalar(0, 127, 0), Scalar(120, 255, 120), dst);

  //开操作去噪点
  morphologyEx(dst, dst, MORPH_OPEN, kernel, Point(-1, -1), 1);

  //膨胀操作把绿灯具体化的显示出来
  dilate(dst, dst, kernel_dilite, Point(-1, -1), 2);
 // imshow("output video", dst);

  vector<vector<Point>>contours;
  vector<Vec4i>hierarchy;
  findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(-1, -1));

  if (contours.size() > 0)
{
   for (size_t i = 0; i < contours.size(); i++)
  {
   double contours_Area = contourArea(contours[static_cast<int>(i)]);//面积
   roi = boundingRect(contours[static_cast<int>(i)]);//外接矩形
   if (contours_Area > area)
    {
    area = contours_Area;   }
  }
}
 else{  roi.x = roi.y = roi.width = roi.height = 0; }

             if(area>=800)  //根据距离调参数 （800）
        {
              cout<<"go"<<endl;
        }
        else{
             cout<<"stop"<<endl;
        }
  //imshow("input video", frame);
  waitKey(0);
  return 0;
}//**********************************************************************

