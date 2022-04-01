#ifndef __IMAGE_COMPARE_H
#define __IMAGE_COMPARE_H

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



void task(void);
Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints);
int library(Mat detect);
float compareImages(Mat cameralmage, Mat librarySymbol);
int compare_task(Mat frame);

extern VideoCapture capture;



#endif
