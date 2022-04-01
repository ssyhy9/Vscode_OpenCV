#ifndef __IMAGE_COMPARE_H
#define __IMAGE_COMPARE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

void task(void);
Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints);
int library(Mat detect);
float compareImages(Mat cameralmage, Mat librarySymbol);
int compare_task(Mat frame);


#endif // IMAGE_COMPARE_H_INCLUDED