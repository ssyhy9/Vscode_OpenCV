#ifndef IMAGE_COMPARE_H_INCLUDED
#define IMAGE_COMPARE_H_INCLUDED

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

float compareImages(Mat cameralmage, Mat librarySymbol);
Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints);
int image(Mat frame); //$$$$$$$$$$
int library(Mat detect);

int shapes(int num);
int light(Mat frame);


#endif // IMAGE_COMPARE_H_INCLUDED
