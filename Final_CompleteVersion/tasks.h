
#ifndef __TASKS_H
#define __TASKS_H


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

#include "image_compare.h"

int active_task(int code,Mat camera);
int task_kickball(void);
int task_dissensor(void);
float disMeasure(void);
void ultraInit(void);
int task_spaker(void);
int task_led(void);
int count_shape(Mat frame);
void getContours(Mat src);
int cheat(int num);
int light(Mat frame);
int traffic_light(void);

extern VideoCapture capture;
extern int pink_num;
extern int robot,lcd;
extern const int RedPin;
extern const int BluePin;



#endif // TASKS_H_INCLUDED
