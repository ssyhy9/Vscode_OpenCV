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

#define leftMotorBaseSpeed 20
#define rightMotorBaseSpeed 20

float draw_centerPoints(Mat gray, Mat im);

int vehicle_init(void);
float PID(int lineDist);
void standardMove(float output);
void stopMotor(void);
void right_sharp_turn(void);
void left_sharp_turn(void);

const float Kp = 0.07, Ki = 0.00, Kd = 0.0;    //encoder output
float leftMotorSpeed = 0; // Variables to hold the current motor speed (+-100%)
float rightMotorSpeed = 0;
int robot;

VideoCapture capture;

int turn_flag, stop_flag;

int main(void)
{
    Mat img, imgGRAY;
    int error;
    float output;

    vehicle_init();

    wiringPiSetup();         //Initialise WiringPi
    capture.open(0);         //640*480
//    capture.set(CAP_PROP_FPS, 60);
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);

    int cnt;

    while(1)
    {
        capture.read(img); //640 480
   //    cout << "Original size" << img.size() << endl;

   //     imshow("Ori IMG", img);


        cvtColor(img, imgGRAY, COLOR_BGR2GRAY);

        error = draw_centerPoints(imgGRAY, img);

        if(turn_flag == 0){
            output = PID(error);
            standardMove(output);
        }

        waitKey(1);
    }

    serialClose(robot);

    return 0;
}


float draw_centerPoints(Mat gray, Mat im){
    int right_num = 0, left_num = 0;
    Mat binary;
    Size dsize=Size(160,100);
    resize(im,im,dsize,0,0,INTER_AREA);

    cvtColor(im, gray, COLOR_BGR2GRAY);
    threshold(gray, binary, 60, 255, THRESH_BINARY_INV);

    imshow("Img Bi", binary);

    int left = 0, right = 0, real_error = 0, x = 0, range = 0, grayValue = 0;
    char text[100] = "";

        for (int i = 0; i < binary.cols; i++ )//for loop to read the grayvalue of each pixel point on row 250
        {
            grayValue = (int)binary.at<uchar>(55, i);//read the grayvalue of point ( 250, c )
            if ( grayValue == 255 ){
                x += i;//gain the sum of c
                range ++;//calculate the number of pixel that satisfy grayvalue = 0
            }
        }

        if (range == 0){
            left_sharp_turn();
            turn_flag = 1;
            cout << "Turn Flag:" << turn_flag << endl;
        }
        else {
            x = x / range;
            stop_flag = 0;
            turn_flag = 0;
        }

        for (int row = 50; row < 60; row++)
        {
            for (int m = 0; m < x ;m++){
                 int intensity1 = binary.at<uchar>(row, m);
                 if(intensity1 < 100) {
                     left++;
                 }
            }

            for (int n = x;n < 160;n++){
                 int intensity2=binary.at<uchar>(row,n);
                 if(intensity2 < 100) {
                     right++;
                 }
            }
        }

        real_error = left - right;
   //         cout << "Real Error" << real_error << endl;
    return real_error;   //|200| - |1400|
}

//*********************************** Vehicle Moving ***********************************************************
int vehicle_init(void){

    robot = serialOpen("/dev/ttyAMA0", 57600 );
    puts("successfully opened999");

    if(wiringPiSetup() == -1){ //when initialize wiring failed,print messageto screen
        printf("setup wiringPi failed !");
        return 1;
    }

    if ( robot == -1 )
    {
        puts("error in openning robot");
        return -1;
    } // end if

    puts("successfully opened515");
}

float PID(int lineDist)
{
  // PID loop

  float error = 0, errorSum = 0, errorOld = 0;  // Variables for the PID loop

  errorOld = error;        // Save the old error for differential component
  error = 0 - lineDist;  // Calculate the error in position
  errorSum += error;

  float proportional = error * Kp;  // Calculate the components of the PID

  float integral = errorSum * Ki;

  float differential = (error - errorOld) * Kd;

  float output = proportional + integral + differential;  // Calculate the result

  return output;
}

void standardMove(float output){
        leftMotorSpeed = leftMotorBaseSpeed - output;
        rightMotorSpeed = rightMotorBaseSpeed + output;

        if(rightMotorSpeed > 0 && leftMotorSpeed > 0)
        {
            if (rightMotorSpeed > 80) rightMotorSpeed = 80;
            if (leftMotorSpeed > 80) leftMotorSpeed = 80;
            serialPrintf(robot, "#Baffff %03d %03d %03d %03d",(int)rightMotorSpeed, (int)rightMotorSpeed, (int)leftMotorSpeed, (int)leftMotorSpeed);
        }


        if(leftMotorSpeed < 0)
        {
            leftMotorSpeed = leftMotorSpeed * 1.2;
            if (rightMotorSpeed > 70) rightMotorSpeed = 70;
            if (leftMotorSpeed < -80) leftMotorSpeed = -80;
            serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", (int)rightMotorSpeed, (int)rightMotorSpeed, -(int)leftMotorSpeed, -(int)leftMotorSpeed);
        }

        if(rightMotorSpeed < 0)
        {
            rightMotorSpeed = rightMotorSpeed * 1.2;
            if (rightMotorSpeed < -80) rightMotorSpeed = -80;
            if (leftMotorSpeed > 70) leftMotorSpeed = 70;
            serialPrintf(robot, "#Barrff %03d %03d %03d %03d", -(int)rightMotorSpeed, -(int)rightMotorSpeed, (int)leftMotorSpeed, (int)leftMotorSpeed);
        }

        printf("speed: %f; %f output:%f\n", leftMotorSpeed, rightMotorSpeed, output);
}

void stopMotor(void){
    serialPrintf(robot, "#ha");
}

// void right_sharp_turn(void){
//     capture.release();
//     serialPrintf(robot, "#Barrff %03d %03d %03d %03d", 50, 50, 10, 10);
//     delay(200);
//     serialPrintf(robot, "#Baffff %03d %03d %03d %03d", 0, 0, 0, 0);
//     delay(5000);
//     capture.open(0);

// }

void left_sharp_turn(void){
    capture.release();
    serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 50, 50, 10, 10);
    delay(500);
     serialPrintf(robot, "#Baffff %03d %03d %03d %03d", 0, 0, 0, 0);
     delay(5000);
    capture.open(0);
}





