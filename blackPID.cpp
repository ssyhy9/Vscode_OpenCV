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
void left_sharp_turn(int cnt);
void right_rect_turn(void);
void left_rect_turn(void);

const float Kp = 0.080, Ki = 0.00, Kd = 0.0;    //encoder output
float leftMotorSpeed = 0; // Variables to hold the current motor speed (+-100%)
float rightMotorSpeed = 0;
int robot;
int cnt = 1;

VideoCapture capture;

int turn_flag, stop_flag;


int main(void)
{
    Mat img;
    Mat mask;
    int error;
    float output;

    vehicle_init();

    wiringPiSetup();         //Initialise WiringPi
    capture.open(0);         //640*480
//    capture.set(CAP_PROP_FPS, 60);
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);

    while(1)
    {
        capture.read(img); //640 480
     //   flip(img, img, -1);
        Size dsize=Size(160,100);
        resize(img,img,dsize,0,0,INTER_AREA);
   //     imshow("Ori IMG", img);

        Scalar lower(0, 0, 0);
        Scalar upper(179, 132, 76);
        inRange(img, lower, upper, mask);


    error = draw_centerPoints(mask, img);

    if(turn_flag == 0){
        output = PID(error);
        standardMove(output);
    }
    cout << "CNTTTTT: " << cnt << endl;
        waitKey(1);
    }

    serialClose(robot);

    return 0;
}


float draw_centerPoints(Mat mask, Mat im){

    int right_num = 0, left_num = 0;
    int num = 0;
    int colour = 0;
    imshow("Img Bi", mask);

    int left = 0, right = 0, real_error = 0, x = 0, range = 0, grayValue = 0;
    char text[100] = "";
        for(int r = 40; r < 60; r++){
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
            stopMotor();
         }
         else{
            x = x / range;
            turn_flag = 0;           
         }

        for (int row = 50; row < 60; row++)
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
//      cout << "Real Error" << real_error << endl;
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
            if (rightMotorSpeed > 80) rightMotorSpeed = 80;
            if (leftMotorSpeed < -60) leftMotorSpeed = -60;
            serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", (int)rightMotorSpeed, (int)rightMotorSpeed, -(int)leftMotorSpeed, -(int)leftMotorSpeed);
        }

        if(rightMotorSpeed < 0)
        {
            rightMotorSpeed = rightMotorSpeed * 1.2;
            if (rightMotorSpeed < -60) rightMotorSpeed = -60;
            if (leftMotorSpeed > 80) leftMotorSpeed = 80;
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

void left_sharp_turn(int cnt){
    if(cnt % 2 == 1){
        capture.release();
        serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 50, 50, 40, 40);
        delay(500);
        serialPrintf(robot, "#Baffff %03d %03d %03d %03d", 0, 0, 0, 0);
        delay(1000);
        capture.open(0);
    }
    else{
        capture.release();
        serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 70, 70, 20, 20);
        delay(500);
        serialPrintf(robot, "#Baffff %03d %03d %03d %03d", 0, 0, 0, 0);
        delay(1000);
        capture.open(0);
    }
}

void right_rect_turn(){
    cout << "righttttttttttttttttttttttttt!" << endl;
    capture.release();
    serialPrintf(robot, "#Barrff %03d %03d %03d %03d", 20, 20, 65, 65);
    delay(300);
    serialPrintf(robot, "#Baffff %03d %03d %03d %03d", 0, 0, 0, 0);
    delay(5000);
    capture.open(0);   
    
}

void left_rect_turn(){
    cout << "leftTTTTTTTTTTTTTTTTTTTTTTTTTTTT" << endl;
    capture.release();
    serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 65, 65, 20, 20);
    delay(300);
    serialPrintf(robot, "#Baffff %03d %03d %03d %03d", 0, 0, 0, 0);
    delay(5000);
    capture.open(0);
}







