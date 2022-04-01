
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
#include "tasks.h"
#include "main.h"

int vehicle_init(void);
void pwm_init(void);

float PID(int lineDist, float* PID);

void standardMove(float output);
void highMove(float output);
void lowMove(float output);
void stopMotor(void);

void left_sharp_turn(int cnt);
void right_rect_turn(void);
void left_rect_turn(void);
void left_turn();

void findColour(Mat img);
int scanMask(Mat MASK);
float draw_centerPoints(Mat gray, Mat im);
void colour_line_following(Mat mask,int num);
int LCD_colour (int num);

int move_cnt;


vector<vector<int>> myColours{
    // {0, 0, 0, 32, 255, 255}, //black
    {140, 125, 94, 165, 255, 255}, //pink    i = 0
    {0, 82, 130, 30, 255, 210}, //yellow      i = 1
    {35, 73, 34, 86, 255, 255},  //green     i = 2
    {101, 64, 70, 125, 208, 130}, //blue        i = 3
    {166, 120, 94, 179, 255, 255} //red       i = 4
};

const float Kp = 0.021, Ki = 0.00, Kd = 0.0;    //blackl
float normal_pid[3] = {0.022, 0 , 0};
float colour_pid[3] = {0.030, 0, 0};
float green_pid[3] = {0.05414, 0.01, 0.015};
float red_pid[3] = {0.028, 0, 0};
const float KP = 0.031, KI = 0, KD = 0;
float leftMotorSpeed = 0; // Variables to hold the current motor speed (+-100%)
float rightMotorSpeed = 0;
int robot,lcd;
int pink_num = 0;
int turn_flag, stop_flag;
int colour_flag =5;
int turn_cnt = 1;
const int RedPin=23;
const int BluePin=22;

VideoCapture capture;

int hmin = 0, smin = 0, vmin = 0;
int hmax = 179, smax = 132, vmax = 76;

int main(void)
{
    Mat img, imgHSV;
    Mat mask;
    int error;
    float output;

    wiringPiSetup();         //Initialise WiringPi
    pwm_init();
    vehicle_init();
    capture.open(0);         //640*480
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);

//   namedWindow("Trackbars", (600, 200));
//        createTrackbar("Hue min", "Trackbars", &hmin, 179);
//        createTrackbar("Hue max", "Trackbars", &hmax, 179);
//        createTrackbar("Sat min", "Trackbars", &smin, 255);
//        createTrackbar("Sat max", "Trackbars", &smax, 255);
//        createTrackbar("Val min", "Trackbars", &vmin, 255);
//        createTrackbar("Val max", "Trackbars", &vmax, 255);

    while(1)
    {
        capture.read(img); //640 480

        Size dsize=Size(160,100);
        resize(img,img,dsize,0,0,INTER_AREA);
        //imshow("Ori", img);
        cvtColor(img, imgHSV, COLOR_BGR2HSV);

        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(imgHSV, lower, upper, mask);

        findColour(img);

        if( colour_flag == 5){
            error = draw_centerPoints(mask, img);
            output = PID(error, normal_pid);
            standardMove(output);
        }
        waitKey(1);
    }

    serialClose(robot);

    return 0;
}


void findColour(Mat img){

    int yellow_cnt = 0, pink_cnt = 0, green_cnt = 0, blue_cnt = 0, red_cnt = 0;

     Mat imgHSV0,imgHSV1,imgHSV2,imgHSV3,imgHSV4;
     Mat mask0,mask1,mask2,mask3,mask4;

       if( pink_num == 1 || pink_num == 9 || pink_num ==17) {

        cvtColor(img, imgHSV1, COLOR_BGR2HSV);

        Scalar lower1(myColours[1][0], myColours[1][1], myColours[1][2]);
        Scalar upper1(myColours[1][3], myColours[1][4], myColours[1][5]);
        inRange(imgHSV1, lower1, upper1, mask1);
        //imshow("yellow", mask1);

           yellow_cnt = scanMask(mask1);
           cout << "yellow cnt:" << yellow_cnt << endl;
           if(yellow_cnt > 50){
               colour_flag = 1;
              colour_line_following(mask1,1);
            }
           else{
               colour_flag = 5;
           }
          } //yellow

          if( pink_num == 2 || pink_num == 10 || pink_num ==18) {

           cvtColor(img, imgHSV2, COLOR_BGR2HSV);

           Scalar lower2(myColours[2][0], myColours[2][1], myColours[2][2]);
           Scalar upper2(myColours[2][3], myColours[2][4], myColours[2][5]);
           inRange(imgHSV2, lower2, upper2, mask2);
            //imshow("Green", mask2);
           green_cnt = scanMask(mask2);
           cout << "green cnt:" << green_cnt << endl;
           if(green_cnt > 50){
               colour_flag = 2;
              colour_line_following(mask2,2);
           }
           else{
               colour_flag = 5;
           }
          } //green

         if( pink_num == 6 || pink_num == 14 || pink_num == 22) {

         cvtColor(img, imgHSV3, COLOR_BGR2HSV);

        Scalar lower3(myColours[3][0], myColours[3][1], myColours[3][2]);
        Scalar upper3(myColours[3][3], myColours[3][4], myColours[3][5]);
        inRange(imgHSV3, lower3, upper3, mask3);
        //imshow("blue", mask3);

           blue_cnt = scanMask(mask3);
           cout << "blue cnt:" << blue_cnt << endl;
           if(blue_cnt > 50){
               colour_flag = 3;
//               float dog = draw_centerPoints(mask3,mask3);
//                float cat = colour_PID(dog);
//                standardMove(cat);
               colour_line_following(mask3,1);
           }
           else{
               colour_flag = 5;
           }
        }//blue

      if( pink_num == 7 || pink_num == 15 || pink_num == 23) {

       cvtColor(img, imgHSV4, COLOR_BGR2HSV);

       Scalar lower4(myColours[4][0], myColours[4][1], myColours[4][2]);
       Scalar upper4(myColours[4][3], myColours[4][4], myColours[4][5]);
       inRange(imgHSV4, lower4, upper4, mask4);
      // imshow("red", mask4);

           red_cnt = scanMask(mask4);
           cout << "red cnt:" << red_cnt << endl;
           if(red_cnt > 50){
               colour_flag = 4;
               colour_line_following(mask4,3);
           }
           else{
               colour_flag = 5;
           }
       } //red

      cvtColor(img, imgHSV0, COLOR_BGR2HSV);

     Scalar lower0(myColours[0][0], myColours[0][1], myColours[0][2]);
     Scalar upper0(myColours[0][3], myColours[0][4], myColours[0][5]);
     inRange(imgHSV0, lower0, upper0, mask0);

           pink_cnt = scanMask(mask0);
           cout << "pink cnt:" << pink_cnt << endl;

           if(pink_cnt > 50) {
            colour_flag = 0;
            pink_num++;
            cout << "Stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            cout << "PINK----NUM:"<< pink_num << endl;
            capture.release();
            delay(255);
            stopMotor();

            if(pink_num !=1 && pink_num != 2 && pink_num !=6 && pink_num !=7 && pink_num !=9 && pink_num !=10 && pink_num !=14 && pink_num !=15&& pink_num !=17 && pink_num !=18 && pink_num !=22 && pink_num !=23 )
             {
              task(); //match and do
              colour_flag = 5;
              cout<<"LINE----FOLLOWING"<<endl;
           }

            else{
                 if(pink_num == 1 || pink_num == 9 || pink_num == 17){
                    lcdPosition(lcd,0,0);
                    lcdPuts(lcd, "Task: ");
                    lcdPosition(lcd,0,1);
                    lcdPuts(lcd, "Short Cut_Yellow");
                    delay(5000);
                    lcdClear(lcd);
                 }
                 if(pink_num == 2 || pink_num == 10|| pink_num == 18){
                    lcdPosition(lcd,0,0);
                    lcdPuts(lcd, "Task: ");
                    lcdPosition(lcd,0,1);
                    lcdPuts(lcd, "Short Cut_Green");
                    delay(5000);
                    lcdClear(lcd);
                 }
                 if(pink_num == 6 || pink_num == 14|| pink_num == 22){
                    lcdPosition(lcd,0,0);
                    lcdPuts(lcd, "Task: ");
                    lcdPosition(lcd,0,1);
                    lcdPuts(lcd, "Short Cut_Blue");
                    delay(5000);
                    lcdClear(lcd);
                 }
                 if(pink_num == 7 || pink_num == 15|| pink_num == 23){
                    lcdPosition(lcd,0,0);
                    lcdPuts(lcd, "Task: ");
                    lcdPosition(lcd,0,1);
                    lcdPuts(lcd, "Short Cut_Red");
                    delay(5000);
                    lcdClear(lcd);
                 }
            capture.open(0);         //640*480
            capture.set(CAP_PROP_FRAME_WIDTH, 640);
            capture.set(CAP_PROP_FRAME_HEIGHT, 480);
    } //end else

        }//pink
       //imshow("000", mask0);
     //    imshow("Image HSV", imgHSV);

} //end fine colour

int scanMask(Mat MASK){
    const int row = 50;
    const int x = 120;
    const int y = 20;
    int colour = 0;
    int cnt = 0;

    for(int r = 0; r < y; r++){
        for(int i = 0; i < x; i++){
            colour = MASK.at<uchar>(row + r, 20 + i);
            if(colour > 100){
                cnt ++;
            }
        }
    }
    return cnt;
}

float draw_centerPoints(Mat mask, Mat im){

    int right_num = 0, left_num = 0;
    int num = 0;
    int colour = 0;
   // imshow("Img Bi", mask);

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
            left_turn();
//          left_sharp_turn(turn_cnt);
            stopMotor();
         }
         else{
            x = x / range;
            turn_flag = 0;
         }

        for (int row = 40; row < 70; row++)
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

void colour_line_following(Mat mask,int num){
    int colour = 0;

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
 //           left_turn();
//          left_sharp_turn(turn_cnt);
 //           stopMotor();
         }
         else{
            x = x / range;
            turn_flag = 0;
         }

        for (int row = 40; row < 70; row++)
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

    float colour_error = 0;

    if(num == 2) { //GREEN
        colour_error = PID(real_error, green_pid);
        highMove(colour_error);
    }
    if(num == 3) {  //RED
        colour_error = PID(real_error, red_pid);
        lowMove(colour_error);
    }
    else{
    colour_error = PID(real_error, normal_pid);
       standardMove(colour_error);
    }
}
//*********************************** pwm ***********************************************************************
void pwm_init(void){
    pinMode(PWM_PIN, OUTPUT);
    softPwmCreate(PWM_PIN,0,200);
    softPwmWrite(PWM_PIN, 17);
    delay(700);
    softPwmWrite(PWM_PIN, 0);
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

    wiringPiSetup();        //Initialise WiringPi
    if(wiringPiSetup() == -1){ //when initialize wiring failed,print messageto screen
        printf("setup wiringPi failed !");
        return 1;
    }

    if (lcd = lcdInit (2, 16,4, LCD_RS, LCD_E ,LCD_D4 , LCD_D5, LCD_D6,LCD_D7,0,0,0,0)){
        printf ("lcdInit failed! \n");
        return -1 ;
    }
}


float PID(int lineDist, float* PID){
  // PID loop

  float error = 0, errorSum = 0, errorOld = 0;  // Variables for the PID loop

  errorOld = error;        // Save the old error for differential component
  error = 0 - lineDist;  // Calculate the error in position
  errorSum += error;

  float proportional = error * PID[0];  // Calculate the components of the PID
  float integral = errorSum * PID[1];
  float differential = (error - errorOld) * PID[2];

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

void highMove(float output){

        leftMotorSpeed = leftMotorHighSpeed - output;     // Calculate the modified motor speed
        rightMotorSpeed = rightMotorHighSpeed + output;


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

         printf("High--speed: %f; %f output:%f\n", leftMotorSpeed, rightMotorSpeed, output);

}
void lowMove(float output){

        leftMotorSpeed = leftMotorLowSpeed - output;     // Calculate the modified motor speed
        rightMotorSpeed = rightMotorLowSpeed + output;

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

        printf("Low__speed: %f; %f output:%f\n", leftMotorSpeed, rightMotorSpeed, output);

}

void stopMotor(void){
    serialPrintf(robot, "#ha");
}

void left_sharp_turn(int cnt){
    if(cnt % 2 == 1){
        capture.release();
        serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 50, 50, 40, 40);
        delay(200);
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

void left_turn(){
    capture.release();
    serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 55, 55, 35, 35);
    delay(200);
    capture.open(0);
}
