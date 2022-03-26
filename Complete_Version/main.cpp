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

#define PWM_PIN     1
#define leftMotorBaseSpeed 30
#define rightMotorBaseSpeed 30
#define min_speed -50
#define max_speed 50

using namespace cv;
using namespace std;

void findColour(Mat img);
int scanMask(Mat MASK);
float draw_centerPoints(Mat gray, Mat im);
void colour_line_following(Mat mask);

void pwm_init(void);
void compare_task(void);

int vehicle_init(void);
float PID(int lineDist);
void standardMove(float output);
void stopMotor(void);

vector<vector<int>> myColours{
    // {0, 0, 0, 32, 255, 255}, //black
     {0, 82, 130, 30, 255, 210}, //yellow     i = 0
    {132, 168, 120, 179, 236, 226} //pink     i = 1
};

float Kp = 0.04, Ki = 0.0, Kd = 0.1;    //encoder output
float leftMotorSpeed = 0; // Variables to hold the current motor speed (+-100%)
float rightMotorSpeed = 0;
int robot;

int colour_flag;

//Mat img;
VideoCapture capture;

int main(void)
{
    Mat img, imgGRAY;
    int error;

    wiringPiSetup();        //Initialise WiringPi
    capture.open(0);    //640*480???
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);    

    pwm_init();
    vehicle_init();

    while(1)
    {
        capture.read(img);
        cout << img.size() << endl; 

        transpose(img, img);
        flip(img, img, 1);
        transpose(img, img);
        flip(img, img, 1);
        imshow("Ori IMG", img);

        cvtColor(img, imgGRAY, COLOR_BGR2GRAY);

        findColour(img);

        if(colour_flag == 0){
            error = draw_centerPoints(imgGRAY, img);
            float output = PID(error); // Calculate the PID output.
            standardMove(output);
        }

        waitKey(2);
    }

    serialClose(robot);

    return 0;
}

//********************************** Colour && Line ***********************************************
void findColour(Mat img){
    Mat imgHSV;
    int yellow_cnt = 0, pink_cnt = 0;
    cvtColor(img, imgHSV, COLOR_BGR2HSV);

    for(int i = 0; i < myColours.size(); i++){
        Mat mask; 

        Scalar lower(myColours[i][0], myColours[i][1], myColours[i][2]);
        Scalar upper(myColours[i][3], myColours[i][4], myColours[i][5]);
        inRange(imgHSV, lower, upper, mask);

        if(i == 0){
            yellow_cnt = scanMask(mask);
            cout << "yellow cnt:" << yellow_cnt << endl;
            if(yellow_cnt > 100){
                colour_flag = 1;
                colour_line_following(mask);
            }
            else{
                colour_flag = 0;
            }
        }

        else if(i == 1){
            pink_cnt = scanMask(mask);
            cout << "pink cnt:" << pink_cnt << endl;
            if(pink_cnt > 200){
                capture.release();
                waitKey(300);
                stopMotor();
                compare_task();
            }
        }

        else if(i == 2){

        }

        else if(i == 3){
            
        }

        imshow(to_string(i), mask);
        imshow("Image HSV", imgHSV);
    }
}

int scanMask(Mat MASK){
    const int row = 300;
    const int x = 400;
    const int y = 5;
    int colour = 0;
    int cnt = 0;

    for(int r = 0; r < y; r++){
        for(int i = 0; i < x; i++){
            colour = MASK.at<uchar>(row + r, 150 + i);
            if(colour > 100){
                cnt ++;
            }
        }
    }
    return cnt;
}

float draw_centerPoints(Mat gray, Mat im){
    Mat binary;
    Size dsize=Size(160,100);
    resize(im,im,dsize,0,0,INTER_AREA);

    imshow("Resized img", im);

    cvtColor(im, gray, COLOR_BGR2GRAY);
    threshold(gray, binary, 10, 255, THRESH_BINARY_INV);

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

        if (range == 0 ) x = 0;
        else x = x / range;

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
            cout << "Real Error" << real_error << endl;

            circle(im, Point(x, 55), 3, Scalar(255, 0, 255), FILLED);
            imshow("Im", im);
            //sprintf(text,"(%d,%d)", x, 55);
            //putText(im, text, Point(start_point + cnt/2 - 50, row - 30), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255), 1);
    return real_error;   //|200| - |1400|
}

void colour_line_following(Mat mask){
    int left = 0, right = 0, real_error = 0, x = 0, range = 0, grayValue = 0;

        for (int i = 0; i < mask.cols(); i++)//for loop to read the grayvalue of each pixel point on row 250
        {
            grayValue = (int)binary.at<uchar>(305, i);//read the grayvalue of point ( 250, c )
            if (grayValue == 255){
                x += i;//gain the sum of c
                range ++;//calculate the number of pixel that satisfy grayvalue = 0
            }
        }

        if (range == 0 ) x = 0;
        else x = x / range;

    for (int row = 300; row < 310; row++)
    {
        for (int m = 0; m < x; m++){
            int intensity1 = binary.at<uchar>(row, m);
            if(intensity1 < 100) {                    
                  left++;
            }
        }

        for (int n = x; n < 160; n++){
            int intensity2=binary.at<uchar>(row, n);
            if(intensity2 < 100) {
                right++;
            }
        }
    }

    real_error = left - right;
    float colour_error = PID(real_error);
    standardMove(colour_error); 
}

//*********************************** compare img ***********************************************************************
void pwm_init(void){
    pinMode(PWM_PIN, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);
    pwmSetRange(2000);
}

void compare_task(void){
    pwmWrite(PWM_PIN, 100);  //up
    waitKey(500);
    capture.open(0);
    for(int i = 0; i < 5; i++){
        Mat img;
        int num;
        capture.read(img);
        //num = img_compare(img);
        if(){
            
            return 
        }
        waitKey(100);
    }
    pwmWrite(PWM_PIN,180);  //down
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

// Function to calculate the PID output.
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
        leftMotorSpeed = leftMotorBaseSpeed - output;     // Calculate the modified motor speed
        rightMotorSpeed = rightMotorBaseSpeed + output;

        printf("speed: %f; %f output:%f\n", leftMotorSpeed, rightMotorSpeed, output);

        if(rightMotorSpeed > 0 && leftMotorSpeed > 0)
        {
            if (rightMotorSpeed > 50) rightMotorSpeed = 50;
            if (leftMotorSpeed > 50) leftMotorSpeed = 50;
            serialPrintf(robot, "#Baffff %d", (int)leftMotorSpeed);
            serialPrintf(robot, "#Baffff %d", (int)leftMotorSpeed);
            serialPrintf(robot, "#Baffff %d", (int)rightMotorSpeed);
            serialPrintf(robot, "#Baffff %d", (int)rightMotorSpeed); 
        }


        if(leftMotorSpeed < 0)
        {
            if (rightMotorSpeed > 50) rightMotorSpeed = 50;
            if (leftMotorSpeed < -50) leftMotorSpeed = -50;
            serialPrintf(robot, "#Barrff %d", -(int)leftMotorSpeed);
            serialPrintf(robot, "#Barrff %d", -(int)leftMotorSpeed);
            serialPrintf(robot, "#Barrff %d", (int)rightMotorSpeed);
            serialPrintf(robot, "#Barrff %d", (int)rightMotorSpeed);
        }

        if(rightMotorSpeed < 0)
        {
            if (rightMotorSpeed < -50) rightMotorSpeed = -50;
            if (leftMotorSpeed > 50) leftMotorSpeed = 50;
            serialPrintf(robot, "#Baffrr %d", (int)leftMotorSpeed);
            serialPrintf(robot, "#Baffrr %d", (int)leftMotorSpeed);
            serialPrintf(robot, "#Baffrr %d", -(int)rightMotorSpeed);
            serialPrintf(robot, "#Baffrr %d", -(int)rightMotorSpeed);
        }
}

void stopMotor(void){
    serialPrintf(robot, "#Baffff %d", 0);
    serialPrintf(robot, "#Baffff %d", 0);
    serialPrintf(robot, "#Baffff %d", 0);
    serialPrintf(robot, "#Baffff %d", 0); 
}

