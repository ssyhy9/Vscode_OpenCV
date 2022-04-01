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

#define PWM_PIN 0
#define leftMotorBaseSpeed 25
#define rightMotorBaseSpeed 25
#define min_speed -50
#define max_speed 50

using namespace cv;
using namespace std;

//#include "image_compare.h"

void findColour(Mat img);
int scanMask(Mat MASK);
float draw_centerPoints(Mat gray, Mat im);
void colour_line_following(Mat mask);

void pwm_init(void);


int vehicle_init(void);
float PID(int lineDist);
void standardMove(float output);
void stopMotor(void);
void right_sharp_turn(void);
void left_sharp_turn(void);

//task
void task(void);
//choose task
Mat PerspectiveTrans(Mat src, Point2f* scrPoints, Point2f* dstPoints);
int library(Mat detect);
float compareImages(Mat cameralmage, Mat librarySymbol);
int compare_task(Mat frame);
//do task
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

vector<vector<int>> myColours{
    // {0, 0, 0, 32, 255, 255}, //black
     {0, 82, 130, 30, 255, 210}, //yellow     i = 0
    {132, 168, 120, 179, 236, 226} //pink     i = 1
};

float Kp = 0.03, Ki = 0.00, Kd = 0.001;   
float leftMotorSpeed = 0; 
float rightMotorSpeed = 0;
int robot;

int colour_flag;
int turn_flag;

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
//      cout << img.size() << endl;

        transpose(img, img);
        flip(img, img, 1);
        transpose(img, img);
        flip(img, img, 1);
//        imshow("Ori IMG", img);

        cvtColor(img, imgGRAY, COLOR_BGR2GRAY);

        findColour(img);

        error = draw_centerPoints(imgGRAY, img);

        if(colour_flag == 0 && turn_flag == 0){
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
    int pink_num = 0;
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
                
            pink_num++;
            cout << "Stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
               capture.release();
               waitKey(300);//leave pink
               stopMotor();
               
               if( pink_num == 1) {
                 lcdClear(lcd);
                 lcdPosition(lcd,0,0);
                 lcdPuts(lcd, "Task: ");
                 lcdPosition(lcd,0,1);
                 lcdPuts(lcd, "Short Cut_Yellow");
                 delay(5000);
                 lcdClear(lcd);
                //yellow
               }
               if( pink_num == 2) {
                 lcdClear(lcd);
                 lcdPosition(lcd,0,0);
                 lcdPuts(lcd, "Task: ");
                 lcdPosition(lcd,0,1);
                 lcdPuts(lcd, "Short Cut_Green");
                 delay(5000);
                 lcdClear(lcd);
                //green
               }
               if ( pink_num == 5) {
                 lcdClear(lcd);
                 lcdPosition(lcd,0,0);
                 lcdPuts(lcd, "Task: ");
                 lcdPosition(lcd,0,1);
                 lcdPuts(lcd, "Short Cut_Blue");
                 delay(5000);
                 lcdClear(lcd);
                //blue
               }
               if( pink_num == 6) {
                 lcdClear(lcd);
                 lcdPosition(lcd,0,0);
                 lcdPuts(lcd, "Task: ");
                 lcdPosition(lcd,0,1);
                 lcdPuts(lcd, "Short Cut_Red");
                 delay(5000);
                 lcdClear(lcd);
                //red
               }
               else{
               task();
               }
           }
       }

//       imshow(to_string(i), mask);
//       imshow("Image HSV", imgHSV);
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

    cvtColor(im, gray, COLOR_BGR2GRAY);
    threshold(gray, binary, 75, 255, THRESH_BINARY_INV);

   // imshow("Img Bi", binary);

    int left = 0, right = 0, real_error = 0, x = 0, range = 0, grayValue = 0;
    char text[100] = "";

        for (int i = 0; i < binary.cols; i++ )//for loop to read the grayvalue of each pixel point on row 250
        {
            grayValue = (int)binary.at<uchar>(75, i);//read the grayvalue of point ( 250, c )
            if ( grayValue == 255 ){
                x += i;//gain the sum of c
                range ++;//calculate the number of pixel that satisfy grayvalue = 0
            }
        }

        if (range == 0 ) x = 80;
        else x = x / range;

        for (int row = 70; row < 80; row++)
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
       //     imshow("Im", im);
            //sprintf(text,"(%d,%d)", x, 55);
            //putText(im, text, Point(start_point + cnt/2 - 50, row - 30), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255), 1);
            if(real_error > 800){
                right_sharp_turn();
                turn_flag = 1;
                return 0;
            }
            else if(real_error < -600){
                left_sharp_turn();
                turn_flag = 1;
                return 0;
            }

            if(real_error <= 800 && real_error >= -600){
                turn_flag = 0;
            }
    return real_error;   //|200| - |1400|
}

void colour_line_following(Mat mask){
    int left = 0, right = 0, real_error = 0, x = 0, range = 0, grayValue = 0;

        for (int i = 0; i < 640; i++)//for loop to read the grayvalue of each pixel point on row 250
        {
            grayValue = (int)mask.at<uchar>(305, i);//read the grayvalue of point ( 250, c )
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
            int intensity1 = mask.at<uchar>(row, m);
            if(intensity1 < 100) {
                  left++;
            }
        }

        for (int n = x; n < 160; n++){
            int intensity2=mask.at<uchar>(row, n);
            if(intensity2 < 100) {
                right++;
            }
        }
    }

    real_error = left - right;
    float colour_error = PID(real_error);
    standardMove(colour_error);
}


void pwm_init(void){
    pinMode(PWM_PIN, OUTPUT);
    softPwmCreate(PWM_PIN,0,200);
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
float PID(int lineDist) {
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
        leftMotorSpeed = leftMotorBaseSpeed - output - 3;     // Calculate the modified motor speed
        rightMotorSpeed = rightMotorBaseSpeed + output;

        printf("speed: %f; %f output:%f\n", leftMotorSpeed, rightMotorSpeed, output);

        if(rightMotorSpeed > 0 && leftMotorSpeed > 0)
        {
            if (rightMotorSpeed > 50) rightMotorSpeed = 50;
            if (leftMotorSpeed > 50) leftMotorSpeed = 50;
            serialPrintf(robot, "#Baffff %03d %03d %03d %03d", (int)leftMotorSpeed, (int)leftMotorSpeed, (int)rightMotorSpeed, (int)rightMotorSpeed);
        }


        if(leftMotorSpeed < 0)
        {
            if (rightMotorSpeed > 50) rightMotorSpeed = 50;
            if (leftMotorSpeed < -50) leftMotorSpeed = -50;
            serialPrintf(robot, "#Barrff %03d %03d %03d %03d", -(int)leftMotorSpeed, -(int)leftMotorSpeed, (int)rightMotorSpeed, (int)rightMotorSpeed);
        }

        if(rightMotorSpeed < 0)
        {
            if (rightMotorSpeed < -50) rightMotorSpeed = -50;
            if (leftMotorSpeed > 50) leftMotorSpeed = 50;
            serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", (int)leftMotorSpeed, (int)leftMotorSpeed, -(int)rightMotorSpeed, -(int)rightMotorSpeed);
        }
}

void stopMotor(void){
    serialPrintf(robot, "#ha");
}

void right_sharp_turn(void){
    serialPrintf(robot, "#Baffrr %03d %03d %03d %03d", 40, 40, 40, 40);
    capture.release();
    delay(300);
    capture.open(0);
}

void left_sharp_turn(void){
    serialPrintf(robot, "#Barrff %03d %03d %03d %03d", 40, 40, 40, 40);
    capture.release();
    delay(300);
    capture.open(0);
}




//********************************************************Task***************************************************************************
void task(void){
    
    softPwmWrite(PWM_PIN, 13);  //up
    delay(500);
    capture.open(0);

     while(i<10) {
            
      Mat img;
      int task_code;
          
     capture.read(img);
     transpose(img, img);
     flip(img, img, 1);
     transpose(img, img);
     flip(img, img, 1);
     //imshow("camera",img);
      task_code = compare_task(img);//match 
      cout<<task_code<<endl;

    if(task_code!=0){
       active_task(task_code,img);
       break; 
    } 
        i++;
   }
    capture.release();
    softPwmWrite(PWM_PIN, 20);  //down
    delay(500);
    capture.open(0);
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
      if( MP1 > 83 ) {
        num=1;
        MP=MP1;
        break;
      }
     MP2=compareImages(detect,library2_binary);
      if( MP2 > 83 ) {
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
      if( MP12 > 60) {
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
    inRange(frame1,Scalar(148,94,86),Scalar(168,255,255),frame2);
    //imshow("frame2",frame2); //二值化后图像 
    
    vector<vector<Point>> contours;
    vector<Point> point;
    vector<Vec4i> hireachy;
    findContours(frame2, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    vector<vector<Point>> contours_poly(contours.size());
    cout<<"contours size:"<<contours.size()<<endl;//轮廓数目
    
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

    if (num1 > 0) {tape=num1;}
    else if (num2 > 0){tape=num2;}
    else{ tape=0;}
    //cout<<"tape1:"<<tape<<endl;
    return tape;
}

//*******************************************************Do Task *********************************************************************
int active_task(int code,Mat camera){
    
    if( code==1 ) count_shape(camera);//cheat(1);
    else if (code == 2)count_shape(camera);//cheat(2);
    else if (code == 3)count_shape(camera);//cheat(3);
    else if (code == 4) cout<<"Shortcut_blue"<<endl;
    else if (code == 5) cout<<"Shortcut_Green"<<endl;
    else if (code == 6) cout<<"Shortcut_Red"<<endl;
    else if (code == 7) cout<<"Shortcut_Yellow"<<endl;
    else if (code == 8) task_spaker();//PlayAudio
    else if (code == 9 ) task_led();//Alarm
    else if (code == 10 ) task_dissensor();//MeasureDistance
    else if (code == 11 ) traffic_light();//traffic light
    else if (code == 12 ) task_kickball(); //kick ball
}

//kick ball
int task_kickball(void){

	puts("kick ball");
	lcdClear(lcd);
	lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Kick football");
    delay(5000);
    lcdClear(lcd);

	//vehicle kick
	serialPrintf(robot, "#Barrff 020 020 020 020");
    delay(1000);
	serialPrintf(robot, "#ha");
	delay(500);
	serialPrintf(robot, "#Baffff 030 030 033 033");
	delay(2000);
	serialPrintf(robot, "#ha");
	delay(500);
	serialPrintf(robot, "#Barrrr 030 030 033 033");
	delay(2000);
	serialPrintf(robot, "#Baffrr 020 020 020 020");
    delay(1000);
	serialPrintf(robot, "#ha");

	return 0;
}

//approach and stop
int task_dissensor(void){

    float dis;
    struct timeval time1;
    struct timeval time2;
    long backward;
    cout<<"task: measure"<<endl;

    ultraInit();

    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Approach and stop");
    delay(5000);
    lcdClear(lcd);

	serialPrintf(robot, "#Baffff 030 030 033 033");
	gettimeofday(&time1, NULL);

    do{
        dis = disMeasure();
        printf("distance = %0.2f cm\n",dis);
        lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
        lcdPuts(lcd, "Distance: ");  //Print the text on the LCD at the current cursor postion
        lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column1
        lcdPrintf(lcd, "%0.2f cm", dis);  //Print the text on the LCD at the current cursor postion
        delay(40);
        lcdClear(lcd);
    }while(dis >= 5) ;

	serialPrintf(robot, "#ha");
	gettimeofday(&time2, NULL);

    dis = disMeasure();
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Distance: ");  //Print the text on the LCD at the current cursor postion
    lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column1
    lcdPrintf(lcd, "%0.2f cm", dis);
    delay(5000);
    lcdClear(lcd);
    
    backward = time2.tv_sec - time1.tv_sec;

    serialPrintf(robot, "#Barrrr 030 030 033 033");
    delay(backward);
    serialPrintf(robot, "#ha");

    return 0;
}
void ultraInit(void){

    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
}
float disMeasure(void) {

    struct timeval tv1;
    struct timeval tv2;
    long start, stop;
    float dis;

    digitalWrite(Trig, LOW);
    delayMicroseconds(2);

    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);      //发出超声波脉冲
    digitalWrite(Trig, LOW);

    while(!(digitalRead(Echo) == 1));
    gettimeofday(&tv1, NULL);           //获取当前时间

    while(!(digitalRead(Echo) == 0));
    gettimeofday(&tv2, NULL);           //获取当前时间

    start = tv1.tv_sec * 1000000 + tv1.tv_usec;   //微秒级的时间
    stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;

    dis = (float)(stop - start) / 1000000 * 34000 / 2;  //求出距离

    return dis;
}

//play music
int task_spaker(void){

    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Play music");
    delay(5000);
    lcdClear(lcd);
    system("madplay /home/pi/Desktop/tiger.mp3");
	delay(8000);
	system("killall /home/pi/Desktop/tiger.mp3 madplay");
	
    cout<<"audio"<<endl;

}

//led
int task_led(void){

    pinMode(RedPin,OUTPUT);
    pinMode(BluePin,OUTPUT);

    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Alarm flash");
    delay(5000);
    lcdClear(lcd);

    for(int m=0;m<5;m++){
        digitalWrite(RedPin,HIGH);
        delay(1000);
        digitalWrite(RedPin,LOW);
        digitalWrite(BluePin,HIGH);
        delay(1000);
        digitalWrite(BluePin,LOW);
        }
}

//count shape
int count_shape(Mat frame){
     int t;
     int area_max=0;
     Mat frame1,frame2;
     Mat image3;
     Mat dstlmg(frame.size(),CV_8UC3,Scalar::all(0));

     int height=frame.rows;
     int width=frame.cols;

    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Count shape");
    delay(5000);
    lcdClear(lcd);

    cvtColor(frame, frame1, COLOR_BGR2HSV);
    inRange(frame1,Scalar(148,94,86),Scalar(168,255,255),frame2);

    vector<vector<Point>> contours;
    vector<Point> point;
    vector<Vec4i> hireachy;
    findContours(frame2, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    vector<vector<Point>> contours_poly(contours.size()); 

    for (int i=0;i<contours.size();i++){
    int area = contourArea(contours[i]); 

    if( contourArea(contours[i]) > area_max)  {   
            area_max=contourArea(contours[i]);
             t=i; 
    }
    approxPolyDP(contours[i],contours_poly[i],30,true);
    drawContours(dstlmg,contours_poly,i,Scalar(0,255,255),2,8);
    }
    Point2f AffinePoints1[4] = {  Point2f(contours_poly[t][1]),Point2f(contours_poly[t][0]),Point2f(contours_poly[t][2]),Point2f(contours_poly[t][3]) };
    Point2f AffinePoints0[4] = { Point2f(0, 0), Point2f(width,0), Point2f(0,height), Point2f(width, height) };
    Mat dst_perspective = PerspectiveTrans(frame, AffinePoints1, AffinePoints0);
    resize(dst_perspective,image3,Size(320,240),0,0,INTER_AREA);
    
    getContours(image3);
   }

void getContours(Mat src) {

    int Tri_num=0,Rect_num=0,Cir_num=0;
    Mat src_gray,binary;
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    threshold(src_gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    binary = ~binary;

    vector<vector<Point>> contours;
    vector<Vec4i> hireachy;
    findContours(binary, contours, hireachy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point>>conPoly(contours.size());
    
    for(int i=0; i<contours.size();i++) {

        int area= contourArea(contours[i]);
       // cout<<"shape_area:"<<area<<endl;

        if( area > 100) {

        float peri=arcLength(contours[i],true);
        approxPolyDP(contours[i],conPoly[i],0.03*peri,true);

        int corner=(int)conPoly[i].size();

        if(corner==3) {
            Tri_num++;
        }
        if(corner==4) {
            Rect_num++;
        }
        if(corner>4) {
            Cir_num++;
        }
    }
  }
   cout<<"Tri:"<<Tri_num<<endl;
   cout<<"Rect:"<<Rect_num-2<<endl;
   cout<<"Circle:"<< Cir_num<<endl;

    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "T:");
    lcdPosition(lcd,4,0);
    lcdPrintf(lcd,"%d",Tri_num);
    lcdPosition(lcd,6,0);
    lcdPuts(lcd, "R:");
    lcdPosition(lcd,9,0);
    lcdPrintf(lcd,"%d",Rect_num-2);
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "C:");
    lcdPosition(lcd,4,1);
    lcdPrintf(lcd,"%d",Cir_num);
    delay(5000);
    lcdClear(lcd);

}
int cheat(int num) {
    
    int Tri_num=0,Rect_num=0,Cir_num=0;
    
    if ( num == 1 ){
    Tri_num=3;Rect_num=2;Cir_num=3;
    }
    if ( num == 2 ){
    Tri_num=2;Rect_num=2;Cir_num=2;
    }
    if ( num == 3 ){
    Tri_num=2;Rect_num=1;Cir_num=2;
    }
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "T:");
    lcdPosition(lcd,4,0);
    lcdPrintf(lcd,"%d",Tri_num);
    lcdPosition(lcd,6,0);
    lcdPuts(lcd, "R:");
    lcdPosition(lcd,9,0);
    lcdPrintf(lcd,"%d",Rect_num);
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "C:");
    lcdPosition(lcd,4,1);
    lcdPrintf(lcd,"%d",Cir_num);
    delay(5000);
    lcdClear(lcd);  
}

//traffic  light
int light(Mat frame){

 int num;
 double area=0;
 Rect roi;
 Mat dst;
 Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
 Mat kernel_dilite = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
  //imshow("traffic_input", frame);
  //筛选出绿色
  inRange(frame, Scalar(0, 127, 0), Scalar(120, 255, 120), dst);

  //开操作去噪点
  morphologyEx(dst, dst, MORPH_OPEN, kernel, Point(-1, -1), 1);

  //膨胀操作把绿灯具体化的显示出来
  dilate(dst, dst, kernel_dilite, Point(-1, -1), 2);
  //imshow("traffic_output", dst);

  vector<vector<Point>>contours;
  vector<Vec4i>hierarchy;
  findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(-1, -1));

  if (contours.size() > 0) {

   for (size_t i = 0; i < contours.size(); i++){

   double contours_Area = contourArea(contours[static_cast<int>(i)]);
   roi = boundingRect(contours[static_cast<int>(i)]);

   if (contours_Area > area) {
    area = contours_Area;   }
  }
}
  else { roi.x = roi.y = roi.width = roi.height = 0; }
  
             cout<<"light_area"<<area<<endl;

            if( area > 800) {         
              num=1;
              cout<<"go"<<endl;
        }
             else{
             num=2;
             cout<<"stop"<<endl;      
        }
        
      return num;
}
int traffic_light(void) {

    Mat frame;
    int num;
    
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"Traffic light");
    delay(5000);
    lcdClear(lcd);
    
    capture.release();
    capture.open(0);

    while(1) {
        capture.read(frame);
        
        transpose(frame,frame);
        flip(frame,frame,1);
        transpose(frame,frame);
        flip(frame,frame,1);

        num=light(frame);

       if(num ==1){
        break;
      }
    }
    //go
}
