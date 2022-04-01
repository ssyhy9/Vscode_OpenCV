
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

#include "tasks.h"
#include "main.h"

int active_task(int code,Mat camera){

    if( code==1 ) count_shape(camera);
    else if (code == 2)count_shape(camera);
    else if (code == 3)count_shape(camera);
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

 cout<<"Kick ball"<<endl;
 lcdClear(lcd);
 lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Kick football");
    delay(5000);
    lcdClear(lcd);

 //vehicle kick
 serialPrintf(robot, "#Barrff 070 070 074 074");
    delay(1600);
 serialPrintf(robot, "#ha");
 delay(500);
 serialPrintf(robot, "#Barrrr 030 030 033 033");
 delay(2000);
 serialPrintf(robot, "#ha");
 delay(500);
 serialPrintf(robot, "#Baffff 030 030 033 033");
 delay(2000);
 serialPrintf(robot, "#ha");
 delay(500);
 serialPrintf(robot, "#Baffrr 070 070 074 074");
    delay(1600);
 serialPrintf(robot, "#ha");
 cout<<"Kick ball finish"<<endl;

 return 0;
}

//approach and stop
int task_dissensor(void){

    float dis;
    struct timeval time1;
    struct timeval time2;
    float backward;
    cout<<"approach and stop"<<endl;

    ultraInit();

    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: Approach");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "and stop");
    delay(5000);
    lcdClear(lcd);

	serialPrintf(robot, "#Baffff 015 015 017 017");
	gettimeofday(&time1, NULL);
	cout<<"1111111"<<endl;

    do{
        dis = disMeasure();
        printf("distance = %0.2f cm\n",dis);
        lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
        lcdPuts(lcd, "Distance: ");  //Print the text on the LCD at the current cursor postion
        lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column1
        lcdPrintf(lcd, "%0.2f cm", dis);  //Print the text on the LCD at the current cursor postion
        delay(30);
//        lcdClear(lcd);
    }while(dis >= 6) ;

	serialPrintf(robot, "#ha");
	gettimeofday(&time2, NULL);

//    dis = disMeasure();
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Distance: ");  //Print the text on the LCD at the current cursor postion
    lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column1
    lcdPrintf(lcd, "%0.2f cm", dis);

    delay(5000);
    lcdClear(lcd);

    backward = time2.tv_sec - time1.tv_sec;

    serialPrintf(robot, "#Barrrr 015 015 017 017");
    backward = backward * 1000 - 300;
    cout<<"time:  "<<backward<<endl;
    delay(backward);
    serialPrintf(robot, "#ha");
    cout<<"approach and stop finish"<<endl;


    delay(5000);

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
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);

    while(!(digitalRead(Echo) == 1));
    gettimeofday(&tv1, NULL);

    while(!(digitalRead(Echo) == 0));
    gettimeofday(&tv2, NULL);

    start = tv1.tv_sec * 1000000 + tv1.tv_usec;
    stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;


    dis = (float)(stop - start) / 1000000 * 34000 / 2;

    return dis;
}

//play music
int task_spaker(void){

    cout<<"audio"<<endl;
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Play music");
    delay(5000);
    lcdClear(lcd);
    system("madplay /home/pi/Desktop/piano.mp3");
	delay(6000);
	system("killall /home/pi/Desktop/tiger.mp3 madplay");

    cout<<"audio finish"<<endl;

}

//led
int task_led(void){
    cout<<"LED"<<endl;
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
    cout<<"LED finish"<<endl;
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
   cout<<"Count Shape"<<endl;
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd, "Task: ");
    lcdPosition(lcd,0,1);
    lcdPuts(lcd, "Count shape");
    delay(5000);
    lcdClear(lcd);

    cvtColor(frame, frame1, COLOR_BGR2HSV);
    inRange(frame1,Scalar(150,100,81),Scalar(170,255,255),frame2);

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
    cout<<"Count shape finish"<<endl;
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
         cout<<"shape_area:"<<area<<endl;

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


//traffic  light
int light(Mat frame){

 int num;
 double area=0;
 Rect roi;
 Mat dst;
 Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
 Mat kernel_dilite = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
  //imshow("traffic_input", frame);

  inRange(frame, Scalar(0, 127, 0), Scalar(120, 255, 120), dst);


  morphologyEx(dst, dst, MORPH_OPEN, kernel, Point(-1, -1), 1);


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

            if( area > 200) {
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
    cout<<"Traffic light"<<endl;
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"Traffic light");
    delay(5000);
    lcdClear(lcd);

    capture.release();
    capture.open(0);
    capture.set(CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CAP_PROP_FRAME_HEIGHT, 1024);

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
    cout<<"Traffic light finish"<<endl;
    //go
}
