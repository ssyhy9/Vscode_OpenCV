#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;

int main(void) {

 VideoCapture capture("D:/Vscode_OpenCV/Resources/CWvideo.mp4"); //read the video

  while (1) { //loop meiyizhen

  Mat frame,gray,thresholded;//定义一个Mat变量，用于存储每一帧的图像
  capture >> frame;  //读取当前帧

  if (frame.empty()){

   break;

  }//若视频播放完成，退出循环

   cvtColor(frame, gray, COLOR_BGR2GRAY);  //转换为黑白图像
   threshold(gray,thresholded, 100, 255, THRESH_BINARY);  //二值化处理


int width  = thresholded.cols;
int height = thresholded.rows;

int col=0;
int row =240;

int col_edge1=0;
int col_edge2=0;

int left,right=thresholded.at<uchar>(row,col);

char file[500];


for( col=0; col<width; col++) {

   left = right;
   right = thresholded.at<uchar>(row,col);

    if( left > right) {
        col_edge1 = col;
    }

    if( left < right) {
        col_edge2 = col;
        break;
    }

} //end for

 if ( col_edge2 > 630 || col_edge1 < 15 ){

 putText(frame,"out of view",Point(250,240),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),4,8);

  } //end if

 else{

 circle(frame,Point(((col_edge1+col_edge2)/2),row),2,Scalar(0,0,255),1,8,0);
 sprintf(file,"(%d,%d)",((col_edge1+col_edge2)/2),row);
 putText(frame,file,Point(((col_edge1+col_edge2)/2)-60,row-40),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),4,8);

  } // end else

  imshow("Processed video",frame);  //显示当前帧
  waitKey(20);

 } //end while
} //end main