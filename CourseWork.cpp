#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

void preProcess(Mat IMG);
void draw_centerPoints(Mat gray, Mat im);

Mat img, imgGRAY;

int main(){
    string path = "D:/Vscode_OpenCV/Resources/CWVideo.mp4";
    VideoCapture cap(path);
    // get one frame from camera to know frame size and type
    cap >> img;
    
    //--- INITIALIZE VIDEOWRITER
    int FPS = cap.get(CAP_PROP_FPS);  //24
    VideoWriter writer;
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    string filename = "D:/Vscode_OpenCV/Resources/Output.avi"; 
    writer.open(filename, codec, FPS, img.size(), true);

     if (!writer.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return -1;
    }   

    while(true){
        cap.read(img);
        preProcess(img);
        
        // cout << img.size() << endl;  //368 * 640
        draw_centerPoints(imgGRAY, img);

        writer.write(img);

        imshow("IMG GRAY", imgGRAY);

        waitKey(100);
    }

    return 0;
}

void preProcess(Mat IMG){
    cvtColor(img, imgGRAY, COLOR_BGR2GRAY);
}

void draw_centerPoints(Mat gray, Mat im){
    const int row = 180;
    int column = 0;
    int colour = 0;
    int cnt = 0;
    int start_flag = 0;
    int start_point = 0;
    char text[100] = "";

    column = gray.cols;  //640
    for(int i = 0; i < column; i++){
        colour = gray.at<uchar>(row, i);
        if(colour < 100 && start_flag == 0){
            start_point = i;
            start_flag = 1;
        }
        if(colour < 100 && start_flag == 1){
            cnt++;
        }
    }
    //cout<<cnt<<endl;
    if(cnt > 300){
        putText(im, "Out of View", Point(220, row - 30), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 255), 1);
    }
    else{
        line(im, Point(0, 180), Point(640, 180), Scalar(0, 255, 0), 1);
        circle(im, Point(start_point + cnt/2, row), 5, Scalar(255, 0, 255), FILLED);
        sprintf(text,"(%d,%d)", start_point + cnt/2, row);
        putText(im, text, Point(start_point + cnt/2 - 50, row - 30), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255), 1);
    }
    imshow("IMG_point", im);
}