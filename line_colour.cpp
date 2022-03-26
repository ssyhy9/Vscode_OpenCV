#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>


using namespace cv;
using namespace std;

float draw_centerPoints(Mat gray, Mat im);
void findColour(Mat img);
int scanMask(Mat MASK);

vector<vector<int>> myColours{
    // {0, 0, 0, 32, 255, 255}, //black
     {0, 82, 130, 30, 255, 210}, //yellow     i = 0
    {132, 168, 120, 179, 236, 226} //pink     i = 1
};



int main()
{
    string path = "D:/Vscode_OpenCV/Resources/Full_video .mp4";
    VideoCapture cap(path);
    Mat img, imgGRAY;
    float error;

    while(true){
        cap.read(img);   //1280 * 720

        cvtColor(img, imgGRAY, COLOR_BGR2GRAY);
        
        findColour(img);

        error = draw_centerPoints(imgGRAY, img);

        

        rectangle(img, Point(150, 360), Point(1150, 365), Scalar(0, 255, 0), 1);

        imshow("Image", img);
        //imshow("Image Gray", imgGRAY);
        //imshow("Image HSV", imgHSV);
        waitKey(10);
    }
    return 0;
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

            float error = real_error / 10;
            cout << "Error" << error << endl;

            circle(im, Point(x, 55), 3, Scalar(255, 0, 255), FILLED);
            imshow("Im", im);
            //sprintf(text,"(%d,%d)", x, 55);
            //putText(im, text, Point(start_point + cnt/2 - 50, row - 30), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255), 1);
    return error;
}

void findColour(Mat img){
    Mat imgHSV;
    int yellow_cnt = 0, pink_cnt = 0;
    cvtColor(img, imgHSV, COLOR_BGR2HSV);

    for(int i = 0; i<myColours.size(); i++){
        Mat mask; 

        Scalar lower(myColours[i][0], myColours[i][1], myColours[i][2]);
        Scalar upper(myColours[i][3], myColours[i][4], myColours[i][5]);
        inRange(imgHSV, lower, upper, mask);

        if(i == 0){
            yellow_cnt = scanMask(mask);
            //cout << "yellow cnt:" << yellow_cnt << endl;
        }
        else if(i == 1){
            pink_cnt = scanMask(mask);
            //cout << "pink cnt:" << pink_cnt << endl;
        }

        imshow(to_string(i), mask);
        imshow("Image HSV", imgHSV);
    }
}

int scanMask(Mat MASK){
    const int row = 360;
    const int x = 1000;
    const int y = 5;
    int colour = 0;
    int cnt = 0;

    for(int r = 0; r < y; r++){
        for(int i = 0; i < x; i++){
            colour = MASK.at<uchar>(row, 150 + i);
            if(colour > 100){
                cnt ++;
            }
        }
    }
    return cnt;
}
