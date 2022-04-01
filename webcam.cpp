#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

void stop(void);

VideoCapture cap;
int cnt = 0;

int main()
{
    cap.open(0);
    Mat img, imgHSV;
    

    while (true)
    {

        cap.read(img);

        cvtColor(img, imgHSV, COLOR_BGR2HSV);

        imshow("IMG", img);
        imshow("IMG HSV", imgHSV);
        waitKey(10);
        cnt ++;
        cout << cnt << endl;

        if(cnt == 1000){
            stop();
        }
        
    }
    return 0;
}

void stop(void){
    cap.release();
    waitKey(5000);
    cap.open(0);
    cnt = 0;
}