#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

///////////////////////////// image ////////////////////////////////
//////// basic img functions /////////
// using namespace cv;
// using namespace std;

// int main()
// {
//     Mat img=imread("D:/Vscode_OpenCV/Resources/R-C.png");
//     Mat imggrey, imgBlur, imgCanny, imgDil, imgEro;

//     //cout << img.size() << endl;//268 * 268

//     cvtColor(img, imggrey, COLOR_BGR2GRAY);
//     // GaussianBlur(img, imgBlur, Size(5,5), 3, 0);
//     // Canny(imgBlur, imgCanny, 30, 35);

//     // Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));  //only use odd number here
//     // dilate(imgCanny, imgDil, kernel);
//     // erode(imgDil, imgEro, kernel);      //canny >> dilate >> erode(A good method to identify the figure of a img)

//     for(int i = 0; i < imggrey.cols; i++){
//     int colour = imggrey.at<uchar>(180, i);
//     cout << colour << endl;
//     waitKey(10);
//     }

//     imshow("IMG",img);
//     imshow("IMG grey",imggrey);
//     // imshow("IMG1",imgBlur);
//     // imshow("IMG2", imgCanny);
//     // imshow("IMG3", imgDil);
//     // imshow("IMG4", imgEro);

//     waitKey(0);
//     return 0;
// }

///////////// img resize and crop ///////////////
using namespace cv;
using namespace std;

int main()
{
    Mat img=imread("D:/Vscode_OpenCV/Resources/RedCar.bmp");
    Mat imgResize, imgCrop;

    cout << img.size() << endl;  //(320 * 240)

    imshow("IMG",img);
    resize(img, imgResize, Size(640, 480));
    imshow("IMGRe",imgResize);

    Rect sqr(50, 50, 100, 150);     //the rect start at the point(50, 50) with a length 100 and with 150
    imgCrop = img(sqr);
    imshow("IMGCrop",imgCrop);

    waitKey(0);
    return 0;
}

///////////////// draw shapes and text //////////////////
// using namespace cv;
// using namespace std;

// int main()
// {
//     Mat img(512 , 512, CV_8UC3, Scalar(255, 255, 255));

//     circle(img, Point(256, 256), 155, Scalar(0, 69, 255), FILLED);
//     rectangle(img, Point(130, 266), Point(382, 286), Scalar(255, 255, 255), FILLED);
//     line(img, Point(130, 266), Point(382, 412), Scalar(100, 255, 100), 4);

//     putText(img, "For the greater good", Point(137, 262), FONT_HERSHEY_PLAIN, 0.75, Scalar(0, 69, 255), 2);

//     imshow("IMG", img);


//     waitKey(0);
//     return 0;
// }


