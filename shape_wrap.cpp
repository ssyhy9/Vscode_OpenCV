#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

void getContours(Mat imgdil, Mat IMG);

Mat img, imggrey, imgHSV, imgBlur, imgCanny, imgDil, imgEro, mask;
Mat imgWarp;

int hmin = 124, smin = 173, vmin = 90;
int hmax = 179, smax = 255, vma = 172; 

int main()
{
    img=imread("D:/Vscode_OpenCV/Resources/real_shape.png");
    cout << img.size << endl;

    cvtColor(img, imgHSV, COLOR_BGR2HSV);

    Scalar lower(hmin, smin, vmin);
    Scalar upper(hmax, smax, vma);
    inRange(imgHSV, lower, upper, mask);

    getContours(mask, img);

    imshow("IMG", img);
    imshow("IMG HSV", imgHSV);
    imshow("IMG MASK", mask);
    imshow("Image Warp", imgWarp);

    waitKey(0);
    return 0;
}

void getContours(Mat MASK, Mat IMG){ 

  int Tri_num = 0, Rect_num = 0, Cir_num = 0; 
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  float max_area = 0;

  findContours(MASK, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
  cout << "counter number: " << contours.size() << endl;

  vector<vector<Point>> conPoly(contours.size());
  vector<RotatedRect> minRect(contours.size());
  string shape = "";
  Point2f real_points[4];

      for (int i = 0; i < contours.size(); i++){
        int area = contourArea(contours[i]);
        if(area > 400){ 
          float peri = arcLength(contours[i], true);
          approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);

          minRect[i] = minAreaRect(contours[i]);
          Point2f vertices[4];  //定义旋转矩形的4个顶点
          minRect[i].points(vertices);

            std::cout << "center:" << minRect[i].center.x << "," << minRect[i].center.y << std::endl;
            std::cout << "Size:" << minRect[i].size.width << "," << minRect[i].size.height << std::endl;
            std::cout << "area:" << minRect[i].size.area()<<std::endl;
            std::cout << "angle:" << minRect[i].angle << std::endl;
            //std::cout << "Points:" << minRect[i].points(vertices) << std::endl;

          if(minRect[i].size.area() > max_area){
            max_area = minRect[i].size.area();
            for(int r = 0; r < 4; r++){
              real_points[r] = vertices[r];
            }
          }  //find the minRect with largest area & save its vertices

          int corner = (int)conPoly[i].size(); 
          if(corner == 3){ 
            shape = "Tri";
            Tri_num ++ ;
           }
          else if(corner == 4){ 
            shape = "Rect"; 
            Rect_num ++;
          }
          else if(corner > 4){ 
            shape = "Circle"; 
            Cir_num ++;
          }
          
        //rectangle(IMG, vertices[0], vertices[2], Scalar(0, 255, 0), 1);
        for (int i = 0; i < 4; i++){
        line(IMG, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 3);
        }
        // putText(IMG, shape, {minRect[i].x, minRect[i].y}, FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 0), 2);
        drawContours(IMG, conPoly, i, Scalar(255, 0, 0), 1); 
        }
    } //end for

    Point2f src[4] = {real_points[0], real_points[1], real_points[2], real_points[3]};
    Point2f dst[4] = {Point2f(0, 0), Point2f(640,0), Point2f(640,480), Point2f(0, 480)};
    Mat matrix = getPerspectiveTransform(src, dst);
    warpPerspective(IMG, imgWarp, matrix, Point(640, 480));

    cout << "Tri number: " << Tri_num << endl;
    cout << "Rect number: " << Rect_num << endl;
    cout << "Cir number: " << Cir_num << endl;
}