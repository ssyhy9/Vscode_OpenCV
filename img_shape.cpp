#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

void getContours(Mat imgdil, Mat IMG);

Mat img, imggrey, imgHSV, imgBlur, imgCanny, imgDil, imgEro, mask;
Mat imgWarp;

int hmin = 141, smin = 102, vmin = 120;
int hmax = 179, smax = 255, vma = 255; 

// int main()
// {
//     img=imread("D:/Vscode_OpenCV/Resources/shape_noise1.jpg");
//     cout << img.size << endl;

//     cvtColor(img, imggrey, COLOR_BGR2GRAY);
//     cvtColor(img, imgHSV, COLOR_BGR2HSV);
//     GaussianBlur(imggrey, imgBlur, Size(5,5), 0, 0);
//     Canny(img, imgCanny, 0, 100);   //last two arguments of Canny: >2nd(Y)  <1st(N)  

//     Scalar lower(hmin, smin, vmin);
//     Scalar upper(hmax, smax, vma);
//     inRange(imgHSV, lower, upper, mask);

//     Mat kernel_dil = getStructuringElement(MORPH_RECT, Size(1, 1));  //only use odd number here(the larger the thicker)
//     Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(3, 3));
//     dilate(imgCanny, imgDil, kernel_dil);
//     erode(imgDil, imgEro, kernel_ero);      //canny >> dilate >> erode(A good method to identify the figure of a img)

//     getContours(mask, img);

//     imshow("IMG", img);
//     imshow("IMG HSV", imgHSV);
//     imshow("IMG MASK", mask);

//     waitKey(0);
//     return 0;
// }

// void getContours(Mat MASK, Mat IMG){ 

//   int Tri_num = 0, Rect_num = 0, Cir_num = 0; 
//   vector<vector<Point>> contours;
//   vector<Vec4i> hierarchy;
  

//   findContours(MASK, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
//   cout << "counter number: " << contours.size() << endl;

//   vector<vector<Point>> conPoly(contours.size());
//   vector<Rect> boundRect(contours.size());
//   string shape = "";

//       for (int i = 0; i < contours.size(); i++){
//         int area = contourArea(contours[i]);
//         //cout << area << endl;
//         if(area > 400){
//           float peri = arcLength(contours[i], true);
//           approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
//           //approxPolyDP(contours[i], conPoly[i], 4, true);
//           boundRect[i] = boundingRect(conPoly[i]);
//           cout << boundRect[i] << endl;

//           int corner = (int)conPoly[i].size(); 
//           if(corner == 3){ 
//             shape = "Tri";
//             Tri_num ++ ;
//            }
//           else if(corner == 4){ 
//             shape = "Rect"; 
//             Rect_num ++;
//           }
//           else if(corner > 4){ 
//             shape = "Circle"; 
//             Cir_num ++;
//           }
          
//           cout << boundRect[i].area() << endl;
//           rectangle(IMG, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 1);
//           putText(IMG, shape, {boundRect[i].x, boundRect[i].y}, FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 0), 2);
//           drawContours(IMG, conPoly, i, Scalar(255, 0, 0), 1); 
//         }
//     } //end for

//     cout << "Tri number: " << Tri_num << endl;
//     cout << "Rect number: " << Rect_num << endl;
//     cout << "Cir number: " << Cir_num << endl;
// }

int main()
{
    img=imread("D:/Vscode_OpenCV/Resources/shape_noise1.jpg");
    cout << img.size << endl;

    cvtColor(img, imggrey, COLOR_BGR2GRAY);
    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    GaussianBlur(imggrey, imgBlur, Size(5,5), 0, 0);
    Canny(img, imgCanny, 0, 100);   //last two arguments of Canny: >2nd(Y)  <1st(N)  

    Scalar lower(hmin, smin, vmin);
    Scalar upper(hmax, smax, vma);
    inRange(imgHSV, lower, upper, mask);

    Mat kernel_dil = getStructuringElement(MORPH_RECT, Size(1, 1));  //only use odd number here(the larger the thicker)
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(imgCanny, imgDil, kernel_dil);
    erode(imgDil, imgEro, kernel_ero);      //canny >> dilate >> erode(A good method to identify the figure of a img)

    getContours(mask, img);

    imshow("IMG", img);
    imshow("IMG HSV", imgHSV);
    imshow("IMG MASK", mask);

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
        //cout << area << endl;
        if(area > 400){
          
          float peri = arcLength(contours[i], true);
          approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
          //approxPolyDP(contours[i], conPoly[i], 4, true);
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
          }

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
          
          //line(IMG, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0),1);
          //rectangle(IMG, vertices[0], vertices[2], Scalar(0, 255, 0), 1);
           for (int i = 0; i < 4; i++)
            line(IMG, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 3);
          // putText(IMG, shape, {minRect[i].x, minRect[i].y}, FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 0), 2);
          drawContours(IMG, conPoly, i, Scalar(255, 0, 0), 1); 
        }
    } //end for

    Point2f src[4] = {real_points[0], real_points[1], real_points[2], real_points[3]};
    //cout << real_points[0] << real_points[1] << real_points[2] << real_points[3] << endl;
    Point2f dst[4] = {Point2f(0, 0), Point2f(640,0), Point2f(640,480), Point2f(0, 480)};
    Mat matrix = getPerspectiveTransform(src, dst);
    warpPerspective(IMG, imgWarp, matrix, Point(640, 480));

    imshow("Image Warp", imgWarp);

    cout << "Tri number: " << Tri_num << endl;
    cout << "Rect number: " << Rect_num << endl;
    cout << "Cir number: " << Cir_num << endl;
}

