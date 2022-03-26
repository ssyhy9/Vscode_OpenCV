void getContours(Mat MASK){ 

  int Tri_num = 0, Rect_num = 0, Cir_num = 0; 
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  float max_area = 0; 

  findContours(MASK, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
  cout << "counter number: " << contours.size() << endl;

  vector<vector<Point>> conPoly(contours.size());

      for (int i = 0; i < contours.size(); i++){
        int area = contourArea(contours[i]);
        //cout << area << endl;
        if(area > 400){
          
          float peri = arcLength(contours[i], true);
          approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);

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
        }
    }
}
