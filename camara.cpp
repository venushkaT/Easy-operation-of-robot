//
//  camara.cpp
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#include "camara.h"


/*

Under fronto-parrallel assumption
Q =[  1 0 0       -cx
    0 1 0      -cy
    0 0 0       f
    0 0 -1/Tx (cx -c'x)/Tx ]
               
               c_x and c_y are the coordinates of the principal point in the left camera
               c_x' is the x-coordinate of the principal point in the right camera
               f is the focal length    mm
               T_x is the baseline length mm
               
               
               By using focal length and basline displarity can obtain the Q value
               
               in order to do that flea2 camera





*/

Mat Camara::makeQMatrix(Point2d image_center, double focal_length, double baseline)
{
    Mat Q = Mat::eye(4, 4, CV_64F);
    Q.at<double>(0, 3) = -image_center.x;
    Q.at<double>(1, 3) = -image_center.y;
    Q.at<double>(2, 3) = focal_length;
    Q.at<double>(3, 3) = 0.0;
    Q.at<double>(2, 2) = 0.0;
    Q.at<double>(3, 2) = 1.0 / baseline;
    
    return Q;
}