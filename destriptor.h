//
//  destriptor.h
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#ifndef easyoperationofrobot_destriptor_h
#define easyoperationofrobot_destriptor_h

#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/contrib/contrib.hpp>
#include "timer.h"
#include <iostream>


using namespace std;

using namespace cv;

class Destriptor
{
    
    public :
    Mat input_center, input_right, input_left;
    Mat input; //To store the keypoints that will be extracted by SIFT
    vector<KeyPoint> keypoints_right, keypoints_center, keypoints_left;//To store the SIFT descriptor of current image
    Mat descriptor_right, descriptor_center, descriptor_left;
    SiftDescriptorExtractor detector;
    KeyPoint con;
    const double baseline = 2000;
    vector<Point2f> rightfun;
    vector<Point2f> leftfun;
   
    
    
    void extract_features_using_dector();
    
    void process_rightview(const char*);
    void process_centerview(const char*);
    void process_leftview(const char*);
    void extract_all_decriptors();
    void keypointMatching();
    
    Mat calculateTheFundamentalMatrix();

    
    
    
    //void compute_depth();
    
    Mat makeQMatrix(Point2d image_center, double focal_length, double baseline);
    
    
    
     
    
};


#endif
