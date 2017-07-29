//
//  projection.h
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#ifndef __easyoperationofrobot__projection__
#define __easyoperationofrobot__projection__

#include <stdio.h>
#include "opencv2/imgproc/types_c.h"
#include "opencv2/nonfree/features2d.hpp"
#include "destriptor.h"










using namespace cv;

class Projection
{
    public :
    Mat  vdisparity, depthMap;
    void stereoRectification(Mat fundamental,Destriptor& des);
    void showDisplaritymap(Destriptor& des,Mat Q);
    void genarate3DPointCloud(const char* filename,Mat& src, Mat& dest, const double focal_length, Point2d imageCenter);
    
};


#endif /* defined(__easyoperationofrobot__projection__) */
