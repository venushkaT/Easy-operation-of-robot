//
//  camara.h
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#ifndef __easyoperationofrobot__camara__
#define __easyoperationofrobot__camara__

#include <stdio.h>
#include "opencv2/imgproc/types_c.h"
#include "opencv2/nonfree/features2d.hpp"

using namespace cv;


class Camara
{
    public :
    Mat Q; // matrix Q
   
    

Mat makeQMatrix(Point2d image_center, double focal_length, double baseline);
    
};




#endif /* defined(__easyoperationofrobot__camara__) */
