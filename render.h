//
//  render.h
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#ifndef __easyoperationofrobot__render__
#define __easyoperationofrobot__render__

#include <stdio.h>
#include "opencv2/imgproc/types_c.h"
#include "opencv2/nonfree/features2d.hpp"
#include "destriptor.h"

class Render
{
public:
    
    
    
    void enable_rendering(double focal_length,double baseline,Mat  vdisparity,Mat depthMap,Mat Q,Destriptor& des);

    
    
};


#endif /* defined(__easyoperationofrobot__render__) */
