//
//  projection.cpp
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#include "projection.h"


void Projection::stereoRectification(Mat fundamental ,Destriptor& des)
{
    
    Mat homRight = Mat::zeros(des.input_left.rows, des.input_left.cols, des.input_left.depth());
    Mat homLeft = Mat::zeros(des.input_right.rows, des.input_right.cols, des.input_right.depth());;
    
    stereoRectifyUncalibrated(des.rightfun, des.leftfun, fundamental, des.input_center.size(), homLeft, homRight);
    
    warpPerspective(des.input_right, des.input_right, homRight, des.input_right.size(), INTER_LINEAR | WARP_INVERSE_MAP, BORDER_CONSTANT);
    warpPerspective(des.input_left, des.input_left, homLeft, des.input_left.size(), INTER_LINEAR | WARP_INVERSE_MAP, BORDER_CONSTANT);
    
    
    
    std::vector<cv::Vec3f> linesLeft;
    cv::computeCorrespondEpilines(
                                  cv::Mat(des.leftfun), // image points
                                  1,                      // in image 1 (can also be 2)
                                  fundamental,            // F matrix
                                  linesLeft);             // vector of epipolar lines
    
    // for all epipolar lines draw in separate mats
    
    cv::Mat copy_input_right=des.input_right.clone();
    for (vector<cv::Vec3f>::const_iterator it = linesLeft.begin(); it != linesLeft.end(); ++it) {
        
        
        // draw the epipolar line between first and last column
        cv::line(copy_input_right, cv::Point(0, -(*it)[2] / (*it)[1]), cv::Point(copy_input_right.cols, -((*it)[2] + (*it)[0] * copy_input_right.cols) / (*it)[1]), cv::Scalar(255, 255, 255));
        
    }
    
    cv::Mat copy_input_left=des.input_left.clone();
    std::vector<cv::Vec3f> linesRight;
    cv::computeCorrespondEpilines(cv::Mat(des.rightfun), 2, fundamental, linesRight);
    for (vector<cv::Vec3f>::const_iterator it = linesRight.begin(); it != linesRight.end(); ++it) {
        
        // draw the epipolar line between first and last column
        cv::line(copy_input_left, cv::Point(0, -(*it)[2] / (*it)[1]), cv::Point(copy_input_left.cols, -((*it)[2] + (*it)[0] * copy_input_left.cols) / (*it)[1]), cv::Scalar(255, 255, 255));
    }
    
    
    
    
    
    //show rectified images
    imshow("out_right", copy_input_right);
    imshow("out_left", copy_input_left);
    
    waitKey(0);

   

}
// call the rendering






// end of the rendering

void Projection::showDisplaritymap(Destriptor& des,Mat Q)
{
    
    /* this function has to be improved */
    StereoSGBM sbm(1,16*2,3,200,255,1,0,0,0,0,true);
    
    
    
    // convert images to grasycale
    
    Mat g1, g2;
    Mat disp;
    
    
   // printf("-----------> %f \n", numberofdis);
    
    
    cvtColor(des.input_left, g1, CV_BGR2GRAY);
    cvtColor(des.input_right, g2, CV_BGR2GRAY);
    
    sbm(g1, g2, disp);
    
    Mat disp16 = Mat(des.input_right.rows, des.input_left.cols, CV_8UC1);
    
    normalize(disp, disp16, 0, 255, CV_MINMAX, CV_8U);
    
    imshow("displarity image", disp16);
    normalize(disp16, vdisparity, 0, 256, NORM_MINMAX);
    reprojectImageTo3D(disp16, depthMap, Q);
    
    
    
    imshow("Disparity", vdisparity);
    imshow("3D", depthMap);
    
    waitKey(0);
    
}

    
    


//use sift-si method

void Projection::genarate3DPointCloud(const char* filename,Mat& src, Mat& dest, const double focal_length, Point2d imageCenter )
{
    
    
    
        FILE* fp = fopen(filename, "wt");
        if (dest.empty())dest = Mat::zeros(src.size().area(), 1, CV_32FC3);
        const double bigZ = 10000.;
        const double hw = (src.cols - 1)*0.5;
        const double hh = (src.rows - 1)*0.5;
        if (imageCenter.x == -1 && imageCenter.y == -1)imageCenter = Point2d(hw,
                                                                             hh);
        double ifocal_length = 1.0 / focal_length;

        for (int j = 0; j<src.rows; j++)
        {
            float* data = dest.ptr<float>(j*src.cols); unsigned short* s = src.ptr<unsigned short>(j); for (int i = 0; i<src.cols; i++)
            {
                data[0] = *s*ifocal_length*(i - imageCenter.x); data[1] = *s*ifocal_length*(j - imageCenter.y);
                if (*s == 0)data[2] = bigZ;
                else data[2] = *s;
               // fprintf(fp, "%f %f %f \n", data[0], data[1], data[2]);
                data += 3;
                s++; }
        }
        fclose(fp);
        // print the file
}




