//
//  destriptor.cpp
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#include "destriptor.h"


void Destriptor::extract_features_using_dector()
{
    
    
    Timer timer;
    
    
    // corner feature detection using  Harris detecter
    Ptr<FeatureDetector> featureDetector = FeatureDetector::create("HARRIS");
    featureDetector->detect(input_right, keypoints_right);// is pointer so i used ->
    featureDetector->detect(input_left, keypoints_left);
    
    
    
    //-- Step 2: Calculate descriptors (feature vectors)
    Ptr<DescriptorExtractor> featureExtractor = DescriptorExtractor::create("SIFT");
    
    
    
    featureExtractor->compute(input_right, keypoints_right, descriptor_right);
    // which is pointer so i used ->
    featureExtractor->compute(input_left, keypoints_left, descriptor_left);
    
    
    
    // show features
    
    // check detected keypoints right
    Mat outputImageright;
    Scalar keypointColor = Scalar(255, 0, 0);     // Blue keypoints.
    drawKeypoints(input_right, keypoints_right, outputImageright, keypointColor, DrawMatchesFlags::DEFAULT);
    
    namedWindow("Right View");
    imshow("Right View", outputImageright);
    
    waitKey(0);
    
    // check detected keypoint left
    
    Mat outputImageleft;
    Scalar keypointColorred = Scalar(0, 0, 255);     // Red keypoints.
    drawKeypoints(input_left, keypoints_left, outputImageleft, keypointColorred, DrawMatchesFlags::DEFAULT);
    
    namedWindow("Left View");
    imshow("Left View", outputImageleft);
    
    waitKey(0);
    
    
    
    
    
    
}

void Destriptor::keypointMatching()
{
    
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match(descriptor_right, descriptor_left, matches);
    double max_dist = 0; double min_dist = 100;
    
    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < descriptor_right.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    
    printf("-- Max dist right descriptor : %f \n", max_dist);
    printf("-- Min dist right descriptor: %f \n", min_dist);
    
    
    
    //-- Draw only "good" matches or a small arbitary value ( 0.02 ) in the event that min_dist is very small
    
    
    std::vector< DMatch > good_matches;
    
    for (int i = 0; i < descriptor_right.rows; i++)
    {
        //here i change octaves and layers then i achieved more
        if (matches[i].distance <= max(3* min_dist, 0.02)) // SIFT recomand 4 or 5 ocatives
        {
            good_matches.push_back(matches[i]);
        }
    }
    
    Mat img_matches;
    
    
    
    // the draw method Change the Flag from default to rich then visible key points with maches DrawMatchesFlags::DEFAULT
    //scale and oriantation as well
    
    drawMatches(input_right, keypoints_right, input_left, keypoints_left,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //-- Show detected matches
    imshow("Good Matches", img_matches);
    
    
    
    waitKey(0);
    
    // match the point cloud
    /*
    
    float xr, yr; // for store x and y cordinates for right pixels
    float xc, yc; // for store x and y cordinates for center pixels
    
    for (int i = 0; i < (int)good_matches.size(); i++)
    {
        //printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
        xr = keypoints_right[good_matches[i].queryIdx].pt.x;
        yr = keypoints_right[good_matches[i].queryIdx].pt.y;
        
        printf("key points of right image x-> %f y-> %f \n", xr, yr);
        
        xc = keypoints_left[good_matches[i].trainIdx].pt.x;
        yc = keypoints_left[good_matches[i].trainIdx].pt.y;
        
        printf("key point of left image x-> %f y-> %f \n ", xc, yc);
        
        printf("\n");
        
        
        
    }
     */
    
    
    //get the keypoints from the FLANN matching keypoints octaves size is changed as
    
    
    
   
    for (int i = 0; i < good_matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        rightfun.push_back(keypoints_right[good_matches[i].queryIdx].pt);
        leftfun.push_back(keypoints_left[good_matches[i].trainIdx].pt);
    }
    
    printf("Number of good matchers %lu \n", good_matches.size());
    
    // Calculate the fundametal matrix
    
    
    
    
}

Mat Destriptor::calculateTheFundamentalMatrix()
{
    Mat fundametal = findFundamentalMat(rightfun, leftfun, FM_RANSAC, 3, 0.99);
    return fundametal;
}



void Destriptor::process_centerview(const char* name_dir)
{
    input_center = imread(name_dir, CV_LOAD_IMAGE_COLOR);
    
    
    printf("done\n");
    
    
    
}



void Destriptor::process_leftview(const char* name_dir)
{
    input_left = imread(name_dir, CV_LOAD_IMAGE_COLOR);
    
    printf("done\n");
    
    
    
    
}

void Destriptor::process_rightview(const char* name_dir)
{
    input_right = imread(name_dir, CV_LOAD_IMAGE_COLOR);
    
    printf("done\n");
   
    
}

