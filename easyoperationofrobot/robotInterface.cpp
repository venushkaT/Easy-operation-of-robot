//
//  main.cpp
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "destriptor.h"
#include "projection.h"
#include "camara.h"
#include "render.h"
#include "timer.h"

#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


Destriptor des;
Projection pro;
Camara cam;
Render rend;

string		g_nameRootFolder; //root folder name

bool			g_isUseCamera, g_isLoadfile;


bool intialiseTheImages();
void InitValues();
void compute_center_descriptors(const char*);
void compute_left_descriptors(const char*);
void compute_right_descriptors(const char*);



int		g_windowWidth, g_windowHeight;



string point_cloud= "/Users/venushka/Desktop/3D/3d_pointcloud.txt";

Mat fundamental_Matrix;

Mat Q; // calculate the Q

const double focal_length = 600;
const double baseline=2000;

Timer timer;


int main( int argc, const char** argv )
{
    
    InitValues();
    
   // timer.Start();
    
    if (intialiseTheImages())
    {
        
        
        
        compute_left_descriptors(g_nameRootFolder.c_str());
        compute_right_descriptors(g_nameRootFolder.c_str());
        des.extract_features_using_dector();
        des.keypointMatching();
        fundamental_Matrix= des.calculateTheFundamentalMatrix();
        
        
        
        // send those images to rectification
        
        pro.stereoRectification(fundamental_Matrix,des);
        
        Q=cam.makeQMatrix(Point2d((des.input_right.cols - 1.0) / 2.0, (des.input_left.rows - 1.0) / 2.0), focal_length, baseline * 16);
       
        pro.showDisplaritymap(des,Q);
        
        Mat xyz;
        
        pro.genarate3DPointCloud(point_cloud.c_str(), pro.vdisparity, xyz, focal_length,Point2d(-1, -1));
        
        rend.enable_rendering(focal_length, baseline, pro.vdisparity, pro.depthMap,Q,des);
        
        // write the point cloud to the file.
    
    }
    
    
   
    system("pause");
    return 0;
  
}

void  InitValues()
{
    // window resize should be formed.
    g_windowWidth = 720;
    g_windowHeight = 480;
    
   // g_loadFileIndex = 0;
    
}

void compute_center_descriptors(const char* name_dir)
{
    
    char namebuf[1024];
    
    for (int i = 0; i < 1; i++)
    {
        //sprintf(namebuf, "%s/center.png", name_dir, g_loadFileIndex++);
        sprintf(namebuf, "%s/Center/%04d_center.bmp", name_dir,i);
        printf("Read %s!!\n", namebuf); //check the buffer output
        
        des.process_centerview(namebuf);
        
        
        
    }
}

void compute_left_descriptors(const char* name_dir)
{
    
    char namebuf[1024];
    
    for (int i = 0; i < 1; i++)
    {
        sprintf(namebuf, "%s/Left/%04d_left.bmp", name_dir, i);
        printf("Read %s!!\n", namebuf); //check the buffer output
        
        des.process_leftview(namebuf);
        
        
        
    }
}

void compute_right_descriptors(const char* name_dir)
{
    
    char namebuf[1024];
    
    for (int i = 0; i < 1; i++)
    {
        sprintf(namebuf, "%s/Right/%04d_right.bmp", name_dir, i);
        //sprintf(namebuf, "%s/right.png", name_dir, g_loadFileIndex++);
        
        printf("Read %s!!\n", namebuf); //check the buffer output
        
        des.process_rightview(namebuf);
        
        
        
    }
}


bool intialiseTheImages()
{
    std::string key;
    std::cout << "Use Camera? -> [y/n] "; std::cin >> key;
    if (key[0] == 'y'){
        g_isUseCamera = true;
        
        std::cerr << "Camera is not avaliable right now" << std::endl;
        
    }
    
    else if (key[0] == 'n'){
        
        std::cout << "Load capture file? -> [y/n] "; std::cin >> key;
        g_isLoadfile = (key[0] == 'y') ? true : false;
        
        if (g_isLoadfile){
            
            std::cout << "Please input capture root folder name -> "; std::cin >> g_nameRootFolder;
            if (g_nameRootFolder.empty()){
                std::cerr << "There was no name" << std::endl;
                return false;
            }
            
            
            
        }
        else{
            std::cerr << "select y or n" << std::endl;
            //return false;
        }
    }
    return true;
}


