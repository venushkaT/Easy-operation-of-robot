//
//  render.cpp
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#include "render.h"

template <class T>
static void fillOcclusion_(Mat& src, T invalidvalue)
{
    int bb=1;
    const int MAX_LENGTH=src.cols*0.5;
    
    for(int j=bb;j<src.rows-bb;j++)
    {
        T* s = src.ptr<T>(j);
      
        s[0]=255;
        s[src.cols-1]=255;
        for(int i=0;i<src.cols;i++)
        {
            if(s[i]<=invalidvalue)
            {
                int t=i;
                do
                {
                    t++;
                    if(t>src.cols-1)break;
                }while(s[t]<=invalidvalue);
                
                const T dd = min(s[i-1],s[t]);
                if(t-i>MAX_LENGTH)
                {
                    for(int n=0;n<src.cols;n++)
                    {
                        s[n]=invalidvalue;
                    }
                }
                else
                {
                    for(;i<t;i++)
                    {
                        s[i]=dd;
                    }
                }
            }
        }
    }
}

template <class T>
static void fillOcclusionInv_(Mat& src, T invalidvalue)
{
    int bb=1;
    const int MAX_LENGTH=src.cols*0.8;
    //#pragma omp parallel for
    for(int j=bb;j<src.rows-bb;j++)
    {
        T* s = src.ptr<T>(j);
    
        
        s[0]=0;
        s[src.cols-1]=0;
        for(int i=0;i<src.cols;i++)
        {
            if(s[i]==invalidvalue)
            {
                int t=i;
                do
                {
                    t++;
                    if(t>src.cols-1)break;
                }while(s[t]==invalidvalue);
                
                const T dd = max(s[i-1],s[t]);
                if(t-i>MAX_LENGTH)
                {
                    for(int n=0;n<src.cols;n++)
                    {
                        s[n]=invalidvalue;
                    }
                }
                else
                {
                    for(;i<t;i++)
                    {
                        s[i]=dd;
                    }
                }
            }
        }
    }
}


void fillOcclusion(Mat& src, int invalidvalue, bool isInv=false)
{
    if(isInv)
    {
        if(src.type()==CV_8U)
        {
            fillOcclusionInv_<uchar>(src, (uchar)invalidvalue);
        }
        else if(src.type()==CV_16S)
        {
            fillOcclusionInv_<short>(src, (short)invalidvalue);
        }
        else if(src.type()==CV_16U)
        {
            fillOcclusionInv_<unsigned short>(src, (unsigned short)invalidvalue);
        }
        else if(src.type()==CV_32F)
        {
            fillOcclusionInv_<float>(src, (float)invalidvalue);
        }
    }
    else
    {
        if(src.type()==CV_8U)
        {
            fillOcclusion_<uchar>(src, (uchar)invalidvalue);
        }
        else if(src.type()==CV_16S)
        {
            fillOcclusion_<short>(src, (short)invalidvalue);
        }
        else if(src.type()==CV_16U)
        {
            fillOcclusion_<unsigned short>(src, (unsigned short)invalidvalue);
        }
        else if(src.type()==CV_32F)
        {
            fillOcclusion_<float>(src, (float)invalidvalue);
        }
    }
}

template <class T>
static void projectImagefromXYZ_(Mat& image, Mat& destimage, Mat& disp, Mat& destdisp, Mat& xyz, Mat& R, Mat& t, Mat& K, Mat& dist, Mat& mask, bool isSub)
{
    if(destimage.empty())destimage=Mat::zeros(Size(image.size()),image.type());
    if(destdisp.empty())destdisp=Mat::zeros(Size(image.size()),disp.type());
    
    vector<Point2f> pt;
    if(dist.empty()) dist = Mat::zeros(Size(5,1),CV_32F);
    projectPoints(xyz,R,t,K,dist,pt);
    
    destimage.setTo(0);
    destdisp.setTo(0);
    
    //pragma omp parallel for
    for(int j=1;j<image.rows-1;j++)
    {
        int count=j*image.cols;
        uchar* img=image.ptr<uchar>(j);
        uchar* m=mask.ptr<uchar>(j);
        for(int i=0;i<image.cols;i++,count++)
        {
            int x=(int)(pt[count].x+0.5);
            int y=(int)(pt[count].y+0.5);
            if(m[i]==255)continue;
            if(pt[count].x>=1 && pt[count].x<image.cols-1 && pt[count].y>=1 && pt[count].y<image.rows-1)
            {
                short v=destdisp.at<T>(y,x);
                if(v<disp.at<T>(j,i))
                {
                    destimage.at<uchar>(y,3*x+0)=img[3*i+0];
                    destimage.at<uchar>(y,3*x+1)=img[3*i+1];
                    destimage.at<uchar>(y,3*x+2)=img[3*i+2];
                    destdisp.at<T>(y,x)=disp.at<T>(j,i);
                    
                    if(isSub)
                    {
                        if((int)pt[count+image.cols].y-y>1 && (int)pt[count+1].x-x>1)
                        {
                            destimage.at<uchar>(y,3*x+3)=img[3*i+0];
                            destimage.at<uchar>(y,3*x+4)=img[3*i+1];
                            destimage.at<uchar>(y,3*x+5)=img[3*i+2];
                            
                            destimage.at<uchar>(y+1,3*x+0)=img[3*i+0];
                            destimage.at<uchar>(y+1,3*x+1)=img[3*i+1];
                            destimage.at<uchar>(y+1,3*x+2)=img[3*i+2];
                            
                            destimage.at<uchar>(y+1,3*x+3)=img[3*i+0];
                            destimage.at<uchar>(y+1,3*x+4)=img[3*i+1];
                            destimage.at<uchar>(y+1,3*x+5)=img[3*i+2];
                            
                            destdisp.at<T>(y,x+1)=disp.at<T>(j,i);
                            destdisp.at<T>(y+1,x)=disp.at<T>(j,i);
                            destdisp.at<T>(y+1,x+1)=disp.at<T>(j,i);
                        }
                        else if((int)pt[count-image.cols].y-y<-1 && (int)pt[count-1].x-x<-1)
                        {
                            destimage.at<uchar>(y,3*x-3)=img[3*i+0];
                            destimage.at<uchar>(y,3*x-2)=img[3*i+1];
                            destimage.at<uchar>(y,3*x-1)=img[3*i+2];
                            
                            destimage.at<uchar>(y-1,3*x+0)=img[3*i+0];
                            destimage.at<uchar>(y-1,3*x+1)=img[3*i+1];
                            destimage.at<uchar>(y-1,3*x+2)=img[3*i+2];
                            
                            destimage.at<uchar>(y-1,3*x-3)=img[3*i+0];
                            destimage.at<uchar>(y-1,3*x-2)=img[3*i+1];
                            destimage.at<uchar>(y-1,3*x-1)=img[3*i+2];
                            
                            destdisp.at<T>(y,x-1)=disp.at<T>(j,i);
                            destdisp.at<T>(y-1,x)=disp.at<T>(j,i);
                            destdisp.at<T>(y-1,x-1)=disp.at<T>(j,i);
                        }
                        else if((int)pt[count+1].x-x>1)
                        {
                            destimage.at<uchar>(y,3*x+3)=img[3*i+0];
                            destimage.at<uchar>(y,3*x+4)=img[3*i+1];
                            destimage.at<uchar>(y,3*x+5)=img[3*i+2];
                            
                            destdisp.at<T>(y,x+1)=disp.at<T>(j,i);
                        }
                        else if((int)pt[count-1].x-x<-1)
                        {
                            destimage.at<uchar>(y,3*x-3)=img[3*i+0];
                            destimage.at<uchar>(y,3*x-2)=img[3*i+1];
                            destimage.at<uchar>(y,3*x-1)=img[3*i+2];
                            
                            destdisp.at<T>(y,x-1)=disp.at<T>(j,i);
                        }
                        else if((int)pt[count+image.cols].y-y>1)
                        {
                            destimage.at<uchar>(y+1,3*x+0)=img[3*i+0];
                            destimage.at<uchar>(y+1,3*x+1)=img[3*i+1];
                            destimage.at<uchar>(y+1,3*x+2)=img[3*i+2];
                            
                            destdisp.at<T>(y+1,x)=disp.at<T>(j,i);
                        }
                        else if((int)pt[count-image.cols].y-y<-1)
                        {
                            destimage.at<uchar>(y-1,3*x+0)=img[3*i+0];
                            destimage.at<uchar>(y-1,3*x+1)=img[3*i+1];
                            destimage.at<uchar>(y-1,3*x+2)=img[3*i+2];
                            
                            destdisp.at<T>(y-1,x)=disp.at<T>(j,i);
                        }
                    }
                }
            }
        }
    }
    
    if(isSub)
    {
        Mat image2;
        Mat disp2;
        destimage.copyTo(image2);
        destdisp.copyTo(disp2);
        const int BS=1;
        //pragma omp parallel for
        for(int j=BS;j<image.rows-BS;j++)
        {
            uchar* img=destimage.ptr<uchar>(j);
            T* m = disp2.ptr<T>(j);
            T* dp = destdisp.ptr<T>(j);
            for(int i=BS;i<image.cols-BS;i++)
            {
                if(m[i]==0)
                {
                    int count=0;
                    int d=0;
                    int r=0;
                    int g=0;
                    int b=0;
                    for(int l=-BS;l<=BS;l++)
                    {
                        T* dp2 = disp2.ptr<T>(j+l);
                        uchar* img2 = image2.ptr<uchar>(j+l);
                        for(int k=-BS;k<=BS;k++)
                        {
                            if(dp2[i+k]!=0)
                            {
                                count++;
                                d+=dp2[i+k];
                                r+=img2[3*(i+k)+0];
                                g+=img2[3*(i+k)+1];
                                b+=img2[3*(i+k)+2];
                            }
                        }
                    }
                    if(count!=0)
                    {
                        double div = 1.0/count;
                        dp[i]=d*div;
                        img[3*i+0]=r*div;
                        img[3*i+1]=g*div;
                        img[3*i+2]=b*div;
                    }
                }
            }
        }
    }
}


void projectImagefromXYZ(Mat& image, Mat& destimage, Mat& disp, Mat& destdisp, Mat& xyz, Mat& R, Mat& t, Mat& K, Mat& dist, bool isSub=true)
{
    
    Mat mask = Mat();
    if(mask.empty())mask=Mat::zeros(image.size(),CV_8U);
    if(disp.type()==CV_8U)
    {
        projectImagefromXYZ_<unsigned char>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
    }
    else if(disp.type()==CV_16S)
    {
        projectImagefromXYZ_<short>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
    }
    else if(disp.type()==CV_16U)
    {
        projectImagefromXYZ_<unsigned short>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
    }
    else if(disp.type()==CV_32F)
    {
        projectImagefromXYZ_<float>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
    }
    else if(disp.type()==CV_64F)
    {
        projectImagefromXYZ_<double>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
    }
}

void eular2rot(double yaw,double pitch, double roll,Mat& dest)
{
    double theta = yaw/180.0*CV_PI;
    double pusai = pitch/180.0*CV_PI;
    double phi = roll/180.0*CV_PI;
    
    double datax[3][3] = {{1.0,0.0,0.0},
        {0.0,cos(theta),-sin(theta)},
        {0.0,sin(theta),cos(theta)}};
    double datay[3][3] = {{cos(pusai),0.0,sin(pusai)},
        {0.0,1.0,0.0},
        {-sin(pusai),0.0,cos(pusai)}};
    double dataz[3][3] = {{cos(phi),-sin(phi),0.0},
        {sin(phi),cos(phi),0.0},
        {0.0,0.0,1.0}};
    Mat Rx(3,3,CV_64F,datax);
    Mat Ry(3,3,CV_64F,datay);
    Mat Rz(3,3,CV_64F,dataz);
    Mat rr=Rz*Rx*Ry;
    rr.copyTo(dest);
}


void lookat(Point3d from, Point3d to, Mat& destR)
{
    double x=(to.x-from.x);
    double y=(to.y-from.y);
    double z=(to.z-from.z);
    
    double pitch =asin(x/sqrt(x*x+z*z))/CV_PI*180.0;
    double yaw   =asin(-y/sqrt(y*y+z*z))/CV_PI*180.0;
    
    eular2rot(yaw, pitch, 0,destR);
}





void Render::enable_rendering(double focal_length,double baseline,Mat  vdisparity,Mat depthMap,Mat Q,Destriptor& des)
{
    Mat destimage,destdisp,dispshow,depth;
    
    StereoSGBM sbm(1,16*2,3,200,255,1,0,0,0,0,true);
    
    
    fillOcclusion(vdisparity,16);
    
    
    
    
    cv::reprojectImageTo3D(vdisparity,depth,Q);
    
    Mat xyz= depth.reshape(3,depth.size().area());
    
    // rendering part begines accordig to change the vier point
    Mat K=Mat::eye(3,3,CV_64F); // q
    K.at<double>(0,0)=focal_length;
    K.at<double>(1,1)=focal_length;
    K.at<double>(0,2)=(des.input_right.cols-1.0)/2.0;
    K.at<double>(1,2)=(des.input_left.rows-1.0)/2.0;
    
    Mat dist=Mat::zeros(5,1,CV_64F);
    Mat R=Mat::eye(3,3,CV_64F);
    Mat t=Mat::zeros(3,1,CV_64F);
    
    Point3d viewpoint(0.0,0.0,baseline*10);
    Point3d lookatpoint(0.0,0.0,-baseline*10.0);
    const double step=baseline;
    int key=0;
    bool isSub=true;

    while(key!='q')
    {
        lookat(viewpoint, lookatpoint , R);
        t.at<double>(0,0)=viewpoint.x;
        t.at<double>(1,0)=viewpoint.y;
        t.at<double>(2,0)=viewpoint.z;
        
        cout<<t<<endl;
        t=R*t;

         projectImagefromXYZ(des.input_right,destimage,vdisparity,destdisp,xyz,R,t,K,dist,isSub);
    
        destdisp.convertTo(dispshow,CV_8U,0.5);
        imshow("depth",dispshow);
        
        
        
        
        
        Mat dst;
        resize(destimage, dst, Size(), 2, 2, INTER_CUBIC);
        
        imshow("3Dinterface",dst);

        key = waitKey(1);
        if(key=='f')
        {
            isSub=isSub?false:true;
        }
        if(key=='w')
        {
            viewpoint.y+=step;
        }
        if(key=='e')
        {
            viewpoint.y-=step;
        }
        if(key=='d')
        {
            viewpoint.x+=step;
        }
        if(key=='a')
        {
            viewpoint.x-=step;
        }
        if(key=='z')
        {
            viewpoint.z+=step;
        }
        if(key=='c')
        {
            viewpoint.z-=step;
        }
    }
}

    
    
    
    
