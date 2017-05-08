#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <librealsense/rs.h>

#include <librealsense/rs.hpp>
/*
#include <librealsense/rscore.h>
#include <librealsense/rsutil.h>
*/

using namespace std;
using namespace cv;
using namespace rs;
class calXYZByDiff{
public:

vector<float> calZ(vector<pair<cv::Point,cv::Point>> matches,const rs::intrinsics  color_intrin,const rs::intrinsics  ir_intrin,const   rs::extrinsics extrin)
{
	/*cout<<"color_intrin.model():"<<color_intrin.model()<<endl;
	cout<<"ir_intrin.model():"<<ir_intrin.model()<<endl;
	*/
	vector<float> z;
 
	float begin=0.0;
	float end=1000.0;//1 m=1000 mm
	int len=matches.size();
	for(int j=0;j<len;j++)
		for(float i=0.001;i<1.0;i+=0.001)
		{


		//cout<<" color_intrin.model()"<< ir_intrin.model()<<endl;
		float depth=i;
		const float2 & pixel={(float)matches[j].second.x,(float)matches[j].second.y};
			/*the color_intrin.modle() is none,is not true*/	
		float3 ir_threeD={0.0,0.0,0.0};//color_intrin.deproject(pixel,depth) ;//pixel[2]+depth------->color's 3D 
		ir_threeD=ir_intrin.deproject( pixel,  depth) ;
			
			
		//cout<<"color_threeD:("<<color_threeD.x<<","<<color_threeD.y<<","<<color_threeD.z<<")"<<endl;
		const float3 & ir_3D=ir_threeD;
		float3 color_threeD=extrin.transform(ir_3D);// color's 3D---->ir's 3D
		//cout<<"depth:"<<color_threeD.z<<endl;
		 
		float2  color_pixel={0.0,0.0};
		 
		 const float3 & color_3D=color_threeD;
		 color_pixel=color_intrin.project(color_3D);
		  /*float x = color_threeD.x / color_threeD.z, y = color_threeD.y / color_threeD.z;
    
        float r2  = x*x + y*y;
        float f = 1 + color_intrin.coeffs[0]*r2 + color_intrin.coeffs[1]*r2*r2 + color_intrin.coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*color_intrin.coeffs[2]*x*y + color_intrin.coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*color_intrin.coeffs[3]*x*y + color_intrin.coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
     
   color_pixel.x = x * color_intrin.fx + color_intrin.ppx;
    color_pixel.y = y * color_intrin.fy + color_intrin.ppy;

		*/
	/*cout<<"color_intrin.ppx"<<color_intrin.ppx<<endl;
	cout<<"color_intrin.ppy"<<color_intrin.ppy<<endl;
	*/
		if(color_pixel.x<0||color_pixel.y<0) continue;
		 
		/*cout<<"match x:"<<matches[j].first.x<<"        "<<color_pixel.x<<endl;
		cout<<"match y:"<<matches[j].first.y<<"        "<<color_pixel.y<<endl;
		*/
		if(abs(matches[j].first.x-color_pixel.x)<8&&abs(matches[j].first.y-color_pixel.y)<8)	
			{
			z.push_back(i);
                        cout<<"final depth:"<<i<<endl;
                        break;
			}
		}
	 return z;

}


vector<float> calZ(vector<pair<cv::Point,cv::Point>> matches,const rs::intrinsics  color_intrin,const rs::intrinsics  ir_intrin,const   rs::extrinsics extrin,const int )
{//cal two distance and average of them 
	//float pixel[2];
	vector<float> z;
	float begin=0.0;
	float end=1000.0;//1 m=1000 mm
	int len=matches.size();
	for(int j=0;j<len;j++)
		for(float i=5.0;i<1000.0;i+=0.1)
		{


		//cout<<" color_intrin.model()"<< ir_intrin.model()<<endl;
		float depth=i;
		const float2 & pixel={(float)matches[j].first.x,(float)matches[j].first.y};
			/*the color_intrin.modle() is none,is not true*/	
			
		float3 color_threeD={0.0,0.0,0.0};//color_intrin.deproject(pixel,depth) ;//pixel[2]+depth------->color's 3D
		 float x = (pixel.x - color_intrin.ppx) / color_intrin.fx;
   		 float y = (pixel.y - color_intrin.ppy) / color_intrin.fy;
   		  
			float r2  = x*x + y*y;
			float f = 1 + color_intrin.coeffs[0]*r2 + color_intrin.coeffs[1]*r2*r2 + color_intrin.coeffs[4]*r2*r2*r2;
			float ux = x*f + 2*color_intrin.coeffs[2]*x*y + color_intrin.coeffs[3]*(r2 + 2*x*x);
			float uy = y*f + 2*color_intrin.coeffs[3]*x*y + color_intrin.coeffs[2]*(r2 + 2*y*y);
			x = ux;
			y = uy;
   			 
		    color_threeD.x = depth * x;
		    color_threeD.y = depth * y;
		    color_threeD.z = depth;
			
			
			
		//cout<<"color_threeD:("<<color_threeD.x<<","<<color_threeD.y<<","<<color_threeD.z<<")"<<endl;
		const float3 & color_3D=color_threeD;
		float3 ir_threeD=extrin.transform(color_3D);// color's 3D---->ir's 3D
		
		//cout<<"ir_intrin.x:"<<ir_intrin.fx<<"ir_intrin.y:"<<ir_intrin.fy<<endl;
		float2  ir_pixel={0.0,0.0};
		/*ir_pixel.x= ir_intrin.fx*ir_threeD.x/ir_threeD.z+ir_intrin.ppx;
		ir_pixel.y= ir_intrin.fx*ir_threeD.y/ir_threeD.z+ir_intrin.ppy;
		*/
		  x = ir_threeD.x / ir_threeD.z;
		  y = ir_threeD.y / ir_threeD.z;
    		 
		r2  = x*x + y*y;
		f = 1 + ir_intrin.coeffs[0]*r2 + ir_intrin.coeffs[1]*r2*r2 + ir_intrin.coeffs[4]*r2*r2*r2;
		x *= f;
		y *= f;
		float dx = x + 2*ir_intrin.coeffs[2]*x*y + ir_intrin.coeffs[3]*(r2 + 2*x*x);
		float dy = y + 2*ir_intrin.coeffs[3]*x*y + ir_intrin.coeffs[2]*(r2 + 2*y*y);
		x = dx;
		y = dy;
		    
    		ir_pixel.x = x * ir_intrin.fx + ir_intrin.ppx;
    		ir_pixel.y = y * ir_intrin.fy + ir_intrin.ppy;		


		
		if(ir_pixel.x<0||ir_pixel.y<0) continue;
		 
		//cout<<"match:"<<matches[j].second.x<<","<<ir_pixel.x<<endl;
		if(abs(matches[j].second.x-ir_pixel.x)<8&&abs(matches[j].second.y-ir_pixel.y)<8)	
			{
			z.push_back(i);
                        cout<<"depth:"<<i<<endl;
                        break;
			}
		}
	 return z;
}
};
