#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <core.hpp>

#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <cmath>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/console/parse.h>

#include "UDP.h"

#define PI 3.14159265
#define UNKNOWN_FLOW_THRESH 1e9  

using namespace cv;
using namespace std;




void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, const Scalar& color) 
{
	for (int y = 0; y < cflowmap.rows; y += step)
	{
		for (int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at< Point2f>(y, x);
			line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
			circle(cflowmap, Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), 1, color, -1);
		}
	}
}


void makecolorwheel(vector<Scalar> &colorwheel)
{
	int RY = 15;
	int YG = 6;
	int GC = 4;
	int CB = 11;
	int BM = 13;
	int MR = 6;

	int i;

	for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255, 255 * i / RY, 0));
	for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255 - 255 * i / YG, 255, 0));
	for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0, 255, 255 * i / GC));
	for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0, 255 - 255 * i / CB, 255));
	for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255 * i / BM, 0, 255));
	for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255, 0, 255 - 255 * i / MR));
}

void motionToColor(Mat flow, Mat &color)
{
	if (color.empty())
		color.create(flow.rows, flow.cols, CV_8UC3);

	static vector<Scalar> colorwheel; //Scalar r,g,b  
	if (colorwheel.empty())
		makecolorwheel(colorwheel);

	// determine motion range:  
	float maxrad = -1;

	// Find max flow to normalize fx and fy  
	for (int i = 0; i < flow.rows; ++i)
	{
		for (int j = 0; j < flow.cols; ++j)
		{
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);
			float fx = flow_at_point[0];
			float fy = flow_at_point[1];
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
				continue;
			float rad = sqrt(fx * fx + fy * fy);
			maxrad = maxrad > rad ? maxrad : rad;
		}
	}

	for (int i = 0; i < flow.rows; ++i)
	{
		for (int j = 0; j < flow.cols; ++j)
		{
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);

			float fx = flow_at_point[0] / maxrad;
			float fy = flow_at_point[1] / maxrad;
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
			{
				data[0] = data[1] = data[2] = 0;
				continue;
			}
			float rad = sqrt(fx * fx + fy * fy);

			float angle = atan2(-fy, -fx) / CV_PI;
			float fk = (angle + 1.0) / 2.0 * (colorwheel.size() - 1);
			int k0 = (int)fk;
			int k1 = (k0 + 1) % colorwheel.size();
			float f = fk - k0;
			//f = 0; // uncomment to see original color wheel  

			for (int b = 0; b < 3; b++)
			{
				float col0 = colorwheel[k0][b] / 255.0;
				float col1 = colorwheel[k1][b] / 255.0;
				float col = (1 - f) * col0 + f * col1;
				if (rad <= 1)
					col = 1 - rad * (1 - col); // increase saturation with radius  
				else
					col *= .75; // out of range  
				data[2 - b] = (int)(255.0 * col);
			}
		}
	}
}

// --------------
// -----Main-----
// --------------

int main(int argc, char** argv)
{
	float AZ=0;
	float lidar_Z[16][1800];
	float relidar_Z[16][1800];
	int fps = 0;
	/*
	Mat preimg(16, 1800, CV_8U);
	Mat img(16, 1800, CV_8U);
	Mat prersimg;
	Mat rsimg;
	Mat motion2color;
	*/
	/*
	UDP(relidar_Z,AZ);

	twoDimg(lidar_Z, preimg);		

	resize(preimg, prersimg, Size(1260, 160), 0, 0, CV_INTER_LINEAR);
	*/
	pcl::visualization::PCLVisualizer viewer("point cloud");
	

	int updatef = 0;

	while (1)
	{
		memset(lidar_Z, 0, sizeof(lidar_Z));

		AZ = 0;

		UDP(lidar_Z,AZ);

		//printf("Azimuth = %.2f \n", AZ);

		//twoDimg(lidar_Z, img);

		//resize(img, rsimg, Size(1260, 160), 0, 0, CV_INTER_LINEAR);

		//imshow("PointTO2dimage", rsimg);
		//cvWaitKey(1);

		pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		float omega = 0;
		float angle = 15;
		float angle2 = AZ;
		uint8_t r(255), g(0), b(0);
		for (int i = 0; i < 16; i++)
		{
			omega = angle * PI / 180;			

			float alpha = 0;
			for (int j = 0; j < 1800; j++)
			{
				pcl::PointXYZ basic_point;

				alpha = angle2 * PI / 180;				

				basic_point.x = lidar_Z[i][j] * cos(omega) * sin(alpha);
				basic_point.y = lidar_Z[i][j] * cos(omega) * cos(alpha);
				basic_point.z = lidar_Z[i][j] * sin(omega);

				basic_cloud_ptr->points.push_back(basic_point);

				/*
				pcl::PointXYZRGBA point;
				point.x = basic_point.x;
				point.y = basic_point.y;
				point.z = basic_point.z;
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

				int t = 0;

				if (lidar_Z[i][j] < 5)
				{
					t = lidar_Z[i][j] * 25;
					if (t>255)t = 255;
					rgb = (static_cast<uint32_t>(t) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				}
				else if (lidar_Z[i][j] >= 10 && lidar_Z[i][j] < 20)
				{
					t = (lidar_Z[i][j] - 10) * 13;
					if (t>255)t = 255;
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(t) << 8 | static_cast<uint32_t>(b));
				}
				else
				{
					t = (lidar_Z[i][j] - 20) * 7;
					if (t>255)t = 255;
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(t));
				}

				point.rgb = *reinterpret_cast<float*>(&rgb);

				point_cloud_ptr->points.push_back(point);*/

				angle2 = angle2 + 0.2;
				if (angle2 >= 360)
					angle2 = angle2 - 360;
				
			}
			angle = angle - 2;
		}
		basic_cloud_ptr->width = 1;
		basic_cloud_ptr->height = 1;
		point_cloud_ptr->width = 1;
		point_cloud_ptr->height = 1;

		//viewer.removePointCloud("cloud");

		stringstream ss;
		ss << "frame " << fps++;	
		if (updatef == 0)
		{			
			viewer.addText(ss.str(), 10, 10, "frameText", 0);
			viewer.addPointCloud(basic_cloud_ptr, "cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			viewer.addCoordinateSystem(1.0);
			viewer.initCameraParameters();
			updatef = 1;
		}
		viewer.updatePointCloud(basic_cloud_ptr, "cloud");
		//viewer.addPointCloud(point_cloud_ptr, "cloud");

		viewer.updateText(ss.str(), 10, 10, "frameText");
				
		viewer.spinOnce(1);
		
		/*
		Mat flow;

        calcOpticalFlowFarneback(prersimg, rsimg, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

	    Mat cflow;
		
	    cvtColor(prersimg, cflow, CV_GRAY2BGR);
		
	    drawOptFlowMap(flow, cflow, 10, CV_RGB(0, 255, 0));
	
	    imshow("OpticalFlowFarneback", cflow);
	    imshow("PointTO2dimage", prersimg);
		*/

	   //motionToColor(flow, motion2color);
	   //imshow("flow", motion2color);		

	    /*Mat xy[2];
		
		split(flow, xy);

		Mat magnitude, angle;

		cartToPolar(xy[0], xy[1], magnitude, angle, true);

		double mag_max;

		minMaxLoc(magnitude, 0, &mag_max);

		magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

		Mat _hsv[3], hsv;

		_hsv[0] = angle;
		_hsv[1] = Mat::ones(angle.size(), CV_32F);
	    _hsv[2] = magnitude;		

		merge(_hsv, 3, hsv);

		Mat bgr;

		cvtColor(hsv, bgr, COLOR_HSV2BGR);

		imshow("SimpleFlow", bgr);*/
				 
		//cvWaitKey(1);

		//preimg = img;		
		//resize(preimg, prersimg, Size(1260, 160), 0, 0, CV_INTER_LINEAR);
	}			
}