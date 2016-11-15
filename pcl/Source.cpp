/*
/////////////////////////////////////////////////
ver.0.05版
-刪除不必要的函式
-刪除opencv 相關
-刪除深度轉二維呼叫
/////////////////////////////////////////////////
*/


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


// --------------
// -----Main-----
// --------------

int main(int argc, char** argv)
{
	float AZ=0;
	float lidar_Z[16][1800];
	float relidar_Z[16][1800];
	int fps = 0;
	
	pcl::visualization::PCLVisualizer viewer("point cloud");
	

	int updatef = 0;

	while (1)
	{
		memset(lidar_Z, 0, sizeof(lidar_Z));

		AZ = 0;

		UDP(lidar_Z,AZ);		

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
	}			
}