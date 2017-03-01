/*
/////////////////////////////////////////////////
ver.0.08版
- 使用kitti資料庫進行處理
/////////////////////////////////////////////////
*/


#include <iostream>
#include <fstream>
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
#include <stdlib.h> 
#include <atlstr.h>
#include <time.h>
#include <vector>
#include"dbscan.h"

#define PI 3.14159265
#define UNKNOWN_FLOW_THRESH 1e9  

using namespace cv;
using namespace std;
double *points2;

// ----------------------------
// -------dbscan所需函式-------
// ----------------------------

struct  point //clustering質心點
{
	double x;
	double y;
	double z;
};

vector<vector<point>> data2; //讀檔時用的
vector<vector<point>> clusters2; //dbscan用
vector<vector<vector<point>>> muti_clusters2; //接收dbscan用
vector<vector<vector<point>>> group2;  //群聚分割用
vector<vector<vector<point>>> resort_group2; //重新將群聚排列用
vector<vector<point>> edge_data2; //邊緣化用
//int layer_out=0; //輸出單一層layer用

bool comp2(const point &a, const point &b, const double &eps)
{
	//return (abs (a.first - b.first) < eps && abs (a.second - b.second) < eps);
	return (pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0) + pow(a.z - b.z, 2.0) < pow(eps, 2));
	//return (pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0) < pow(eps, 2));
}

float Euclid_dist2(const point &a, const point &b) //2D
{
	//return (sqrt((pow(a.x - b.x, 2.0) + pow(a.z - b.z, 2.0))));
	return (sqrt((pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0))));
}

float find_max2(vector<point> clu)
{
	point max;
	max.x = -99;
	for (int i = 0; i<clu.size(); i++)
	{
		if (max.x<clu[i].x)
			max = clu[i];
	}
	return max.x;
}

float find_min2(vector<point> clu)
{
	point min;
	min.x = -99;
	for (int i = 0; i<clu.size(); i++)
	{
		if (min.x<clu[i].x)
			min = clu[i];
	}
	return min.x;
}

int clustering2(vector<point> src_data) //dbscan
{
	unsigned int minimumPointsToFormCluster = 0;
	unsigned int minimumPointsForCluster = 2;
	double epsilon = 200;
	DBSCAN<point> dbscan(minimumPointsToFormCluster, minimumPointsForCluster, epsilon, comp2);
	dbscan.setData(src_data);
	dbscan.scan();
	src_data.clear();
	clusters2 = dbscan.getClusters();

	//輸出其中一層dbscan並上色
	/*
	Mat One_Layer_dbScan(1000, 1000, CV_8UC3);
	int px, py;
	for (int i = 0; i<clusters.size(); i++)
	{
	int r = rand() % 255;
	int g = rand() % 255;
	int b = rand() % 255;
	for (int j = 0; j<clusters[i].size(); j++)
	{
	px = (clusters[i][j].x / 100) + 400;
	py = (clusters[i][j].y / 100) + 400;
	One_Layer_dbScan.at<Vec3b>(px, py)[0] = r;
	One_Layer_dbScan.at<Vec3b>(px, py)[1] = g;
	One_Layer_dbScan.at<Vec3b>(px, py)[2] = b;
	}
	}
	imshow("One_Layer_dbScan", One_Layer_dbScan);
	cvWaitKey(1);
	*/
	//layer_out++;
	//if(layer_out==10)
	//{
	//	FILE *group_obj=fopen("layer.pcd","w"); 
	//	fprintf(group_obj,"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n");
	//	fprintf(group_obj,"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n",409);
	//	for(int i=0; i<clusters.size(); i++)
	//	{
	//		for(int j=0; j<clusters[i].size(); j++)
	//		{
	//			fprintf(group_obj,"%f %f %f %f\n",clusters[i][j].x, clusters[i][j].y, clusters[i][j].z, float(i)*5+float(i)/10);
	//		}
	//	}
	//}

	return 0;
}
// --------------
// -----Main-----
// --------------

int main(int argc, char** argv)
{
	kitti();	
	
	//------------------16_Layer_Lidar---------------------
	//float AZ =0;
	/*
	float AZ[905];
	float lidar_Z[16][1809];
	int fps = 0;
	points2 = new double[870000];
	pcl::visualization::PCLVisualizer viewer("point cloud");
	
	int updatef = 0;

	while (1)
	{
		memset(lidar_Z, 0, sizeof(lidar_Z));
		memset(AZ, 0, sizeof(AZ));
		//AZ = 0;

		//UDP(lidar_Z,AZ);		
		UDP(lidar_Z, AZ);
		float omega = 0;
		float angle = 15;
		float angle2 = AZ[0];
		for (int i = 0; i < 16; i++)
		{
			omega = angle * PI / 180;

			float alpha = 0;
			int k = 0;
			int f = 0;
			for (int j = 0; j < 1808; j++)
			{
				alpha = angle2 * PI / 180;
				points2[(i * 1808 + j ) * 3] = lidar_Z[i][j] * cos(omega) * sin(alpha);
				points2[(i * 1808 + j ) * 3 + 1] = lidar_Z[i][j] * cos(omega) * cos(alpha);
				points2[(i * 1808 + j ) * 3 + 2] = lidar_Z[i][j] * sin(omega);
								
				if (f == 0)
				{
					angle2 = AZ[k];
					angle2 = angle2 + 0.2;
					f = 1;
				}
				else if (f == 1)
				{
					k = k + 1;
					angle2 = AZ[k];
					f = 0;
				}
				if (angle2 >= 360)
					angle2 = angle2 - 360;			
			}
			angle = angle - 2;
		}
		/*
		vector<point> temp_data;		
		for (int i = 0; i < 28928; i=i+3)
		{					
			point temp_point;
			temp_point.x = points2[i];
			temp_point.y = points2[i + 2];
			temp_point.z = points2[i + 1];
			if (points2[i + 2] < -1.2)
			{
				temp_point.x = 0;
				temp_point.y = 0;
				temp_point.z = 0;			}
				
			if (points2[i] > 10 || points2[i + 1] > 10)
			{
				temp_point.x = 0;
				temp_point.y = 0;
				temp_point.z = 0;
			}				
			if (points2[i] < -10 || points2[i + 1] < -10)
			{
				temp_point.x = 0;
				temp_point.y = 0;
				temp_point.z = 0;
			}						
			temp_data.push_back(temp_point);	
				
		}
		if (temp_data.size()>0)
			data2.push_back(temp_data);
		    temp_data.clear();
		for (int j = 0; j<data2.size(); j++)
		{
				clustering2(data2[j]);
				muti_clusters2.push_back(clusters2);
				//clusters.clear();
				cout << "layer" << j << endl;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		uint8_t r(0), g(0), b(0);
		for (int i = 0; i<clusters2.size(); i++)
		{
			int r = rand() % 255;
			int g = rand() % 255;
			int b = rand() % 255;
			for (int j = 0; j<clusters2[i].size(); j++)
			{								

				pcl::PointXYZ basic_point;

				basic_point.x = clusters2[i][j].x;
				basic_point.y = clusters2[i][j].y;
				basic_point.z = clusters2[i][j].z;

				basic_cloud_ptr->points.push_back(basic_point);

				pcl::PointXYZRGBA point;

				point.x = basic_point.x;
				point.y = basic_point.y;
				point.z = basic_point.z;

				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

				point.rgb = *reinterpret_cast<float*>(&rgb);
				point_cloud_ptr->points.push_back(point);
			}
		}
		*/
/*
		pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);		
		uint8_t r(0), g(0), b(0);

		for (int i = 0; i < 16; i++)
		{			
			for (int j = 0; j < 1808; j++)
			{
				pcl::PointXYZ basic_point;				
				
				basic_point.x = points2[(i * 1808 + j) * 3];
				basic_point.y = points2[(i * 1808 + j) * 3 + 1];
				basic_point.z = points2[(i * 1808 + j) * 3 + 2];

				basic_cloud_ptr->points.push_back(basic_point);

				pcl::PointXYZRGBA point;

				point.x = basic_point.x;
				point.y = basic_point.y;
				point.z = basic_point.z;

				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				int t = 255;
				rgb = (static_cast<uint32_t>(t) << 16 | static_cast<uint32_t>(t) << 8 | static_cast<uint32_t>(t));
				
				if (points2[(i * 1808 + j) * 3 + 2] < -1.2)
				rgb = (static_cast<uint32_t>(0) << 16 | static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(0));				

				point.rgb = *reinterpret_cast<float*>(&rgb);
				point_cloud_ptr->points.push_back(point);				
													
			}		
		}
		
		basic_cloud_ptr->width = 1;
		basic_cloud_ptr->height = 1;
		point_cloud_ptr->width = 1;
		point_cloud_ptr->height = 1;

		//viewer.removePointCloud("cloud");

		//stringstream ss;
		//ss << "frame " << fps++;	
		if (updatef == 0)
		{			
			//viewer.addText(ss.str(), 10, 10, "frameText", 0);
			//viewer.addPointCloud(basic_cloud_ptr, "cloud");
			viewer.addPointCloud(point_cloud_ptr, "cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			viewer.addCoordinateSystem(1.0);
			viewer.initCameraParameters();
			updatef = 1;
		}
		//viewer.updatePointCloud(basic_cloud_ptr, "cloud");
		viewer.updatePointCloud(point_cloud_ptr, "cloud");
		//viewer.addPointCloud(point_cloud_ptr, "cloud");
		//viewer.updateText(ss.str(), 10, 10, "frameText");				
		viewer.spinOnce(1);		
	}			
	*/
	
}