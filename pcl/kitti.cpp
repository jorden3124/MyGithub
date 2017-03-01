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

#include <stdlib.h> 
#include <atlstr.h>
#include <time.h>
#include <vector>
#include"dbscan.h"

using namespace cv;
using namespace std;
// ----------------------------
// ----------預設參數----------
// ----------------------------
double *points;
double *prepoints;
double *PA;
double readedpoints[89][400129];
UCHAR pm2[89][400129];
UCHAR *pMarks;
//double LidarH = 173;
int e[100];
int C = 133376;
const int LayerLth = 180;
const int LidarH = 780;
int gpsize = 0;
struct  point //clustering質心點
{
	double x;
	double y;
	double z;
};

vector<vector<point>> data; //讀檔時用的
vector<vector<point>> clusters; //dbscan用
vector<vector<vector<point>>> muti_clusters; //接收dbscan用
vector<vector<vector<point>>> group;  //群聚分割用
vector<vector<vector<point>>> resort_group; //重新將群聚排列用
vector<vector<point>> edge_data; //邊緣化用
//int layer_out=0; //輸出單一層layer用
// ----------------------------
// -----移除地面點所需函式-----
// ----------------------------
double calDistH(int P1){

	double d = 0;
	d = LidarH + points[P1 * 3 + 1];

	return d;
}

double calDist(int P1, int P2){
	double d = 0;
	d = points[P2 * 3 + 1] - points[P1 * 3 + 1];
	return d;
}

double calDistP(int P1, int P2){

	double d = 0, x = 0, z = 0;
	x = points[P2 * 3] - points[P1 * 3];
	z = points[P2 * 3 + 2] - points[P1 * 3 + 2];

	d = sqrt((x*x) + (z*z));

	return d;
}

double HSpd_calDistPlane(int P1, int P2){

	double d = 0, x = 0, z = 0;
	x = points[P2 * 3] - points[P1 * 3];
	z = points[P2 * 3 + 2] - points[P1 * 3 + 2];

	d = (x*x) + (z*z);

	return d;
}

double HSpd_calSLOPE(int P1, int P2, double Pd){

	double d = 0, y = 0;
	y = points[P2 * 3 + 1] - points[P1 * 3 + 1];
	d = (y*y / Pd); //Slope的2次方
	return d;
}



double calTheta(int P1, int P2){

	double d;
	double x, y, z;

	x = points[P2 * 3] - points[P1 * 3];
	y = points[P2 * 3 + 1] - points[P1 * 3 + 1];
	z = points[P2 * 3 + 2] - points[P1 * 3 + 2];
	d = (abs(y) / sqrt(x*x + z*z));

	return d;
}
int FUN_PATHDETECT(int k){

	for (int i = 0; i < k; i = i + 3)
	{
		if (i<e[22] && i> e[21])
			pMarks[i] = 1;
		//else
		//pMarks[ i ] = 2;

	}

	return 0;
}
int FUN_SAVEPLY(int fc){

	string s;
	string fileb = ".ply";
	string root = "C:\\Users\\user\\Desktop\\new data\\";
	char sfc[3];
	sprintf(sfc, "%d", fc);

	s = root + "group_" + sfc + fileb;


	fstream cloudFileOut;
	cloudFileOut.open(s, ::fstream::out); // read only
	if (!cloudFileOut){
		return 1;//FAIL
	}

	cloudFileOut << "ply" << endl
		<< "format ascii 1.0" << endl
		<< "element vertex " << gpsize << endl
		<< "property double x" << endl
		<< "property double y" << endl
		<< "property double z" << endl
		<< "property uchar blue" << endl
		<< "property uchar green" << endl
		<< "property uchar red" << endl
		<< "end_header" << endl
		;
	/*
	for (int i = 0; i < C; i++){

		cloudFileOut << points[i * 3] << " " << points[i * 3 + 1] << " " << points[i * 3 + 2] << " ";

		switch (pMarks[i]){

		case 0:
			cloudFileOut << "255 255 255" << endl;
			break;
		case 1:
			cloudFileOut << "0 255 0" << endl;
			break;
		case 2:
			cloudFileOut << "0 0 255" << endl;
			break;
		}
	}
	*/
	for (int i = 0; i<group.size(); i++)
	{
		int r = rand() % 255;
		int g = rand() % 255;
		int b = rand() % 255;
		for (int j = 0; j<group[i].size(); j++)
		{
			for (int k = 0; k<group[i][j].size(); k++)
			{
				cloudFileOut << group[i][j][k].x <<" "<<group[i][j][k].y<<" "<< group[i][j][k].z<<" "<< r <<" " << g << " " << b<<endl;				
			}
		}
	}
	cloudFileOut.close();
	return 0;
}

int VK(int fc){
	//20160712 new ver
	int GGC = 0; // global ground counter 
	int LGC = 0; //  local ground counter 
	/* fstream CFO;
	CFO.open("adj_5cm.ply",::fstream::out); // read only
	if (!CFO){
	return 1;//FAIL
	}*/
	int numPperL = 64;
	int numL = C / numPperL;
	bool CTN = false;
	for (int scL = 0; scL < 2084; scL++){
		LGC = 0;
		int prevGround = -1;
		//if  ( (scL >300)&& (scL <340)) continue;
		//every Layer first point
		for (int i = 10; i < numPperL; i++){
			if ((points[(numPperL*scL + i) * 3] == 0) && (points[(numPperL*scL + i) * 3 + 1] == 0) && (points[(numPperL*scL + i) * 3 + 2] == 0)){
				continue;
			} // null point, sky points
			else
			{
				pMarks[numPperL*scL + i] = 2;
				pm2[fc][numPperL*scL + i] = 2;
			}
			//if(i>90)continue;		

			if (prevGround  <  0){
				if (abs(calDistH(numPperL*scL + i) + 80) <= 150){
					prevGround = i;

					CTN = true;
				}
				else
				{
					pMarks[numPperL*scL + i] = 2;
					pm2[fc][numPperL*scL + i] = 2;
				}
				continue;
			}//find start ground point and next point 
			double disp = HSpd_calDistPlane(numPperL*scL + i, numPperL*scL + prevGround);
			double thH = 75;
			double thT = 0.3;

			if (disp >= 1000 * 1000){
				thT += (1 - (disp / 1000000)) / 100;
				if (thT<0)thT = 0;

				//if (disp>1500*1500) CTN = false;	
			}
			//thH = ( disp /4 ) +  25 ;
			if (disp< 625){

				thT += ((625 - disp) / 625)*0.9;
				//thT = 0.3;
			}
			/*	if (calDistP(numPperL*scL+prevGround,numPperL*scL)<calDistP(numPperL*scL+i,numPperL*scL))
			if ((calTheta(numPperL*scL+i,numPperL*scL+prevGround ) <= abs(thT)  ) && (abs(calDist(numPperL*scL+i,numPperL*scL+prevGround)) <= thH*100) )	 {
			*/
			double HCS = HSpd_calSLOPE(numPperL*scL + i, numPperL*scL + prevGround, disp);
			// 資料蒐集
			//  if (disp< 900){	 CFO <<  sqrt(disp) <<  " "  <<  sqrt(HCS) <<   endl ;  }
			if (HSpd_calSLOPE(numPperL*scL + i, numPperL*scL + prevGround, disp) <= (thT*thT)) {

				if (HSpd_calDistPlane(numPperL*scL + prevGround, numPperL*scL) < HSpd_calDistPlane(numPperL*scL + i, numPperL*scL))
				{
					if (CTN == true){
						pMarks[numPperL*scL + i] = 1;
						pMarks[numPperL*scL + prevGround] = 1;
						pm2[fc][numPperL*scL + i] = 1;
						pm2[fc][numPperL*scL + prevGround] = 1;
						// {LGC++;}
					}
					else{
						CTN = true;
					}
					prevGround = i;
				}
				// 資料蒐集
				// if (disp> 900*900){	 CFO <<  sqrt(disp)/5 <<  " "  <<  sqrt(HCS) <<  " " <<   1  << endl ;  }
			}
			/*	 else{
			// 資料蒐集
			if (disp> 900*900){	 CFO << sqrt(disp)/5 <<  " "  <<  sqrt(HCS) <<  " " <<  2  << endl ;}
			}// Tslope*/
			if ((i - prevGround)>1){ //地面連續中斷
				pMarks[numPperL*scL + i] = 2;
				pm2[fc][numPperL*scL + i] = 2;
				CTN = false;
			}
			/* else{
			}*/
		}
		//GGC+=LGC;
	}
	// CFO.close();
	return 0;
}
int open_kt(string filename, int j)
{
	fstream cloudFile;
	cloudFile.open(filename, ::fstream::in); // read only
	if (!cloudFile){
		return 1;//FAIL
	}
	float maxvalue=0;
	C = 133376;
	points = new double[C * 3]; //每個點有X Y Z 三筆浮點數
	pMarks = new UCHAR[C];
	double AA = 0;
	for (int i = C - 1; i >= 0; i--){
		cloudFile >> AA;
		points[(i * 3)] = AA * 500;
		readedpoints[j][i * 3] = points[(i * 3)];
		cloudFile >> AA;
		points[(i * 3 + 2)] = AA * 500;
		readedpoints[j][i * 3 + 2] = points[(i * 3) + 2];
		cloudFile >> AA;
		points[(i * 3 + 1)] = AA * 500;
		readedpoints[j][i * 3 + 1] = points[(i * 3) + 1];
		//cloudFile >> AA;				
		pMarks[i] = 1;
		pm2[j][i] = 1;
	}
	cloudFile.close();
	return 0;
}

// ----------------------------
// -------dbscan所需函式-------
// ----------------------------



bool comp(const point &a, const point &b, const double &eps)
{
	//return (abs (a.first - b.first) < eps && abs (a.second - b.second) < eps);
	return (pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0) + pow(a.z - b.z, 2.0) < pow(eps, 2));
	//return (pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0) < pow(eps, 2));
}

float Euclid_dist(const point &a, const point &b) //2D
{
	//return (sqrt((pow(a.x - b.x, 2.0) + pow(a.z - b.z, 2.0))));
	return (sqrt((pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0))));
}

float find_max(vector<point> clu)
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

float find_min(vector<point> clu)
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

int clustering(vector<point> src_data) //dbscan
{
	unsigned int minimumPointsToFormCluster = 0;
	unsigned int minimumPointsForCluster = 2;
	double epsilon = 200;
	DBSCAN<point> dbscan(minimumPointsToFormCluster, minimumPointsForCluster, epsilon, comp);
	dbscan.setData(src_data);
	dbscan.scan();
	src_data.clear();
	clusters = dbscan.getClusters();

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

void read_file(int fc)
{
	vector<point> temp_data;
	int forc = 0;
	for (int j = 0; j < 64; j++)
	{
		for (int i = 0; i < 2084; i++)
		{
			point temp_point;
			if (pm2[fc][(j + i * 64)] == 2)
			{
				temp_point.x = points[(j + i * 64) * 3];
				temp_point.y = points[(j + i * 64) * 3 + 2];
				temp_point.z = points[(j + i * 64) * 3 + 1];
			}
			else
			{
				temp_point.x = 0;
				temp_point.y = 0;
				temp_point.z = 0;
			}
			temp_data.push_back(temp_point);
			forc++;
		}		
		if (temp_data.size()>0)
			data.push_back(temp_data);
		temp_data.clear();
	}	
	
	cout <<"for times" <<forc<<endl;
}

void centre_group()
{
	bool isGroup[100][10000] = { false };
	float centre_diff = 750; //質心距離差
	for (int i = 0; i<muti_clusters.size(); i++) //比對層
	{
		for (int j = 0; j<muti_clusters[i].size(); j++)
		{
			vector<vector<point>> tempGroup;
			if (isGroup[i][j] == false)
			{
				tempGroup.push_back(muti_clusters[i][j]); //先放入暫存群集
				isGroup[i][j] = true;
				point curr_cen;
				curr_cen.x = 0;
				curr_cen.y = 0;
				curr_cen.z = 0;
				for (int num = 0; num<muti_clusters[i][j].size(); num++)
				{
					curr_cen.x += muti_clusters[i][j][num].x;
					curr_cen.y += muti_clusters[i][j][num].y;
					curr_cen.z += muti_clusters[i][j][num].z;
				}
				curr_cen.x = curr_cen.x / muti_clusters[i][j].size();
				curr_cen.y = curr_cen.y / muti_clusters[i][j].size();
				curr_cen.z = curr_cen.z / muti_clusters[i][j].size();

				vector<point> muti_cen;
				for (int m = i; m<muti_clusters.size(); m++) //被比對層
				{
					int cluster_count = 0; //紀錄同一曾被加幾個cluster
					int muti_cen_flag = 0;  //紀錄質心被改變幾個 0沒有改變 1第一個質心改變 2第二個質心改變 3兩個都改變
					for (int n = 0; n<muti_clusters[m].size(); n++)
					{
						if (isGroup[m][n] == false)
						{
							point compair_cen;
							compair_cen.x = 0;
							compair_cen.y = 0;
							compair_cen.z = 0;
							for (int num = 0; num<muti_clusters[m][n].size(); num++)
							{
								compair_cen.x += muti_clusters[m][n][num].x;
								compair_cen.y += muti_clusters[m][n][num].y;
								compair_cen.z += muti_clusters[m][n][num].z;
							}
							compair_cen.x = compair_cen.x / muti_clusters[m][n].size();
							compair_cen.y = compair_cen.y / muti_clusters[m][n].size();
							compair_cen.z = compair_cen.z / muti_clusters[m][n].size();
							// 比對質心間的距離
							if (muti_cen.size() == 2)
							{
								float centre_EU1 = Euclid_dist(muti_cen[0], compair_cen);
								float centre_EU2 = Euclid_dist(muti_cen[1], compair_cen);
								if (centre_EU1<centre_diff)
								{
									tempGroup.push_back(muti_clusters[m][n]);
									cluster_count++;
									muti_cen[0] = compair_cen;
									isGroup[m][n] = true;
									if (muti_cen_flag == 0)
										muti_cen_flag = 1;
									else
										muti_cen_flag = 3;
								}
								else if (centre_EU2<centre_diff)
								{
									tempGroup.push_back(muti_clusters[m][n]);
									cluster_count++;
									muti_cen[1] = compair_cen;
									isGroup[m][n] = true;
									if (muti_cen_flag == 0)
										muti_cen_flag = 2;
									else
										muti_cen_flag = 3;
								}
							}
							else
							{
								float centre_EU = Euclid_dist(curr_cen, compair_cen);
								if (centre_EU<centre_diff)
								{
									tempGroup.push_back(muti_clusters[m][n]);
									cluster_count++;
									if (cluster_count == 1)
									{
										curr_cen = compair_cen; //換比對的質心
									}
									else if (cluster_count == 2)
									{
										muti_cen.push_back(curr_cen);
										muti_cen.push_back(compair_cen);
									}
									isGroup[m][n] = true;
								}
							}
							if (cluster_count <= 1)
							{
								if (muti_cen_flag == 1)
								{
									curr_cen = muti_cen[0];
								}
								else if (muti_cen_flag == 2)
								{
									curr_cen = muti_cen[1];
								}
								muti_cen.clear();
								muti_cen_flag = 0;
							}
						}
					}  //end for_loop n
				} //end for_loop m
				group.push_back(tempGroup);
				tempGroup.clear();
			}
		} //end for_loop j
	} //end for_loop i

	//FILE *group_obj=fopen("group_obj.pcd","w"); //輸出group並上色
	//fprintf(group_obj,"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n");
	//fprintf(group_obj,"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n",8487);
	//for(int i=0; i<group.size();i++)
	//{
	//	for(int j=0; j<group[i].size();j++)
	//	{
	//		for(int k=0; k<group[i][j].size();k++)
	//		{
	//			fprintf(group_obj,"%f %f %f %f\n",group[i][j][k].x, group[i][j][k].y, group[i][j][k].z, float(i)*4+float(i)/10);
	//		}
	//	}
	//}
	//free(isGroup);
}

void edge_detect(char file_edge_obj[123], char file_edge_txt[123])
{
	/*FILE * edge_obj=fopen(file_edge_obj,"w");
	FILE * edge_txt=fopen(file_edge_txt,"w");*/
	float edge_depth = 0.14;
	for (int i = 0; i<resort_group.size(); i++)
	{
		bool isEdge[64][300] = { false };
		vector<point> temp_edge;
		for (int j = 0; j<resort_group[i].size(); j++)
		{
			for (int k = 0; k<resort_group[i][j].size(); k++)
			{
				if (j == 0 || j == resort_group[i].size() - 1 || k == 0 || k == resort_group[i][j].size() - 1)
					temp_edge.push_back(resort_group[i][j][k]); //點在圖形四邊
				//else if(Euclid_dist(resort_group[i][j][k],resort_group[i][j][k-1])>edge_depth || Euclid_dist(resort_group[i][j][k],resort_group[i][j][k+1]))
				//	temp_edge.push_back(resort_group[i][j][k]); //點與點之間太遠
				//else if(resort_group[i][j][k].x>find_max(resort_group[i][j-1]) || resort_group[i][j][k].x<find_min(resort_group[i][j-1]) || resort_group[i][j][k].x>find_max(resort_group[i][j+1]) || resort_group[i][j][k].x<find_min(resort_group[i][j+1]))
				//	temp_edge.push_back(resort_group[i][j][k]); //點超過上下層的寬度
			}
		}
		edge_data.push_back(temp_edge);
		temp_edge.clear();
	}

	/*for(int m=0; m<edge_data.size(); m++)
	{
	fprintf(edge_obj,"object\n");
	fprintf(edge_txt,"object\n");
	for(int n=0; n<edge_data[m].size(); n++)
	{
	fprintf(edge_obj,"v %f %f %f\n",edge_data[m][n].z, edge_data[m][n].x, edge_data[m][n].y);
	fprintf(edge_txt,"%f %f %f\n",edge_data[m][n].z, edge_data[m][n].x, edge_data[m][n].y);
	}
	}
	fclose(edge_obj);
	fclose(edge_txt);*/
}

void sort()
{
	for (int i = 0; i<group.size(); i++)
	{
		vector<vector<point>> temp_cluster;
		for (int j = 0; j<group[i].size(); j++)
		{
			bool isSort[300] = { false };
			vector<point> temp_point;
			for (int k = 0; k<group[i][j].size(); k++)
			{
				point max_value;
				int max_position = -1;
				max_value.x = 99;
				for (int n = 0; n<group[i][j].size(); n++)
				{
					if (isSort[n] == false)
					{
						if (group[i][j][n].x<max_value.x)
						{
							max_value = group[i][j][n];
							max_position = n;
						}
					}
				}
				if (max_value.x != 99)
				{
					temp_point.push_back(max_value);
					isSort[max_position] = true;
				}
			}
			temp_cluster.push_back(temp_point);
		}
		resort_group.push_back(temp_cluster);
	}
}
/*
int _tmain(int argc, _TCHAR* argv[])
{
	float max_time = 0;
	float min_time = 99999;
	float mean_time = 0;
	for (int i = 2; i <= 69; i++)
	{
		char file_read[125] = "D:/devkit/karlsruhe_dataset/2011_09_28/2011_09_28_drive_0138_extract/velodyne_points/data/r_ground/";
		char file_write_obj[125] = "D:/devkit/karlsruhe_dataset/2011_09_28/2011_09_28_drive_0138_extract/velodyne_points/data/Group/group";
		char file_write_txt[125] = "D:/devkit/karlsruhe_dataset/2011_09_28/2011_09_28_drive_0138_extract/velodyne_points/data/Group/group";
		char file_edge_obj[125] = "D:/devkit/karlsruhe_dataset/2011_09_28/2011_09_28_drive_0138_extract/velodyne_points/data/Edge/edge";
		char file_edge_txt[125] = "D:/devkit/karlsruhe_dataset/2011_09_28/2011_09_28_drive_0138_extract/velodyne_points/data/Edge/edge";

		char path_i[] = { ' ', ' ', ' ', ' ' };
		char format_txt[] = ".txt";
		char format_obj[] = ".obj";
		sprintf(path_i, "%d", i);
		strcat(file_read, path_i);
		strcat(file_write_obj, path_i);
		strcat(file_write_txt, path_i);
		strcat(file_edge_obj, path_i);
		strcat(file_edge_txt, path_i);

		strcat(file_read, format_txt);
		strcat(file_write_obj, format_obj);
		strcat(file_write_txt, format_txt);
		strcat(file_edge_obj, format_obj);
		strcat(file_edge_txt, format_txt);

		read_file(file_read);

		clock_t start_time, end_time;
		start_time = clock();

		for (int j = 0; j<data.size(); j++)
		{
			clustering(data[j]);
			muti_clusters.push_back(clusters);
			clusters.clear();
		}

		centre_group();
		sort();
		edge_detect(file_edge_obj, file_edge_txt);

		end_time = clock();
		float tota_time = (float)(end_time - start_time) / CLOCKS_PER_SEC;
		if (tota_time>max_time)
			max_time = tota_time;
		if (tota_time<min_time)
			min_time = tota_time;
		mean_time += tota_time;
		//printf("time:%f\n",tota_time);

		//write_file(file_write_obj, file_write_txt);

		muti_clusters.clear();
		group.clear();
		resort_group.clear();
		data.clear();
		edge_data.clear();
	}
	mean_time = mean_time / 68;
	printf("min:%f max:%f mean:%f\n", min_time, max_time, mean_time);
	system("pause");
	return 0;
}
*/

// ----------------------------
// ----讀取kitti資料主程式-----
// ----------------------------
int kitti(){
	int fc = 0;
	int px = 0, py = 0;
	int endOffc = 10;
	string s;
	string root = "C:\\Users\\user\\Desktop\\data\\NEW000000";
	string root2 = "C:\\Users\\user\\Desktop\\data\\NEW00000";
	string fileb = ".txt";
	char sfc[2];
	int updatef = 0;
	Mat Mcluster(1000,1000,CV_8UC3);
	//--------------讀檔迴圈------------------
	for (fc = 0; fc < endOffc; fc++)
	{
		sprintf(sfc, "%d", fc);

		if (fc < endOffc)
			s = root + sfc + fileb;
		else
			s = root2 + sfc + fileb;

		open_kt(s, fc);
		VK(fc);		
		cout << "file: " << fc << " removed ground "<< endl;

		//------------------dbscan---------------
		read_file(fc);
		cout << "data size: "<<data.size() << endl;	
		

		for (int j = 0; j<data.size(); j++)
		{
			clustering(data[j]);
			muti_clusters.push_back(clusters);	
			clusters.clear();
			cout << "layer" << j << endl;
		}
		centre_group();
		
		for (int i = 0; i<group.size(); i++)
		{			
			for (int j = 0; j<group[i].size(); j++)
			{
				gpsize = gpsize + group[i][j].size();
				
			}
		}
		/*
		for (int i = 0; i<group.size(); i++)
		{
			int r = rand() % 255;
			int g = rand() % 255;
			int b = rand() % 255;
			for(int j=0; j<group[i].size();j++)
			{
				for(int k=0; k<group[i][j].size();k++)
				{
						px = (group[i][j][k].x / 100) + 400;
						py = (group[i][j][k].y / 100) + 400;
						Mcluster.at<Vec3b>(px, py)[0] = r;
						Mcluster.at<Vec3b>(px, py)[1] = g;
						Mcluster.at<Vec3b>(px, py)[2] = b;
				}
			}
		}
		*/
		//-------------check_data----------------
		
		pcl::visualization::PCLVisualizer viewer("check point cloud");
		pcl::PointCloud<pcl::PointXYZ>::Ptr check_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

		for (int i = 0; i<group.size(); i++)
		{
			int r = rand() % 255;
			int g = rand() % 255;
			int b = rand() % 255;
			for (int j = 0; j<group[i].size(); j++)
			{
				for (int k = 0; k<group[i][j].size(); k++)
				{
					pcl::PointXYZ basic_point;

					basic_point.x = group[i][j][k].x;

					basic_point.y = group[i][j][k].y;

					basic_point.z = group[i][j][k].z;

					check_cloud_ptr->points.push_back(basic_point);

					pcl::PointXYZRGBA PclPoints;

					PclPoints.x = basic_point.x;

					PclPoints.y = basic_point.y;

					PclPoints.z = basic_point.z;

					int t = 255;
					uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

					PclPoints.rgb = *reinterpret_cast<float*>(&rgb);

					point_cloud_ptr->points.push_back(PclPoints);
				}
			}
		}		
		FUN_SAVEPLY(fc);
		if (updatef == 0)
		{
		//viewer.addPointCloud(check_cloud_ptr, "cloud");
		viewer.addPointCloud(point_cloud_ptr, "cloud2");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		viewer.addCoordinateSystem(100.0);
		viewer.initCameraParameters();
		updatef = 1;
		}
		//viewer.updatePointCloud(check_cloud_ptr, "cloud");
		while (1)
		{
			viewer.updatePointCloud(point_cloud_ptr, "cloud2");
			viewer.spinOnce(100);
		}
		

		
		
		imshow("PointProjectXY", Mcluster);
		cvWaitKey(1);
		system("pause");
		//sort();
		

		//--------------清除vector-------------
		muti_clusters.clear();
	    group.clear();
		resort_group.clear();
		data.clear();
		edge_data.clear();
		
	}

	pcl::visualization::PCLVisualizer viewer("point cloud");
	//--------------播放PointCloudViewer和2dImages------------------
	for (fc = 0; fc < endOffc; fc++)
	{
		if (fc == endOffc-1)
			fc = 0;
		Mat projection(1000, 1000, CV_8U);

		pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		uint8_t r(0), g(0), b(0);


		for (int j = 0; j < 2084; j++)
		{
			for (int i = 0; i < 64; i++)
			{
				pcl::PointXYZ basic_point;

				basic_point.x = readedpoints[fc][(64 * j + i) * 3];

				basic_point.y = readedpoints[fc][(64 * j + i) * 3 + 2];

				basic_point.z = readedpoints[fc][(64 * j + i) * 3 + 1];

				basic_cloud_ptr->points.push_back(basic_point);

				pcl::PointXYZRGBA PclPoints;

				PclPoints.x = basic_point.x;
				PclPoints.y = basic_point.y;
				PclPoints.z = basic_point.z;

				int t = 255;
				uint32_t rgb = (static_cast<uint32_t>(t) << 16 | static_cast<uint32_t>(t) << 8 | static_cast<uint32_t>(t));

				//rgb = (static_cast<uint32_t>(t) << 16 | static_cast<uint32_t>(t) << 8 | static_cast<uint32_t>(t));

				switch (pm2[fc][(64 * j + i)])
				{
				case 0:
					rgb = (static_cast<uint32_t>(t) << 16 | static_cast<uint32_t>(t) << 8 | static_cast<uint32_t>(t));
					break;
				case 1:
					rgb = (static_cast<uint32_t>(t) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));				
					break;
				case 2:
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(t) << 8 | static_cast<uint32_t>(b));
					px = (readedpoints[fc][(64 * j + i) * 3] / 100) + 400;
					py = (readedpoints[fc][(64 * j + i) * 3 + 2] / 100) + 400;
					projection.at<uchar>(px, py) = 0;
					break;
				}
				PclPoints.rgb = *reinterpret_cast<float*>(&rgb);

				point_cloud_ptr->points.push_back(PclPoints);
			}
		}

		basic_cloud_ptr->width = 100;
		basic_cloud_ptr->height = 100;
		point_cloud_ptr->width = 100;
		point_cloud_ptr->height = 100;

		//viewer.addPointCloud(basic_cloud_ptr, "cloud");
		if (updatef == 0)
		{
			viewer.addPointCloud(point_cloud_ptr, "cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			viewer.addCoordinateSystem(100.0);
			viewer.initCameraParameters();
			updatef = 1;
		}
		viewer.updatePointCloud(point_cloud_ptr, "cloud");
		viewer.spinOnce(1);
		imshow("PointProjectXY", projection);
		cvWaitKey(1);
	}
	return 0;
}