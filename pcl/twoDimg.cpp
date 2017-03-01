/*
/////////////////////////////////////////////////
ver.0.05版
-註解掉轉成RGB色彩方法
/////////////////////////////////////////////////
*/

#include "UDP.h"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <core.hpp>

using namespace cv;

cv::Mat twoDimg(float matrix[16][1800], Mat img)
{
	
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < 1800; j++)
		{
			int t = 0;			
			if (matrix[i][j] < 10)
			{
				t = matrix[i][j] * 25.5;
				if (t>255)t = 255;		
				img.at<uchar>(i, j)= 255-t;
				//img.at<Vec3b>(i  , j)[0] = 15+t;
				//img.at<Vec3b>(i  , j)[1] = 15+t;
				//img.at<Vec3b>(i  , j)[2] = 15+t;		

			}
			else
			{
				img.at<uchar>(i, j) = 0;
			}
			
			/*else if (matrix[i][j] >= 15 && matrix[i][j] < 20)
			{
				t = (matrix[i][j]-15) * 25.5 ;
				if (t>255)t = 255;
				img.at<Vec3b>(i, j)[0] = t;
				img.at<Vec3b>(i, j)[1] = t;
				img.at<Vec3b>(i, j)[2] = t;
				
			}
			else if (matrix[i][j] >= 10 && matrix[i][j] < 15)
			{
				t = (matrix[i][j] - 10) * 51;
				if (t>255)t = 255;
				img.at<Vec3b>(i, j)[0] = 0;
				img.at<Vec3b>(i, j)[1] = 255-t;
				img.at<Vec3b>(i, j)[2] = 255;
			}
			else if (matrix[i][j] >= 15 && matrix[i][j] < 20)
			{
				t = (matrix[i][j] - 15) * 51;
				if (t>255)t = 255;
				img.at<Vec3b>(i, j)[0] = 0;
				img.at<Vec3b>(i, j)[1] = 0;
				img.at<Vec3b>(i, j)[2] = 255-t;
			}
			else if (matrix[i][j] >= 20 && matrix[i][j] < 25)
			{
				t = (matrix[i][j] - 20) * 51;
				if (t>255)t = 255;
				img.at<Vec3b>(i, j)[0] = t;
				img.at<Vec3b>(i, j)[1] = t;
				img.at<Vec3b>(i, j)[2] = t;
			}
			else
			{
				t = (matrix[i][j]-20) * 3;
				if (t>255)t = 255;
				img.at<uchar>(i, j) = 255-t;
			//	img.at<Vec3b>(i  , j)[0] = 255;
			//	img.at<Vec3b>(i  , j)[1] = 0;
			//	img.at<Vec3b>(i  , j)[2] = 255;
			}*/
		}
	}

	//imshow("PointTO2dimage", img);
	//cvWaitKey(1);	
	return img;
}


