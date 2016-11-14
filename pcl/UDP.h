#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <core.hpp>

using namespace cv;

float UDP(float PC_matrix[16][1800],float &AZ);

cv::Mat twoDimg(float PC_matrix[16][1800], Mat img);