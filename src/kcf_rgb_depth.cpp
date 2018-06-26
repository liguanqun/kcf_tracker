/*
 * kcf_rgb_depth.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: orbbec
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include<unistd.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<sys/types.h>
#include<dirent.h>
#include<map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "kcftracker.hpp"

using namespace cv;
using namespace std;
void gettimeandk(string str, int & t, int & k);
bool get_groundtruth_and_k(string str, cv::Rect & r, int & k);
bool bRenewROI = true;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;

bool HOG = true;
bool FIXEDWINDOW = true;
bool MULTISCALE = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
string base_path = "/home/orbbec/cmake_ws/track/data/ValidationSet/ValidationSet/bear_front/";
ifstream in_groundtruth("/home/orbbec/cmake_ws/track/data/ValidationSet/ValidationSet/bear_front/bear_front.txt");
ifstream in_init("/home/orbbec/cmake_ws/track/data/ValidationSet/ValidationSet/bear_front/init.txt");

int main(int argc, char** argv)
	{

		/*******************************************************************/
		std::map<int, string> k_path;
		std::map<int, int> k_t;

		std::map<int, string> k_path_d;
		std::map<int, int> k_t_d;

		std::map<int, cv::Rect> k_groundth;
		/**************************读取RGB*****************************/
		DIR *dp;
		struct dirent *dirp;
		string path = base_path + "rgb/";

		if ((dp = opendir(path.c_str())) == NULL)
			{
				perror("opendir error");
				exit(1);
			}
		while ((dirp = readdir(dp)) != NULL)
			{

				if ((strcmp(dirp->d_name, ".") == 0) || (strcmp(dirp->d_name, "..") == 0))
					continue;

				// char dirname[] = base_path +"rgb/";
				char dirname[] = "/home/orbbec/cmake_ws/track/data/ValidationSet/ValidationSet/bear_front/rgb/";
				int t, k;
				gettimeandk(dirp->d_name, t, k);

				k_path[k] = strcat(dirname, dirp->d_name);
				k_t[k] = t;

			}
		/*****************************读取depth image *******************************/

		path = base_path + "depth/";

		cv::namedWindow("depth", 0);
		if ((dp = opendir(path.c_str())) == NULL)
			{
				perror("opendir error");
				exit(1);
			}

		while ((dirp = readdir(dp)) != NULL)
			{

				if ((strcmp(dirp->d_name, ".") == 0) || (strcmp(dirp->d_name, "..") == 0))
					continue;

				char dirname[] = "/home/orbbec/cmake_ws/track/data/ValidationSet/ValidationSet/bear_front/depth/";
				int t, k;
				gettimeandk(dirp->d_name, t, k);

				k_path_d[k] = strcat(dirname, dirp->d_name);
				k_t_d[k] = t;

			}
		/****************************读取groundtruth************************************/
		string line;
		int k;
		while (getline(in_groundtruth, line))
			{  //按行读取
				cv::Rect r;
				string rowdata;
				rowdata = line.c_str();

				if(!get_groundtruth_and_k(rowdata, r, k))
					{
						k=k+1;
					}
				k_groundth[k] = r;
			}

		/***************************读取初始化目标框*************************/
		cv::Rect initRect;

		while (getline(in_init, line))
			{  //按行读取
				string str = line.c_str();

				int pose = str.find(",");
				initRect.x = atoi(str.substr(0, pose).c_str());
				str.erase(0, pose + 1);

				pose = str.find(",");
				initRect.y = atoi(str.substr(0, pose).c_str());
				str.erase(0, pose + 1);

				pose = str.find(",");
				initRect.width = atoi(str.substr(0, pose).c_str());
				str.erase(0, pose + 1);

				initRect.height = atoi(str.data());

			}

		/*******************************开始跟踪*********************************/
		cv::Mat image;
		cv::Mat depth;
		Mat track_show, init_show;

		cv::Rect result;
		cv::namedWindow("RGB", 0);
		//cv::namedWindow("Track", 0);
		cv::namedWindow("depth", 0);
		cout << "k_path.size()==" << k_path.size() << "  " << k_t.size() << endl;

		for (int i = 1; i <= k_path.size(); ++i)
			{
				cout << k_path[i] << endl;
				image = cv::imread(k_path[i]);

				cout << k_path_d[i] << endl;
				depth = cv::imread(k_path_d[i]);

				result = k_groundth[i];
				cv::rectangle(image, result, cv::Scalar(255, 0, 0), 2, 8);
				cv::rectangle(depth, result, cv::Scalar(255, 0, 0), 2, 8);

				cv::imshow("RGB", image);
				cv::imshow("depth", depth);

				cv::waitKey(0);
				/*if (bRenewROI)
				 {
				 tracker.init(initRect, image);
				 bBeginKCF = true;
				 bRenewROI = false;
				 image.copyTo(init_show);
				 cv::rectangle(init_show, initRect, cv::Scalar(255, 0, 0), 2, 8);
				 cv::imshow("RGB", init_show);

				 }

				 if (bBeginKCF)
				 {
				 result = tracker.update(image);
				 image.copyTo(track_show);
				 cv::rectangle(track_show, result, cv::Scalar(0, 255, 0), 2, 8);
				 cv::imshow("Track", track_show);
				 cvWaitKey(0);
				 }
				 */
			}

		cvWaitKey(0);

		cv::destroyWindow("RGB");
		//cv::destroyWindow("Track");
		cv::destroyWindow("depth");
		return 0;
	}

void gettimeandk(string str, int & t, int & k)
	{
		string abc = str;

		abc.erase(0, 2);

		int pose = abc.find("-");
		t = atoi(abc.substr(0, pose).c_str());

		abc.erase(0, pose + 1);

		pose = abc.find(".");
		k = atoi(abc.substr(0, pose).c_str());

	}
bool get_groundtruth_and_k(string str, cv::Rect & r, int & k)
	{
		if(str.substr(0,3)=="NaN")
			{
				r.x=0;r.y=0;r.height=1;r.width=1;
				k=0;
				cout<<"no target found"<<endl;
				return false;
			}
		int pose = str.find(",");
		r.x = atoi(str.substr(0, pose).c_str());
		str.erase(0, pose + 1);

		pose = str.find(",");
		r.y = atoi(str.substr(0, pose).c_str());
		str.erase(0, pose + 1);

		pose = str.find(",");
		r.width = atoi(str.substr(0, pose).c_str());
		str.erase(0, pose + 1);

		pose = str.find(",");
		r.height = atoi(str.substr(0, pose).c_str());
		str.erase(0, pose + 1);

		k = atoi(str.data());
		cout<<"frame =="<<k<<endl;
		return true;
	}

