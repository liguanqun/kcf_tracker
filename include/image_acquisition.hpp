
#ifndef IMAGE_ACQUISITION_HPP_
#define IMAGE_ACQUISITION_HPP_

#include "opencv2/highgui/highgui.hpp"
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

class ImageAcquisition
	{
	public:
		ImageAcquisition();
		virtual ~ImageAcquisition();
		void Init();
		bool Get_Depth_Image(int k,cv::Mat& image);
		bool Get_RGB_Image(int k, cv::Mat& image);
		void Get_Time_And_K(std::string str, int & t, int & k);
		cv::Mat Shift_Bit_Depth_Image(cv::Mat& image);
        cv::Rect Get_Init_Rect(void);
       // cv::Rect ImageAcquisition::Get_Groundtruth(int k);
	private:
        const std::string _path;
		std::string _RGB_path ;
		std::string _DEPTH_path ;
		std::map<int, std::string> _k_path, _k_path_d;
		std::map<int, int> _k_t, _k_t_d;
		int _size, _rgb_k, _depth_k;

	};

#endif
