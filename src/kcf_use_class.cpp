/*
 * kcf_use_class.cpp
 *
 *  Created on: Apr 12, 2018
 *      Author: orbbec
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "kcftracker.hpp"
#include <iostream>
#include "image_acquisition.hpp"

int main()
	{
		ImageAcquisition ic;
		ic.Init();

		cv::namedWindow("depth", 0);
		cv::namedWindow("image", 0);

		cv::Mat rgb, depth;
		 ic.Get_RGB_Image(0,rgb);
		 ic.Get_Depth_Image(0,depth);


		cv::imshow("image", rgb);
		cv::imshow("depth", depth);
		cv::waitKey(33);
		while (1)
			{
				 ic.Get_RGB_Image(1,rgb);
				 ic.Get_Depth_Image(1,depth);

				cv::imshow("image", rgb);
				cv::imshow("depth", depth);
				cv::waitKey(33);
			}

		return 0;
	}

