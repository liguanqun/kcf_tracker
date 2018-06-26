/*
 * kcf_use_benchmark.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: orbbec
 */

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "kcftracker.hpp"
#include "image_acquisition.hpp"

using namespace cv;
using namespace std;
void gettimeandk(string str, int & t, int & k);

bool bRenewROI = true;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;

bool HOG = true;
bool FIXEDWINDOW = true;
bool MULTISCALE = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

int main()
	{
		ImageAcquisition ic;
		ic.Init();

		//	cv::namedWindow("RGB", 0);
		//	cv::moveWindow("RGB", 100, 200);
		cv::namedWindow("Track", 0);
		cv::moveWindow("Track", 100, 20);

		cv::namedWindow("res_for_show", 0);
		cv::moveWindow("res_for_show", 100, 500);

		cv::namedWindow("f(z)-f(x)", 0);
		cv::moveWindow("f(z)-f(x)", 500, 20);

		cv::namedWindow("hist_picture", 0);
		cv::moveWindow("hist_picture", 1000, 20);

		cv::namedWindow("occlusion_r", 0);
		cv::moveWindow("occlusion_r", 1400, 20);

		std::map<int, string> k_path;
		std::map<int, int> k_t;
		cv::Mat image;
		Mat track_show, init_show;
		Mat occlusion_plot(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
		;
		cv::Rect selectRect;
		cv::Rect result;
		selectRect.x = 186;
		selectRect.y = 176;
		selectRect.width = 104;
		selectRect.height = 144;
		//180	79	37	114
		/*		selectRect.x = 180;
		 selectRect.y = 79;
		 selectRect.width = 37;
		 selectRect.height = 114;*/

		/*		for (int i = 1; i < 308; i++)
		 {

		 char dirname[] = "/home/orbbec/Jogging/img/0";
		 char frame[4];
		 if(i<10) strcat(dirname, "00");
		 else if(i<100)strcat(dirname, "0");
		 sprintf(frame, "%d", i);
		 k_path[i] = strcat(dirname, frame);
		 k_path[i] = strcat(dirname, ".jpg");
		 //	cout << k_path[i] << endl;
		 }*/
		//cout << "k_path.size()==" << k_path.size() << "  " << k_t.size() << endl;
		cout << "ROI width height == " << selectRect.width << "*" << selectRect.height << endl;

		for (int i = 1; i < 250; i++)
			{

				 ic.Get_RGB_Image(1,image);


				cout << "  image.cols==" << image.cols << "   image.rows==" << image.rows << endl;

				if (bRenewROI)
					{
						image.copyTo(init_show);
						cv::rectangle(init_show, selectRect, cv::Scalar(255, 0, 0), 2, 8);
						cv::imshow("RGB", init_show);
						tracker.init(selectRect, image);
						bBeginKCF = true;
						bRenewROI = false;

					}

				else if (bBeginKCF)
					{
						result = tracker.update(image);
						image.copyTo(track_show);
						cv::rectangle(track_show, result, cv::Scalar(0, 255, 0), 2, 8);
						cv::imshow("Track", track_show);

						float r = 100 * (log(tracker.occlusion_r) / log(10) + 5);
						cv::Point p1 = cv::Point(i * 3, occlusion_plot.rows);
						cv::Point p2 = cv::Point(i * 3, occlusion_plot.rows - r);
						cv::line(occlusion_plot, p1, p2, cv::Scalar(0, 0, 255), 3);

						cv::imshow("occlusion_r", occlusion_plot);
						cvWaitKey(0);
					}
			}
		cvWaitKey(0);

		//	cv::destroyWindow("RGB");
		cv::destroyWindow("Track");
		return 0;
	}
