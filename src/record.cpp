/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <openni2/OpenNI.h>
//#include "Viewer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>


using namespace cv;
using namespace openni;
using namespace std;

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		fprintf(stderr, " Error:Missing the parameter of video path, eg. video.oni");
		return -1;
	}

	//set saving data path
	char videopath[255];
	sprintf(videopath, ".\\Data\\%s", argv[1]);

	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream vsDepth, vsColor;

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf(" initialization:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	//����ģʽ
	VideoMode vdmode;
	vdmode.setResolution(320,240);
	vdmode.setPixelFormat(PIXEL_FORMAT_RGB888);
	vdmode.setFps(30);

	VideoMode vdmodedepth;
	vdmodedepth.setResolution(320, 240);
	vdmodedepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	vdmodedepth.setFps(30);

	rc = device.open(ANY_DEVICE);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	// Create depth stream
	rc = vsDepth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		vsDepth.setVideoMode(vdmodedepth);

		rc = vsDepth.start();

		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			vsDepth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = vsColor.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		vsColor.setVideoMode(vdmode);
		rc = vsColor.start();

		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			vsColor.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);//�����ͼ��ת������ɫͼ������ϵ��

	// create recorder
	Recorder recRecorder;
	recRecorder.create(videopath);
	recRecorder.attach(vsDepth);
	recRecorder.attach(vsColor);

	// start
	recRecorder.start();

	cv::namedWindow("color image", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("depth image", CV_WINDOW_AUTOSIZE);

	int MAXdepth = vsDepth.getMaxPixelValue();
	int numFrame = 0;

	char buffImg[255];
	memset(buffImg,0,255*sizeof(char));

	while (1)
	{
		//color
		VideoFrameRef vfcolor;
		vsColor.readFrame(&vfcolor);

		const cv::Mat mImgBGR(vfcolor.getHeight(), vfcolor.getWidth(), CV_8UC3, (void*)vfcolor.getData());
		cv::Mat rgb;
		cvtColor(mImgBGR, rgb, CV_RGB2BGR);
		imshow("color image", rgb);
		vfcolor.release();

		//depth
		VideoFrameRef vfdepth;
		vsDepth.readFrame(&vfdepth);
		const cv::Mat mImgDepth(vfdepth.getHeight(), vfdepth.getWidth(), CV_16UC1, (void*)vfdepth.getData());
	

		Mat mScaledepth;
		mImgDepth.convertTo(mScaledepth,CV_8U,255.0/MAXdepth);
		imshow("depth image", mScaledepth);
		vfdepth.release();
		//printf("numFrame��%d\n",numFrame);
		if (cv::waitKey(1)=='q')
		{
			break;
		}
		

	}

	vsColor.stop();
	vsColor.destroy();
	recRecorder.stop();
	recRecorder.destroy();
	vsDepth.stop();
	vsDepth.destroy();
	device.close();
	OpenNI::shutdown();

}
