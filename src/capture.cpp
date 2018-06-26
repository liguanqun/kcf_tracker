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
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color;

	//¶šÒåÄ£Êœ
	VideoMode vdmode;
	vdmode.setResolution(320, 240);
	vdmode.setPixelFormat(PIXEL_FORMAT_RGB888);
	vdmode.setFps(30);

	VideoMode vdmodedepth;
	vdmodedepth.setResolution(320, 240);
	vdmodedepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	vdmodedepth.setFps(30);

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf(" initialization:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	const char* deviceURI = openni::ANY_DEVICE;
	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		depth.setVideoMode(vdmodedepth);
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}


	rc = color.create(device, openni::SENSOR_COLOR);

	if (rc == openni::STATUS_OK)
	{
#if 0
		//»ñÈ¡color ËùÖ§³ÖµÄÄ£Êœ
		const SensorInfo & rinfo = color.getSensorInfo();
		const Array<VideoMode> &amodes = rinfo.getSupportedVideoModes();
		for (size_t i = 0; i < amodes.getSize(); i++)
		{
			const VideoMode &rmode = amodes[i];
			cout << "video mode" << rmode.getResolutionX();
			cout << "*" << rmode.getResolutionY();
			cout << "format:" << rmode.getPixelFormat();
			cout << "  @" << rmode.getFps()<<endl;
		}
#endif
		//œ«ÉÏÊö¶šÒåµÄÄ£ÊœÉè¶šÓÚvideostream colorÉÏ
		rc=color.setVideoMode(vdmode);

		device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);//ÉèÖÃÉî¶ÈºÍ²ÊÉ«ÎªÍ¬Ò»ÊÓÒ°

		rc = color.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			color.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}
	//VideoMode mode = color.getVideoMode();

	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}
	cv::namedWindow("color image", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("depth image", CV_WINDOW_AUTOSIZE);

	int MAXdepth = depth.getMaxPixelValue();
	int numFrame = 0;

	char buffImg[255];
	memset(buffImg,0,255*sizeof(char));

	while (1)
	{
		numFrame++;
		VideoFrameRef vfcolor;
		color.readFrame(&vfcolor);

		const cv::Mat mImgBGR(vfcolor.getHeight(), vfcolor.getWidth(), CV_8UC3, (void*)vfcolor.getData());
		cv::Mat rgb;
		cvtColor(mImgBGR,rgb,CV_RGB2BGR);

		//±£ŽæºÍÏÔÊŸ
		sprintf(buffImg, ".\\rgb\\%06d.png", numFrame);
		imwrite(buffImg, rgb);
		imshow("color image", rgb);
		vfcolor.release();


		//depth
		VideoFrameRef vfdepth;
		depth.readFrame(&vfdepth);
		const cv::Mat mImgDepth(vfdepth.getHeight(), vfdepth.getWidth(), CV_16UC1, (void*)vfdepth.getData());
		//printf("depth h:%d\n", vfdepth.getHeight());

		sprintf(buffImg, ".\\depth\\%06d.png", numFrame);
		imwrite(buffImg, mImgDepth);

		Mat mScaledepth;
		mImgDepth.convertTo(mScaledepth,CV_8U,255.0/MAXdepth);
		imshow("depth image", mScaledepth);
		vfdepth.release();
		//printf("numFrame£º%d\n",numFrame);
		if (cv::waitKey(1)=='q')
		{
			break;
		}


	}

	//color.stop();
	color.destroy();
	//depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

}
