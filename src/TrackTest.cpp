/*
 * TrackTest.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: orbbec
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include "kcftracker.hpp"
#include "openni2/OpenNI.h"

//#include <OpenNI.h>

//#pragma comment(lib, "opencv_world310d.lib")

using namespace std;
using namespace cv;
using namespace openni;

//OPENNI
openni::Device device;
openni::VideoStream vsDepth;
openni::VideoStream vsColor;
VideoMode vdmode;

VideoMode vdmodedepth;

class BoxExtractor
	{
	public:
		Rect2d extract(Mat img);
		Rect2d extract(const std::string& windowName, Mat img, bool showCrossair = true);

		struct handlerT
			{
				bool isDrawing;
				Rect2d box;
				Mat image;

				// initializer list
				handlerT()
						: isDrawing(false)
					{
					}
				;
			} params;

	private:
		static void mouseHandler(int event, int x, int y, int flags, void *param);
		void opencv_mouse_callback(int event, int x, int y, int, void *param);
	};

//int OpenAstra(const char *path)
int OpenAstra(void)
	{
		vdmode.setResolution(640, 480);
		vdmode.setPixelFormat(PIXEL_FORMAT_RGB888);
		vdmode.setFps(30);

		vdmodedepth.setResolution(640, 480);
		vdmodedepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
		vdmodedepth.setFps(30);

		const char* deviceURI = openni::ANY_DEVICE;
		openni::Status rc = openni::STATUS_OK;

		rc = openni::OpenNI::initialize();
		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

		//rc = device.open(path);
		rc = device.open(deviceURI);
		if (rc != openni::STATUS_OK)
			{
				printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
				openni::OpenNI::shutdown();
				return 1;
			}

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
				rc = vsColor.setVideoMode(vdmode);
				//深度图转换到rgb视角下，对齐
				device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
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

		if (!vsDepth.isValid() || !vsColor.isValid())
			{
				printf("SimpleViewer: No valid streams. Exiting\n");
				openni::OpenNI::shutdown();
				return 2;
			}
		return 0;

	}

void CloseAstra()
	{
		//销毁视频流
		vsDepth.stop();
		vsDepth.destroy();

		vsColor.stop();
		vsColor.destroy();

		//关闭设备
		device.close();

		OpenNI::shutdown();

	}

double calculateCOM(Mat depth, Rect roi)
	{
		Mat depthroi = depth(roi);
		//把 depthroi 大于0的点设置为 255
		Mat mask = depthroi > 0;
		Scalar meandepth = mean(depthroi, mask);
		return meandepth(0);

	}
int main()
	{

		bool HOG = true;            //使用HOG特征
		bool FIXEDWINDOW = true;
		bool MULTISCALE = false;
		bool SILENT = true;
		bool LAB = false;          //使用人脸识别的特征

		// Create KCFTracker object
		KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
		//img path
		/*	string path = "data\\rgb";

		 // Read groundtruth for the 1st frame
		 ifstream groundtruthFile;
		 const char* groundtruth = ".\\Basketball\\groundtruth_rect.txt";
		 groundtruthFile.open(groundtruth);

		 string firstLine;
		 getline(groundtruthFile, firstLine);
		 groundtruthFile.close();*/

		//获取第一帧的图片和区域roi
		Mat frame;
		Rect roi;
		roi.x = 300;
		roi.y = 100;
		roi.width = 120;
		roi.height = 300;

		//open device
//		const char *path_oni = argv[1];
//	if (OpenAstra(path_oni)!=0)
		if (OpenAstra() != 0)
			{
				printf("open device failed");
				return 0;
			}

		bool g_isRunning = true;
		bool calib = false;
		int nframe = 0;
		openni::VideoFrameRef vfdepth;
		openni::VideoFrameRef vfcolor;
		Rect result;

		while (g_isRunning)
			{
				nframe++;
				cout << "nframe =" << nframe << endl;
				/******数据**************/

				vsDepth.readFrame(&vfdepth);
				cv::Mat mImgDepth(vfdepth.getHeight(), vfdepth.getWidth(), CV_16SC1, (void*) vfdepth.getData());

				//cout<<"depth image channel is "<<mImgDepth.channels()<<endl;
               // cv::imwrite("/home/orbbec/cmake_ws/kcf_track/depth.png",mImgDepth);

				imshow("depth", mImgDepth);

				vsColor.readFrame(&vfcolor);
				cv::Mat mImgBGR(vfcolor.getHeight(), vfcolor.getWidth(), CV_8UC3, (void*) vfcolor.getData());

				cvtColor(mImgBGR, frame, CV_RGB2BGR);
				Mat cloneframe;

				frame.copyTo(cloneframe);
			//	cout << "width is  " << cloneframe.cols << "  height is " << cloneframe.rows << endl;
				double tranCOM, trackCOM;

				if (!calib)  //训练
					{
						if (roi.width == 0 || roi.height == 0)
							return 0;
						//训练第一张图片

						rectangle(cloneframe, roi, Scalar(0, 255, 255), 1, 8);
						imshow("calibration", cloneframe);
						waitKey(1);
						if (nframe > 210) //标定时给了一个固定框，人站在这个框框内进行标定
							{
								tracker.init(roi, frame);
								tranCOM = calculateCOM(mImgDepth, roi);
								calib = true;
							}
						continue;

					}
				else  //跟踪
					{
						result = tracker.update(frame);
						trackCOM = calculateCOM(mImgDepth, roi);
						cout << "trackCOM =" << trackCOM << endl;

						if (result.width > 0)
							{
								rectangle(frame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);
							}
					}

				imshow("test", frame);
				//quit on ESC button
				if (waitKey(1) == 27)
					break;
			}

		vfdepth.release();
		vfcolor.release();

		CloseAstra();
		return 0;

	}

void BoxExtractor::mouseHandler(int event, int x, int y, int flags, void *param)
	{
		BoxExtractor *self = static_cast<BoxExtractor*>(param);
		self->opencv_mouse_callback(event, x, y, flags, param);
	}

void BoxExtractor::opencv_mouse_callback(int event, int x, int y, int, void *param)
	{
		handlerT * data = (handlerT*) param;
		switch (event)
			{
			// update the selected bounding box
			case EVENT_MOUSEMOVE:
				if (data->isDrawing)
					{
						data->box.width = x - data->box.x;
						data->box.height = y - data->box.y;
					}
				break;

				// start to select the bounding box
			case EVENT_LBUTTONDOWN:
				data->isDrawing = true;
				data->box = cvRect(x, y, 0, 0);
				break;

				// cleaning up the selected bounding box
			case EVENT_LBUTTONUP:
				data->isDrawing = false;
				if (data->box.width < 0)
					{
						data->box.x += data->box.width;
						data->box.width *= -1;
					}
				if (data->box.height < 0)
					{
						data->box.y += data->box.height;
						data->box.height *= -1;
					}
				break;
			}
	}

Rect2d BoxExtractor::extract(Mat img)
	{
		return extract("Bounding Box Extractor", img);
	}

Rect2d BoxExtractor::extract(const std::string& windowName, Mat img, bool showCrossair)
	{

		int key = 0;

		// show the image and give feedback to user
		imshow(windowName, img);
		printf("Select an object to track and then press SPACE/BACKSPACE/ENTER button!\n");

		// copy the data, rectangle should be drawn in the fresh image
		params.image = img.clone();

		// select the object
		setMouseCallback(windowName, mouseHandler, (void *) &params);

		// end selection process on SPACE (32) BACKSPACE (27) or ENTER (13)
		while (!(key == 32 || key == 27 || key == 13))
			{
				// draw the selected object
				rectangle(params.image, params.box, Scalar(255, 0, 0), 2, 1);

				// draw cross air in the middle of bounding box
				if (showCrossair)
					{
						// horizontal line
						line(params.image, Point((int) params.box.x, (int) (params.box.y + params.box.height / 2)),
								Point((int) (params.box.x + params.box.width), (int) (params.box.y + params.box.height / 2)), Scalar(255, 0, 0), 2, 1);

						// vertical line
						line(params.image, Point((int) (params.box.x + params.box.width / 2), (int) params.box.y),
								Point((int) (params.box.x + params.box.width / 2), (int) (params.box.y + params.box.height)), Scalar(255, 0, 0), 2, 1);
					}

				// show the image bouding box
				imshow(windowName, params.image);

				// reset the image
				params.image = img.clone();

				//get keyboard event
				key = waitKey(1);
			}

		return params.box;
	}

