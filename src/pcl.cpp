/*
 * pcl.cpp
 *
 *  Created on: May 15, 2018
 *      Author: orbbec
 */

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "image_acquisition.hpp"

// PCL 库

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include <string>
using namespace std;
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

const double camera_factor = 1000;
const double camera_cx = 320;
const double camera_cy = 240;
const double camera_fx = 575.8157496;
const double camera_fy = 575.8157496;

int main()
	{
		ImageAcquisition ic;
		ic.Init();

		cv::namedWindow("depth", 0);

		int i = 0;
		cv::Mat image_depth, image_rgb;
		 ic.Get_Depth_Image(0,image_depth);
		 ic.Get_RGB_Image(0,image_rgb);

		cv::imshow("depth",image_depth);

		PointCloud::Ptr cloud(new PointCloud);

		// 遍历深度图
		for (int m = 0; m < image_depth.rows; m++)
			for (int n = 0; n < image_depth.cols; n++)
				{
					// 获取深度图中(m,n)处的值
					ushort d = image_depth.ptr<ushort>(m)[n];
					// d 可能没有值，若如此，跳过此点
					if (d == 0)
						continue;
					// d 存在值，则向点云增加一个点
					PointT point;

					// 计算这个点的空间坐标
					point.z = double(d) / camera_factor;
					point.x = (n - camera_cx) * point.z / camera_fx;
					point.y = (m - camera_cy) * point.z / camera_fy;

					// 从rgb图像中获取它的颜色
					// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
					point.b = image_rgb.ptr<uchar>(m)[n * 3];
					point.g = image_rgb.ptr<uchar>(m)[n * 3 + 1];
					point.r = image_rgb.ptr<uchar>(m)[n * 3 + 2];
                  //  p.a =0;
					// 把p加入到点云中
					cloud->points.push_back(point);
				}

		cloud->height = 1;
		cloud->width = cloud->points.size();
		cout << "point cloud size = " << cloud->points.size() << endl;
		cloud->is_dense = false;
		pcl::io::savePCDFile("/home/orbbec/222.pcd", *cloud);
		// 清除数据并退出
		cloud->points.clear();
		cout << "Point cloud saved." << endl;

	    pcl::visualization::CloudViewer viewer("pcd viewer");

	    viewer.showCloud(cloud);
/*	    while (!viewer.wasStopped ())
	      {
	      }*/

		cv::waitKey(0);
		return 0;
	}
