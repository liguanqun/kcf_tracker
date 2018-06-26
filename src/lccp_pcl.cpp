/*
 * lccp_pcl.cpp
 *
 *  Created on: May 16, 2018
 *      Author: orbbec
 */

#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <string>
using namespace std;
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

float voxel_resolution = 0.008f;
float seed_resolution = 0.1f;
float color_importance = 0.2f;
float spatial_importance = 0.4f;
float normal_importance = 1.0f;

int main()
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // 创建点云（指针）

		//if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>("/home/orbbec/222.pcd", *cloud) == -1)
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("/home/orbbec/222.pcd", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
			{
				PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
				return (-1);
			}

		//一个面显示
		pcl::visualization::CloudViewer viewer("Viewer");
		viewer.showCloud(cloud);

		pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);


		  //和点云形式有关
/*		  if (cloud->disable_transform)
		    super.setUseSingleCameraTransform (false);*/
		  //输入点云及结晶参数
		  super.setInputCloud (cloud);
		  super.setColorImportance (color_importance);
		  super.setSpatialImportance (spatial_importance);
		  super.setNormalImportance (normal_importance);
		  //输出结晶分割结果：结果是一个映射表
		  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
		  super.extract (supervoxel_clusters);

		  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
		  super.getSupervoxelAdjacency (supervoxel_adjacency);







		return 0;

	}

