#include "image_acquisition.hpp"
#include <iostream>
#include <ctype.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <fstream>
#include <stdio.h>
ImageAcquisition::ImageAcquisition()
		: _path("/media/orbbec/7024AED824AEA118/EvaluationSet/cc_occ1")
	{
//遮挡物特别大，遮挡之后的恢复问题
//cc_occ1  express1_occ express2_occ  new_ex_occ1 new_ex_occ3  rose1.2"
		_rgb_k = 1;
		_depth_k = 1;
		//  /home/orbbec/Downloads/data/ValidationSet/child_no1/
		//zcup_move_1  bear_front child_no1 new_ex_occ4 face_occ5

		//  /media/orbbec/7024AED824AEA118/EvaluationSet/bag1/
		// bag1 basketball1 basketball2(效果不好) basketball2.2 basketballnew bdog_occ2
		//bear_back bear_change bird1.1_no  bird3.1_no book_move1 book_turn book_turn2
		//box_no_occ  br_occ1 br_occ_0 br_occ_turn0 cafe_occ1 cc_occ1 cf_difficult cf_no_occ
		//cf_occ2 cf_occ3 computerbar1 computerBar2 cup_book dog_no_1 dog_occ_2 dog_occ_3
		//express1_occ express2_occ express3_static_occ face_move1 face_occ2  face_occ3
		//face_turn2 flower_red_occ gre_book  hand_no_occ hand_occ  library2.1_occ library2.2_occ
		//mouse_no1 new_ex_no_occ new_ex_occ1 new_ex_occ2 new_ex_occ3 new_ex_occ5_long new_ex_occ6 new_ex_occ7.1
		//new_student_center1 new_student_center2 new_student_center3 new_student_center4  new_student_center_no_occ
		//new_ye_no_occ new_ye_occ one_book_move rose1.2 static_sign1 studentcenter2.1 studentcenter3.1 studentcenter3.2
		//three_people toy_car_no toy_car_occ toy_green_occ toy_mo_occ toy_no   toy_no_occ toy_wg_no_occ toy_wg_occ toy_wg_occ1
		//toy_yellow_no tracking4 tracking7.1 two_book two_dog_occ1 two_people_1.1 two_people_1.2 two_people_1.3
		//walking_no_occ walking_occ1 walking_occ_long  wdog_no1 wdog_occ3 wr_no  wr_no1    wr_occ2 wuguiTwo_no zball_no1 zball_no2 zball_no2
		//zballpat_no1
		_RGB_path = _path + "/rgb/";
		_DEPTH_path = _path + "/depth/";
		//_RGB_path = "/home/orbbec/Downloads/data/ValidationSet/child_no1/rgb/";
		//_DEPTH_path = "/home/orbbec/Downloads/data/ValidationSet/child_no1/depth/";
	}
void ImageAcquisition::Init()
	{
		/*******************读RGB图*************************/
		int t, k;
		DIR *dp;
		struct dirent *dirp;
		if ((dp = opendir(_RGB_path.c_str())) == NULL)
			{
				perror("opendir error");
				exit(1);
			}

		while ((dirp = readdir(dp)) != NULL)
			{

				if ((strcmp(dirp->d_name, ".") == 0) || (strcmp(dirp->d_name, "..") == 0))
					continue;

				char dirname[100];
				std::string tmp = _path + "/rgb/";
				tmp.copy(dirname, tmp.size(), 0);
				dirname[tmp.size()] ='\0';


				Get_Time_And_K(dirp->d_name, t, k);
				_k_path[k] = strcat(dirname, dirp->d_name);

				_k_t[k] = t;
				//_k_path.insert(std::make_pair<int,std::string>(k, strcat(dirname, dirp->d_name)));
				//_k_t.insert(std::make_pair<int,int>(k,t));

			}

		/**************************读深度图*******************************/

		DIR *dp_d;
		struct dirent *dirp_d;
		if ((dp_d = opendir(_DEPTH_path.c_str())) == NULL)
			{
				perror("opendir error");
				exit(1);
			}
		while ((dirp_d = readdir(dp_d)) != NULL)
			{

				if ((strcmp(dirp_d->d_name, ".") == 0) || (strcmp(dirp_d->d_name, "..") == 0))
					continue;

				char  dirname[100];
				std::string tmp = _path + "/depth/";
				tmp.copy(dirname, tmp.size(), 0);
				dirname[tmp.size()] = '\0';


				Get_Time_And_K(dirp_d->d_name, t, k);

				_k_path_d[k] = strcat(dirname, dirp_d->d_name);
				_k_t_d[k] = t;
				//_k_path_d.insert(std::make_pair<int,std::string>(k, strcat(dirname, dirp_d->d_name)));
				//_k_t_d.insert(std::make_pair<int,int>(k,t));

			}

		_size = std::min(_k_path_d.size(), _k_path.size());
		std::cout << "total image is " << _size << std::endl;

		/*         char k;
		 std::cin>>k;*/
	}
bool ImageAcquisition::Get_RGB_Image(int k, cv::Mat& image)
	{

		if (k == 0)
			{
				image = cv::imread(_k_path[1]);
				double time_diff = ((double) (_k_t_d[1] - _k_t[1])) / 33333;
				std::cout << "the " << _rgb_k << " frame RGB and depth time diff is " << time_diff << std::endl;

				return true;
			}

		else if (k == 1)
			{
				_rgb_k += 1;
				if (_rgb_k < _size)
					{
						image = cv::imread(_k_path[_rgb_k]);

						double time_diff = ((double) (_k_t_d[_rgb_k] - _k_t[_rgb_k])) / 33333;
						std::cout << "the " << _rgb_k << " frame RGB and depth time diff is " << time_diff << std::endl;

						return true;
					}
				else
					return false;
			}
		else
			return false;
	}
bool ImageAcquisition::Get_Depth_Image(int k, cv::Mat& image)
	{

		if (k == 0)
			{
				image = cv::imread(_k_path_d[1], CV_LOAD_IMAGE_ANYDEPTH);
				image = Shift_Bit_Depth_Image(image);
//              image =image/8;
				/*				cv::Mat image_1 = image.clone();
				 cv::Rect _boundingBox = Get_Init_Rect();
				 cv::rectangle(image_1, _boundingBox.tl(), _boundingBox.br(), cv::Scalar(65536));
				 cv::imshow("depth", image_1);*/

				/*				 cv::Mat image_1(image.rows, image.cols, image.type(), cv::Scalar::all(8));
				 image_1 = image_1.mul(image);*/

				/*
				 cv::Mat image_1 = image.clone();
				 cv::Rect _boundingBox = Get_Init_Rect();
				 cv::rectangle(image_1, _boundingBox.tl(), _boundingBox.br(), cv::Scalar(255));
				 cv::imshow("image_000", image_1);
				 */

				//cv::imshow("depth_8u", image_8u);
				/*
				 char name[100];
				 sprintf(name, "/home/orbbec/Downloads/data/ValidationSet/bear_front/depth_b/%d.png", k);
				 cv::imwrite(name, image);
				 */

				return true;
			}
		else if (k == 1)
			{
				_depth_k += 1;
				if (_depth_k < _size)
					{

						image = cv::imread(_k_path_d[_depth_k], CV_LOAD_IMAGE_ANYDEPTH);
						image = Shift_Bit_Depth_Image(image);
						//image =image/8;
						/*						cv::Mat image_1 = image.clone();
						 cv::Rect _boundingBox = Get_Init_Rect();
						 cv::rectangle(image_1, _boundingBox.tl(), _boundingBox.br(), cv::Scalar(255));
						 cv::imshow("depth", image_1);*/

						/*						cv::Mat image_1 = image.clone();
						 cv::Rect _boundingBox = Get_Init_Rect();
						 cv::rectangle(image_1, _boundingBox.tl(), _boundingBox.br(), cv::Scalar(255));
						 cv::imshow("image_000", image_1);*/

						//image = Shift_Bit_Depth_Image(image_);
						//image = Shift_Bit_Depth_Image(cv::imread(_k_path_d[_depth_k]));
						//cv::imshow("depth", image);
						//保存图片
						/*					char name[100];
						 sprintf(name, "/home/orbbec/Downloads/data/ValidationSet/bear_front/depth_b/%d.png", _depth_k);

						 cv::imwrite(name, image);*/

						return true;
					}
				else
					{
						std::cout << "there is no depth image " << std::endl;
						return false;
					}
			}
		else
			return false;
	}
void ImageAcquisition::Get_Time_And_K(std::string str, int & t, int & k)
	{
		std::string abc = str;

		abc.erase(0, 2);

		int pose = abc.find("-");
		t = atoi(abc.substr(0, pose).c_str());

		abc.erase(0, pose + 1);

		pose = abc.find(".");
		k = atoi(abc.substr(0, pose).c_str());

	}
cv::Mat ImageAcquisition::Shift_Bit_Depth_Image(cv::Mat& image)
	{

		/*		cv::Mat image_show;
		 image.copyTo(image_show);
		 //cv::rectangle(image_show, Get_Init_Rect, cv::Scalar(255), 2);
		 //cv::rectangle(image_show, Get_Init_Rect,cv::Scalar(255,0,0));
		 cv::Rect rect = Get_Init_Rect();
		 cv::rectangle(image_show,rect.tl(),rect.br(),cv::Scalar(255));
		 cv::imshow("depth_rectangle",image_show);*/
		cv::Mat image_2(image.rows, image.cols, image.type(), cv::Scalar(0));

		//image.copyTo(image_2);
		image_2 = image.clone();
		/*		image.copyTo(image_2);
		 int nl = image.rows;
		 int nc = image.cols;

		 for (int j = 0; j < nl; j++)
		 {
		 //unsigned short *pt = image_2.ptr<unsigned short>(j);

		 unsigned short *pt = image.ptr<unsigned short>(j);
		 unsigned short *ptt = image_2.ptr<unsigned short>(j);
		 //unsigned short temp;
		 for (int i = 0; i < nc; i++)
		 {
		 //对像素进行某些操作
		 ptt[i] = (unsigned short) (pt[i] / 8);
		 }
		 }*/
		for (int i = 0; i < image.rows; i++)
			{
				for (int j = 0; j < image.cols; j++)
					{

						unsigned short temp = image.at<unsigned short>(i, j);
						//std::cout<<temp<<" ";
						temp = temp / 8 + (0xffff & (temp << 13));
						//temp = temp /8 ;
						image_2.at<unsigned short>(i, j) = temp;
					}
			}
		std::cout << std::endl;

		return image_2;
	}
cv::Rect ImageAcquisition::Get_Init_Rect(void)
	{
		using namespace std;
		cv::Rect r;
		std::string path = _RGB_path;
		path = path.substr(0, path.find_last_of('/'));
		path = path.substr(0, path.find_last_of('/') + 1);
		path = path + "init.txt";

		//	std::cout << "path == " << path << std::endl;
		ifstream myfile(path.c_str());
		if (!myfile.is_open())
			{
				cout << "can not open the init file" << endl;
			}
		//std::cout << "read finish" << std::endl;

		string temp;
		getline(myfile, temp);
		myfile.close();   //关闭文件

		//std::cout << temp << std::endl;

		//std::cout<<temp.substr(0,temp.find_first_of(',')).c_str()<<std::endl;

		r.x = atoi(temp.substr(0, temp.find_first_of(',')).c_str());
		temp.erase(0, temp.find_first_of(',') + 1);

		r.y = atoi(temp.substr(0, temp.find_first_of(',')).c_str());
		temp.erase(0, temp.find_first_of(',') + 1);

		r.width = atoi(temp.substr(0, temp.find_first_of(',')).c_str());
		temp.erase(0, temp.find_first_of(',') + 1);

		r.height = atoi(temp.c_str());

		//std::cout << "r.x y width height " << r.x << "*" << r.y << "*" << r.width << "*" << r.height << std::endl;

		return r;
	}

/*cv::Rect ImageAcquisition::Get_Groundtruth(int k)
 {

 cv::Rect r;
 return r;
 }*/
ImageAcquisition::~ImageAcquisition()
	{

	}
