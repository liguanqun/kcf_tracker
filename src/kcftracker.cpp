/*

 Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure with Kernels (CSK) [2].
 CSK is implemented by using raw gray level features, since it is a single-channel filter.
 KCF is implemented by using HOG features (the default), since it extends CSK to multiple channels.

 [1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
 "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

 [2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
 "Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.

 Authors: Joao Faro, Christian Bailer, Joao F. Henriques
 Contacts: joaopfaro@gmail.com, Christian.Bailer@dfki.de, henriques@isr.uc.pt
 Institute of Systems and Robotics - University of Coimbra / Department Augmented Vision DFKI


 Constructor parameters, all boolean:
 hog: use HOG features (default), otherwise use raw pixels　用HOG特征点还是使用原像素
 fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
 multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)
 //如果使用固定窗口（fixed_window = true）的话就不能使用多尺度multiscale
 Default values are set for all properties of the tracker depending on the above choices.
 Their values can be customized further before calling init():
 interp_factor: linear interpolation factor for adaptation
 sigma: gaussian kernel bandwidth
 lambda: regularization  归一化
 cell_size: HOG cell size
 padding: area surrounding the target, relative to its size
 output_sigma_factor: bandwidth of gaussian target
 template_size: template size in pixels, 0 to use ROI size
 scale_step: scale step for multi-scale estimation, 1 to disable it
 scale_weight: to downweight detection scores of other scales for added stability
 为了速度   template_size/cell_size 应该是2的倍数
 For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

 Inputs to init():
 image is the initial frame.
 roi is a cv::Rect with the target positions in the initial frame

 Inputs to update():
 image is the current frame.

 Outputs of update():
 cv::Rect with target positions for the current frame

 版权声明
 By downloading, copying, installing or using the software you agree to this license.
 If you do not agree to this license, do not download, install,
 copy or use the software.


 License Agreement
 For Open Source Computer Vision Library
 (3-clause BSD License)

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the names of the copyright holders nor the names of the contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 This software is provided by the copyright holders and contributors "as is" and
 any express or implied warranties, including, but not limited to, the implied
 warranties of merchantability and fitness for a particular purpose are disclaimed.
 In no event shall copyright holders or contributors be liable for any direct,
 indirect, incidental, special, exemplary, or consequential damages
 (including, but not limited to, procurement of substitute goods or services;
 loss of use, data, or profits; or business interruption) however caused
 and on any theory of liability, whether in contract, strict liability,
 or tort (including negligence or otherwise) arising in any way out of
 the use of this software, even if advised of the possibility of such damage.
 */
#include <iostream>
#ifndef _KCFTRACKER_HEADERS
#include "kcftracker.hpp"
#include "ffttools.hpp"
#include "recttools.hpp"
#include "fhog.hpp"
#include "labdata.hpp"

#include "math.h"

#endif
#include "math.h"
#include "vector"
#include <algorithm>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
// Constructor
KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab)
	{

		// Parameters equal in all cases
		//所有情况下都适用的参数
		//lambda = 0.01;  xwx
		lambda = 0.0001;
		padding = 2.5;
		//output_sigma_factor = 0.2; xwx
		output_sigma_factor = 0.125;
		//遮挡标志位
		occlusion_flag = false;

		if (hog)
			{
				//interp_factor = 0.025; xwx
				interp_factor = 0.012;
				sigma = 0.6;
				cell_size = 4;
				_hogfeatures = true;

				if (lab)
					{
						interp_factor = 0.005;
						sigma = 0.4;
						output_sigma_factor = 0.025;

						_labfeatures = true;
						_labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
						cell_sizeQ = cell_size * cell_size;
					}
				else
					{
						_labfeatures = false;
					}
			}
		else
			{   // RAW 灰度图
				interp_factor = 0.075;
				sigma = 0.2;
				cell_size = 1;
				_hogfeatures = false;

				if (lab)
					{
						printf("Lab features are only used with HOG features.\n");
						_labfeatures = false;
					}
			}

		if (multiscale)
			{ // multiscale
				//template_size = 1;
				template_size = 192;
				scale_step = 1.05;
				scale_weight = 0.95;
				if (!fixed_window)
					{
						std::cout << "Multiscale does not support non-fixed window." << std::endl;
						fixed_window = true;
					}
			}
		else if (fixed_window)
			{  // fit correction without multiscale
				template_size = 1;
				//template_size = 100;
				scale_step = 1;
			}
		else
			{
				template_size = 1;
				scale_step = 1;
			}
	}

// Initialize tracker 
void KCFTracker::init(const cv::Rect &roi, cv::Mat image)
	{
		_roi = roi;
		assert(roi.width >= 0 && roi.height >= 0);
		_tmpl = getFeatures(image, 1);
		_prob = createGaussianPeak(size_patch[0], size_patch[1]);  // 最小二乘 的y
		_alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
		//_num = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
		//_den = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
		train(_tmpl, 1.0); // train with initial frame
	}
// Update position based on the new frame
cv::Rect KCFTracker::update(cv::Mat image)
	{
		if (_roi.x + _roi.width <= 0)
			_roi.x = -_roi.width + 1;
		if (_roi.y + _roi.height <= 0)
			_roi.y = -_roi.height + 1;
		if (_roi.x >= image.cols - 1)
			_roi.x = image.cols - 2;
		if (_roi.y >= image.rows - 1)
			_roi.y = image.rows - 2;

		float cx = _roi.x + _roi.width / 2.0f;
		float cy = _roi.y + _roi.height / 2.0f;

		occlusion_flag = false;

		float peak_value;
		//cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value);
		cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value, true);
		if (occlusion_flag)
			return _roi;
		if (scale_step != 1) //意味着使用了multiscale
			{

				cout << "multiscale is entering" << endl;
				// Test at a smaller _scale
				float new_peak_value;
				cv::Point2f new_res = detect(_tmpl, getFeatures(image, 0, 1.0f / scale_step), new_peak_value, false);

				if (scale_weight * new_peak_value > peak_value)
					{
						res = new_res;
						peak_value = new_peak_value;
						_scale /= scale_step;
						_roi.width /= scale_step;
						_roi.height /= scale_step;
						cout << "a small scale has a good effict" << endl;
					}

				// Test at a bigger _scale
				new_res = detect(_tmpl, getFeatures(image, 0, scale_step), new_peak_value, false);

				if (scale_weight * new_peak_value > peak_value)
					{
						res = new_res;
						peak_value = new_peak_value;
						_scale *= scale_step;
						_roi.width *= scale_step;
						_roi.height *= scale_step;
						cout << "a bigger scale has a good effict" << endl;
					}
			}

		// Adjust by cell size and _scale
		_roi.x = cx - _roi.width / 2.0f + ((float) res.x * cell_size * _scale);
		_roi.y = cy - _roi.height / 2.0f + ((float) res.y * cell_size * _scale);

		if (_roi.x >= image.cols - 1)
			_roi.x = image.cols - 1;
		if (_roi.y >= image.rows - 1)
			_roi.y = image.rows - 1;
		if (_roi.x + _roi.width <= 0)
			_roi.x = -_roi.width + 2;
		if (_roi.y + _roi.height <= 0)
			_roi.y = -_roi.height + 2;

		assert(_roi.width >= 0 && _roi.height >= 0);
		cv::Mat x = getFeatures(image, 0);
		train(x, interp_factor);

		return _roi;
	}

// Detect object in the current frame.
cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value, bool flag)
	{
		using namespace FFTTools;
		using namespace cv;
		cv::Mat k = gaussianCorrelation(x, z);
		cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true)));

	//	cout<<"res == "<<endl<<""<<res<<endl<<endl;
		cv::Mat res_for_show(600, res.cols*res.rows, CV_8UC3, cv::Scalar(255, 255, 255));

		for (size_t y = 0; y < res.rows; ++y)				//行数代表y
			{
				float* ptr = res.ptr<float>(y);
				for (size_t x = 0; x < res.cols; x++)				//列数代表x
					{
						float pix = ptr[x];
						cv::Point p1 = cv::Point(y * res.cols + x, 300);
						cv::Point p2 = cv::Point(y * res.cols + x, 300 - (int) (pix * 300));
						cv::line(res_for_show, p1, p2, cv::Scalar(255, 0, 0));
					}
			}
		cv::imshow("res_for_show", res_for_show);
	//	cv::waitKey(0);
		/********************计算三阶累计量*******************/
		if (flag)
			{


				cv::Mat xx = gaussianCorrelation(x, x);
				cv::Mat res_xx = (real(fftd(complexMultiplication(_alphaf, fftd(xx)), true)));

				cv::subtract(res_xx, res, res_xx);

				double r = 0;
				float step = 0.0015;
				float step_min = -0.15, step_max = 0.15;
				vector<int> hist(200, 0);
				cv::Mat picture(600, res_xx.cols * res_xx.rows, CV_8UC3, cv::Scalar(255, 255, 255));
				cout << res_xx.cols << " * " << res_xx.rows << endl;
				for (size_t y = 0; y < res_xx.rows; ++y)				//行数代表y
					{
                          float* ptr =res_xx.ptr<float>(y);
						for (size_t x = 0; x < res_xx.cols; x++)				//列数代表x
							{

								float pix = ptr[x];
								if (pix > step_min && pix < step_max)
									{
										hist[(int) ((pix - step_min) / step)] += 1;

									}
								cv::Point p1 = cv::Point(y * res_xx.cols + x, 300);
								cv::Point p2 = cv::Point(y * res_xx.cols + x, 300 - (int) (pix *300));
								cv::line(picture, p1, p2, cv::Scalar(255, 0, 0));
								r += pow((double) pix, 3);
								//	cout << (int) (pix * 10000) << " ";
							}
					}
				cv::Mat hist_picture(400, 200, CV_8UC3, cv::Scalar(255, 255, 255));
				for (int i = 0; i < hist.size(); i++)
					{
						cout << hist[i] << " ";
						cv::Point p1 = cv::Point(i, hist_picture.rows);
						cv::Point p2 = cv::Point(i, hist_picture.rows - hist[i]);
						cv::line(hist_picture, p1, p2, cv::Scalar(0, 0, 255));
					}
				cout << endl;

				imshow("f(z)-f(x)", picture);
				imshow("hist_picture", hist_picture);

				cv::waitKey(1);

				r = r /(res_xx.cols*res_xx.rows);
				occlusion_r = r;
				cout << "r == " << r << endl;
				if (r > 0.02)
					{

						occlusion_flag = true;
						cout << endl << endl << "occlusion " << endl << endl;
					}

			}
		/*************************************************/
//minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
		cv::Point2i pi;
		double pv;

		cv::Point2i pi_min;
		double pv_min;
		cv::minMaxLoc(res, &pv_min, &pv, &pi_min, &pi);
		peak_value = (float) pv;
		std::cout << "min reponse : " << pv_min << " max response :" << pv << std::endl;

//subpixel peak estimation, coordinates will be non-integer
		cv::Point2f p((float) pi.x, (float) pi.y);
//微调 peak 的位置
		if (pi.x > 0 && pi.x < res.cols - 1)
			{
				p.x += subPixelPeak(res.at<float>(pi.y, pi.x - 1), peak_value, res.at<float>(pi.y, pi.x + 1));
			}

		if (pi.y > 0 && pi.y < res.rows - 1)
			{
				p.y += subPixelPeak(res.at<float>(pi.y - 1, pi.x), peak_value, res.at<float>(pi.y + 1, pi.x));
			}

		p.x -= (res.cols) / 2;
		p.y -= (res.rows) / 2;

		return p;
	}
// train tracker with a single image
void KCFTracker::train(cv::Mat x, float train_interp_factor)
	{
		using namespace FFTTools;

// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y,
//which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
		cv::Mat k = gaussianCorrelation(x, x);
		cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));		//KCF 公式17

		_tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
		_alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor) * alphaf;

	}

// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts
//between input images X and Y, which must both be MxN.
//They must    also be periodic (ie., pre-processed with a cosine window).
cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2)
	{
		using namespace FFTTools;
		cv::Mat c = cv::Mat(cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0));
// HOG features
		if (_hogfeatures)
			{
				cv::Mat caux;
				cv::Mat x1aux;
				cv::Mat x2aux;
				//每个hog特征有31维，每个维度分别卷积（先变换，再相乘，再求逆），31维累加
				for (int i = 0; i < size_patch[2]; i++)
					{

						x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
						x1aux = x1aux.reshape(1, size_patch[0]);
						x2aux = x2.row(i).reshape(1, size_patch[0]);

						cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true);//求频谱
						caux = fftd(caux, true);						//求逆
						rearrange(caux);						//重排函数 分成四块，对角块互换
						caux.convertTo(caux, CV_32F);
						c = c + real(caux);
					}
			}
// Gray features
		else
			{
				cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
				c = fftd(c, true);
				rearrange(c);
				c = real(c);
			}

		cv::Mat d;
	//(    cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0]   ) - 2. * c 公式31
		cv::max(( (    cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0]   ) - 2. * c) / (size_patch[0] * size_patch[1] * size_patch[2]), 0, d);

		cv::Mat k;
		cv::exp((-d / (sigma * sigma)), k);
		return k;
	}

// Create Gaussian Peak. Function called only in the first frame.
//创建高斯峰，在第一张图片调用
cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex)
	{
		cv::Mat_<float> res(sizey, sizex);

		int syh = (sizey) / 2;
		int sxh = (sizex) / 2;

		float output_sigma = std::sqrt((float) sizex * sizey) / padding * output_sigma_factor;
		float mult = -0.5 / (output_sigma * output_sigma);

//	cout<<"GaussianPeak  "<<sizey<<"*"<<sizex<<endl;
		for (int i = 0; i < sizey; i++)
			{
				for (int j = 0; j < sizex; j++)
					{
						int ih = i - syh;
						int jh = j - sxh;
						res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
						//			cout<< res(i, j)<<" ";
					}
				//	cout<<endl;
			}
		return FFTTools::fftd(res);//公式中的y的F里叶变换  即 y帽
	}

// Obtain sub-window from image, with replication-padding and extract features
//从图像中获取子窗口，带有复制填充和提取特征
cv::Mat KCFTracker::getFeatures(const cv::Mat & image, bool inithann, float scale_adjust)
	{
		cv::Rect extracted_roi;

		float cx = _roi.x + _roi.width / 2;
		float cy = _roi.y + _roi.height / 2;

		if (inithann)   //汉明窗初始化，，第一次获取特征的时候进入
			{
				int padded_w = _roi.width * padding;   //260
				int padded_h = _roi.height * padding;   //360
				cout << "padded_w*padded_h== " << padded_w << "*" << padded_h << endl;
				//         cout<<"template_size  == "<<template_size<<endl;

				if (template_size > 1)
					{  // Fit largest dimension to the given template size
						if (padded_w >= padded_h)  //fit to width
							_scale = padded_w / (float) template_size;
						else
							_scale = padded_h / (float) template_size; //360/96=3.75

						cout << "_scale  == " << _scale << endl;
						_tmpl_sz.width = padded_w / _scale;
						_tmpl_sz.height = padded_h / _scale;
						//		cout<<"_tmpl_sz.width*_tmpl_sz.height  "<<_tmpl_sz.width<<"*"<<_tmpl_sz.height<<endl;
						//	cv::waitKey(0);
					}
				else
					{
						//不会进入
						//No template size given, use ROI size
						_tmpl_sz.width = padded_w;
						_tmpl_sz.height = padded_h;
						_scale = 1;

					}

				if (_hogfeatures)
					{
						// Round to cell size and also make it even
						//调整大小
						_tmpl_sz.width = (((int) (_tmpl_sz.width / (2 * cell_size))) * 2 * cell_size) + cell_size * 2;
						_tmpl_sz.height = (((int) (_tmpl_sz.height / (2 * cell_size))) * 2 * cell_size) + cell_size * 2;

						cout << " hogfeatures  tmpl_sz.width*_tmpl_sz.height" << _tmpl_sz.width << "*" << _tmpl_sz.height << endl;
						//	cv::waitKey(0);
					}
				else
					{  //Make number of pixels even (helps with some logic involving half-dimensions)
						_tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
						_tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
					}
			}  //if (inithann)

		extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;  //270
		extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;  //390

		// 	cout<<"extracted_roi.width==  "<<extracted_roi.width<<"   extracted_roi.height=="<<extracted_roi.height<<endl;
// center roi with new size
		extracted_roi.x = cx - extracted_roi.width / 2;
		extracted_roi.y = cy - extracted_roi.height / 2;

		cv::Mat FeaturesMap;
		//	cv::waitKey(0);
		cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);
//		cout << "_tmpl_sz.width == " << _tmpl_sz.width << "  _tmpl_sz.height" << _tmpl_sz.height << endl;
//		cout << "z.cols*z.rows== " << z.cols << "*" << z.rows << endl;
//    	cv::imshow("z",z);
//  cout<<"z.channels== "<<z.channels()<<endl;

		if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height)
			{
				cv::resize(z, z, _tmpl_sz);
			}
//		cout << "resize z.cols*z.rows== " << z.cols << "*" << z.rows << endl;
		cv::imshow("zz", z);
		//cv::waitKey(0);
// HOG features
		if (_hogfeatures)
			{
				IplImage z_ipl = z;
				CvLSVMFeatureMapCaskade *map;
				getFeatureMaps(&z_ipl, cell_size, &map);  //提取ROI中的特征
				normalizeAndTruncate(map, 0.2f);  //归一化并且截断
				PCAFeatureMaps(map);  //
				size_patch[0] = map->sizeY;//行数
				size_patch[1] = map->sizeX;//列数
				size_patch[2] = map->numFeatures;

				cout << "size_patch[0 1 2]" << size_patch[0] << "  " << size_patch[1] << "   " << size_patch[2] << endl;

				// Procedure do deal with cv::Mat multichannel bug
				//hog特征的numFeatures为31，FeaturesMap目前每行为一个cell的hog特征，行数为图像包含的cell数
				FeaturesMap = cv::Mat(cv::Size(map->numFeatures, map->sizeX * map->sizeY), CV_32F, map->map);

				//转置之后每行为一个cell的hog特征 FeaturesMap 为 31行 size_patch[0］＊size_patch[1] 列
				FeaturesMap = FeaturesMap.t();
                std::cout<<"FeaturesMap.rows == "<<FeaturesMap.rows<<"    FeaturesMap.cols == "<<FeaturesMap.cols<<"   FeaturesMap.channel =="<<FeaturesMap.channels()<<std::endl;
				freeFeatureMapObject(&map);

				// Lab features 人脸特征 没有使用
				if (_labfeatures)
					{
						cv::Mat imgLab;
						cvtColor(z, imgLab, CV_BGR2Lab);
						unsigned char *input = (unsigned char*) (imgLab.data);

						// Sparse output vector
						cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch[0] * size_patch[1], CV_32F, float(0));

						int cntCell = 0;
						// Iterate through each cell
						for (int cY = cell_size; cY < z.rows - cell_size; cY += cell_size)
							{
								for (int cX = cell_size; cX < z.cols - cell_size; cX += cell_size)
									{
										// Iterate through each pixel of cell (cX,cY)
										for (int y = cY; y < cY + cell_size; ++y)
											{
												for (int x = cX; x < cX + cell_size; ++x)
													{
														// Lab components for each pixel
														float l = (float) input[(z.cols * y + x) * 3];
														float a = (float) input[(z.cols * y + x) * 3 + 1];
														float b = (float) input[(z.cols * y + x) * 3 + 2];

														// Iterate trough each centroid
														float minDist = FLT_MAX;
														int minIdx = 0;
														float *inputCentroid = (float*) (_labCentroids.data);
														for (int k = 0; k < _labCentroids.rows; ++k)
															{
																float dist = ((l - inputCentroid[3 * k]) * (l - inputCentroid[3 * k]))
																		+ ((a - inputCentroid[3 * k + 1]) * (a - inputCentroid[3 * k + 1]))
																		+ ((b - inputCentroid[3 * k + 2]) * (b - inputCentroid[3 * k + 2]));
																if (dist < minDist)
																	{
																		minDist = dist;
																		minIdx = k;
																	}
															}
														// Store result at output
														outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
														//((float*) outputLab.data)[minIdx * (size_patch[0]*size_patch[1]) + cntCell] += 1.0 / cell_sizeQ;
													}
											}
										cntCell++;
									}
							}
						// Update size_patch[2] and add features to FeaturesMap
						size_patch[2] += _labCentroids.rows;
						FeaturesMap.push_back(outputLab);
					}

			}				 //if (_hogfeatures)
		else
			{
				FeaturesMap = RectTools::getGrayImage(z);
				FeaturesMap -= (float) 0.5; // In Paper;
				size_patch[0] = z.rows;
				size_patch[1] = z.cols;
				size_patch[2] = 1;
			}

		if (inithann)
			{
				////生成的hann窗 为31行＊size_patch[0] * size_patch[1]列
				createHanningMats();
			}
//加窗，汉明窗
		//	cout<<"hann.rows == "<<hann.rows<<"  hann.cols == "<<hann.cols<<endl;
		FeaturesMap = hann.mul(FeaturesMap);
		return FeaturesMap;
	}

// Initialize Hanning window. Function called only in the first frame.
void KCFTracker::createHanningMats()
	{
		cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1], 1), CV_32F, cv::Scalar(0));
		cv::Mat hann2t = cv::Mat(cv::Size(1, size_patch[0]), CV_32F, cv::Scalar(0));

		for (int i = 0; i < hann1t.cols; i++)
			hann1t.at<float>(0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
		for (int i = 0; i < hann2t.rows; i++)
			hann2t.at<float>(i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

		cv::Mat hann2d = hann2t * hann1t;
// HOG features HOG特征
		if (_hogfeatures)
			{
				//增加一个通道，增加一行
				cv::Mat hann1d = hann2d.reshape(1, 1); // Procedure do deal with cv::Mat multichannel bug

				hann = cv::Mat(cv::Size(size_patch[0] * size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
				for (int i = 0; i < size_patch[2]; i++) //每个cell的hog特征数 31
					{
						for (int j = 0; j < size_patch[0] * size_patch[1]; j++)
							{ //生成的hann窗 为31行＊size_patch[0] * size_patch[1]列
								hann.at<float>(i, j) = hann1d.at<float>(0, j);
							}
					}
			}
// Gray features 灰度特征
		else
			{
				hann = hann2d;
			}
	}

// Calculate sub-pixel peak for one dimension
float KCFTracker::subPixelPeak(float left, float center, float right)
	{
		float divisor = 2 * center - right - left;

		if (divisor == 0)
			return 0;

		return 0.5 * (right - left) / divisor;
	}
