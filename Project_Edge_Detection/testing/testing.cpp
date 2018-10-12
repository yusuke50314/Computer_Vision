//
// testing.cpp : Defines the entry point for the console application.
//

//Visual Studio
#include "stdafx.h"

//C++ Standard Library
#include <iostream>
#include <iomanip>
#include <vector>

//Math Library
#define _USE_MATH_DEFINES // for C++ 
#include <cmath> 

//OpenCV Library
#include "cv_library.h"

//Namespace
using namespace std;
using namespace cv;

//////////////////////////////////////////////////////////////////////////////////
/*	Gaussian Filter	*/
//////////////////////////////////////////////////////////////////////////////////
Mat gaussianFilter(Mat image, Mat kernel) {
	int ky = kernel.rows;
	int kx = kernel.cols;
	int space = kx / 2;
	int h = image.rows;
	int w = image.cols;
	Mat filtered_image = Mat_<double>(h, w);
	vector<double> filtered_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			double filtered_pix = 0;
			for (int kr = (-1)*space; kr <= space; kr++) {
				for (int kc = (-1)*space; kc <= space; kc++) {
					int er = r + kr;
					int ec = c + kc;
					if (0 <= er && er < h && 0 <= ec && ec < w) {
						filtered_pix= filtered_pix + (kernel.at<double>(kr+space, kc+space))*((double)image.at<uchar>(er, ec));
					}
					else {
						filtered_pix = filtered_pix + (kernel.at<double>(kr + space, kc + space)) * 255;
					}
				}
			}
			filtered_array.push_back(filtered_pix);
		}
	}
	Mat m1 = Mat_<double>(filtered_array);
	Mat m2 = Mat_<double>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0+i*w,w+i*w).copyTo(m2.row(i).colRange(0,w));
	}
	Mat m3;
	m2.convertTo(m3, CV_8U, 1);
	return m3;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Sobel Filter	*/
//////////////////////////////////////////////////////////////////////////////////
Mat sobelFilter(Mat image, Mat kernel) {
	int ky = kernel.rows;
	int kx = kernel.cols;
	int space = kx / 2;
	int h = image.rows;
	int w = image.cols;
	vector<int> filtered_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			int filtered_pix = 0;
			for (int kr = (-1)*space; kr <= space; kr++) {
				for (int kc = (-1)*space; kc <= space; kc++) {
					int er = r + kr;
					int ec = c + kc;
					if (0 <= er && er < h && 0 <= ec && ec < w) {
						filtered_pix = filtered_pix + (kernel.at<int>(kr + space, kc + space))*((int)image.at<uchar>(er, ec));
					}
					else {
						filtered_pix = filtered_pix + (kernel.at<int>(kr + space, kc + space))*255;
					}
				}
			}
			filtered_array.push_back(filtered_pix);
		}
	}
	Mat m1 = Mat_<int>(filtered_array);
	Mat m2 = Mat_<int>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Gradient Magnitude	*/
//////////////////////////////////////////////////////////////////////////////////
Mat mag(Mat image_x, Mat image_y) {
	int h = image_x.rows;
	int w = image_y.cols;
	vector<uchar> mag_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			int gx = image_x.at<int>(r, c);
			int gy = image_y.at<int>(r, c);
			int mag = sqrt((gx*gx) + (gy*gy));
			if (mag > 255) {
				mag = 255;
			}
			mag_array.push_back(mag);
		}
	}
	Mat m1 = Mat_<uchar>(mag_array);
	Mat m2 = Mat_<uchar>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Thresholding (Sobel)	*/
//////////////////////////////////////////////////////////////////////////////////
Mat sobelThreshold(Mat image, int threshold) {
	int h = image.rows;
	int w = image.cols;
	vector<uchar> sobel_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			int pix = image.at<uchar>(r, c);
			if (pix > threshold) {
				pix = 255;
			}
			else {
				pix = 0;
			}
			sobel_array.push_back(pix);
		}
	}
	Mat m1 = Mat_<uchar>(sobel_array);
	Mat m2 = Mat_<uchar>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Gradient Angle	*/
//////////////////////////////////////////////////////////////////////////////////
Mat ang(Mat image_x, Mat image_y) {
	int h = image_x.rows;
	int w = image_y.cols;
	vector<double> ang_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			double gx = image_x.at<int>(r, c);
			double gy = (-1) * image_y.at<int>(r, c);
			ang_array.push_back(atan2(gy, gx));
		}
	}
	Mat m1 = Mat_<double>(ang_array);
	Mat m2 = Mat_<double>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Approximate Angle	*/
//////////////////////////////////////////////////////////////////////////////////
Mat app(Mat image) {
	int h = image.rows;
	int w = image.cols;
	vector<uchar> ang_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
		double angleR = image.at<double>(r, c);
		double angleD = angleR * 180 / M_PI;
		if (-180 <= angleD && angleD < -157.5) {
			angleD = 0;
		}
		else if (-157.5 <= angleD && angleD < -112.5) {
			angleD = 45;
		}
		else if (-112.5 <= angleD && angleD < -67.5) {
			angleD = 90;
		}
		else if (-67.5 <= angleD && angleD < -22.5) {
			angleD = 135;
		}
		else if (-22.5 <= angleD && angleD < 22.5) {
			angleD = 0;
		}
		else if (22.5 <= angleD && angleD < 67.5) {
			angleD = 45;
		}
		else if (67.5 <= angleD && angleD < 112.5) {
			angleD = 90;
		}
		else if (112.5 <= angleD && angleD < 157.5) {
			angleD = 135;
		}
		else {
			angleD = 0;
		}
		ang_array.push_back((uchar)angleD);
		}
	}
	Mat m1 = Mat_<uchar>(ang_array);
	Mat m2 = Mat_<uchar>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Non-maximum Suppression	*/
//////////////////////////////////////////////////////////////////////////////////
Mat non_max(Mat mag, Mat ang) {
	int h = mag.rows;
	int w = mag.cols;
	vector<uchar> max_array;
	int er1, ec1;
	int er2, ec2;
	double cur_ang = ang.at<uchar>(0, 0);
	double cur_mag = mag.at<uchar>(0, 0);
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			double cur_ang = ang.at<uchar>(r, c);
			double cur_mag = mag.at<uchar>(r, c);
			double com_mag;
			double dst=cur_mag;
			if (cur_ang == 0) {
				er1 = r;
				ec1 = c + 1;
				if (0 <= er1 && er1 < h && 0 <= ec1 && ec1 < w) {
					com_mag= mag.at<uchar>(er1, ec1);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
				er2 = r;
				ec2 = c - 1;
				if (0 <= er2 && er2 < h && 0 <= ec2 && ec2 < w) {
					com_mag = mag.at<uchar>(er2, ec2);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
			}
			else if (cur_ang == 45) {
				er1 = r - 1;
				ec1 = c + 1;
				if (0 <= er1 && er1 < h && 0 <= ec1 && ec1 < w) {
					com_mag = mag.at<uchar>(er1, ec1);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
				er2 = r + 1;
				ec2 = c - 1;
				if (0 <= er2 && er2 < h && 0 <= ec2 && ec2 < w) {
					com_mag = mag.at<uchar>(er2, ec2);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
			}
			else if (cur_ang == 90) {
				er1 = r + 1;
				ec1 = c;
				if (0 <= er1 && er1 < h && 0 <= ec1 && ec1 < w) {
					com_mag = mag.at<uchar>(er1, ec1);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
				er2 = r - 1;
				ec2 = c;
				if (0 <= er2 && er2 < h && 0 <= ec2 && ec2 < w) {
					com_mag = mag.at<uchar>(er2, ec2);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
			}
			else if (cur_ang == 135) {	
				er1 = r + 1;
				ec1 = c + 1;
				if (0 <= er1 && er1 < h && 0 <= ec1 && ec1 < w) {
					com_mag = mag.at<uchar>(er1, ec1);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
				er2 = r - 1;
				ec2 = c - 1;
				if (0 <= er2 && er2 < h && 0 <= ec2 && ec2 < w) {
					com_mag = mag.at<uchar>(er2, ec2);
					if (cur_mag < com_mag) {
							dst = 0;
					}
				}
				else {
					dst = 0;
				}
			}
			max_array.push_back(dst);
		}
	}
	Mat m1 = Mat_<uchar>(max_array);
	Mat m2 = Mat_<uchar>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Hysteresis Thresholding	*/
//////////////////////////////////////////////////////////////////////////////////
Mat hyst(Mat image, uchar min, uchar max) {
	int h = image.rows;
	int w = image.cols;
	vector<uchar> hyst_array;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			uchar cur = image.at<uchar>(r, c);
			uchar dst = 0;
			uchar er,ec,com;
			if (cur >= max) {
				dst = 255;
			}
			else if (cur < min) {
				dst = 0;
			}
			else {
				for (int kr = -1; kr <= 1; kr++) {
					for (int kc = -1; kc <= 1; kc++) {
						er = r + kr;
						ec = c + kc;
						if (0 <= er && er < h && 0 <= ec && ec < w) {
							com = image.at<uchar>(er, ec);
							if (com >= max) {
								dst = 255;
							}
						}
						else {
							dst = 255;
						}
					}
				}
			}
			hyst_array.push_back(dst);
		}
	}
	Mat m1 = Mat_<uchar>(hyst_array);
	Mat m2 = Mat_<uchar>(h, w);
	m1 = m1.t();
	for (int i = 0; i < h; i++) {
		m1.row(0).colRange(0 + i*w, w + i*w).copyTo(m2.row(i).colRange(0, w));
	}
	return m2;
}

//////////////////////////////////////////////////////////////////////////////////
/*	Edge Detector	*/
//////////////////////////////////////////////////////////////////////////////////
void edgeDetector(Mat image) {
	//Gaussian Filter (3~3)
	
	Mat kernel_g = (Mat_<double>(3, 3) 
	<<	(double)1 / 16, (double)1 / 8, (double)1 / 16, 
		(double)1 / 8,	(double)1 / 4, (double)1 / 8,
		(double)1 / 16, (double)1 / 8, (double)1 / 16);
	
	//Gaussian Filter (5~5)
	/**
	Mat kernel_g = (Mat_<double>(5, 5)
	<<  (double)2/159, (double)4/159, (double)5/159, (double)4 / 159, (double)2 / 159,
		(double)4 / 159, (double)9 / 159, (double)12 / 159, (double)9 / 159, (double)4 / 159,
		(double)5 / 159, (double)12 / 159, (double)15 / 159, (double)12 / 159, (double)5 / 159,
		(double)4 / 159, (double)9 / 159, (double)12 / 159, (double)9 / 159, (double)4 / 159,
		(double)2 / 159, (double)4 / 159, (double)5 / 159, (double)4 / 159, (double)2 / 159);
	**/
	//Sobel Filter (X Direction)
	Mat kernel_x = (Mat_<int>(3, 3)
	<<  (int)-1, (int)0, (int)1,
		(int)-2, (int)0, (int)2,
		(int)-1, (int)0, (int)1);
	//Sobel Filter (Y Direction)
	Mat kernel_y = (Mat_<int>(3, 3)
	<<  (int)-1, (int)-2, (int)-1,
		(int)0, (int)0, (int)0,
		(int)1, (int)2, (int)1);
	Mat filtered_image = gaussianFilter(image, kernel_g);
	namedWindow("Gaussian Filtered Image");
	imshow("Gaussian Filtered Image", filtered_image);
	Mat cv_canny;
	Canny(filtered_image, cv_canny, 30, 60);
	namedWindow("OpenCV Canny Sample");
	imshow("OpenCV Canny Sample", cv_canny);
	Mat grad;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	Sobel(filtered_image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
	Sobel(filtered_image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
	imshow("OpenCV Sobel Sample", grad);
	Mat sobelCVImage = sobelThreshold(grad, 33);
	namedWindow("OpenCV Sobel Sample (Threshold)");
	imshow("OpenCV Sobel Sample (Threshold)", sobelCVImage);
	Mat sobel_x_image = sobelFilter(filtered_image, kernel_x);
	Mat sobel_y_image = sobelFilter(filtered_image, kernel_y);
	Mat gradientMag = mag(sobel_x_image, sobel_y_image);
	namedWindow("Gradient Magnitude Image");
	imshow("Gradient Magnitude Image", gradientMag);
	Mat gradientAngle = ang(sobel_x_image, sobel_y_image);
	Mat sobelImage = sobelThreshold(gradientMag, 45);
	namedWindow("Sobel Image");
	imshow("Sobel Image", sobelImage);
	Mat appgradientAngle = app(gradientAngle);
	Mat nonMaxSuppression = non_max(gradientMag, appgradientAngle);
	namedWindow("Non Maximum Suppression Image");
	imshow("Non Maximum Suppression Image", nonMaxSuppression);
	Mat hystThreshold = hyst(nonMaxSuppression, 20, 40);
	namedWindow("Hysteresis Threshold Image");
	imshow("Hysteresis Threshold Image", hystThreshold);
	namedWindow("Personal Canny Image");
	imshow("Personal Canny Image", hystThreshold);
}

//////////////////////////////////////////////////////////////////////////////////
/*	Main */
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char const *argv[]) {
	//Original Image
	Mat src_image = imread("puppy.jpg");
	namedWindow("Original Image");
	imshow("Original Image", src_image);
	//Converting to Gray Image
	Mat gray_image;
	cvtColor(src_image, gray_image, CV_BGR2GRAY);
	namedWindow("Gray Image");
	imshow("Gray Image", gray_image);
	//Edge Detection Function
	edgeDetector(gray_image);
	waitKey(0);
	return 0;
}