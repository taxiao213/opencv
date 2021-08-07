#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class QuickDemo
{
public:
	QuickDemo();
	void colorSpace(Mat* mat);// 色彩转换
	void matCreate(Mat  mat);// 创建Mat
};