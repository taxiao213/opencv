#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class QuickDemo
{
public:
	QuickDemo();
	void colorSpace(Mat* mat);// 色彩转换
	void matCreate(Mat mat);// 创建Mat
	void pixelTransformation(Mat mat);// 像素转换
	void pixelTransformation2(Mat mat);// 像素转换,使用指针
	void pixelScale(Mat mat);//放大
	void tracking_bar_demo(Mat& mat);// 滚动条
	void keyDemo(Mat& mat);// 键盘响应事件
	void colorStyle(Mat& mat);// 颜色表
	void bitWise_demo(Mat& mat);//像素逻辑操作
	void channels_demo(Mat& image);//通道分离
	void inrange_demo(Mat& image);//图像色彩空间转换
	void pixel_statistic_demo(Mat& image);//图像像素值统计
	void drawing_demo(Mat& image);//图像几何形状的绘制
	void random_drawing(Mat& image);//随机数与随机颜色
	void polyline_drawing_demo(Mat& image);//多边形填充与绘制
	void mouse_drawing_demo(Mat& image);//鼠标事件
	void norm_demo(Mat& image);//归一化
	void resize_demo(Mat& image);//图像缩放和插值
	void flip_demo(Mat& image);//图像翻转
	void rotate_demo(Mat& image);//图像旋转
};