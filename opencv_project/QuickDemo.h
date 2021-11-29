#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
using namespace cv;
using namespace dnn;
using namespace std;

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
	void video_demo(Mat& image);//摄像头
	void show_histogram_demo(Mat& image);//直方图
	void histogram_2d_demo(Mat& image);//2D 直方图
	void histogram_eq_demo(Mat& image);//直方图均衡化
	void blur_demo(Mat& image);//图像的卷积操作 模糊效果
	void gaussian_blur_demo(Mat& image);//高斯模糊
	void bifilter_demo(Mat& image);//双边模糊
	void face_detection_demo();//人脸识别
	void loadCaffe(Mat& image);//加载caffe 物体识别
	void loadTensorFlow(Mat& image);//加载TensorFlow 物体识别
	void nv2rgb(Mat& image);//nv21转rgb
};