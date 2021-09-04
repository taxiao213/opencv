#include <iostream>
#include <opencv2/opencv.hpp>
#include "QuickDemo.h"

using namespace cv;
int main() {
	// ¶ÁÈ¡ÎÄ¼þ
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/zhifang_ball.png", IMREAD_UNCHANGED);
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/gaoyy.png", IMREAD_UNCHANGED);
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/girl.jpg", IMREAD_UNCHANGED);
	Mat src = imread("C:/project/c/opencv/project/opencv_01/images/flower.png", IMREAD_UNCHANGED);
	if (src.empty()) {
		return -1;
	}
	namedWindow("input", WINDOW_FREERATIO);
	imshow("input", src);
	QuickDemo quickDemo;
	//quickDemo.colorSpace(&src);
	//quickDemo.matCreate(src);
	//quickDemo.pixelTransformation(src);
	//quickDemo.pixelTransformation2(src);
	//quickDemo.pixelScale(src);
	//quickDemo.tracking_bar_demo(src);
	//quickDemo.keyDemo(src);
	//quickDemo.colorStyle(src);
	//quickDemo.bitWise_demo(src);
	//quickDemo.channels_demo(src);
	//quickDemo.inrange_demo(src);
	//quickDemo.pixel_statistic_demo(src);
	//quickDemo.drawing_demo(src);
	quickDemo.random_drawing(src);

	waitKey(0);
	destroyAllWindows();
	return 0;
}