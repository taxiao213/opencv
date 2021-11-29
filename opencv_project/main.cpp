#include <iostream>
#include <opencv2/opencv.hpp>
#include "QuickDemo.h"

using namespace cv;
int main() {
	// ¶ÁÈ¡ÎÄ¼þ
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/zhifang_ball.png", IMREAD_UNCHANGED);
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/dog.jpg", IMREAD_UNCHANGED);
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/girl.jpg", IMREAD_UNCHANGED);
	Mat src = imread("C:/project/c/opencv/project/opencv_01/images/persons.png", IMREAD_UNCHANGED);
	//Mat src = imread("C:/project/c/opencv/project/opencv_01/images/flower.png", IMREAD_UNCHANGED);
	//Mat src = imread("C:/Users/yin13/Desktop/qq.jpg", IMREAD_UNCHANGED);
	if (src.empty()) {
		return -1;
	}
	namedWindow("input", WINDOW_AUTOSIZE);
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
	//quickDemo.random_drawing(src);
	//quickDemo.polyline_drawing_demo(src);
	//quickDemo.mouse_drawing_demo(src);
	//quickDemo.norm_demo(src);
	//quickDemo.resize_demo(src);
	//quickDemo.flip_demo(src);
	//quickDemo.rotate_demo(src);
	//quickDemo.video_demo(src);
	//quickDemo.show_histogram_demo(src);
	//quickDemo.histogram_2d_demo(src);
	//quickDemo.histogram_eq_demo(src);
	//quickDemo.blur_demo(src);
	//quickDemo.gaussian_blur_demo(src);
	//quickDemo.bifilter_demo(src);
	//quickDemo.face_detection_demo();
	//quickDemo.loadCaffe(src);
	//quickDemo.loadTensorFlow(src);
	quickDemo.nv2rgb(src);

	waitKey(0);
	destroyAllWindows();
	return 0;
}