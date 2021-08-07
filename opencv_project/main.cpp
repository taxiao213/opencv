#include <iostream>
#include <opencv2/opencv.hpp>
#include "QuickDemo.h"

using namespace cv;
int main() {
	// ¶ÁÈ¡ÎÄ¼þ
	Mat src = imread("C:/project/c/opencv/project/opencv_01/images/age_gender.jpg", IMREAD_UNCHANGED);
	if (src.empty()) {
		return -1;
	}
	namedWindow("input", WINDOW_FREERATIO);
	imshow("input", src);
	QuickDemo quickDemo;
	//quickDemo.colorSpace(&src);
	quickDemo.matCreate(src);

	waitKey(0);
	destroyAllWindows();
	return 0;
}