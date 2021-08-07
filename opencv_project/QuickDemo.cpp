#include "QuickDemo.h"

QuickDemo::QuickDemo()
{
}

void QuickDemo::colorSpace(Mat* src)
{
	Mat dst;
	cvtColor(*src, dst, COLOR_BGR2GRAY);
	// 写入文件
	imwrite("C:/project/c/opencv/project/opencv_01/images/opencv_01.jpg", dst);
	imshow("color spacer", dst);
	waitKey(0);
	destroyAllWindows();
}

void QuickDemo::matCreate(Mat  mat) {
	Mat m1, m2;
	// 克隆
	m1 = mat.clone();
	// 复制
	mat.copyTo(m2);

	// 创建空白图像 三种方式 
	// CV_8UC3  表示 8位 U 无符号 C char类型 3(3通道)
	Mat m3 = Mat::zeros(Size(50,50), CV_8UC3);

	//Mat m3 = Mat::ones(Size(50,50), CV_8UC3);

	//Mat m3 = Mat::eye(Size(50,50), CV_8UC3);

	// 赋值操作
	m3 = Scalar(0, 0, 255);

	// 获取宽度 高度 通道数
	std::cout << "width:"<<m3.cols<<" height:"<< m3.rows <<"channel: "<< m3.channels()  <<std::endl;
	
	std::cout << m3 << std::endl;

	imshow("空白图像",m3);
}
