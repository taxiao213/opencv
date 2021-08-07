#include "QuickDemo.h"

QuickDemo::QuickDemo()
{
}

void QuickDemo::colorSpace(Mat* src)
{
	Mat dst;
	cvtColor(*src, dst, COLOR_BGR2GRAY);
	// д���ļ�
	imwrite("C:/project/c/opencv/project/opencv_01/images/opencv_01.jpg", dst);
	imshow("color spacer", dst);
	waitKey(0);
	destroyAllWindows();
}

void QuickDemo::matCreate(Mat  mat) {
	Mat m1, m2;
	// ��¡
	m1 = mat.clone();
	// ����
	mat.copyTo(m2);

	// �����հ�ͼ�� ���ַ�ʽ 
	// CV_8UC3  ��ʾ 8λ U �޷��� C char���� 3(3ͨ��)
	Mat m3 = Mat::zeros(Size(50,50), CV_8UC3);

	//Mat m3 = Mat::ones(Size(50,50), CV_8UC3);

	//Mat m3 = Mat::eye(Size(50,50), CV_8UC3);

	// ��ֵ����
	m3 = Scalar(0, 0, 255);

	// ��ȡ��� �߶� ͨ����
	std::cout << "width:"<<m3.cols<<" height:"<< m3.rows <<"channel: "<< m3.channels()  <<std::endl;
	
	std::cout << m3 << std::endl;

	imshow("�հ�ͼ��",m3);
}
