#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class QuickDemo
{
public:
	QuickDemo();
	void colorSpace(Mat* mat);// ɫ��ת��
	void matCreate(Mat mat);// ����Mat
	void pixelTransformation(Mat mat);// ����ת��
	void pixelTransformation2(Mat mat);// ����ת��,ʹ��ָ��
	void pixelScale(Mat mat);//�Ŵ�
	void tracking_bar_demo(Mat& mat);// ������
	void keyDemo(Mat& mat);// ������Ӧ�¼�
	void colorStyle(Mat& mat);// ��ɫ��
	void bitWise_demo(Mat& mat);//�����߼�����
	void channels_demo(Mat& image);//ͨ������
	void inrange_demo(Mat& image);//ͼ��ɫ�ʿռ�ת��
	void pixel_statistic_demo(Mat& image);//ͼ������ֵͳ��
	void drawing_demo(Mat& image);//ͼ�񼸺���״�Ļ���
	void random_drawing(Mat& image);//������������ɫ
	void polyline_drawing_demo(Mat& image);//�������������
	void mouse_drawing_demo(Mat& image);//����¼�
	void norm_demo(Mat& image);//��һ��
	void resize_demo(Mat& image);//ͼ�����źͲ�ֵ
	void flip_demo(Mat& image);//ͼ��ת
	void rotate_demo(Mat& image);//ͼ����ת
};