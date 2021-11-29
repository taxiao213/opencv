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
	void video_demo(Mat& image);//����ͷ
	void show_histogram_demo(Mat& image);//ֱ��ͼ
	void histogram_2d_demo(Mat& image);//2D ֱ��ͼ
	void histogram_eq_demo(Mat& image);//ֱ��ͼ���⻯
	void blur_demo(Mat& image);//ͼ��ľ������ ģ��Ч��
	void gaussian_blur_demo(Mat& image);//��˹ģ��
	void bifilter_demo(Mat& image);//˫��ģ��
	void face_detection_demo();//����ʶ��
	void loadCaffe(Mat& image);//����caffe ����ʶ��
	void loadTensorFlow(Mat& image);//����TensorFlow ����ʶ��
	void nv2rgb(Mat& image);//nv21תrgb
};