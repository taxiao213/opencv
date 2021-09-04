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

void QuickDemo::matCreate(Mat mat) {
	Mat m1, m2;
	// ��¡
	m1 = mat.clone();
	// ����
	mat.copyTo(m2);

	// �����հ�ͼ�� ���ַ�ʽ 
	// CV_8UC3  ��ʾ 8λ U �޷��� C char���� 3(3ͨ��)
	Mat m3 = Mat::zeros(Size(50, 50), CV_8UC3);

	//Mat m3 = Mat::ones(Size(50,50), CV_8UC3);

	//Mat m3 = Mat::eye(Size(50,50), CV_8UC3);

	// ��ֵ����
	m3 = Scalar(0, 0, 255);

	// ��ȡ��� �߶� ͨ����
	std::cout << "width:" << m3.cols << " height:" << m3.rows << "channel: " << m3.channels() << std::endl;

	std::cout << m3 << std::endl;

	imshow("�հ�ͼ��", m3);
}

void QuickDemo::pixelTransformation(Mat mat)
{
	Mat dist;
	dist = mat.clone();
	int w = dist.cols;
	int h = dist.rows;
	int channel = dist.channels();
	std::cout << "w:" << w << " h:" << h << " channel:" << channel << std::endl;
	for (int row = 0; row < w; row++) {
		for (int col = 0; col < w; col++) {
			if (channel == 1) {
				// �Ҷ�ͼ��
				int pv = dist.at<char>(row, col);
				dist.at<char>(row, col) = 255 - pv;
			}
			if (channel == 3) {
				// ��ɫͼ��
				Vec3b rgb = dist.at<Vec3b>(row, col);
				dist.at<Vec3b>(row, col)[0] = 255 - rgb[0];
				dist.at<Vec3b>(row, col)[1] = 255 - rgb[1];
				dist.at<Vec3b>(row, col)[2] = 255 - rgb[2];
			}
		}
	}
	namedWindow("����ת��", WINDOW_FREERATIO);
	imshow("����ת��", dist);
}


void QuickDemo::pixelTransformation2(Mat mat)
{
	Mat dist;
	dist = mat.clone();
	int w = dist.cols;
	int h = dist.rows;
	int channel = dist.channels();
	std::cout << "w:" << w << " h:" << h << " channel:" << channel << std::endl;
	for (int row = 0; row < w; row++) {
		uchar* current_ptr = dist.ptr(row);// ��ȡָ��
		for (int col = 0; col < w; col++) {
			if (channel == 1) {
				// �Ҷ�ͼ��
				int pv = *current_ptr;
				dist.at<char>(row, col) = 255 - pv;
			}
			if (channel == 3) {
				// ��ɫͼ��
				*current_ptr++ = 255 - *current_ptr;
				*current_ptr++ = 255 - *current_ptr;
				*current_ptr++ = 255 - *current_ptr;
			}
		}
	}
	namedWindow("����ת�� ָ��", WINDOW_FREERATIO);
	imshow("����ת�� ָ��", dist);
}

void QuickDemo::pixelScale(Mat mat)
{
	Mat copy;
	Mat dst;
	copy = Mat::zeros(mat.size(), mat.type());
	copy = Scalar(2, 2, 2);
	//add(mat, copy, dst);//��
	//subtract(mat, copy, dst);//��
	//multiply(mat, copy, dst);//��
	divide(mat, copy, dst);//��
	imshow("pixelScale", dst);
}


// ���ȵ��� image*1.0 + m*0.0 + b
static void on_lightness(int b, void* userdata) {
	Mat image = *((Mat*)userdata);
	Mat dst = Mat::zeros(image.size(), image.type());
	Mat m = Mat::zeros(image.size(), image.type());
	addWeighted(image, 1.0, m, 0, b, dst);
	imshow("���ȵ���", dst);
}

// �Աȶȵ��� image*contrast + m*0.0 + 0
static void on_contrast(int b, void* userdata) {
	Mat image = *((Mat*)userdata);
	Mat dst = Mat::zeros(image.size(), image.type());
	Mat m = Mat::zeros(image.size(), image.type());
	double contrast = b / 100.0;
	addWeighted(image, contrast, m, 0.0, 0, dst);
	imshow("���ȵ���", dst);
}

void QuickDemo::tracking_bar_demo(Mat& image)
{
	namedWindow("���ȵ���", WINDOW_AUTOSIZE);
	int max_value = 100;
	int lightness = 50;
	int contrast_max_value = 200;
	int contrast_value = 100;
	createTrackbar("Value Bar:", "���ȵ���", &lightness, max_value, on_lightness, (void*)(&image));
	createTrackbar("Contrast Bar:", "���ȵ���", &contrast_value, contrast_max_value, on_contrast, (void*)(&image));
	on_lightness(50, &image);
}

void QuickDemo::keyDemo(Mat& mat)
{
	Mat dst = Mat::zeros(mat.size(), mat.type());
	while (true)
	{
		int key = waitKey(100);
		if (key == 27)break;
		if (key == 49) {
			std::cout << key << std::endl;
			cvtColor(mat, dst, COLOR_BGR2GRAY);
		}
		if (key == 50) {
			std::cout << key << std::endl;
			cvtColor(mat, dst, COLOR_BGR2HLS);
		}
		if (key == 51) {
			std::cout << key << std::endl;
			dst = Scalar(10, 10, 10);
			add(mat, dst, dst);
		}

		imshow("key", dst);
	}
}

void QuickDemo::colorStyle(Mat& mat)
{
	int colorMap[] = {
		COLORMAP_AUTUMN,
		COLORMAP_BONE,
		COLORMAP_CIVIDIS,
		COLORMAP_DEEPGREEN,
		COLORMAP_HOT,
		COLORMAP_HSV,
		COLORMAP_INFERNO,
		COLORMAP_JET,
		COLORMAP_MAGMA,
		COLORMAP_OCEAN,
		COLORMAP_PINK,
		COLORMAP_PARULA,
		COLORMAP_RAINBOW,
		COLORMAP_SPRING,
		COLORMAP_TWILIGHT,
		COLORMAP_TURBO,
		COLORMAP_TWILIGHT,
		COLORMAP_VIRIDIS,
		COLORMAP_TWILIGHT_SHIFTED,
		COLORMAP_WINTER
	};

	Mat dst;
	int index = 0;
	while (true)
	{
		int key = waitKey(100);
		if (key == 27)break;
		if (key == 49) {
			// ���� 1 ʱ����ͼƬ��ָ��λ��
			std::cout << key << std::endl;
			imwrite("C:/project/c/opencv/project/opencv_01/images/save/imwrite.png", dst);
		}
		applyColorMap(mat, dst, colorMap[index % 19]);
		//ѭ��չʾ
		index++;
		imshow("ѭ������ͼ", dst);
	}
}

void QuickDemo::bitWise_demo(Mat& mat)
{
	Mat m1 = Mat::zeros(mat.size(), mat.type());
	Mat m2 = Mat::zeros(mat.size(), mat.type());
	rectangle(m1, Rect(100, 100, 80, 80), Scalar(0, 255, 0), -1, LINE_8, 0);
	rectangle(m2, Rect(150, 150, 80, 80), Scalar(255, 255, 0), -1, LINE_8, 0);
	imshow("M1", m1);
	imshow("M2", m2);
	Mat dst;
	//bitwise_and(m1, m2, dst); //λ������
	bitwise_or(m1, m2, dst); //λ������
	//bitwise_not(m1, m2, dst); //ȡ������
	//bitwise_xor(m1, m2, dst); //������
	imshow("λ����", dst);
}

void QuickDemo::channels_demo(Mat& image)
{
	std::vector<Mat> mv;
	split(image, mv);
	imshow("��ɫ", mv[0]);
	imshow("��ɫ", mv[1]);
	imshow("��ɫ", mv[2]);
	//0,1,2����ͨ���ֱ����BGR��
	//�ر�2��ͨ����ζ�ſ���һ��ͨ����
	Mat dst = Mat::zeros(image.size(), image.type());
	mv[0] = 0;
	mv[2] = 0;
	merge(mv, dst);
	imshow("�ϲ�ͨ��", dst);

	int from_to[] = { 0,2,1,1,2,0 };
	//��ͨ���໥��������0->��2����һ->��һ���ڶ�->��0
	mixChannels(&image, 1, &dst, 1, from_to, 3);//3��ʾ3��ͨ��
	//����1ָ������ͼ��->����2���õ�dst
	imshow("ͨ�����", dst);
}

void QuickDemo::inrange_demo(Mat& image)
{
	Mat hsv;
	cvtColor(image, hsv, COLOR_BGR2HSV);
	Mat mask;
	inRange(hsv, Scalar(35, 43, 46), Scalar(77, 255, 255), mask);
	//35,43,46 ����ͼƬ����ɫ�����ȷ����Сֵ��
	//77,255,255 ����ͼƬ����ɫ�����ȷ�����ֵ�� 
	//����1�ͷ�Χ������2�߷�Χ
	//��hsv�е��ɵ͵��ߵ����ص���ȡ�������Ҵ洢��mask���С�
	//����Ϊ0�Ǻ�ɫ��1�ǰ�ɫ
	imshow("mask", hsv);
	Mat redback = Mat::zeros(image.size(), image.type());
	redback = Scalar(255, 0, 0);
	bitwise_not(mask, mask);
	imshow("mask", mask);
	image.copyTo(redback, mask);//��redback���Ƶ�mask��maskͨ��inRange�õ���
	imshow("roi��ȡ", hsv);
	imshow("roi��ȡ1", redback);
}

void QuickDemo::pixel_statistic_demo(Mat& image)
{
	double minv, maxv;//������ֵ
	Point minLoc, maxLoc;//������ֵ��ַ
	std::vector<Mat>mv;//mv��һ��Mat���͵����� װ�����������
	split(image, mv);
	for (int i = 0; i < mv.size(); i++)
	{
		//�ֱ��ӡ����ͨ������ֵ
		minMaxLoc(mv[i], &minv, &maxv, &minLoc, &maxLoc);//���ͼ������ֵ����Сֵ��
		std::cout << "No.channels:" << i << " minvalue:" << minv << " maxvalue:" << maxv << std::endl;
	}
	Mat mean, stddev;
	meanStdDev(image, mean, stddev);//���ͼ��ľ�ֵ�ͷ���
	std::cout << "mean:" << mean << std::endl;
	std::cout << "stddev:" << stddev << std::endl;
}

void QuickDemo::drawing_demo(Mat& image)
{
	Mat bg = Mat::zeros(image.size(), image.type());
	// ����
	rectangle(bg, Rect(100, 100, 60, 60), Scalar(0, 0, 255), -1, LINE_8, 0);
	// Բ
	circle(image, Point(100, 100), 100, Scalar(255, 0, 0), 0, LINE_8, 0);
	// ֱ��
	line(image, Point(100, 100), Point(200, 200), Scalar(0, 255, 0), 3, LINE_8, 0);
	// ��Բ
	ellipse(image, RotatedRect(Point(100, 100), Size(100, 200), 90.0), Scalar(40, 40, 40), 0, LINE_8);
	Mat dst;
	addWeighted(image, 0.5, bg, 0.3, 0, dst);
	imshow("line", dst);
}

void QuickDemo::random_drawing(Mat& image)
{
	Mat canvas = Mat::zeros(Size(512, 512), CV_8UC3);
	int w = canvas.cols;
	int h = canvas.rows;
	RNG rng(12345);
	double d1 = rng.operator double();
	int int1 = rng;
	std::cout << d1 << std::endl;
	std::cout << int1 << std::endl;
	while (true)
	{
		int c = waitKey(10);
		if (c == 27)
		{
			break;
		}
		int x1 = rng.uniform(0, canvas.cols);
		int y1 = rng.uniform(0, h);
		int x2 = rng.uniform(0, canvas.cols);
		int y2 = rng.uniform(0, h);
		int b = rng.uniform(0, 255);
		int g = rng.uniform(0, 255);
		int r = rng.uniform(0, 255);
		//canvas = Scalar(0, 0, 0); // �򿪺�ֻ��ʾ����һ������
		line(canvas, Point(x1, y1), Point(x2, y2), Scalar(b, g, r), 8, LINE_AA, 0);//line_AA��ʾȥ����� 
		imshow("���������ʾ", canvas);
	}
}
