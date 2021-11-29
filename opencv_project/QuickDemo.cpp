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
static void on_lightness(int b, void* userdata)
{
	Mat image = *((Mat*)userdata);
	Mat dst = Mat::zeros(image.size(), image.type());
	Mat m = Mat::zeros(image.size(), image.type());
	addWeighted(image, 1.0, m, 0, b, dst);
	imshow("���ȵ���", dst);
}

// �Աȶȵ��� image*contrast + m*0.0 + 0
static void on_contrast(int b, void* userdata)
{
	Mat image = *((Mat*)userdata);
	Mat dst = Mat::zeros(image.size(), image.type());
	Mat m = Mat::zeros(image.size(), image.type());
	double contrast = b / 100.0;
	addWeighted(image, contrast, m, 0.0, 0, dst);
	imshow("���ȵ���", dst);
}

void QuickDemo::tracking_bar_demo(Mat& image)
{
	namedWindow("������", WINDOW_AUTOSIZE);
	int max_value = 100;
	int lightness = 50;
	int contrast_max_value = 200;
	int contrast_value = 100;
	createTrackbar("Value Bar:", "������", &lightness, max_value, on_lightness, (void*)(&image));
	createTrackbar("Contrast Bar:", "������", &contrast_value, contrast_max_value, on_contrast, (void*)(&image));
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

void QuickDemo::polyline_drawing_demo(Mat& image)
{
	Mat canvas = Mat::zeros(Size(500, 500), CV_8UC3);
	Point p1(100, 100);
	Point p2(350, 100);
	Point p3(450, 280);
	Point p4(320, 450);
	Point p5(80, 400);
	std::vector<Point> list;
	list.push_back(p1);
	list.push_back(p2);
	list.push_back(p3);
	list.push_back(p4);
	list.push_back(p5);
	//fillPoly(canvas, list, Scalar(50, 250, 0), LINE_AA,0 );
	//polylines(canvas, list, 0, Scalar(255, 0, 0), 2, LINE_AA, 0);
	std::vector<std::vector<Point>> construct;
	construct.push_back(list);
	// �߿�Ϊ���� ���
	drawContours(canvas, construct, -1, Scalar(0, 0, 255), -1, LINE_AA);
	imshow("�����", canvas);
}


Point sp(-1, -1);//���Ŀ�ʼ��λ��
Point ep(-1, -1);
Mat temp;
static void mouse(int event, int x, int y, int flags, void* userdata) {
	Mat  image = *(Mat*)(userdata);
	Rect rect;
	int dx, dy;
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		sp.x = x;
		sp.y = y;
		std::cout << "x: " + x << std::endl;
		std::cout << "y: " + y << std::endl;
		break;

	case EVENT_LBUTTONUP:
		ep.x = x;
		ep.y = y;
		rect.x = sp.x;
		rect.y = sp.y;
		dx = ep.x - sp.x;
		dy = ep.y - sp.y;
		rect.width = dx;
		rect.height = dy;
		if (dx > 0 && dy > 0) {
			rectangle(image, rect, Scalar(0, 0, 255), 1, LINE_AA, 0);
			imshow("roi", image);
			imshow("������", image);
			sp.x = -1;
			sp.y = -1;//��λ��Ϊ��һ����׼��
			ep.x = -1;
			ep.y = -1;
		}
		break;

	case EVENT_MOUSEMOVE:
		ep.x = x;
		ep.y = y;
		rect.x = sp.x;
		rect.y = sp.y;
		dx = ep.x - sp.x;
		dy = ep.y - sp.y;
		rect.width = dx;
		rect.height = dy;
		if (dx > 0 && dy > 0) {
			temp.copyTo(image);
			rectangle(image, rect, Scalar(0, 0, 255), 1, LINE_AA, 0);
			imshow("������", image);
		}
		break;
	}
}

void QuickDemo::mouse_drawing_demo(Mat& image)
{
	namedWindow("������", WINDOW_AUTOSIZE);
	setMouseCallback("������", mouse, &image);
	imshow("������", image);
	temp = image.clone();
}

void QuickDemo::norm_demo(Mat& image)
{
	Mat dst;
	std::cout << image.type() << std::endl;
	image.convertTo(image, CV_32F);
	std::cout << image.type() << std::endl;
	// ��һ������
	normalize(image, dst, 1.0, 0, NORM_MINMAX);
	std::cout << dst.type() << std::endl;
	imshow("ͼ��Ĺ�һ��", dst);//��ʾ��һ����ͼ��
}

void QuickDemo::resize_demo(Mat& image)
{
	Mat dst;
	resize(image, dst, Size(image.rows / 2, image.cols / 2), 0, 0, INTER_LINEAR);
	imshow("resize", dst);
}

void QuickDemo::flip_demo(Mat& image)
{
	Mat dst;
	flip(image, dst, 0);//���·�ת x�Գ�
	flip(image, dst, 1);//���ҷ�ת y�Գ�
	flip(image, dst, -1);//��ת180��
	imshow("flip", dst);
}

void QuickDemo::rotate_demo(Mat& image)
{
	Mat dst, m;
	int h = image.rows;// �߶�
	int w = image.cols;// ���
	m = getRotationMatrix2D(Point(w / 2, h / 2), 20, 1.0);
	double cos = abs(m.at<double>(0, 0));
	double sin = abs(m.at<double>(0, 1));
	int nw = cos * w + sin * h;
	int nh = sin * w + cos * h;
	m.at<double>(0, 2) += (nw / 2 - w / 2);
	m.at<double>(1, 2) += (nh / 2 - h / 2);
	warpAffine(image, dst, m, Size(nw, nh));
	imshow("rotate", dst);
}

void QuickDemo::video_demo(Mat& image)
{
	// ������ͷ
	//VideoCapture capture(0);
	// ����Ƶ�ļ�
	VideoCapture capture("C:/project/c/opencv/project/opencv_01/images/balltest.mp4");
	int width = capture.get(CAP_PROP_FRAME_WIDTH);// ���
	int height = capture.get(CAP_PROP_FRAME_HEIGHT);// �߶�
	int count = capture.get(CAP_PROP_FRAME_COUNT);// ֡��
	int fps = capture.get(CAP_PROP_FPS);//FPS
	int fourcc = capture.get(CAP_PROP_FOURCC);//FOURCC
	std::cout << "width:" << width << std::endl;
	std::cout << "height:" << height << std::endl;
	std::cout << "count:" << count << std::endl;
	std::cout << "fps:" << fps << std::endl;
	std::cout << "fourcc:" << fourcc << std::endl;
	VideoWriter write("C:/project/c/opencv/project/opencv_01/images/write.mp4", fourcc, fps,
		Size(width, height), true);
	Mat frame;
	while (true)
	{
		capture.read(frame);
		if (frame.empty())break;

		imshow("camera", frame);
		write.write(frame);
		int c = waitKey(100);
		if (c == 27)break;
	}
	// �ͷ���Դ
	capture.release();
	write.release();
}

void QuickDemo::show_histogram_demo(Mat& image)
{
	std::vector<Mat> bgr_plane;
	split(image, bgr_plane);
	// �����������
	const int channel[1] = { 0 };
	const int bins[1] = { 256 };
	float hranges[2] = { 0,255 };
	const float* ranges[1] = { hranges };
	Mat b_hist;
	Mat g_hist;
	Mat r_hist;
	// ����Blue,green,redͨ����ֱ��ͼ
	calcHist(&bgr_plane[0], 1, 0, Mat(), b_hist, 1, bins, ranges);
	calcHist(&bgr_plane[1], 1, 0, Mat(), g_hist, 1, bins, ranges);
	calcHist(&bgr_plane[2], 1, 0, Mat(), r_hist, 1, bins, ranges);
	// ��ʾֱ��ͼ
	int hist_w = 512;
	int hist_h = 400;
	int bin_w = cvRound((double)hist_w / bins[0]);
	Mat histogram = Mat::zeros(hist_w, hist_h, CV_8SC3);
	// ��һ��ֱ��ͼ����
	normalize(b_hist, b_hist, 0, histogram.rows, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, histogram.rows, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, histogram.rows, NORM_MINMAX, -1, Mat());
	// ����ֱ��ͼ����
	for (int i = 0; i < bins[0]; i++)
	{
		line(histogram, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), 2, LINE_AA, 0);
		line(histogram, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(0, 255, 0), 2, LINE_AA, 0);
		line(histogram, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(0, 0, 255), 2, LINE_AA, 0);
	}
	namedWindow("histogram");
	imshow("histogram", histogram);
}

void QuickDemo::histogram_2d_demo(Mat& image) {
	// 2D ֱ��ͼ
	Mat hsv, hs_hist;
	cvtColor(image, hsv, COLOR_BGR2HSV);
	int hbins = 30, sbins = 32;
	int hist_bins[] = { hbins, sbins };
	float h_range[] = { 0, 180 };
	float s_range[] = { 0, 256 };
	const float* hs_ranges[] = { h_range, s_range };
	int hs_channels[] = { 0, 1 };
	calcHist(&hsv, 1, hs_channels, Mat(), hs_hist, 2, hist_bins, hs_ranges, true, false);
	double maxVal = 0;
	minMaxLoc(hs_hist, 0, &maxVal, 0, 0);
	int scale = 10;
	Mat hist2d_image = Mat::zeros(sbins * scale, hbins * scale, CV_8UC3);
	for (int h = 0; h < hbins; h++) {
		for (int s = 0; s < sbins; s++)
		{
			float binVal = hs_hist.at<float>(h, s);
			int intensity = cvRound(binVal * 255 / maxVal);
			rectangle(hist2d_image, Point(h * scale, s * scale),
				Point((h + 1) * scale - 1, (s + 1) * scale - 1),
				Scalar::all(intensity),
				-1);
		}
	}
	applyColorMap(hist2d_image, hist2d_image, COLORMAP_JET);
	imshow("H-S Histogram", hist2d_image);
	imwrite("E:/hist_2d.png", hist2d_image);
}

void QuickDemo::histogram_eq_demo(Mat& image) {
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	imshow("�Ҷ�ͼ��", gray);
	Mat dst;
	equalizeHist(gray, dst);
	imshow("ֱ��ͼ���⻯��ʾ", dst);
}

// ģ��Ч��	
void QuickDemo::blur_demo(Mat& image) {
	Mat dst;
	// 3*3 �ľ������-1��-1������
	blur(image, dst, Size(3, 3), Point(-1, -1));
	imshow("ͼ��ģ��", dst);
}

// ��˹ģ��	
void QuickDemo::gaussian_blur_demo(Mat& image) {
	Mat dst;
	// 3*3 �ľ������-1��-1������
	GaussianBlur(image, dst, Size(3, 3), 15);
	imshow("��˹ģ��", dst);
}

void QuickDemo::bifilter_demo(Mat& image) {
	Mat dst;
	namedWindow("˫��ģ��", WINDOW_FREERATIO);
	bilateralFilter(image, dst, 0, 100, 10);
	imshow("˫��ģ��", dst);
}

void QuickDemo::face_detection_demo() {
	std::string root_dir = "D:/opencv-4.4.0/opencv/sources/samples/dnn/face_detector/";
	dnn::Net net = dnn::readNetFromTensorflow(root_dir + "opencv_face_detector_uint8.pb", root_dir + "opencv_face_detector.pbtxt");
	VideoCapture capture("D:/images/video/example_dsh.mp4");
	Mat frame;
	while (true) {
		capture.read(frame);
		if (frame.empty()) {
			break;
		}
		Mat blob = dnn::blobFromImage(frame, 1.0, Size(300, 300), Scalar(104, 177, 123), false, false);
		net.setInput(blob);// NCHW
		Mat probs = net.forward(); // 
		Mat detectionMat(probs.size[2], probs.size[3], CV_32F, probs.ptr<float>());
		// �������
		for (int i = 0; i < detectionMat.rows; i++) {
			float confidence = detectionMat.at<float>(i, 2);
			if (confidence > 0.5) {
				int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
				int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
				int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
				int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);
				Rect box(x1, y1, x2 - x1, y2 - y1);
				rectangle(frame, box, Scalar(0, 0, 255), 2, 8, 0);
			}
		}
		imshow("���������ʾ", frame);
		int c = waitKey(1);
		if (c == 27) { // �˳�
			break;
		}
	}
}
// ��Ҫ����ͷ�ļ� #include <iostream> #include <fstream>
std::vector<std::string> loadLabel(std::string str) {
	vector<string> className;
	ifstream fp(str);
	string name;
	while (!fp.eof()) {
		// ���ж�ȡ
		getline(fp, name);
		if (name.length()) {
			className.push_back(name);
		}
	}
	fp.close();
	return className;
}

void QuickDemo::loadCaffe(Mat& image) {

	// ģ��
	std::string model = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/caffe/MobileNetSSD_deploy.caffemodel";
	// ����
	std::string config = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/caffe/MobileNetSSD_deploy.prototxt";
	// ���
	std::string detection_class = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/caffe/object_detection_classes_pascal_voc.txt";
	// ���� caffe ģ�� ���ַ�ʽ
	//Net net = readNet(model, config, "caffe");
	Net net = readNetFromCaffe(config, model);
	// ���ü�����
	net.setPreferableBackend(DNN_BACKEND_OPENCV);
	// ���ü���Ŀ��
	net.setPreferableTarget(DNN_TARGET_CPU);
	// ��ȡ�ֲ���Ϣ
	std::vector<String> layer = net.getLayerNames();
	for (int i = 0; i < layer.size(); i++) {
		int id = net.getLayerId(layer[i]);
		Ptr<Layer> p_layer = net.getLayer(id);
		printf("layer id:%d,type:%s,name:%s \n", id, p_layer->type.c_str(), p_layer->name.c_str());
	}
	// ��������
	Mat blob = blobFromImage(image, 0.007843, Size(300, 300), Scalar(127.5, 127.5, 127.5), false, false);
	net.setInput(blob);

	// ����,Ĭ�������һ�� detection_out 1,1,100,7 
	Mat forward = net.forward("detection_out");
	printf("forward:%d,%d,%d,%d \n", forward.size[0], forward.size[1], forward.size[2], forward.size[3]);
	// 1,1,100,7 
	// ���룺[1x3x300x300] �����[1x1x100x7], 7 ��Ӧ�ĸ����������£� [image_id, label, conf, x_min, y_min, x_max, y_max]
	Mat detection(forward.size[2], forward.size[3], CV_32F, forward.ptr<float>());
	// ��ȡ�������
	vector<string> label = loadLabel(detection_class);
	for (int i = 0; i < detection.rows; i++) {
		float conf = detection.at<float>(i, 2);
		printf("conf:%f \n", conf);
		// ʶ�𾫶�
		if (conf > 0.5) {
			int index = detection.at<float>(i, 1);
			float min_x = detection.at<float>(i, 3) * image.cols;
			float min_y = detection.at<float>(i, 4) * image.rows;
			float max_x = detection.at<float>(i, 5) * image.cols;
			float max_y = detection.at<float>(i, 6) * image.rows;
			Rect box(min_x, min_y, max_x - min_x, max_y - min_y);
			rectangle(image, box, Scalar(0, 0, 255), 2, LINE_AA, 0);
			// ����1 Mat����
			// ����2 label ��ǩ
			// ����3 ��ʼ���꣬ȡbox���Ͻ����� top_left
			// ����4 ����
			// ����5 ��������
			// ����6 ������ɫ BGR
			// ����7 �����߿�
			putText(image, format("conf:%.2f,%s", conf, label[index - 1].c_str()), box.tl(), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(255, 0, 0),
				1, LINE_AA);
			printf("conf:%f,%d,%d", conf, box.x, box.y);
		}
	}
	namedWindow("caffe detection", WINDOW_KEEPRATIO);
	imshow("caffe detection", image);
}

void QuickDemo::loadTensorFlow(Mat& image) {

	// ģ��
	std::string model = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/TensorFlow/frozen_inference_graph.pb";
	// ����
	std::string config = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/TensorFlow/ssd_mobilenet_v2_coco_2018_03_29.pbtxt";
	// ���
	std::string detection_class = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/TensorFlow/object_detection_classes_coco.txt";
	// ���� caffe ģ�� ���ַ�ʽ
	Net net = readNetFromTensorflow(model, config);
	// ���ü�����
	net.setPreferableBackend(DNN_BACKEND_OPENCV);
	// ���ü���Ŀ��
	net.setPreferableTarget(DNN_TARGET_CPU);
	// ��ȡ�ֲ���Ϣ
	std::vector<String> layer = net.getLayerNames();
	for (int i = 0; i < layer.size(); i++) {
		int id = net.getLayerId(layer[i]);
		Ptr<Layer> p_layer = net.getLayer(id);
		printf("layer id:%d,type:%s,name:%s \n", id, p_layer->type.c_str(), p_layer->name.c_str());
	}
	// ��������
	Mat blob = blobFromImage(image, 1.0, Size(300, 300), Scalar(0, 0, 0), true, false);
	net.setInput(blob);

	// ����,Ĭ�������һ�� detection_out 1,1,100,7 
	Mat forward = net.forward("detection_out");
	printf("forward:%d,%d,%d,%d \n", forward.size[0], forward.size[1], forward.size[2], forward.size[3]);
	// 1,1,100,7 
	// ���룺[1x3x300x300] �����[1x1x100x7], 7 ��Ӧ�ĸ����������£� [image_id, label, conf, x_min, y_min, x_max, y_max]
	Mat detection(forward.size[2], forward.size[3], CV_32F, forward.ptr<float>());
	// ��ȡ�������
	vector<string> label = loadLabel(detection_class);
	for (int i = 0; i < detection.rows; i++) {
		float conf = detection.at<float>(i, 2);
		printf("conf:%f \n", conf);
		// ʶ�𾫶�
		if (conf > 0.5) {
			int index = detection.at<float>(i, 1);
			float min_x = detection.at<float>(i, 3) * image.cols;
			float min_y = detection.at<float>(i, 4) * image.rows;
			float max_x = detection.at<float>(i, 5) * image.cols;
			float max_y = detection.at<float>(i, 6) * image.rows;
			Rect box(min_x, min_y, max_x - min_x, max_y - min_y);
			rectangle(image, box, Scalar(0, 0, 255), 2, LINE_8, 0);
			// ����1 Mat����
			// ����2 label ��ǩ
			// ����3 ��ʼ���꣬ȡbox���Ͻ����� top_left
			// ����4 ����
			// ����5 ��������
			// ����6 ������ɫ BGR
			// ����7 �����߿�
			putText(image, format("conf:%.2f,%s", conf, label[index - 1].c_str()), box.tl(), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(255, 0, 0),
				1, LINE_8);
			printf("conf:%f,%d,%d", conf, box.x, box.y);
		}
	}
	namedWindow("TensorFlow detection", WINDOW_KEEPRATIO);
	imshow("TensorFlow detection", image);
}

// NV21 ת��RGB ����
void QuickDemo::nv2rgb(Mat& image)
{
	int width = 1920;
	int height = 1080;
	FILE* file = fopen("C:/project/c/opencv/project/opencv_01/images/camera_mark.jpg", "rb+");
	// NV21 ת��RGB ����
	auto* yuvBuf = new unsigned char[width * height * 3 / 2];
	fread(yuvBuf, height * 3 / 2, width, file);
	cv::Mat yuvMat = cv::Mat(height * 3 / 2, width, CV_8UC1, yuvBuf);
	cv::Mat rgbMat = cv::Mat(height, width, CV_8UC3);
	cv::cvtColor(yuvMat, rgbMat, COLOR_YUV2BGR_NV21);

	namedWindow("nv2rgb", WINDOW_KEEPRATIO);
	imshow("nv2rgb", rgbMat);
}
