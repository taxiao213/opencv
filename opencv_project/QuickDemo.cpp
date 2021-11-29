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

void QuickDemo::matCreate(Mat mat) {
	Mat m1, m2;
	// 克隆
	m1 = mat.clone();
	// 复制
	mat.copyTo(m2);
	// 创建空白图像 三种方式 
	// CV_8UC3  表示 8位 U 无符号 C char类型 3(3通道)
	Mat m3 = Mat::zeros(Size(50, 50), CV_8UC3);
	//Mat m3 = Mat::ones(Size(50,50), CV_8UC3);
	//Mat m3 = Mat::eye(Size(50,50), CV_8UC3);
	// 赋值操作
	m3 = Scalar(0, 0, 255);
	// 获取宽度 高度 通道数
	std::cout << "width:" << m3.cols << " height:" << m3.rows << "channel: " << m3.channels() << std::endl;
	std::cout << m3 << std::endl;
	imshow("空白图像", m3);
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
				// 灰度图像
				int pv = dist.at<char>(row, col);
				dist.at<char>(row, col) = 255 - pv;
			}
			if (channel == 3) {
				// 彩色图像
				Vec3b rgb = dist.at<Vec3b>(row, col);
				dist.at<Vec3b>(row, col)[0] = 255 - rgb[0];
				dist.at<Vec3b>(row, col)[1] = 255 - rgb[1];
				dist.at<Vec3b>(row, col)[2] = 255 - rgb[2];
			}
		}
	}
	namedWindow("像素转变", WINDOW_FREERATIO);
	imshow("像素转变", dist);
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
		uchar* current_ptr = dist.ptr(row);// 获取指针
		for (int col = 0; col < w; col++) {
			if (channel == 1) {
				// 灰度图像
				int pv = *current_ptr;
				dist.at<char>(row, col) = 255 - pv;
			}
			if (channel == 3) {
				// 彩色图像
				*current_ptr++ = 255 - *current_ptr;
				*current_ptr++ = 255 - *current_ptr;
				*current_ptr++ = 255 - *current_ptr;
			}
		}
	}
	namedWindow("像素转变 指针", WINDOW_FREERATIO);
	imshow("像素转变 指针", dist);
}

void QuickDemo::pixelScale(Mat mat)
{
	Mat copy;
	Mat dst;
	copy = Mat::zeros(mat.size(), mat.type());
	copy = Scalar(2, 2, 2);
	//add(mat, copy, dst);//加
	//subtract(mat, copy, dst);//减
	//multiply(mat, copy, dst);//乘
	divide(mat, copy, dst);//除
	imshow("pixelScale", dst);
}


// 亮度调整 image*1.0 + m*0.0 + b
static void on_lightness(int b, void* userdata)
{
	Mat image = *((Mat*)userdata);
	Mat dst = Mat::zeros(image.size(), image.type());
	Mat m = Mat::zeros(image.size(), image.type());
	addWeighted(image, 1.0, m, 0, b, dst);
	imshow("亮度调整", dst);
}

// 对比度调整 image*contrast + m*0.0 + 0
static void on_contrast(int b, void* userdata)
{
	Mat image = *((Mat*)userdata);
	Mat dst = Mat::zeros(image.size(), image.type());
	Mat m = Mat::zeros(image.size(), image.type());
	double contrast = b / 100.0;
	addWeighted(image, contrast, m, 0.0, 0, dst);
	imshow("亮度调整", dst);
}

void QuickDemo::tracking_bar_demo(Mat& image)
{
	namedWindow("滚动条", WINDOW_AUTOSIZE);
	int max_value = 100;
	int lightness = 50;
	int contrast_max_value = 200;
	int contrast_value = 100;
	createTrackbar("Value Bar:", "滚动条", &lightness, max_value, on_lightness, (void*)(&image));
	createTrackbar("Contrast Bar:", "滚动条", &contrast_value, contrast_max_value, on_contrast, (void*)(&image));
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
			// 按下 1 时保存图片到指定位置
			std::cout << key << std::endl;
			imwrite("C:/project/c/opencv/project/opencv_01/images/save/imwrite.png", dst);
		}
		applyColorMap(mat, dst, colorMap[index % 19]);
		//循环展示
		index++;
		imshow("循环播放图", dst);
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
	//bitwise_and(m1, m2, dst); //位操作与
	bitwise_or(m1, m2, dst); //位操作或
	//bitwise_not(m1, m2, dst); //取反操作
	//bitwise_xor(m1, m2, dst); //异或操作
	imshow("位操作", dst);
}

void QuickDemo::channels_demo(Mat& image)
{
	std::vector<Mat> mv;
	split(image, mv);
	imshow("蓝色", mv[0]);
	imshow("绿色", mv[1]);
	imshow("红色", mv[2]);
	//0,1,2三个通道分别代表BGR。
	//关闭2个通道意味着开启一个通道。
	Mat dst = Mat::zeros(image.size(), image.type());
	mv[0] = 0;
	mv[2] = 0;
	merge(mv, dst);
	imshow("合并通道", dst);

	int from_to[] = { 0,2,1,1,2,0 };
	//把通道相互交换，第0->第2，第一->第一，第二->第0
	mixChannels(&image, 1, &dst, 1, from_to, 3);//3表示3个通道
	//参数1指针引用图像->参数2引用到dst
	imshow("通道混合", dst);
}

void QuickDemo::inrange_demo(Mat& image)
{
	Mat hsv;
	cvtColor(image, hsv, COLOR_BGR2HSV);
	Mat mask;
	inRange(hsv, Scalar(35, 43, 46), Scalar(77, 255, 255), mask);
	//35,43,46 根据图片中绿色最低来确定最小值。
	//77,255,255 根据图片中绿色最高来确定最大值。 
	//参数1低范围，参数2高范围
	//将hsv中的由低到高的像素点提取出来并且存储到mask当中。
	//像素为0是黑色，1是白色
	imshow("mask", hsv);
	Mat redback = Mat::zeros(image.size(), image.type());
	redback = Scalar(255, 0, 0);
	bitwise_not(mask, mask);
	imshow("mask", mask);
	image.copyTo(redback, mask);//把redback复制到mask，mask通过inRange得到。
	imshow("roi提取", hsv);
	imshow("roi提取1", redback);
}

void QuickDemo::pixel_statistic_demo(Mat& image)
{
	double minv, maxv;//定义最值
	Point minLoc, maxLoc;//定义最值地址
	std::vector<Mat>mv;//mv是一个Mat类型的容器 装在这个容器内
	split(image, mv);
	for (int i = 0; i < mv.size(); i++)
	{
		//分别打印各个通道的数值
		minMaxLoc(mv[i], &minv, &maxv, &minLoc, &maxLoc);//求出图像的最大值和最小值。
		std::cout << "No.channels:" << i << " minvalue:" << minv << " maxvalue:" << maxv << std::endl;
	}
	Mat mean, stddev;
	meanStdDev(image, mean, stddev);//求出图像的均值和方差
	std::cout << "mean:" << mean << std::endl;
	std::cout << "stddev:" << stddev << std::endl;
}

void QuickDemo::drawing_demo(Mat& image)
{
	Mat bg = Mat::zeros(image.size(), image.type());
	// 矩形
	rectangle(bg, Rect(100, 100, 60, 60), Scalar(0, 0, 255), -1, LINE_8, 0);
	// 圆
	circle(image, Point(100, 100), 100, Scalar(255, 0, 0), 0, LINE_8, 0);
	// 直线
	line(image, Point(100, 100), Point(200, 200), Scalar(0, 255, 0), 3, LINE_8, 0);
	// 椭圆
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
		//canvas = Scalar(0, 0, 0); // 打开后只显示绘制一个线条
		line(canvas, Point(x1, y1), Point(x2, y2), Scalar(b, g, r), 8, LINE_AA, 0);//line_AA表示去掉锯齿 
		imshow("随机绘制演示", canvas);
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
	// 线宽为负数 填充
	drawContours(canvas, construct, -1, Scalar(0, 0, 255), -1, LINE_AA);
	imshow("多边形", canvas);
}


Point sp(-1, -1);//鼠标的开始的位置
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
			imshow("鼠标绘制", image);
			sp.x = -1;
			sp.y = -1;//复位，为下一次做准备
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
			imshow("鼠标绘制", image);
		}
		break;
	}
}

void QuickDemo::mouse_drawing_demo(Mat& image)
{
	namedWindow("鼠标绘制", WINDOW_AUTOSIZE);
	setMouseCallback("鼠标绘制", mouse, &image);
	imshow("鼠标绘制", image);
	temp = image.clone();
}

void QuickDemo::norm_demo(Mat& image)
{
	Mat dst;
	std::cout << image.type() << std::endl;
	image.convertTo(image, CV_32F);
	std::cout << image.type() << std::endl;
	// 归一化处理
	normalize(image, dst, 1.0, 0, NORM_MINMAX);
	std::cout << dst.type() << std::endl;
	imshow("图像的归一化", dst);//显示归一化的图像
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
	flip(image, dst, 0);//上下翻转 x对称
	flip(image, dst, 1);//左右翻转 y对称
	flip(image, dst, -1);//旋转180°
	imshow("flip", dst);
}

void QuickDemo::rotate_demo(Mat& image)
{
	Mat dst, m;
	int h = image.rows;// 高度
	int w = image.cols;// 宽度
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
	// 打开摄像头
	//VideoCapture capture(0);
	// 打开视频文件
	VideoCapture capture("C:/project/c/opencv/project/opencv_01/images/balltest.mp4");
	int width = capture.get(CAP_PROP_FRAME_WIDTH);// 宽度
	int height = capture.get(CAP_PROP_FRAME_HEIGHT);// 高度
	int count = capture.get(CAP_PROP_FRAME_COUNT);// 帧数
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
	// 释放资源
	capture.release();
	write.release();
}

void QuickDemo::show_histogram_demo(Mat& image)
{
	std::vector<Mat> bgr_plane;
	split(image, bgr_plane);
	// 定义参数变量
	const int channel[1] = { 0 };
	const int bins[1] = { 256 };
	float hranges[2] = { 0,255 };
	const float* ranges[1] = { hranges };
	Mat b_hist;
	Mat g_hist;
	Mat r_hist;
	// 计算Blue,green,red通道的直方图
	calcHist(&bgr_plane[0], 1, 0, Mat(), b_hist, 1, bins, ranges);
	calcHist(&bgr_plane[1], 1, 0, Mat(), g_hist, 1, bins, ranges);
	calcHist(&bgr_plane[2], 1, 0, Mat(), r_hist, 1, bins, ranges);
	// 显示直方图
	int hist_w = 512;
	int hist_h = 400;
	int bin_w = cvRound((double)hist_w / bins[0]);
	Mat histogram = Mat::zeros(hist_w, hist_h, CV_8SC3);
	// 归一化直方图数据
	normalize(b_hist, b_hist, 0, histogram.rows, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, histogram.rows, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, histogram.rows, NORM_MINMAX, -1, Mat());
	// 绘制直方图曲线
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
	// 2D 直方图
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
	imshow("灰度图像", gray);
	Mat dst;
	equalizeHist(gray, dst);
	imshow("直方图均衡化演示", dst);
}

// 模糊效果	
void QuickDemo::blur_demo(Mat& image) {
	Mat dst;
	// 3*3 的卷积，（-1，-1）中心
	blur(image, dst, Size(3, 3), Point(-1, -1));
	imshow("图像模糊", dst);
}

// 高斯模糊	
void QuickDemo::gaussian_blur_demo(Mat& image) {
	Mat dst;
	// 3*3 的卷积，（-1，-1）中心
	GaussianBlur(image, dst, Size(3, 3), 15);
	imshow("高斯模糊", dst);
}

void QuickDemo::bifilter_demo(Mat& image) {
	Mat dst;
	namedWindow("双边模糊", WINDOW_FREERATIO);
	bilateralFilter(image, dst, 0, 100, 10);
	imshow("双边模糊", dst);
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
		// 解析结果
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
		imshow("人脸检测演示", frame);
		int c = waitKey(1);
		if (c == 27) { // 退出
			break;
		}
	}
}
// 需要导入头文件 #include <iostream> #include <fstream>
std::vector<std::string> loadLabel(std::string str) {
	vector<string> className;
	ifstream fp(str);
	string name;
	while (!fp.eof()) {
		// 整行读取
		getline(fp, name);
		if (name.length()) {
			className.push_back(name);
		}
	}
	fp.close();
	return className;
}

void QuickDemo::loadCaffe(Mat& image) {

	// 模型
	std::string model = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/caffe/MobileNetSSD_deploy.caffemodel";
	// 配置
	std::string config = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/caffe/MobileNetSSD_deploy.prototxt";
	// 类别
	std::string detection_class = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/caffe/object_detection_classes_pascal_voc.txt";
	// 加载 caffe 模型 两种方式
	//Net net = readNet(model, config, "caffe");
	Net net = readNetFromCaffe(config, model);
	// 设置计算后端
	net.setPreferableBackend(DNN_BACKEND_OPENCV);
	// 设置计算目标
	net.setPreferableTarget(DNN_TARGET_CPU);
	// 获取分层信息
	std::vector<String> layer = net.getLayerNames();
	for (int i = 0; i < layer.size(); i++) {
		int id = net.getLayerId(layer[i]);
		Ptr<Layer> p_layer = net.getLayer(id);
		printf("layer id:%d,type:%s,name:%s \n", id, p_layer->type.c_str(), p_layer->name.c_str());
	}
	// 构建输入
	Mat blob = blobFromImage(image, 0.007843, Size(300, 300), Scalar(127.5, 127.5, 127.5), false, false);
	net.setInput(blob);

	// 推理,默认是最后一个 detection_out 1,1,100,7 
	Mat forward = net.forward("detection_out");
	printf("forward:%d,%d,%d,%d \n", forward.size[0], forward.size[1], forward.size[2], forward.size[3]);
	// 1,1,100,7 
	// 输入：[1x3x300x300] 输出：[1x1x100x7], 7 对应的浮点数据如下： [image_id, label, conf, x_min, y_min, x_max, y_max]
	Mat detection(forward.size[2], forward.size[3], CV_32F, forward.ptr<float>());
	// 读取类别名称
	vector<string> label = loadLabel(detection_class);
	for (int i = 0; i < detection.rows; i++) {
		float conf = detection.at<float>(i, 2);
		printf("conf:%f \n", conf);
		// 识别精度
		if (conf > 0.5) {
			int index = detection.at<float>(i, 1);
			float min_x = detection.at<float>(i, 3) * image.cols;
			float min_y = detection.at<float>(i, 4) * image.rows;
			float max_x = detection.at<float>(i, 5) * image.cols;
			float max_y = detection.at<float>(i, 6) * image.rows;
			Rect box(min_x, min_y, max_x - min_x, max_y - min_y);
			rectangle(image, box, Scalar(0, 0, 255), 2, LINE_AA, 0);
			// 参数1 Mat对象
			// 参数2 label 标签
			// 参数3 开始坐标，取box左上角坐标 top_left
			// 参数4 字体
			// 参数5 字体缩放
			// 参数6 字体颜色 BGR
			// 参数7 绘制线宽
			putText(image, format("conf:%.2f,%s", conf, label[index - 1].c_str()), box.tl(), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(255, 0, 0),
				1, LINE_AA);
			printf("conf:%f,%d,%d", conf, box.x, box.y);
		}
	}
	namedWindow("caffe detection", WINDOW_KEEPRATIO);
	imshow("caffe detection", image);
}

void QuickDemo::loadTensorFlow(Mat& image) {

	// 模型
	std::string model = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/TensorFlow/frozen_inference_graph.pb";
	// 配置
	std::string config = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/TensorFlow/ssd_mobilenet_v2_coco_2018_03_29.pbtxt";
	// 类别
	std::string detection_class = "C:/project/opencv/opencv_windows/opencv/sources/samples/dnn/TensorFlow/object_detection_classes_coco.txt";
	// 加载 caffe 模型 两种方式
	Net net = readNetFromTensorflow(model, config);
	// 设置计算后端
	net.setPreferableBackend(DNN_BACKEND_OPENCV);
	// 设置计算目标
	net.setPreferableTarget(DNN_TARGET_CPU);
	// 获取分层信息
	std::vector<String> layer = net.getLayerNames();
	for (int i = 0; i < layer.size(); i++) {
		int id = net.getLayerId(layer[i]);
		Ptr<Layer> p_layer = net.getLayer(id);
		printf("layer id:%d,type:%s,name:%s \n", id, p_layer->type.c_str(), p_layer->name.c_str());
	}
	// 构建输入
	Mat blob = blobFromImage(image, 1.0, Size(300, 300), Scalar(0, 0, 0), true, false);
	net.setInput(blob);

	// 推理,默认是最后一个 detection_out 1,1,100,7 
	Mat forward = net.forward("detection_out");
	printf("forward:%d,%d,%d,%d \n", forward.size[0], forward.size[1], forward.size[2], forward.size[3]);
	// 1,1,100,7 
	// 输入：[1x3x300x300] 输出：[1x1x100x7], 7 对应的浮点数据如下： [image_id, label, conf, x_min, y_min, x_max, y_max]
	Mat detection(forward.size[2], forward.size[3], CV_32F, forward.ptr<float>());
	// 读取类别名称
	vector<string> label = loadLabel(detection_class);
	for (int i = 0; i < detection.rows; i++) {
		float conf = detection.at<float>(i, 2);
		printf("conf:%f \n", conf);
		// 识别精度
		if (conf > 0.5) {
			int index = detection.at<float>(i, 1);
			float min_x = detection.at<float>(i, 3) * image.cols;
			float min_y = detection.at<float>(i, 4) * image.rows;
			float max_x = detection.at<float>(i, 5) * image.cols;
			float max_y = detection.at<float>(i, 6) * image.rows;
			Rect box(min_x, min_y, max_x - min_x, max_y - min_y);
			rectangle(image, box, Scalar(0, 0, 255), 2, LINE_8, 0);
			// 参数1 Mat对象
			// 参数2 label 标签
			// 参数3 开始坐标，取box左上角坐标 top_left
			// 参数4 字体
			// 参数5 字体缩放
			// 参数6 字体颜色 BGR
			// 参数7 绘制线宽
			putText(image, format("conf:%.2f,%s", conf, label[index - 1].c_str()), box.tl(), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(255, 0, 0),
				1, LINE_8);
			printf("conf:%f,%d,%d", conf, box.x, box.y);
		}
	}
	namedWindow("TensorFlow detection", WINDOW_KEEPRATIO);
	imshow("TensorFlow detection", image);
}

// NV21 转成RGB 数据
void QuickDemo::nv2rgb(Mat& image)
{
	int width = 1920;
	int height = 1080;
	FILE* file = fopen("C:/project/c/opencv/project/opencv_01/images/camera_mark.jpg", "rb+");
	// NV21 转成RGB 数据
	auto* yuvBuf = new unsigned char[width * height * 3 / 2];
	fread(yuvBuf, height * 3 / 2, width, file);
	cv::Mat yuvMat = cv::Mat(height * 3 / 2, width, CV_8UC1, yuvBuf);
	cv::Mat rgbMat = cv::Mat(height, width, CV_8UC3);
	cv::cvtColor(yuvMat, rgbMat, COLOR_YUV2BGR_NV21);

	namedWindow("nv2rgb", WINDOW_KEEPRATIO);
	imshow("nv2rgb", rgbMat);
}
