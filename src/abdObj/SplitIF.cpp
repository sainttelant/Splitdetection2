#include "SplitIF.hpp"
#include "UA-DETRAC.h"
// iou relevant
#include "FrameDiff.h"


#define yolov5 0
#define debug 0
#define DISPLAY 0

// ��960X720 �ȱ���С3��
#define RESIZE_WIDTH 960
#define RESIZE_HEIGHT 720
#define CHECK_INTERVAL 10

// define use video or rtsp
#define LowVersionOpencv 1
#define UsingOpenvx 1
#define READIMGONLY 0
#define RTSP 1
using namespace cv;
//using namespace std;


//Some constants for the algorithm
const double pi = 3.142;
const double cthr = 0.00001;
const double alpha = 0.002;
const double cT = 0.05;
const double covariance0 = 11.0;
const double cf = 0.1;
const double cfbar = 1.0 - cf;
const double temp_thr = 9.0 * covariance0 * covariance0;
const double prune = -alpha * cT;
const double alpha_bar = 1.0 - alpha;
//Temperory variable
int overall = 0;

//Structure used for saving various components for each pixel
struct gaussian
{
	double mean[3], covariance;
	double weight;								// Represents the measure to which a particular component defines the pixel value
	gaussian* Next;
	gaussian* Previous;
} *ptr, * start, * rear, * g_temp, * save, * gnext, * previous, * nptr, * temp_ptr;

struct MYNode
{
	gaussian* pixel_s;
	gaussian* pixel_r;
	int no_of_components;
	MYNode* Next;
} *N_ptr, * N_start, * N_rear;




struct Node1
{
	cv::Mat gauss;
	int no_of_comp;
	Node1* Next;
} *N1_ptr, * N1_start, * N1_rear;



//Some function associated with the structure management
 MYNode* Create_Node(double info1, double info2, double info3);
void Insert_End_Node(MYNode* np);
gaussian* Create_gaussian(double info1, double info2, double info3);

std::vector<std::string> LoadNames(const std::string& path) {
	// load class names
	std::vector<std::string> class_names;
	std::ifstream infile(path);
	if (infile.is_open()) {
		std::string line;
		while (getline(infile, line)) {
			class_names.emplace_back(line);
		}
		infile.close();
	}
	else {
		std::cerr << "Error loading the class names!\n";
	}

	return class_names;
}


void Demo(cv::Mat& img,
	const std::vector<std::tuple<cv::Rect, float, int>>& data_vec,
	const std::vector<std::string>& class_names,
	bool label = true) {
	for (const auto& data : data_vec) {
		cv::Rect box;
		float score;
		int class_idx;
		std::tie(box, score, class_idx) = data;

		cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

		if (label) {
			std::stringstream ss;
			ss << std::fixed << std::setprecision(2) << score;
			std::string s = class_names[class_idx] + " " + ss.str();

			auto font_face = cv::FONT_HERSHEY_DUPLEX;
			auto font_scale = 1.0;
			int thickness = 1;
			int baseline = 0;
			auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
			cv::rectangle(img,
				cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
				cv::Point(box.tl().x + s_size.width, box.tl().y),
				cv::Scalar(0, 0, 255), -1);
			cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
				font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
		}
	}
}


MYNode* Create_Node(double info1, double info2, double info3)
{
	N_ptr = new MYNode;
	if (N_ptr != NULL)
	{
		N_ptr->Next = NULL;
		N_ptr->no_of_components = 1;
		N_ptr->pixel_s = N_ptr->pixel_r = Create_gaussian(info1, info2, info3);
	}
	return N_ptr;
}

gaussian* Create_gaussian(double info1, double info2, double info3)
{
	ptr = new gaussian;
	if (ptr != NULL)
	{
		ptr->mean[0] = info1;
		ptr->mean[1] = info2;
		ptr->mean[2] = info3;
		ptr->covariance = covariance0;
		ptr->weight = alpha;
		ptr->Next = NULL;
		ptr->Previous = NULL;
	}
	return ptr;
}

void Insert_End_Node(MYNode* np)
{
	if (N_start != NULL)
	{
		N_rear->Next = np;
		N_rear = np;
	}
	else
		N_start = N_rear = np;
}

void Insert_End_gaussian(gaussian* nptr)
{
	if (start != NULL)
	{
		rear->Next = nptr;
		nptr->Previous = rear;
		rear = nptr;
	}
	else
		start = rear = nptr;
}

gaussian* Delete_gaussian(gaussian* nptr)
{
	previous = nptr->Previous;
	gnext = nptr->Next;
	if (start != NULL)
	{
		if (nptr == start && nptr == rear)
		{
			start = rear = NULL;
			delete nptr;
		}
		else if (nptr == start)
		{
			gnext->Previous = NULL;
			start = gnext;
			delete nptr;
			nptr = start;
		}
		else if (nptr == rear)
		{
			previous->Next = NULL;
			rear = previous;
			delete nptr;
			nptr = rear;
		}
		else
		{
			previous->Next = gnext;
			gnext->Previous = previous;
			delete nptr;
			nptr = gnext;
		}
	}
	else
	{
		std::cout << "Underflow........";
		//_getch();
		exit(0);
	}
	return nptr;
}

bool judgeInorNot(std::vector<cv::Point2d>& pt, const std::vector<cv::Point2d>& polygons)
{
	if (pt.size() < 4)
	{
		printf("input rect failure!!<<<<<<");
		return false;
	}
	int numofNonintersection = 0;

	for (int j = 0; j < pt.size(); j++)
	{
		int nCross = 0;    // 定义变量，统计目标点向右画射线与多边形相交次�?
		for (int i = 0; i < polygons.size(); i++)
		{   //遍历多边形每一个节�?

			cv::Point2d p1;
			cv::Point2d p2;

			p1 = polygons[i];
			p2 = polygons[(i + 1) % polygons.size()];  // p1是这个节点，p2是下一个节点，两点连线是多边形的一条边
	// 以下算法是用是先以y轴坐标来判断�?

			if (p1.y == p2.y)
				continue;   //如果这条边是水平的，跳过


			if (pt[j].y < min(p1.y, p2.y)) //如果目标点低于这个线段，跳过
				continue;

			if (pt[j].y >= max(p1.y, p2.y)) //如果目标点高于这个线段，跳过
				continue;
			//那么下面的情况就是：如果过p1画水平线，过p2画水平线，目标点在这两条线中�?
			double x = (double)(pt[j].y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;
			// 这段的几何意义是 过目标点，画一条水平线，x是这条线与多边形当前边的交点x坐标
			if (x > pt[j].x)
				nCross++; //如果交点在右边，统计加一。这等于从目标点向右发一条射线（ray），与多边形各边的相交（crossing）次�?
		}

		if (nCross % 2 == 1) {

			return true; //如果是奇数，说明在多边形�?
		}
		else {
			//numofNonintersection++;
			//return false; //否则在多边形�?�?边上
		}
	}
	return false;
}

//CheckMode: 0����ȥ��������1����ȥ��������; NeihborMode��0����4����1����8����;  
void RemoveSmallRegion(Mat& Src, Mat& Dst, int AreaLimit, int CheckMode, int NeihborMode)
{
	int RemoveCount = 0;       //��¼��ȥ�ĸ���  
	//��¼ÿ�����ص����״̬�ı�ǩ��?����δ���?�������ڼ��?2������鲻�ϸ���Ҫ��ת��ɫ����?�������ϸ������? 
	Mat Pointlabel = Mat::zeros(Src.size(), CV_8UC1);

	if (CheckMode == 1)
	{
		std::cout << "Mode: ȥ��С����. ";
		for (int i = 0; i < Src.rows; ++i)
		{
			uchar* iData = Src.ptr<uchar>(i);
			uchar* iLabel = Pointlabel.ptr<uchar>(i);
			for (int j = 0; j < Src.cols; ++j)
			{
				if (iData[j] < 10)
				{
					iLabel[j] = 3;
				}
			}
		}
	}
	else
	{
		std::cout << "Mode: ȥ���׶�. ";
		for (int i = 0; i < Src.rows; ++i)
		{
			uchar* iData = Src.ptr<uchar>(i);
			uchar* iLabel = Pointlabel.ptr<uchar>(i);
			for (int j = 0; j < Src.cols; ++j)
			{
				if (iData[j] > 10)
				{
					iLabel[j] = 3;
				}
			}
		}
	}

	std::vector<Point2i> NeihborPos;  //��¼�����λ��? 
	NeihborPos.push_back(Point2i(-1, 0));
	NeihborPos.push_back(Point2i(1, 0));
	NeihborPos.push_back(Point2i(0, -1));
	NeihborPos.push_back(Point2i(0, 1));
	if (NeihborMode == 1)
	{
		std::cout << "Neighbor mode: 8����." << std::endl;
		NeihborPos.push_back(Point2i(-1, -1));
		NeihborPos.push_back(Point2i(-1, 1));
		NeihborPos.push_back(Point2i(1, -1));
		NeihborPos.push_back(Point2i(1, 1));
	}
	else std::cout << "Neighbor mode: 4����." << std::endl;
	int NeihborCount = 4 + 4 * NeihborMode;
	int CurrX = 0, CurrY = 0;
	//��ʼ���? 
	for (int i = 0; i < Src.rows; ++i)
	{
		uchar* iLabel = Pointlabel.ptr<uchar>(i);
		for (int j = 0; j < Src.cols; ++j)
		{
			if (iLabel[j] == 0)
			{
				//********��ʼ�õ㴦�ļ��?*********  
				std::vector<cv::Point2i> GrowBuffer;                                      //��ջ�����ڴ洢������  
				GrowBuffer.push_back(cv::Point2i(j, i));
				Pointlabel.at<uchar>(i, j) = 1;
				int CheckResult = 0;                                               //�����жϽ�����Ƿ񳬳���С����?Ϊδ������1Ϊ����  

				for (int z = 0; z < GrowBuffer.size(); z++)
				{

					for (int q = 0; q < NeihborCount; q++)                                      //����ĸ������  
					{
						CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
						CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
						if (CurrX >= 0 && CurrX < Src.cols && CurrY >= 0 && CurrY < Src.rows)  //��ֹԽ��  
						{
							if (Pointlabel.at<uchar>(CurrY, CurrX) == 0)
							{
								GrowBuffer.push_back(Point2i(CurrX, CurrY));  //��������buffer  
								Pointlabel.at<uchar>(CurrY, CurrX) = 1;           //���������ļ���ǩ�������ظ����? 
							}
						}
					}
				}
				if (GrowBuffer.size() > AreaLimit) CheckResult = 2;                 //�жϽ�����Ƿ񳬳��޶��Ĵ�С����?Ϊδ������2Ϊ����  
				else { CheckResult = 1;   RemoveCount++; }
				for (int z = 0; z < GrowBuffer.size(); z++)                         //����Label��¼  
				{
					CurrX = GrowBuffer.at(z).x;
					CurrY = GrowBuffer.at(z).y;
					Pointlabel.at<uchar>(CurrY, CurrX) += CheckResult;
				}
				//********�����õ㴦�ļ��?*********  


			}
		}
	}

	CheckMode = 255 * (1 - CheckMode);
	//��ʼ��ת�����С������? 
	for (int i = 0; i < Src.rows; ++i)
	{
		uchar* iData = Src.ptr<uchar>(i);
		uchar* iDstData = Dst.ptr<uchar>(i);
		uchar* iLabel = Pointlabel.ptr<uchar>(i);
		for (int j = 0; j < Src.cols; ++j)
		{
			if (iLabel[j] == 2)
			{
				iDstData[j] = CheckMode;
			}
			else if (iLabel[j] == 3)
			{
				iDstData[j] = iData[j];
			}
		}
	}

	std::cout << RemoveCount << " objects removed." << std::endl;
}

//��ֵ�˲�
Mat myAverage(Mat& srcImage)
{
	
	Mat dstImage = Mat::zeros(srcImage.size(), srcImage.type());
	//Mat mask = Mat::ones(3, 3, srcImage.type());

	for (int k = 1; k < srcImage.rows - 1; k++)
	{
		for (int n = 1; n < srcImage.cols - 1; n++)
		{
			uchar f = 0;
			for (int i = -1; i <= 1; i++)
			{
				for (int j = -1; j <= 1; j++)
				{
					f += srcImage.at<uchar>(k + i, n + j);

				}
			}
			dstImage.at<uchar>(k, n) = uchar(f / 9);
		}
	}
	return dstImage;
}

void removePepperNoise(Mat& mask)
{
	for (int y = 2; y < mask.rows - 2; ++y)
	{
		uchar* pThis = mask.ptr(y);
		uchar* pUp1 = mask.ptr(y - 1);
		uchar* pUp2 = mask.ptr(y - 2);
		uchar* pDown1 = mask.ptr(y + 1);
		uchar* pDown2 = mask.ptr(y + 2);

		pThis += 2; pUp1 += 2; pUp2 += 2; pDown1 += 2; pDown2 += 2;

		int x = 2;
		while (x < mask.cols - 2)
		{
			uchar v = *pThis;
			// ��ǰ��Ϊ��ɫ
			if (v == 0)
			{
				// 5 * 5 ��������
				bool allAbove = *(pUp2 - 2) && *(pUp2 - 1) && *(pUp2) && *(pUp2 + 1) && *(pUp2 + 2);
				bool allBelow = *(pDown2 - 2) && *(pDown2 - 1) && *(pDown2) && *(pDown2 + 1) && *(pDown2 + 2);
				bool allLeft = *(pUp1 - 2) && *(pThis - 2) && *(pDown1 - 2);
				bool allRight = *(pUp1 + 2) && *(pThis + 2) && *(pDown1 + 2);
				bool surroundings = allAbove && allBelow && allLeft && allRight;

				if (surroundings)
				{
					// 5*5 ������ڲ�?*3��С����
					*(pUp1 - 1) = *(pUp1) = *(pUp1 + 1) = 255;
					*(pThis - 1) = *pThis = *(pThis + 1) = 255;
					*(pDown1 - 1) = *pDown1 = *(pDown1 + 1) = 255;
					//(*pThis) = ~(*pThis);
											// 0 ? 255
				}
				pUp2 += 2; pUp1 += 2; pThis += 2; pDown1 += 2; pDown2 += 2;
				x += 2;
			}
			++pThis; ++pUp2; ++pUp1; ++pDown1; ++pDown2; ++x;
		}
	}
}

 SplitObjIF::SplitIF::SplitIF(/* args */)
 :trigger(false)
 , innerframecount(0)
 , count4tracker(0)
 , openvxframe(0)
 , stationary_threshold(0.9f)
 , lazy_threshold(0.7f)
 , sigma_h(0.7f)
 , sigma_iou(0.2f)
 , t_min(3.0f)
	
{

};
    
SplitObjIF::SplitIF::~SplitIF()
{

};

void SplitObjIF::SplitIF::Setdata(SplitObjReceiver inferout)
{
	m_Data.timestamp = inferout.timestamp;
	m_Data.v_inferout = inferout.v_inferout;
	m_Data.framenum = inferout.framenum;
	m_Data.imageData = inferout.imageData;
};

bool SplitObjIF::SplitIF::InitData()
{
	return true;
}

SplitObjIF::SplitObjReceiver SplitObjIF::SplitIF::GetReceiverData()
{
	return m_Data;
};

void SplitObjIF::SplitIF::Setinnerframecount(unsigned int framecount)
{
	innerframecount = framecount;
};


unsigned int SplitObjIF::SplitIF::Getinnerframecount()
{
	return innerframecount;
};


void SplitObjIF::SplitIF::RunSplitDetect(SplitObjReceiver &datain,std::vector<SplitObjIF::SplitObjSender>& dataout,bool run)
{
	
	SplitObjIF::SplitIF::Setdata(datain);

	if (run)
	{
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		printf(">>>>>>>>>>>>>>now, Turn splitobj detection ON!<<<<<<<<<<<<<<<\n");
		
		work(dataout);
	}
	else
	{
		printf("turn SplitObj detection off!");
		if(!dataout.empty())
		{
			dataout.clear();
		}
	}
	
}

void SplitObjIF::SplitIF::work(std::vector<SplitObjIF::SplitObjSender> &senderpin)
{


	double del[3], mal_dist;
	double sum = 0.0;
	double sum1 = 0.0;
	int count = 0;
	bool close = false;
	int background;
	double mult;
	double duration, duration1, duration2, duration3;
	double temp_cov = 0.0;
	double weight = 0.0;
	double var = 0.0;
	double muR, muG, muB, dR, dG, dB, rVal, gVal, bVal;	
	cv::VideoCapture capture;
	cv::Mat roiregion;

	int i, j, k;
	i = j = k = 0;
	int nL, nC;

	cv::Mat orig_img,drawingorig, bin_img, vxMat,vxMat1;

	
	vx_context context =vxCreateContext();
	vx_matrix vxmatrix = 0;
    vx_graph vxgraph = 0;
    vx_node vxnode = 0;
	cv::Mat signalDraw(RESIZE_HEIGHT, RESIZE_WIDTH, CV_8UC3);
	std::vector< Track > iou_tracks;	

	cv::Vec3f val;
	uchar* r_ptr;
	uchar* b_ptr;
	int splitID=1;
	xueweiImage::ImageAnalysis Analysis;
	#if yolov5

	std::ofstream outfile("../results/yolov5.txt");

#else

#if RTSP
	
	// 接入inferout
	SplitObjIF::SplitObjReceiver inferData = SplitObjIF::SplitIF::Instance().GetReceiverData();
	cout<<"trying fetch img srcs<<<<<<<<<<<<"<<endl;
	cout<<"trying fetch img srcs<<<<<<<<<<<<"<<endl;
	cout<<"trying fetch img srcs<<<<<<<<<<<<"<<endl;
	cout<<"trying fetch img srcs<<<<<<<<<<<<"<<endl;
	cout<<"trying fetch img srcs<<<<<<<<<<<<"<<endl;

	//memset(&SenderResults,0,sizeof(SplitObjIF::SplitObjSender));
#else
	std::ifstream infile("../results/yolov5_xuewei_960_720.txt");

#if debug
 	std::ofstream debuglog("../results/log1.txt");
#endif

	std::vector< std::vector<BoundingBox> > yolov5_detections;
	// �޸Ķ�ȡ��ͼ���ʵ���������

	int scalefactor = 960/RESIZE_WIDTH;
	read_detections(infile, yolov5_detections,scalefactor);

#endif // DEBUG
	

#endif

	
	//Declare a VideoCapture object to store incoming frame and initialize it
#if RTSP
	
	if (!inferData.imageData.empty())
	{
		orig_img = inferData.imageData;
		cout<<"refer to imageData<<<<<<<<<<<<<<"<<endl;
		cout<<"refer to imageData<<<<<<<<<<<<<<"<<endl;
		cout<<"refer to imageData<<<<<<<<<<<<<<"<<endl;
		cout<<"refer to imageData<<<<<<<<<<<<<<"<<endl;
	}
	else
	{
		char rtsp[1000];
		int image_width = 2560;
		int image_height = 1440;
		std::string rtsp_latency = "0";
		std::string uri = "rtsp://admin:Ucit2021@10.203.204.198:554/h264/ch1/main/av_stream";
		sprintf(rtsp, "rtspsrc location=%s latency=%s ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink sync=false",uri.c_str(),rtsp_latency.c_str(),image_width,image_height);
		// 嵌入式运行不成功，需要网络情况良�?
		if (!capture.open(rtsp))
	{
		std::cout << "it can not open rtsp!!!!" << std::endl;
		return;
	}	

	}
	

#if READIMGONLY
	while (1)
	{
		
		bool ret = capture.grab();
		capture >> orig_img;
		if (orig_img.empty())
		{
			continue;
		}
		else
		{
			cv::imshow("RTSP_display",orig_img);
			cv::waitKey(5);
		}
	};
	
#endif


#else
	cv::VideoCapture capture("../data/out_xuewei.mp4");
#endif // RTSP


	//orig_img = cv::imread("../data/back1.jpg");
	cv::resize(orig_img, orig_img, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), INTER_NEAREST);
	
	if(!SplitIF::Instance().trigger )
	{
	
	
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Hits <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
	printf(">>>>>>>>>>>>>>>>>>please Rectangle the ROI area to run obj detection!!!<<<<<<<<<<<\n");
	printf(">>>>>>>>>>>>>>>>>>please Rectangle the ROI area to run obj detection!!!<<<<<<<<<<<\n");
	printf(">>>>>>>>>>>>>>>>>>please Rectangle the ROI area to run obj detection!!!<<<<<<<<<<<\n");
	printf(">>>>>>>>>>>>>>>>>>please Rectangle the ROI area to run obj detection!!!<<<<<<<<<<<\n");
	printf(">>>>>>>>>>>>>>>>>>please Rectangle the ROI area to run obj detection!!!<<<<<<<<<<<\n");
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");

	roi = cv::selectROI(orig_img);
	// prepare for judgeinornot 
	
	Point2d p1(roi.x, roi.y);
	Point2d p2(roi.x + roi.width, roi.y);
	Point2d p3(roi.x + roi.width, roi.y + roi.height);
	Point2d p4(roi.x, roi.y + roi.height);
	regions.push_back(p1);
	regions.push_back(p2);
	regions.push_back(p3);
	regions.push_back(p4);

	roiregion = orig_img(roi);
	cv::cvtColor(roiregion, roiregion, cv::COLOR_BGR2YCrCb);



	//cv::GaussianBlur(orig_img, orig_img, cv::Size(3,3), 3.0);

	//Initializing the binary image with the same dimensions as original image
	//bin_img = cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1, cv::Scalar(0));
	vxMat= cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1,cv::Scalar(0));
	vxMat1= cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1,cv::Scalar(0));	
	double value[3];
	

	//Step 1: initializing with one gaussian for the first time and keeping the no. of models as 1
	
	for (i = 0; i < roiregion.rows; i++)
	{
		r_ptr = roiregion.ptr(i);
		for (j = 0; j < roiregion.cols; j++)
		{

			N_ptr = Create_Node(*r_ptr, *(r_ptr + 1), *(r_ptr + 2));
			if (N_ptr != NULL) {
				N_ptr->pixel_s->weight = 1.0;
				Insert_End_Node(N_ptr);
			}
			else
			{
				std::cout << "Memory limit reached... ";
				//_getch();
				exit(0);
			}
		}
	}



	if (roiregion.isContinuous() == true)
	{
		nL = 1;
		nC = roiregion.rows * roiregion.cols * roiregion.channels();
	}

	else
	{
		nL = roiregion.rows;
		nC = roiregion.cols * roiregion.channels();
	}

	

	//Step 2: Modelling each pixel with Gaussian
	duration1 = static_cast<double>(cv::getTickCount());
	bin_img = cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1, cv::Scalar(0));
	
	std::cout<<"fps: "<<capture.get(cv::CAP_PROP_FPS)<<std::endl;
	
	SplitIF::Instance().trigger = true;


	};



#if yolov5
	// �����������������yolov5
	// load class names from dataset for visualization
	std::vector<std::string> class_names = LoadNames("../weights/coco.names");
	if (class_names.empty()) {
		std::cout << "load className failed!" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "load className success!" << std::endl;
	}
	// load network
	std::string weights = "../weights/yolov5s.torchscript.pt";
	auto detector = Detector(weights, device_type);

	// inference
	float conf_thres = 0.2f;
	float iou_thres = 0.5f;
	
#endif




		duration3 = static_cast<double>(cv::getTickCount());
		std::vector<BoundingBox> v_bbnd;
		v_bbnd.clear();
		
#if RTSP
		if(orig_img.empty())
		{
			cout<<"it is not possible the orig_img is empty in this place"<<endl;
		}
		else{
			orig_img = inferData.imageData;
			cv::resize(orig_img, orig_img, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), INTER_NEAREST);
			roiregion = orig_img(roi);
			bin_img = cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1, cv::Scalar(0));
			vxMat= cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1,cv::Scalar(0));
			vxMat1= cv::Mat(roiregion.rows, roiregion.cols, CV_8UC1,cv::Scalar(0));	
		}


#else
		if (!capture.read(orig_img)) {
			break;
		}
		else
		{
			cv::resize(orig_img, orig_img, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), INTER_NEAREST);
			roiregion = orig_img(roi);
		}
#endif

		//break;
		int count1 = 0;
		iou_tracks.clear();
		roiregion.copyTo(drawingorig);


		N_ptr = N_start;
		duration = static_cast<double>(cv::getTickCount());
		for (i = 0; i < nL; i++)
		{
			r_ptr = roiregion.ptr(i);
			// ��ֵ����ͼ��ÿ�����ص�ĵ�ַָ��?
			b_ptr = bin_img.ptr(i);

			for (j = 0; j < nC; j += 3)
			{
				sum = 0.0;
				sum1 = 0.0;
				close = false;
				background = 0;
				rVal = *(r_ptr++);
				gVal = *(r_ptr++);
				bVal = *(r_ptr++);
				start = N_ptr->pixel_s;
				rear = N_ptr->pixel_r;
				ptr = start;

				temp_ptr = NULL;

				if (N_ptr->no_of_components > 4)
				{
					Delete_gaussian(rear);
					N_ptr->no_of_components--;
				}

				for (k = 0; k < N_ptr->no_of_components; k++)
				{


					weight = ptr->weight;
					mult = alpha / weight;
					weight = weight * alpha_bar + prune;
					if (close == false)
					{
						muR = ptr->mean[0];
						muG = ptr->mean[1];
						muB = ptr->mean[2];

						dR = rVal - muR;
						dG = gVal - muG;
						dB = bVal - muB;

						/*del[0] = value[0]-ptr->mean[0];
						del[1] = value[1]-ptr->mean[1];
						del[2] = value[2]-ptr->mean[2];*/


						var = ptr->covariance;

						mal_dist = (dR * dR + dG * dG + dB * dB);

						if ((sum < cfbar) && (mal_dist < 16.0 * var * var))
							// ���������� 
							background = 255;

						if (mal_dist < 9.0 * var * var)
						{
							weight += alpha;
							//mult = mult < 20.0*alpha ? mult : 20.0*alpha;

							close = true;

							ptr->mean[0] = muR + mult * dR;
							ptr->mean[1] = muG + mult * dG;
							ptr->mean[2] = muB + mult * dB;
							//if( mult < 20.0*alpha)
							//temp_cov = ptr->covariance*(1+mult*(mal_dist - 1));
							temp_cov = var + mult * (mal_dist - var);
							ptr->covariance = temp_cov < 5.0 ? 5.0 : (temp_cov > 20.0 ? 20.0 : temp_cov);
							temp_ptr = ptr;
						}

					}

					if (weight < -prune)
					{
						ptr = Delete_gaussian(ptr);
						weight = 0;
						N_ptr->no_of_components--;
					}
					else
					{
						//if(ptr->weight > 0)
						sum += weight;
						ptr->weight = weight;
					}

					ptr = ptr->Next;
				}



				if (close == false)
				{
					ptr = new gaussian;
					ptr->weight = alpha;
					ptr->mean[0] = rVal;
					ptr->mean[1] = gVal;
					ptr->mean[2] = bVal;
					ptr->covariance = covariance0;
					ptr->Next = NULL;
					ptr->Previous = NULL;
					//Insert_End_gaussian(ptr);
					if (start == NULL)
						// ??
						start = rear = NULL;
					else
					{
						ptr->Previous = rear;
						rear->Next = ptr;
						rear = ptr;
					}
					temp_ptr = ptr;
					N_ptr->no_of_components++;
				}

				ptr = start;
				while (ptr != NULL)
				{
					ptr->weight /= sum;
					ptr = ptr->Next;
				}

				while (temp_ptr != NULL && temp_ptr->Previous != NULL)
				{
					if (temp_ptr->weight <= temp_ptr->Previous->weight)
						break;
					else
					{
						//count++;
						gnext = temp_ptr->Next;
						previous = temp_ptr->Previous;
						if (start == previous)
							start = temp_ptr;
						previous->Next = gnext;
						temp_ptr->Previous = previous->Previous;
						temp_ptr->Next = previous;
						if (previous->Previous != NULL)
							previous->Previous->Next = temp_ptr;
						if (gnext != NULL)
							gnext->Previous = previous;
						else
							rear = previous;
						previous->Previous = temp_ptr;
					}

					temp_ptr = temp_ptr->Previous;
				}



				N_ptr->pixel_s = start;
				N_ptr->pixel_r = rear;

				//if(background == 1)
				//printf("current bin_image pixel's background %d \n", background);
				*b_ptr++ = background;
				//else
					//bin_img.at<uchar>(i,j) = 0;
				N_ptr = N_ptr->Next;
			}
		}

#if DISPLAY
		imshow("before xingtai", bin_img);
		waitKey(5);
#endif

		// xuewei add some Morphology relevant processing
	if (openvxframe > 20)
		{
		//step one, filter tiny points
		//RemoveSmallRegion(bin_img, bin_img, 20, 0, 0);	
		
		// ��Ч��֤��ֵ�˲��ͱղ����Լ۱���ߣ�ҲЧ���Ϻá�?
		// ��ֵ�˲�
		//cv::medianBlur(bin_img, bin_img, 3);

		//  using openvx to substitute for opencv morphology operation(first dilate and then erode!) 

		// �������?
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarcy;

#if UsingOpenvx
		// try to map memory, not copy
     	
     	vx_image vx_bin = nvx_cv::createVXImageFromCVMat(context,bin_img);
		vx_image vx_Mat = nvx_cv::createVXImageFromCVMat(context,vxMat);
	

 		vx_status statusofdilate = vxuDilate3x3(context,vx_bin,vx_Mat); 
   		printf("dilate3x3:%d \n",statusofdilate);

		// recopy back to bin_img,
		// refer to https://github.com/Chevreau-Maxime/Polytech_TegraTX1/blob/ad418b4b509d1cd7fa0e37e1abd710b525247cf0/player/filtrage.h
		vx_status copyback = nvxuCopyImage(context, vx_Mat, vx_bin);
		printf("copyback:%d \n",copyback);

		vxuErode3x3(context, vx_bin, vx_Mat); 
		nvxuCopyImage(context, vx_Mat, vx_bin);

		vxReleaseImage(&vx_bin); 
		vxReleaseImage(&vx_Mat);
		cv::bitwise_not(bin_img, bin_img);


		vx_image vx_bin1 = nvx_cv::createVXImageFromCVMat(context,bin_img);
		vx_image vx_Mat1 = nvx_cv::createVXImageFromCVMat(context,vxMat1);
		vxmatrix = vxCreateMatrixFromPattern(context, VX_PATTERN_BOX, 9, 9);
		vx_status nonfilter = vxuNonLinearFilter(context, VX_NONLINEAR_FILTER_MAX, vx_bin1, vxmatrix, vx_Mat1); 
		printf("nonfilter:%d \n",nonfilter);
		nvxuCopyImage(context, vx_Mat1, vx_bin1);
		vxReleaseImage(&vx_bin1); 
		vxReleaseImage(&vx_Mat1);
		
#else
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
		cv::morphologyEx(bin_img, bin_img, CV_MOP_CLOSE, kernel);
		cv::bitwise_not(bin_img, bin_img);
		cv::Mat dilatekernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
		cv::dilate(bin_img, bin_img, dilatekernel, Point(-1, -1), 1, 0);
#endif 

#if DISPLAY
		cv::namedWindow("after xingtai", WINDOW_NORMAL);
		imshow("after xingtai", bin_img);
		waitKey(5);
#endif
		cv::findContours(bin_img, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		std::vector<RotatedRect> box(contours.size());
		std::vector<Rect> boundRect(contours.size());
		Point2f m_rect[4];
		BoundingBox m_BBtemp;
		memset(&m_BBtemp, 0, sizeof(BoundingBox));
		std::vector<BoundingBox> yolov5_currentobj;
#if yolov5
		// �ȸ�һ�������������?
		auto result = detector.Run(roiregion, conf_thres, iou_thres);
		// ��yolov5�Ľ��д��txt
		bool ret = write2file(outfile, count4tracker, result);
		if (1) {
			Demo(roiregion, result, class_names,false);
		}
#else
	#if RTSP		
	BoundingBox tempbb;
	for(int i=0; i<inferData.v_inferout.size();i++ )
	{
		tempbb.x = static_cast<float>(inferData.v_inferout[i].x);
		tempbb.y = static_cast<float>(inferData.v_inferout[i].y);
		tempbb.width = static_cast<float>(inferData.v_inferout[i].width);
		tempbb.height = static_cast<float>(inferData.v_inferout[i].height);
		tempbb.score = 1;
		tempbb.m_status = UnkownObj;
		yolov5_currentobj.push_back(tempbb);

	}
	SplitObjIF::SplitIF::Instance().Setinnerframecount(count4tracker);
	#else
		
		if (count4tracker< yolov5_detections.size())
		{
			yolov5_currentobj = yolov5_detections[count4tracker];
		}
		else
		{
			printf("it is not possible!, and yolov5 detections frames are less than videos!");
			return ;
		}
	
	#endif // RTSP


		
#endif

		//  ��ȡ��ǰ���������̽����?
		std::vector<Point2d> yolov5Points;
		std::cout << "begin to draw yolov5 detections'results!!" << std::endl;
		for (std::vector<BoundingBox>::iterator iters_b = yolov5_currentobj.begin();iters_b< yolov5_currentobj.end();)
		{
				yolov5Points.clear();
				cv::Point topleft, topright, bottomleft, bottomright;
				topleft.x = (int)iters_b->x;
				topleft.y = (int)iters_b->y;
				topright.x = (int)iters_b->x + (int)iters_b->width;
				topright.y = (int)iters_b->y;
				bottomleft.x = (int)iters_b->x;
				bottomleft.y = (int)iters_b->y + (int)iters_b->height;
				bottomright.x = (int)iters_b->x + (int)iters_b->width;
				bottomright.y = (int)iters_b->y + (int)iters_b->height;
				yolov5Points.push_back(topleft);
				yolov5Points.push_back(topright);
				yolov5Points.push_back(bottomright);
				yolov5Points.push_back(bottomleft);
				bool insideornot = judgeInorNot(yolov5Points, regions);
				if (insideornot)
				{
					iters_b->x = iters_b->x - roi.x;
					iters_b->y = iters_b->y - roi.y;
					
					rectangle(drawingorig,
						Point((int)iters_b->x-roi.x,
							(int)iters_b->y-roi.y),
						Point((int)iters_b->x + (int)iters_b->width-roi.x,
							(int)iters_b->y + (int)iters_b->height-roi.y),
						Scalar(0, 0, 255),
						2,
						8);
					iters_b++;
				}
				else
				{
					iters_b = yolov5_currentobj.erase(iters_b);
				}

		}
		
		printf("contours'size:%d \n",contours.size());
		for (int i=0; i < contours.size();i++)
		{
			box[i] = minAreaRect(Mat(contours[i]));
			boundRect[i] = cv::boundingRect(Mat(contours[i]));
			if (box[i].size.width < 15*RESIZE_WIDTH/960 || box[i].size.height<15*RESIZE_WIDTH/960)
			{
				continue;
			}
			else
			{
				//rectangle(drawingorig, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
				/*	m_BBtemp.x = boundRect[i].x;
					m_BBtemp.y = boundRect[i].y;
					m_BBtemp.w = boundRect[i].width;
					m_BBtemp.h = boundRect[i].height;
					m_BBtemp.score = 1;
					m_BBtemp.m_status = UnkownObj;
					v_bbnd.push_back(m_BBtemp);*/
				//circle(roiregion, Point(box[i].center.x, box[i].center.y), 5, Scalar(0, 255, 0), -1, 8);
				box[i].points(m_rect);
#if debug
        	debuglog << "m_rect:" << m_rect[0] << "\t" << m_rect[1]<<"\t" << m_rect[2]<<"\t" << m_rect[3] << endl;
#endif 

#if LowVersionOpencv
        m_BBtemp.x = m_rect[1].x;
				m_BBtemp.y = m_rect[1].y;
				m_BBtemp.width = m_rect[2].x - m_rect[1].x;
				m_BBtemp.height = m_rect[3].y - m_rect[2].y;
				m_BBtemp.score = 1;
				m_BBtemp.m_status = UnkownObj;
				v_bbnd.push_back(m_BBtemp);
  
#else
      	m_BBtemp.x = m_rect[0].x;
				m_BBtemp.y = m_rect[0].y;
				m_BBtemp.width = m_rect[1].x - m_rect[0].x;
				m_BBtemp.height = m_rect[2].y - m_rect[1].y;
				m_BBtemp.score = 1;
				m_BBtemp.m_status = UnkownObj;
				v_bbnd.push_back(m_BBtemp);
#endif
			
			
				// keep ��С��Ӿ���?
				for (int j = 0; j < 4; j++)
				{
					//line(roiregion, m_rect[j], m_rect[(j + 1) % 4], Scalar(0, 255, 0), 2, 8);
				}
			}
		}

		// ��һ��Yolo�ͱ������ϲ�filter
		// step one ,��ƥ������˶�Ŀ����yolo��̽��

		char yichu[255];
		for (int i =0; i<v_bbnd.size();i++)
		{
			int indexofmatch = highestIOU(v_bbnd[i], yolov5_currentobj);
			// �ж��Ƿ��˶�������yolov5 ���ճ��������ǣ��򲻼�������׷��iou
			if (indexofmatch != -1 \
				&& intersectionOverUnion(v_bbnd[i], yolov5_currentobj[indexofmatch]) >= 0.05)
			{
				v_bbnd[i].m_status = Ejected;
				rectangle(drawingorig,
					Point(v_bbnd[i].x, v_bbnd[i].y),
					Point(v_bbnd[i].x + v_bbnd[i].width,
						v_bbnd[i].y + v_bbnd[i].height),
					Scalar(0, 150, 50), 2, 8);

				sprintf(yichu, "Ejected");

				cv::putText(drawingorig, yichu,
					cv::Point((v_bbnd[i].x + v_bbnd[i].width - v_bbnd[i].width / 2) - 30,
						v_bbnd[i].y + v_bbnd[i].height + 10),
					1,
					1.2,
					Scalar(0, 150, 50),
					1.2, LINE_4);

				v_bbnd.erase(v_bbnd.begin() + i);
			}
			else
			{
				v_bbnd[i].m_status = Suspected;
				rectangle(drawingorig,
					Point(v_bbnd[i].x, v_bbnd[i].y),
					Point(v_bbnd[i].x + v_bbnd[i].width,
						v_bbnd[i].y + v_bbnd[i].height),
					Scalar(0, 150, 50), 2, 8);

				sprintf(yichu, "Split_Unsure");

				cv::putText(drawingorig, yichu,
					cv::Point((v_bbnd[i].x + v_bbnd[i].width - v_bbnd[i].width / 2) - 30,
						v_bbnd[i].y + v_bbnd[i].height + 10),
					1,
					1.2,
					Scalar(150, 0, 50),
					1.2, LINE_4);
			}
		}
		// ÿ֡ѭ������v_bbnd���뵽Ƕ��vv�У�
		vv_detections.push_back(v_bbnd);

		// �����ۼ�����ѭ����ʼ iou track

		if (count4tracker>25)
		{
			//begin to iou track
			iou_tracks = track_iou(stationary_threshold, lazy_threshold,sigma_h, sigma_iou, t_min, vv_detections);
			std::cout << "tracks'size" << iou_tracks.size() << std::endl;
			//std::cout << "Last Track ID > " << iou_tracks.back().id << std::endl;
		}
		std::cout << "this is" << count4tracker << "frame" << std::endl;

		

		char info[256];
		for (auto &dt : iou_tracks)
		{
			int box_index = count4tracker - dt.start_frame;
			if (box_index < dt.boxes.size())
			{
				BoundingBox b = dt.boxes[box_index];
				
				cv::rectangle(drawingorig, cv::Point(b.x, b.y), cv::Point(b.x + b.width, b.y + b.height), cv::Scalar(255, 0, 100), 2);

				std::string s_status;
				cv::Scalar blue(255, 0, 0);
				cv::Scalar red(0, 0, 255);
				cv::Scalar green(0, 255, 0);
				switch (dt.status)
				{
				case Moving:
					s_status = "Moving";
					sprintf(info, "ID:%d_AppearingT:%d_%s", dt.id, dt.total_appearing, s_status.c_str());
					//cv::putText(drawingorig, info, cv::Point((b.x + b.w - b.w / 2) - 30, b.y + b.h - 5), 1, 1, blue, 1);
					break;
				case Stopping:
					s_status = "Stopping";
					sprintf(info, "ID:%d_AppearingT:%d_%s", dt.id, dt.total_appearing, s_status.c_str());
					//cv::putText(drawingorig, info, cv::Point((b.x + b.w - b.w / 2) - 30, b.y + b.h - 5), 1, 1, green, 1);
					break;
				case Static_Sure:

					if (b.width * b.height > pow(RESIZE_WIDTH*200/960,2))
					{
						break;
					}

					s_status = "SplitObj_Sure";
					sprintf(info, "ID:%d_%s", dt.id,  s_status.c_str());
					cv::putText(drawingorig, info, cv::Point((b.x + b.width - b.width / 2) - 30, b.y + b.height - 5), 1, 1.5, red, 1);
					
					xueweiImage::SplitObject tmpSplitObj;
					tmpSplitObj.ID = splitID;
					tmpSplitObj.m_postion.x = static_cast<int>(b.x);
					tmpSplitObj.m_postion.y = static_cast<int>(b.y);
					tmpSplitObj.m_postion.width = static_cast<int>(b.width);
					tmpSplitObj.m_postion.height = static_cast<int>(b.height);
					tmpSplitObj.moved = false;
					tmpSplitObj.firstshowframenum = count4tracker;
#if UsingOpenvx
					//printf("b[%d,%d,%d,%d]\n",static_cast<int>(b.x),static_cast<int>(b.y),static_cast<int>(b.width),static_cast<int>(b.height));
					if (static_cast<int>(b.x)<0 || static_cast<int>(b.y)<0 || static_cast<int>(b.width)<8 || static_cast<int>(b.height)<8)
					{
						continue;
					}
#endif 					
					tmpSplitObj.imgdata = roiregion(tmpSplitObj.m_postion);
					tmpSplitObj.haschecked = false;
					tmpSplitObj.checktimes = 1;
					// copy a result to senderpin
#if RTSP
					SenderResults.m_gps.latititude = 0;
					SenderResults.m_gps.longtitude = 0;
					SenderResults.m_radarpos.x = 0.0f;
					SenderResults.m_radarpos.y = 0.0f;
					SenderResults.SplitID = splitID;
					// this timestamp is when the object
					SenderResults.appearing_timestamp = inferData.timestamp;
					SenderResults.m_postion.x = static_cast<int>(b.x);
					SenderResults.m_postion.y = static_cast<int>(b.y);
					SenderResults.m_postion.width = static_cast<int>(b.width);
					SenderResults.m_postion.height = static_cast<int>(b.height);
					SenderResults.origlayout.x = (SenderResults.m_postion.x + (int)roi.x)* (float)2560/(float)RESIZE_WIDTH;
					SenderResults.origlayout.y = (SenderResults.m_postion.y + (int)roi.y) * (float)1440 / (float)RESIZE_HEIGHT;
					SenderResults.origlayout.width = SenderResults.m_postion.width* (float)2560 / (float)RESIZE_WIDTH;
					SenderResults.origlayout.height = SenderResults.m_postion.height* (float)1440 / (float)RESIZE_HEIGHT;	

					SenderResults.moved = false;
					SenderResults.firstshowframenum = count4tracker;
					SenderResults.imgdata = roiregion(tmpSplitObj.m_postion);
					SenderResults.haschecked = false;
					SenderResults.checktimes = 1;	
	#endif
					char display[256];
					#if RTSP
					if (!v_forSenders.empty())
					{
						// �����������ж��Ƿ����µ����������?
						int index = Analysis.CheckHighestIOU(tmpSplitObj.m_postion, SplitObjForSure);
						if (index != -1 \
							&& Analysis.intersectionOU(tmpSplitObj.m_postion, SplitObjForSure[index].m_postion) >= 0.75)
						{
								
						}
						else
						{
							v_forSenders.push_back(SenderResults);
							splitID++;
						}
						
					}
					else
					{
						v_forSenders.push_back(SenderResults);
						splitID++;
					}

					#else
					if (!SplitObjForSure.empty())
					{
						// �����������ж��Ƿ����µ����������?
						int index = Analysis.CheckHighestIOU(tmpSplitObj.m_postion, SplitObjForSure);
						if (index != -1 \
							&& Analysis.intersectionOU(tmpSplitObj.m_postion, SplitObjForSure[index].m_postion) >= 0.75)
						{
								
						}
						else
						{
                #if RTSP
							senderpin.push_back(SenderResults);
                                                                                                  #endif
							SplitObjForSure.push_back(tmpSplitObj);
							splitID++;
						}
						
					}
					else
					{
           #if RTSP
						senderpin.push_back(SenderResults);
                                                     #endif
						SplitObjForSure.push_back(tmpSplitObj);
						splitID++;
					}
					#endif
					break;
				}
			}
		}



		char displayindex[256],judge[256];
		int offset = 1; 
		// for  destroy corresponding patch 
		char destroypatchname[256];


		#if RTSP

		for (vector<SplitObjIF::SplitObjSender>::iterator iter = v_forSenders.begin(); iter < v_forSenders.end();)
		{
			int timeinterval = count4tracker - iter->firstshowframenum;
			if (timeinterval > CHECK_INTERVAL && !iter->haschecked)
			{
				/*sprintf(judge,"judge whether the obj[%d] is moved out \n", iter->ID);*/
				// 
				cv::Mat currentpatchhere = roiregion(iter->m_postion);
				bool movedout = Analysis.BemovedOut(iter->imgdata, currentpatchhere, 0);
				iter->haschecked = true;
				iter->checktimes++;
				if (movedout)
				{
					sprintf(judge,"obj[%d] has been moved out \n", iter->ID);
					cv::putText(drawingorig, judge, cv::Point(200, 5 + (100 * offset)), 3, 1.25, cv::Scalar(100, 0, 200));
					offset++;
					sprintf(destroypatchname, "patch_%d", iter->ID);
					cv::destroyAllWindows();
					iter = v_forSenders.erase(iter);
				}
				else
				{
					sprintf(judge, "obj[%d] is still there, check first times \n", iter->ID);
					cv::putText(drawingorig, judge, cv::Point(8, 5 + (100 * offset)), 3, 1.25, cv::Scalar(100, 0, 200));
					offset++;
					iter->moved = false;
					iter++;
				}
			}
			else if (iter->haschecked && timeinterval>(CHECK_INTERVAL*iter->checktimes))
			{
				cv::Mat currentpatchhere = roiregion(iter->m_postion);
				bool movedout = Analysis.BemovedOut(iter->imgdata, currentpatchhere, 0);
				iter->checktimes++;
				if (movedout)
				{
					sprintf(judge,"obj[%d] has been moved out \n", iter->ID);
					cv::putText(drawingorig, judge, cv::Point(8, 5 + (100 * offset)), 3, 1.25, cv::Scalar(0, 0, 255));
					offset++;
					sprintf(destroypatchname, "patch_%d", iter->ID);
					cv::destroyAllWindows();
					iter = v_forSenders.erase(iter);
				}
				else
				{
					sprintf(judge, "obj[%d] is still there, check[%d] times \n", iter->ID,iter->checktimes);
					cv::putText(drawingorig, judge, cv::Point(8, 5 + (100 * offset)), 3, 1.25, cv::Scalar(0, 0, 255));
					iter->moved = false;
					iter++;
				}
			}
			else
			{
				if (CHECK_INTERVAL>timeinterval)
				{
					sprintf(judge, "ID_%d_%d_fleft",iter->ID, CHECK_INTERVAL - timeinterval);
				}
				else
				{
					if (!iter->haschecked)
					{
						sprintf(judge, "ID_%d to check now", iter->ID);
					}
				}
				cv::putText(drawingorig,judge,cv::Point(8,5+(100*offset)),3,1.25,cv::Scalar(0,0,255));
				offset++;
				sprintf(displayindex, "patch_%d ", iter->ID);
				/* cv::namedWindow(displayindex, WINDOW_NORMAL);
				cv::imshow(displayindex, iter->imgdata);
				cv::waitKey(5); */
				iter++;
			}
		}

		SplitObjSender tempersend;
		for (int i=0; i< v_forSenders.size();i++)
		{
			tempersend.appearing_timestamp = v_forSenders[i].appearing_timestamp;
			tempersend.checktimes = v_forSenders[i].checktimes;
			tempersend.dispearing_timestamp = v_forSenders[i].dispearing_timestamp;
			tempersend.firstshowframenum = v_forSenders[i].firstshowframenum;
			tempersend.haschecked = v_forSenders[i].haschecked;
			tempersend.ID = v_forSenders[i].ID;
			tempersend.m_gps = v_forSenders[i].m_gps;
			tempersend.m_radarpos = v_forSenders[i].m_radarpos;
			tempersend.moved = v_forSenders[i].moved;
			tempersend.SplitID = v_forSenders[i].SplitID;
			tempersend.origlayout = v_forSenders[i].origlayout;
			senderpin.push_back(tempersend);	
		}

		#else
			for (vector <xueweiImage::SplitObject>::iterator iter = SplitObjForSure.begin(); iter < SplitObjForSure.end();)
		{
			int timeinterval = count4tracker - iter->firstshowframenum;
			if (timeinterval > CHECK_INTERVAL && !iter->haschecked)
			{
				/*sprintf(judge,"judge whether the obj[%d] is moved out \n", iter->ID);*/
				// 
				cv::Mat currentpatchhere = roiregion(iter->m_postion);
				bool movedout = Analysis.BemovedOut(iter->imgdata, currentpatchhere, 0);
				iter->haschecked = true;
				iter->checktimes++;
				if (movedout)
				{
					sprintf(judge,"obj[%d] has been moved out \n", iter->ID);
					cv::putText(drawingorig, judge, cv::Point(200, 5 + (100 * offset)), 3, 1.25, cv::Scalar(100, 0, 200));
					offset++;
					sprintf(destroypatchname, "patch_%d", iter->ID);
					cv::destroyAllWindows();
					iter = SplitObjForSure.erase(iter);
				}
				else
				{
					sprintf(judge, "obj[%d] is still there, check first times \n", iter->ID);
					cv::putText(drawingorig, judge, cv::Point(8, 5 + (100 * offset)), 3, 1.25, cv::Scalar(100, 0, 200));
					offset++;
					iter->moved = false;
					iter++;
				}
			}
			else if (iter->haschecked && timeinterval>(CHECK_INTERVAL*iter->checktimes))
			{
				cv::Mat currentpatchhere = roiregion(iter->m_postion);
				bool movedout = Analysis.BemovedOut(iter->imgdata, currentpatchhere, 0);
				iter->checktimes++;
				if (movedout)
				{
					sprintf(judge,"obj[%d] has been moved out \n", iter->ID);
					cv::putText(drawingorig, judge, cv::Point(8, 5 + (100 * offset)), 3, 1.25, cv::Scalar(0, 0, 255));
					offset++;
					sprintf(destroypatchname, "patch_%d", iter->ID);
					cv::destroyAllWindows();
					iter = SplitObjForSure.erase(iter);
				}
				else
				{
					sprintf(judge, "obj[%d] is still there, check[%d] times \n", iter->ID,iter->checktimes);
					cv::putText(drawingorig, judge, cv::Point(8, 5 + (100 * offset)), 3, 1.25, cv::Scalar(0, 0, 255));
					iter->moved = false;
					iter++;
				}
			}
			else
			{
				if (CHECK_INTERVAL>timeinterval)
				{
					sprintf(judge, "ID_%d_%d_fleft",iter->ID, CHECK_INTERVAL - timeinterval);
				}
				else
				{
					if (!iter->haschecked)
					{
						sprintf(judge, "ID_%d to check now", iter->ID);
					}
				}
				cv::putText(drawingorig,judge,cv::Point(8,5+(100*offset)),3,1.25,cv::Scalar(0,0,255));
				offset++;
				sprintf(displayindex, "patch_%d ", iter->ID);
				cv::namedWindow(displayindex, WINDOW_NORMAL);
				cv::imshow(displayindex, iter->imgdata);
				cv::waitKey(5);
				iter++;
			}
		}

		#endif
			count4tracker++;



	}

		
		openvxframe++;
		duration = static_cast<double>(cv::getTickCount()) - duration3;
		duration /= cv::getTickFrequency();
		std::cout << "\n per frame duration :" << duration;
		std::cout << "\n per frame duration :" << duration;
		std::cout << "\n per frame duration :" << duration;

		cv::namedWindow("orig", WINDOW_NORMAL);
		cv::imshow("orig", drawingorig);
		cv::waitKey(3);



	//}
#if debug
 	debuglog.close();
#endif

#if yolov5
	outfile.close();
#endif

}



