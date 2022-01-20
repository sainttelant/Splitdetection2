#include "adas_em.h"
#include "topic.h"
#include "sys/time.h"
using namespace std;
using nlohmann::json;

namespace AdasEM
{
adas::adas()
{
}

adas::~adas()
{
}

void adas::ecal_init()
{
	eCAL::Initialize(0,nullptr,"adasServer");
}

void adas::createAbdObjRectPub()
{
	abdObjRectPub.Create("abdObjRect");
}

void adas::subTopic()
{
    inferBboxSub.Create("inferBbox");
    inferBboxSub.AddReceiveCallback(bind(&adas::receive_callback, this,placeholders::_1, placeholders::_2));
}

void adas::ecal_finalize()
{
	eCAL::Finalize();
}

void adas::receive_callback(const char *topic_name, const std::string &message)
{
	Ucitcout << "Receive topic message from: " << string(topic_name) << endl;
    	string name = string(topic_name);
	//Ucitcout << "msg:"<<message<<endl;
	json j = json::parse(string(message));
	try
	{
		if( name == "inferBbox")
		{
			auto data = j.get<inferBboxNs::InferBbox>();
			Ucitcout<< "timestamp:"<<data.timestamp<<endl;
			Ucitcout<< "camNo:"<<data.camNo<<endl;
			Ucitcout<< "num:"<<data.Bbox.size()<<endl;
			//vector<Rect>::iterator it;
			mux_infer.lock();
			if (!inferBbox.empty())
			{
				inferBbox.clear();
			}
			// the following it parameters should be changed to referring
			for(auto &it:data.Bbox)
			{
				cv::Rect rect_temp(int(it.left),int(it.top),int(it.width),int(it.height));
				// change it with emplace_back, rect maybe have move construct functions
				inferBbox.emplace_back(rect_temp);
			}
			mux_infer.unlock();
			Ucitcout <<"inferBbox Out size:"<<inferBbox.size()<<endl;
		}
	}
	catch (json::exception &e)
	{
		Ucitcout << "error msg: " << e.what() << endl;
	}
}

void adas::sendAbdObjRect(AbdObjRectInfo &AbdObjOut)
{
	
	abdObjRectNs::Rect rect_temp;
	abdObjRectNs::AbdObjRect AbdObjRectData;

	#if 0
	AbdObjRectInfo AbdObjOut;
	Rect    myRect;
	myRect.top = 400;
	myRect.left = 400;
	myRect.width = 600;
	myRect.height = 600;
	AbdObjOut.vec.push_back(myRect);

	myRect.top = 700;
	myRect.left = 200;
	myRect.width = 300;
	myRect.height = 300;
	AbdObjOut.vec.push_back(myRect);

	myRect.top = 700;
	myRect.left = 600;
	myRect.width = 300;
	myRect.height = 300;
	AbdObjOut.vec.push_back(myRect);
	#endif

	vector<Rect>::iterator it;
	for(it = AbdObjOut.vec.begin();it!=AbdObjOut.vec.end();++it)
	{
		rect_temp.top = it->top;
		rect_temp.left = it->left;
		rect_temp.width = it->width;
		rect_temp.height = it->height;
		AbdObjRectData.Bbox.push_back(rect_temp);
	}
	AbdObjRectData.timestamp = 222222222222;
	AbdObjRectData.camNo = "002";
	json j;
	j["timestamp"] = AbdObjRectData.timestamp;
	j["camNo"]     = AbdObjRectData.camNo;
	j["Bbox"]      = AbdObjRectData.Bbox;
	abdObjRectPub.Send(j.dump());
	
}
void adas::sendAbdObjRect()
{
	
	abdObjRectNs::Rect rect_temp;
	abdObjRectNs::AbdObjRect AbdObjRectData;
	AbdObjRectData.Bbox.clear();
	AbdObjRectInfo AbdObjOut;
	#if 0
	Rect    myRect;
	myRect.top = 400;
	myRect.left = 200;
	myRect.width = 200;
	myRect.height = 200;
	AbdObjOut.vec.push_back(myRect);

	myRect.top = 600;
	myRect.left = 400;
	myRect.width = 300;
	myRect.height = 300;
	AbdObjOut.vec.push_back(myRect);

	myRect.top = 800;
	myRect.left = 800;
	myRect.width = 400;
	myRect.height = 400;
	AbdObjOut.vec.push_back(myRect);
	#endif

	vector<Rect>::iterator it;
	for(it = AbdObjOut.vec.begin();it!=AbdObjOut.vec.end();++it)
	{
		rect_temp.top = it->top;
		rect_temp.left = it->left;
		rect_temp.width = it->width;
		rect_temp.height = it->height;
		AbdObjRectData.Bbox.push_back(rect_temp);
	}
	AbdObjRectData.timestamp = 222555555555;
	AbdObjRectData.camNo = "003";
	json j;
	j["timestamp"] = AbdObjRectData.timestamp;
	j["camNo"]     = AbdObjRectData.camNo;
	j["Bbox"]      = AbdObjRectData.Bbox;
	abdObjRectPub.Send(j.dump());
	
}
int adas::capture1Thrd()
{
	cv::VideoCapture capture;
	cv::Mat orig_img;
	char rtsp[1000];
	int image_width = 2560;
	int image_height = 1440;
	std::string rtsp_latency = "0";
	//std::string uri = "rtsp://admin:Ucit-2119@10.8.5.210:554/h264/ch1/main/av_stream";
	std::string uri = "rtsp://admin:Ucit2021@10.203.204.198:554/h264/ch1/main/av_stream";
	sprintf(rtsp, "rtspsrc location=%s latency=%s ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink 		sync=false",uri.c_str(),rtsp_latency.c_str(),image_width,image_height);
		// åµå¥åŒè¿è¡äžæåïŒéèŠçœç»æåµè¯ï¿?
	if (!capture.open(uri))
	{
		Ucitcout << "it can not open rtsp!!!!" << std::endl;
		//return -1 ;
	}	

	while (1)
	{
		
		bool ret = capture.grab();
		mux_cap.lock();
		capture >> frame;
		
		if (frame.empty())
		{
			continue;
		}
		else
		{
			//cv::imshow("RTSP_display",frame);
			//cv::waitKey(5);
			//sendAbdObjRect();
		}
		mux_cap.unlock();
	};
	return 0;
	
}
int adas::process1Thrd()
{
	struct timeval tv;
	long nowTimeMs = 0;
	//cv::Mat img;
	while(1)
	{
		static unsigned int count = 0;
		count ++;
		Ucitcout<<"process1Thrd...."<<endl;
		bool Runokay = true;
		std::vector<SplitObjIF::SplitObjSender> v_objsender;
		v_objsender.clear();
		SplitObjIF::SplitObjReceiver datain;
		datain.framenum = count;
		
		if (frame.empty())
		{
			continue;
		}
		mux_cap.lock();
		//img = frame.clone();
		datain.imageData = frame.clone();
		mux_cap.unlock();
	
		mux_infer.lock();
		// interBbox is only used for SplitIF, so change it with move construct
		datain.v_inferout = std::move(inferBbox);
		datain.timestamp = infer_timestamp;
		mux_infer.unlock();
		
		
		//cv::waitKey(5);
		gettimeofday(&tv,0);
		Ucitcout<<"start time:"<<(tv.tv_sec*1000+tv.tv_usec/1000)<<endl;
		Ucitcout<<" SPlit module receive"<<datain.v_inferout.size()<<"Objects"<<endl;
   		SplitObjIF::SplitIF::Instance().RunSplitDetect(datain,v_objsender,Runokay);
		gettimeofday(&tv,0);
		Ucitcout<<"_____"<<endl;
		Ucitcout<<"end time:"<<(tv.tv_sec*1000+tv.tv_usec/1000);
		AbdObjRectInfo AbdObjOut;
		AbdObjOut.timestamp = 222222222222222;
		AbdObjOut.camNo = "cam1out";
		std::vector<SplitObjIF::SplitObjSender>::iterator it;
		Rect rect_temp;
		AbdObjOut.vec.clear();
		Ucitcout<<"______________________________________________________________________________-_abdout size:"<<v_objsender.size()<<endl;
		for(it = v_objsender.begin();it!=v_objsender.end();++it)
		{
			rect_temp.top = it->origlayout.y;
			rect_temp.left = it->origlayout.x;
			rect_temp.width = it->origlayout.width;
			rect_temp.height = it->origlayout.height;
			AbdObjOut.vec.push_back(rect_temp);
			Ucitcout<<"______________________________________________________x:"<<rect_temp.left<<"    y:"<<rect_temp.top<<"    width"<<rect_temp.width<<"     height:"<<rect_temp.height<<endl;
		}
		sendAbdObjRect(AbdObjOut);
		
	}
	return 0;
}
};
