//#pragma once
#ifndef __ADAS_EM_h___
#define __ADAS_EM_h___
#include <iostream>
#include <string>
#include <unistd.h>
#include <mutex>
#include <string>
#include <vector>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/string/subscriber.h>
#include <ecal/msg/string/publisher.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


#include <nlohmann/json.hpp>
#include "SplitIF.hpp"

using namespace std;

namespace AdasEM 
{

typedef struct
{
	float left;
	float top;
	float width;
	float height;
}Rect;

typedef struct 
{
        int64_t   timestamp;
        string   camNo;
        vector<Rect> vec;
}AbdObjRectInfo;

class adas
{
public:
	adas();
	~adas();
	cv::Mat frame;
	std::vector<cv::Rect> inferBbox;
	long infer_timestamp;
	std::mutex mux_cap;
	std::mutex mux_infer;
	eCAL::string::CPublisher<std::string> abdObjRectPub;
	eCAL::string::CSubscriber<std::string> inferBboxSub;

	void init();
	void run();
	void ecal_init();
	void ecal_finalize();
	void createAbdObjRectPub();
	void subTopic();
	void sendAbdObjRect(AbdObjRectInfo &AbdObjOut);
	void sendAbdObjRect();
	void receive_callback(const char *topic_name, const std::string &message);
	int capture1Thrd();
	int process1Thrd();
private:
        //abdObj lcll;
};
};
#endif
