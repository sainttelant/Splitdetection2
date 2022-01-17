#ifndef _SPLITIF_HPP_
#define _SPLITIF_HPP

#include <iostream>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <memory>
#include <time.h>
#include <string>
#include "IOUT.h"
#include "ImageAnalysis.hpp"

namespace SplitObjIF
{

    struct GpsPostion
    {
        double longtitude;
        double latititude;
    };
    struct RadarPostion
    {
        float x;
        float y;
    };

    struct SplitObjSender
    {
        int SplitID;
        GpsPostion m_gps;
        RadarPostion m_radarpos;
        long appearing_timestamp;
        long dispearing_timestamp;
        cv::Rect m_postion;
        cv::Rect origlayout;
		cv::Mat imgdata;
		unsigned int firstshowframenum;
		int ID;
		bool moved;
		bool haschecked;
		int checktimes;
    };

    struct SplitObjReceiver
    {
       unsigned int framenum;
       long timestamp;
       cv::Mat imageData;
       // 安全起见拷贝一次从inferout，等号操作深拷贝一次
       std::vector<cv::Rect> v_inferout;
        /* data */
    };


    
    

    class SplitIF
    {
    public:
        static SplitIF  &Instance()
		{
			static SplitIF m_SplitIF;
			return m_SplitIF;
		};

        void RunSplitDetect(SplitObjReceiver &datain,std::vector<SplitObjIF::SplitObjSender>& dataout,bool run);
        void work(std::vector<SplitObjIF::SplitObjSender> &senderpin);

        bool InitData();

        void Setdata(SplitObjReceiver inferout);
        void Setinnerframecount(unsigned int framecount);
        unsigned int Getinnerframecount();
        SplitObjReceiver GetReceiverData();
        bool trigger;
    private:

        SplitIF();
        virtual ~SplitIF();
        SplitObjReceiver m_Data;
        unsigned int innerframecount;
        cv::Rect2d roi;
        std::vector< std::vector<BoundingBox>> vv_detections;
        std::vector<xueweiImage::SplitObject> SplitObjForSure;
        unsigned int count4tracker;
	    unsigned int openvxframe;
	    std::vector<cv::Point2d> regions;
        SplitObjIF::SplitObjSender SenderResults;
	    std::vector<SplitObjIF::SplitObjSender> v_forSenders;

        float stationary_threshold;		// low detection threshold
        float lazy_threshold;
        float sigma_h;		// high detection threshold
        float sigma_iou;	// IOU threshold
        float t_min;		// minimum track length in frames

        
    };
    
    
} // namespace SplitObj

#endif // !_SPLITIF_HPP_








