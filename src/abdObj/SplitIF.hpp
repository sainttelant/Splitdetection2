#ifndef _SPLITIF_HPP_
#define _SPLITIF_HPP

#include <iostream>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/video/background_segm.hpp>
#include <memory>
#include <time.h>
#include <string>
#include "IOUT.h"
#include "ImageAnalysis.hpp"



#define TRACE
#ifndef TRACE
 #define Ucitcout 0 && std::cout//或者NULL && Ucitcout
#else
 #define Ucitcout std::cout
#endif

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
       std::vector<cv::Rect> v_inferout;
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
	 //封装一下background abstract~~~
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
        Ptr<cv::cuda::BackgroundSubtractorMOG2> bgsubtractor; 
    };
    
    
} // namespace SplitObj

#endif // !_SPLITIF_HPP_








