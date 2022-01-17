#include "ImageAnalysis.hpp"


namespace xueweiImage
{
	ImageAnalysis::ImageAnalysis()
	{

	}

	ImageAnalysis::~ImageAnalysis()
	{

	}


	float ImageAnalysis::intersectionOU(cv::Rect box1, cv::Rect box2)
	{
		float minx1 = box1.x;
		float maxx1 = box1.x + box1.width;
		float miny1 = box1.y;
		float maxy1 = box1.y + box1.height;

		float minx2 = box2.x;
		float maxx2 = box2.x + box2.width;
		float miny2 = box2.y;
		float maxy2 = box2.y + box2.height;

		if (minx1 > maxx2 || maxx1 < minx2 || miny1 > maxy2 || maxy1 < miny2)
			return 0.0f;
		else
		{
			float dx = std::min(maxx2, maxx1) - std::max(minx2, minx1);
			float dy = std::min(maxy2, maxy1) - std::max(miny2, miny1);
			float area1 = (maxx1 - minx1) * (maxy1 - miny1);
			float area2 = (maxx2 - minx2) * (maxy2 - miny2);
			float inter = dx * dy; // Intersection
			float uni = area1 + area2 - inter; // Union
			float IoU = inter / uni;
			return IoU;
		}
		//	return 0.0f;
	}

	int ImageAnalysis::CheckHighestIOU(cv::Rect& box, std::vector<SplitObject>& Split)
	{
		float iou = 0, highest = 0;
		int index = -1;
		for (int i = 0; i < Split.size(); i++)
		{
			iou = intersectionOU(box, Split[i].m_postion);
			if (iou >= highest)
			{
				highest = iou;
				index = i;
			}
		}
		return index;

	}

	bool ImageAnalysis::BemovedOut(cv::Mat& patch, cv::Mat& compared, int intervals, int method)
	{
		if (method == 0)
		{
			// ����hist�İ취����ƥ��patch
			Mat tmpImg;
			resize(patch, tmpImg, Size(compared.cols, compared.rows));



			//HSV��ɫ����ģ��(ɫ��H,���Ͷ�S������V)
			cvtColor(tmpImg, tmpImg, COLOR_BGR2HSV);
			cvtColor(compared, compared, COLOR_BGR2HSV);
			
			
			int hBins = 256, sBins = 256;
			int histSize[] = { hBins,sBins };
			//H:0~180, S:0~255,V:0~255
			//Hɫ��ȡֵ��Χ
			float hRanges[] = { 0,180 };
			//S���Ͷ�ȡֵ��Χ
			float sRanges[] = { 0,255 };
			const float* ranges[] = { hRanges,sRanges };
			int channels[] = { 0,1 };//��άֱ��ͼ
			MatND hist1, hist2;
			calcHist(&tmpImg, 1, channels, Mat(), hist1, 2, histSize, ranges, true, false);
			normalize(hist1, hist1, 0, 1, NORM_MINMAX, -1, Mat());
			calcHist(&compared, 1, channels, Mat(), hist2, 2, histSize, ranges, true, false);
			normalize(hist2, hist2, 0, 1, NORM_MINMAX, -1, Mat());
			double similarityValue = compareHist(hist1, hist2, HISTCMP_CORREL);

			if (similarityValue >= 0.3)
			{
				std::cout << "not be moved out"<<"\n"<< std::endl;
				return false;
			}
			if (similarityValue<0)
			{
				std::cout << "even though it is not possible, return false" << "\n" << std::endl;
				return false;
			}

			return true;
		}
		else
		{

			Mat tmpImg;
			resize(patch, tmpImg, Size(compared.cols, compared.rows));


			cv::Mat matDst1, matDst2;

			cv::resize(tmpImg, matDst1, cv::Size(8, 8), 0, 0, cv::INTER_CUBIC);
			cv::resize(compared, matDst2, cv::Size(8, 8), 0, 0, cv::INTER_CUBIC);

			cv::cvtColor(matDst1, matDst1, COLOR_BGR2GRAY);
			cv::cvtColor(matDst2, matDst2, COLOR_BGR2GRAY);

			int iAvg1 = 0, iAvg2 = 0;
			int arr1[64], arr2[64];

			for (int i = 0; i < 8; i++) {
				uchar* data1 = matDst1.ptr<uchar>(i);
				uchar* data2 = matDst2.ptr<uchar>(i);

				int tmp = i * 8;

				for (int j = 0; j < 8; j++) {
					int tmp1 = tmp + j;

					arr1[tmp1] = data1[j] / 4 * 4;
					arr2[tmp1] = data2[j] / 4 * 4;

					iAvg1 += arr1[tmp1];
					iAvg2 += arr2[tmp1];
				}
			}

			iAvg1 /= 64;
			iAvg2 /= 64;

			for (int i = 0; i < 64; i++) {
				arr1[i] = (arr1[i] >= iAvg1) ? 1 : 0;
				arr2[i] = (arr2[i] >= iAvg2) ? 1 : 0;
			}

			int iDiffNum = 0;

			for (int i = 0; i < 64; i++)
				if (arr1[i] != arr2[i])
					++iDiffNum;

			cout << "iDiffNum = " << iDiffNum << endl;

			if (iDiffNum > 10)
			{
				cout << "Be moved out" << endl;
				return true;
			}
			else
			{
				cout << "not be moved out" << endl;
				return false;
			}

		}
	}
}
