#ifndef NATIVEVISION_H
#define NATIVEVISION_H

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "camera_control.h"
#include <iostream>
#include <chrono>
#include <stdlib.h>
#include "LogWindows.h"
#include "CellDetect.h"
#include <filesystem>
#include <mutex>
#include <shared_mutex>

using namespace cv;

using namespace std::literals;
typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<long long int, std::ratio<1, 1000000000>>> std_time;
typedef std::unordered_map<int, double> CameraParameters;

struct filter_info {               //ɸѡ����
	float dia_min;
	float dia_max;
	float peri_min;
	float peri_max;
	float area_min;
	float area_max;
	float vol_min;
	float vol_max;
	float ell_min;
	float ell_max;
};

class NativeVision {
public:
	NativeVision();
	NativeVision(LogWindows* pLogWindow, const int inThreadNums);
	~NativeVision();
	bool Init();
	uint32_t GetDeviceNum();
	std::vector<CellInfo> GetFrame(bool bSave, bool bSaveAsync, const std::string inSaveName, const std::string inSaveRootPath);
	bool GetImages(int inId, int inSecond, const std::string inName, bool inBSave,const int ins);
	void AnalyzeImages(const std::string inSampleName, const std::string inRootPath, bool save_cell, bool save_original_img);
	bool SetCamParas(int inWidth, int inHeight, int inOffsetX, double inExposureTime, double inAcquisitionFrameRate);
	bool StartCapture();
	bool StopCapture();
	bool BSaveValid();
	void SaveImageFromQueue();
	void InsertImageQueue(cv::Mat image, const std::string& imagePath);
	bool SaveAsync();
	cv::Mat OperateImageQueue(cv::Mat inImage, bool bInsert);
	void InsertQueue(cv::Mat inImage);
	std::vector<cv::Mat> OperateDetectImageQueue(cv::Mat inImage, bool bInsert, int in_class_index);
	int Classify(double inDia);
	int Filter(filter_info inSetfilter, const std::string inStore_path, vector<CellInfo> inTotalcellinfo);
	float GetAnalyzeProgress();
	int GetTotalImageSize();
	std::vector<CellInfo> GetTotalCells();
	void Clear();
	void AnalyzeImages0_9(const std::string inSampleName, const std::string inRootPath, const int ins, bool insave_cell, bool save_original_img);

private:

	std::string m_save_path;
	int m_num;
	GENICAM_StreamSource* pStreamSource;
	GENICAM_Camera* pCamera = NULL;
	std::shared_mutex steamsource_mutex;
	std::shared_mutex image_path_mutex;
	std::shared_mutex image_mutex;
	std::shared_mutex images_mutex;
	std::shared_mutex detect_images_mutex;
	std::queue<std::string> image_path_queue;
	std_time m_time_old = std::chrono::system_clock::now();
	int image_preview_interval = 100;
	int image_save_interval = 100;
	std::queue<std::pair<cv::Mat, std::string> > image_queue;
	std::queue<cv::Mat> images_queue;

	std::vector<std::queue<cv::Mat>> detect_images;
	bool m_bSaveAsync = false;
	bool m_bSave = false;
	LogWindows* m_log_window;
	int m_class_num;
	int m_detect_image_preview_num;

	std::vector<ImageInfo> m_total_images;
	std::vector<CellInfo> m_total_cells;
	std::vector < std::vector<ImageInfo>> m_s_images;
	std::vector<std::vector<CellInfo>> m_s_cells;


	int m_thread_nums = 0;
	std::vector<float> m_analyze_progress;
	std::shared_mutex m_s_images_mutex;



};


#endif