#include "NativeVision.h"

NativeVision::NativeVision(LogWindows* pLogWindow)
{	
	m_log_window = pLogWindow;                      //输出窗口
	/*m_class_num = in_class_num;                   //分类种类
	m_detect_image_preview_num = in_detect_image_preview_num;*/     //显示分类的种类
	pStreamSource = NULL;
	m_celldetect = CellDetect();
	detect_images = std::vector<std::queue<cv::Mat>>(m_class_num, std::queue<cv::Mat>());    //切割出细胞的信息，包含图像与类别
	for (int i = 0; i < 10; i++) {
		std::vector<ImageInfo> tempImageInfos;
		std::vector<CellInfo> tempCellInfos;
		m_s_images.push_back(tempImageInfos);
		m_s_cells.push_back(tempCellInfos);
	}

}

NativeVision::NativeVision() {
}

NativeVision::~NativeVision()
{
}

//设置相机参数
bool NativeVision::SetCamParas(int inWidth, int inHeight, int inOffsetX, double inExposureTime, double inAcquisitionFrameRate) {
	if (pCamera != NULL) {

		return (
			modifyCameraWidth(pCamera, inWidth) == 0 &&
			modifyCameraHeight(pCamera, inHeight) == 0 &&
			modifyCameraOffsetX(pCamera, inOffsetX) == 0 &&
			modifyCameraExposureTime(pCamera, inExposureTime) == 0 &&
			modifyCameraAcquisitionFrameRate(pCamera, inAcquisitionFrameRate) == 0
			);
	}
	else {
		m_log_window->AddLog("no camera when set paras\n");
		return false;
	}
}

//开始捕获图像
bool NativeVision::StartCapture() {
	if (pCamera != NULL && pStreamSource == NULL) {
		std::lock_guard<std::mutex> guard(steamsource_mutex);

		int ret = GENICAM_CreateStreamSource(pCamera, &pStreamSource);
		if ((ret != 0) || (NULL == pStreamSource)) {
			m_log_window->AddLog("create stream obj  fail.\r\n");
			//getchar();
			return false;
		}
		// 开始拉流  
		// Start grabbing    
		ret = GENICAM_startGrabbing(pStreamSource);
		if (ret != 0) {
			m_log_window->AddLog("StartGrabbing  fail.\n");
			//注意：需要释放pStreamSource内部对象内存    
			//Attention: should release pStreamSource internal object before return     
			pStreamSource->release(pStreamSource);
			pStreamSource = NULL;
			//getchar();
			return false;
		}
		m_log_window->AddLog("start capture\n");
		return true;
	}
	else {
		m_log_window->AddLog("no camera when start capture \n");
		return false;
	}
}

//停止捕获图像
bool NativeVision::StopCapture() {
	if (pCamera != NULL && pStreamSource != NULL) {
		std::lock_guard<std::mutex> guard(steamsource_mutex);

		int ret = GENICAM_stopGrabbing(pStreamSource);
		if (ret != 0) {
			m_log_window->AddLog("StopGrabbing  fail.\n");
			//注意：需要释放pStreamSource内部对象内存    
			//Attention: should release pStreamSource internal object before return     
			pStreamSource->release(pStreamSource);
			//getchar();
			return false;
		}
		pStreamSource->release(pStreamSource);
		pStreamSource = NULL;

		m_log_window->AddLog("stop capture\n");
		return true;
	}
	else {
		m_log_window->AddLog("no camear when stop capture \n");
		return false;
	}
}

void NativeVision::SaveImageFromQueue() {
	std_time time_start = std::chrono::system_clock::now();
	while (true) {
		if (!m_bSave || std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - time_start).count() > image_save_interval) {
			cv::Mat image;
			std::string path = "";
			{
				std::lock_guard<std::mutex> guard(image_mutex);
				if (!image_queue.empty()) {
					auto ele = image_queue.front();
					path = ele.second;
					image = ele.first;
					m_log_window->AddLog("get image queue size : %d \n", image_queue.size());
					image_queue.pop();
				}
			}
			if (!image.empty() && path.size() > 0) {
				//std::cout << "saving : " << path << std::endl;
				cv::imwrite(path, image);
			}
			image.release();
			time_start = std::chrono::system_clock::now();
		}
	}
}
void NativeVision::InsertImageQueue(cv::Mat image, const std::string& imagePath) {
	{
		std::lock_guard<std::mutex> guard(image_mutex);
		if (!image.empty() && imagePath.size() > 0) {
			image_queue.push(std::make_pair(image, imagePath));
			m_log_window->AddLog("set image queue size : %d \n", image_queue.size());
		}
	}
}

cv::Mat NativeVision::OperateImageQueue(cv::Mat inImage, bool bInsert) {
	std::lock_guard<std::mutex> guard(images_mutex);
	if (bInsert) {
		//std::cout << "insert " << std::endl;
		while (!images_queue.empty()) {
			images_queue.pop();
		}
		images_queue.push(inImage);
	}
	else {
		//std::cout << "get  " << std::endl;
		if (!images_queue.empty()) {
			return images_queue.front();
		}
	}
	return cv::Mat();
}

std::vector<cv::Mat> NativeVision::OperateDetectImageQueue(cv::Mat inImage, bool bInsert, int in_class_index) {
	std::lock_guard<std::mutex> guard(detect_images_mutex);
	std::vector<cv::Mat> res;
	if (bInsert) {
		//std::cout << "insert " << std::endl;
		detect_images.at(in_class_index).push(inImage);
		while (detect_images[in_class_index].size() > m_detect_image_preview_num) {
			detect_images[in_class_index].pop();
		}
	}
	else {
		//std::cout << "get  " << std::endl;
		while (!detect_images.at(in_class_index).empty()) {
			res.push_back(detect_images[in_class_index].back());
			detect_images[in_class_index].pop();
		}
	}
	return res;
}



bool NativeVision::Init()
{
	int32_t ret;
	GENICAM_System* pSystem = NULL;
	GENICAM_Camera* pCameraList = NULL;
	GENICAM_AcquisitionControl* pAcquisitionCtrl = NULL;
	uint32_t cameraCnt = 0;
	HANDLE threadHandle;
	unsigned threadID;
	int cameraIndex = -1;
	ret = GENICAM_getSystemInstance(&pSystem);
	if (-1 == ret) {
		m_log_window->AddLog("pSystem is null.\r\n");
		getchar();
		return false;
	}    // 发现设备     // discover camera     
	ret = pSystem->discovery(pSystem, &pCameraList, &cameraCnt, typeAll);
	if (-1 == ret) {
		m_log_window->AddLog("discovery device fail.\r\n");
		//getchar();
		return false;
	}
	if (cameraCnt < 1) {
		m_log_window->AddLog("no camera when init\r\n");
		//getchar();
		return false;
	}
	// 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）     
	// Print camera info (Index, Type, Vendor, Model, Serial number, DeviceUserID, IP Address)     
	//displayDeviceInfo(pCameraList, cameraCnt);

	// 选择需要连接的相机     
	// Select one camera to connect to      
	//cameraIndex = selectDevice(cameraCnt);
	cameraIndex = 0;
	pCamera = &pCameraList[cameraIndex];
	// 连接设备     
	// Connect to camera     
	ret = GENICAM_connect(pCamera);
	if (0 != ret) {
		m_log_window->AddLog("connect camera failed.\n");
		//getchar();
		return false;
	}
	else {
		m_log_window->AddLog("connect success ! \n");
	}
	return true;
}

bool NativeVision::BSaveValid() {
	//std::lock_guard<std::mutex> guard(steamsource_mutex);
	return pStreamSource == NULL ? false : true;
}

void createAlphaMat(cv::Mat& mat) // 创建一个图像
{
	int randNum = rand() % UCHAR_MAX;
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			mat.at<uchar>(i, j) = (randNum + 100 * i + j) % UCHAR_MAX;
		}
	}
}



bool NativeVision::SaveAsync() {
	return true;
	return m_bSaveAsync;
}

//分类算法,后续补充
int NativeVision::Classify(double inDia) {
	// to do: classify 
	if (inDia <= 5) {
		return 0;
	}
	else if (5 < inDia && inDia < 15)
	{
		return 1;
	}

	return 2;
}


//筛选算法
int NativeVision::Filter(filter_info inSetfilter, const std::string inStore_path, vector<CellInfo> inTotalcellinfo)
{
	int s_cell_num = 0;
	return s_cell_num;
}


//捕获图像
bool NativeVision::GetImages(int inId, int inSecond, const std::string inName, bool inBGet ,const int ins) {

	bool b_time_to_preview = false;
	if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_time_old).count() > image_preview_interval) {
		b_time_to_preview = true;
		m_time_old = std::chrono::system_clock::now();
	}
	int32_t ret = 0;
	uint64_t blockId = 0;
	GENICAM_Frame* pFrame;
	std_time ts = std::chrono::system_clock::now();
	{
		if (NULL == pStreamSource)
		{
			return false;
		}
		else {
			if (pStreamSource != NULL) {
				ret = pStreamSource->getFrame(pStreamSource, &pFrame, 1);
				if (ret < 0) {
					m_log_window->AddLog("getFrame  fail.\n");
					this->StopCapture();
					Sleep(1);
					this->StartCapture();
					return false;
				}

				ret = pFrame->valid(pFrame);
				if (ret < 0) {
					m_log_window->AddLog("frame is invalid!\n");
					pFrame->release(pFrame);
					return false;
				}
				cv::Mat image = cv::Mat(pFrame->getImageHeight(pFrame),
					pFrame->getImageWidth(pFrame),
					CV_8U,
					(uint8_t*)((pFrame->getImage(pFrame)))
				);

				if (inBGet && ins < 10) {
					ImageInfo image_info;
					image_info.m_image = image.clone();
					image_info.m_id = inId;
					image_info.m_name = inName;
					image_info.m_second = inSecond;
					m_s_images.at(ins).push_back(image_info);
				}


				if (b_time_to_preview) {
					OperateImageQueue(image.clone(), true);
				}

				image.release();
				pFrame->release(pFrame);
			}
		}
	}
	return true;

}

////原始图像分析，切割出细胞
//void NativeVision::AnalyzeImages(const std::string inSampleName, const std::string inRootPath, bool insave_cell, bool save_original_img) {
//	m_total_cells.reserve(10000);
//	int cell_num = 0;                       //代表样本中细胞数目编号
//	std::vector<CellInfo> cellInfos;
//	for (int i = 0; i < m_total_images.size(); ++i) {
//		m_analyze_progress = static_cast<float>(i) / m_total_images.size();
//		cellInfos = m_celldetect.GetResult(m_total_images[i].m_image);
//		for (int j = 0; j < cellInfos.size(); ++j) {
//			CellInfo cell_info = cellInfos[j];
//			cell_info.m_second = m_total_images[i].m_second;
//			//cell_info.m_class = Classify(cell_info.m_diameter);
//			cell_info.m_id = m_total_images[i].m_id;
//			cell_info.m_name = inSampleName + "_" + m_total_images[i].m_name + std::to_string(cell_num) + "_" + std::to_string(cell_info.m_class) + "_"  + ".bmp";
//			m_total_cells.push_back(cell_info);
//			cell_num++;
//			std::filesystem::path root_path = std::filesystem::path(inRootPath) / std::filesystem::path(std::to_string(cell_info.m_id));
//			//std::filesystem::path root_path = std::filesystem::path(inRootPath);
//			if (!std::filesystem::exists(root_path)) {
//				std::filesystem::create_directories(root_path);
//			}
//			std::filesystem::path save_path = root_path / std::filesystem::path(cell_info.m_name);
//			cell_info.m_path = save_path.string();
//			//m_log_window->AddLog("save img in %s \n", save_path.string().c_str());
//			if (insave_cell) {
//				cv::imwrite(save_path.string(), cell_info.m_image);
//			}
//		}
//		if (save_original_img)
//		{
//			std::filesystem::path img_root_path = std::filesystem::path(inRootPath) / std::filesystem::path(std::string("raw"));
//			if (!std::filesystem::exists(img_root_path)) {
//				std::filesystem::create_directories(img_root_path);
//			}
//			std::filesystem::path save_path = img_root_path / std::filesystem::path(m_total_images[i].m_name + std::to_string(i) + ".bmp");
//			cv::imwrite(save_path.string(), m_total_images[i].m_image);
//		}
//	}
//	std::vector<ImageInfo> temp;
//	m_total_images.swap(temp);
//}


float NativeVision::GetAnalyzeProgress() {
	float sum = 0.0;
	for (int i = 0; i < 10; i++)
	{
		sum += m_analyze_progress[i];
	}
	return sum/10.0;
}

int NativeVision::GetTotalImageSize() {
	int sum = 0.0;
	for (int i = 0; i<10; i++)
	{
		sum += m_s_images.at(i).size();
	}
	return sum;
}

std::vector<CellInfo> NativeVision::GetTotalCells() {
	m_total_cells.reserve(10000);
	for (int i = 0; i < 10; i++)
	{
		m_total_cells.insert(m_total_cells.end(), m_s_cells[i].begin(), m_s_cells[i].end());
		
	}

	for (int i = 0; i < 10; i++)
	{
		std::vector<CellInfo> temp_cell_infos;
		m_s_cells[i].swap(temp_cell_infos);

	}


	return m_total_cells;
}

void NativeVision::Clear() {
	std::vector<CellInfo> temp_cell_infos;
	std::vector<ImageInfo> temp_images;
	
	m_total_cells.swap(temp_cell_infos);
	m_total_images.swap(temp_images);

	for (int i = 0; i < 10; i++)
	{
		std::vector<CellInfo> temp_cell_infos;
		m_s_cells.at(i).swap(temp_cell_infos);

		std::vector<ImageInfo> temp_images;
		m_s_images.at(i).swap(temp_images);

		m_analyze_progress[i] = 0.0;
	}

}


//10个不同分析线程调用的函数
void NativeVision::AnalyzeImages0_9(const std::string inSampleName, const std::string inRootPath, const int ins, bool insave_cell, bool save_original_img) {
	m_s_cells.at(ins).reserve(10000);
	int cell_num = 0;                       //代表样本中细胞数目编号
	std::vector<CellInfo> cellInfos;
	CellInfo cell_info;
	for (int i = 0; i < m_s_images.at(ins).size(); ++i) {
		cell_num = 0;

		m_analyze_progress[ins] = (static_cast<float>(i)+1) / m_s_images.at(ins).size();
		cellInfos = m_celldetect.GetResult(m_s_images.at(ins)[i].m_image);

		for (int j = 0; j < cellInfos.size(); ++j) {
			cell_info = cellInfos[j];
			cell_info.m_second = m_s_images.at(ins)[i].m_second;
			//cell_info.m_class = Classify(cell_info.m_diameter);
			cell_info.m_id = m_s_images.at(ins)[i].m_id;
			cell_info.m_name = inSampleName + "_" + m_s_images.at(ins)[i].m_name +"_"+ std::to_string(cell_num) + "_" + std::to_string(cell_info.m_class) + "_"  + ".bmp";
			m_s_cells.at(ins).push_back(cell_info);
			cell_num++;
			std::filesystem::path root_path = std::filesystem::path(inRootPath) / std::filesystem::path(std::to_string(cell_info.m_id));
			//std::filesystem::path root_path = std::filesystem::path(inRootPath);
			if (!std::filesystem::exists(root_path)) {
				std::filesystem::create_directories(root_path);
			}
			std::filesystem::path save_path = root_path / std::filesystem::path(cell_info.m_name);
			cell_info.m_path = save_path.string();
			//m_log_window->AddLog("save img in %s \n", save_path.string().c_str());
			if (insave_cell) {
				cv::imwrite(save_path.string(), cell_info.m_image);
			}
		}
		if (save_original_img)
		{
			std::filesystem::path img_root_path = std::filesystem::path(inRootPath) / std::filesystem::path(std::string("raw"));
			if (!std::filesystem::exists(img_root_path)) {
				std::filesystem::create_directories(img_root_path);
			}
			std::filesystem::path save_path = img_root_path / std::filesystem::path(m_s_images.at(ins)[i].m_name + std::to_string(i) + ".bmp");
			cv::imwrite(save_path.string(), m_s_images.at(ins)[i].m_image);
		}
	}

	std::vector<ImageInfo> temp;
	m_s_images.at(ins).swap(temp);

}

