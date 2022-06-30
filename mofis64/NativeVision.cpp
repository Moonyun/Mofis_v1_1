#include "NativeVision.h"

std::vector<CellInfo> GetResult(cv::Mat src) {
	int width = 100, height = 100;
	int img_w = 480, img_h = 480;
	float beta = 0.5;
	std::vector<CellInfo> res;
	CellInfo cellinfo;
	/*if (!src.data)
		std::cerr << "Problem loading image!!!" << std::endl;*/
	cv::Mat gray, edge, src_1, src_2, src_c, src_4;

	resize(src, src_c, Size(0, 0), beta, beta, 4);

	boxFilter(src_c, src_1, -1, Size(3, 3));

	Scalar mean = cv::mean(src_1);
	src_1 = src_1 + 140 - mean[0];

	threshold(src_1, src_2, 110, 255, THRESH_BINARY);
	dilate(src_2, src_2, getStructuringElement(2, Size(4, 4)));

	//waitKey(0);
	vector<vector<Point>> contours_small_img;
	findContours(255 - src_2, contours_small_img, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	if (!contours_small_img.size())
		return res;

	Mat src_3 = Mat::zeros(src_2.size(), CV_8U);
	drawContours(src_3, contours_small_img, -1, Scalar(255), 1);

	//src_3 = 255 - src_3;
	resize(src_3, src_4, Size(img_w, img_h));

	std::vector<std::vector<Point> > contours_big_img;
	vector<Vec4i> hierarchys;
	findContours(src_4, contours_big_img, hierarchys,
		cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));  //寻找轮廓
	for (unsigned int i = 0; i < contours_big_img.size(); ++i)
	{
		float cell_area = contourArea(contours_big_img[i]) * UM_2_PER_PIXEL;          //求细胞面积

		RotatedRect rectPoint = minAreaRect(contours_big_img[i]);

		if (rectPoint.center.x - width / 2 <= 0 || rectPoint.center.x + width / 2 > img_w - 1 || rectPoint.center.y - height / 2 <= 0 || rectPoint.center.y + height / 2 > img_h - 1)
			continue;

		if (rectPoint.center.x - width / 2 <= 0)
			rectPoint.center.x = width / 2 + 1;
		if (rectPoint.center.x + width / 2 > img_w - 1)
			rectPoint.center.x = img_w - 1 - width / 2;
		if (rectPoint.center.y - height / 2 <= 0)
			rectPoint.center.y = height / 2 + 1;
		if (rectPoint.center.y + height / 2 > img_h - 1)
			rectPoint.center.y = img_h - 1 - height / 2;

		cv::Mat roiImg = src(Rect(rectPoint.center.x - width / 2, rectPoint.center.y - height / 2, width, height));
		//cv::Mat roiImg = src;               //不分割，直接保存原图；
		float cell_peri = arcLength(contours_big_img[i], true) * UM_PER_PIXEL;     //求细胞周长

		if (cell_peri > 900.0 || cell_peri < 6)
			//roiImg.release();
			continue;

		cv::Point2f center;
		double cell_dia = 0.0;
		double cell_short_axis = 0.0;
		double cell_long_axis = 0.0;
		double cell_vol = 0.0;
		double cell_eccentricity = 0.0;
		double cell_roundness = 0.0;
		RotatedRect ell;
		ell = minAreaRect(contours_big_img[i]);
		cell_short_axis = MIN(ell.size.height, ell.size.width);
		cell_long_axis = MAX(ell.size.height, ell.size.width);
		cell_dia = 2 * sqrt(cell_area / pi);                //等效直径
		//cell_dia = UM_PER_PIXEL*(cell_short_axis+ cell_long_axis)/2;               
		cell_vol = pow(cell_dia, 3) * pi / 6;    //等效体积
		cell_eccentricity = (cell_long_axis - cell_short_axis) / cell_long_axis;   //根据长短轴求偏心率

		cell_roundness = cell_area / pow(cell_peri, 2) * 4 * pi;

		cellinfo = { roiImg, cell_area, cell_peri, cell_dia ,cell_short_axis * UM_PER_PIXEL,cell_long_axis * UM_PER_PIXEL,cell_vol,cell_eccentricity, cell_roundness };
		res.push_back(cellinfo);
		//roiImg.release();
	}
	//释放cv::mat
	//gray.release(), edge.release(), src_1.release(), src_2.release(), src_c.release(), src_4.release();

	return res;
}

NativeVision::NativeVision(LogWindows* pLogWindow,const int inThreadNums)
{	
	m_log_window = pLogWindow;                      //输出窗口
	/*m_class_num = in_class_num;                   //分类种类
	m_detect_image_preview_num = in_detect_image_preview_num;*/     //显示分类的种类
	pStreamSource = NULL;
	detect_images = std::vector<std::queue<cv::Mat>>(m_class_num, std::queue<cv::Mat>());    //切割出细胞的信息，包含图像与类别
	for (int i = 0; i < inThreadNums; i++) {
		std::vector<ImageInfo> tempImageInfos;
		std::vector<CellInfo> tempCellInfos;
		m_s_cells.push_back(tempCellInfos);
		m_s_images.push_back(tempImageInfos);
	}

	m_thread_nums = inThreadNums;
	m_analyze_progress = std::vector<float>(inThreadNums, 0.0);
}

NativeVision::NativeVision() {
}

NativeVision::~NativeVision()
{
}

//相机初始化
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
		//getchar();
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

//设置相机参数
bool NativeVision::SetCamParas(int inWidth, int inHeight, int inOffsetX, double inExposureTime, double inAcquisitionFrameRate) {
	if (pCamera != NULL) {
		m_log_window->AddLog("camera !\n");

		if (modifyCameraWidth(pCamera, inWidth) == 0 &&
			modifyCameraHeight(pCamera, inHeight) == 0 &&
			modifyCameraOffsetX(pCamera, inOffsetX) == 0 &&
			modifyCameraExposureTime(pCamera, inExposureTime) == 0 &&
			modifyCameraAcquisitionFrameRate(pCamera, inAcquisitionFrameRate) == 0) {
			m_log_window->AddLog("set camera paras success!\n");
		}

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
		std::shared_lock<std::shared_mutex> guard(steamsource_mutex);

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
		std::shared_lock<std::shared_mutex> guard(steamsource_mutex);

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
				std::shared_lock<std::shared_mutex> guard(image_mutex);
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
		std::shared_lock<std::shared_mutex> guard(image_mutex);
		if (!image.empty() && imagePath.size() > 0) {
			image_queue.push(std::make_pair(image, imagePath));
			m_log_window->AddLog("set image queue size : %d \n", image_queue.size());
		}
	}
}

cv::Mat NativeVision::OperateImageQueue(cv::Mat inImage, bool bInsert) {
	std::shared_lock<std::shared_mutex> guard(images_mutex);
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

void NativeVision::InsertQueue(cv::Mat inImage) {
	std::shared_lock<std::shared_mutex> guard(images_mutex);
	while (!images_queue.empty()) {
		images_queue.pop();
	}
	images_queue.push(inImage);
}

std::vector<cv::Mat> NativeVision::OperateDetectImageQueue(cv::Mat inImage, bool bInsert, int in_class_index) {
	std::shared_lock<std::shared_mutex> guard(detect_images_mutex);
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





bool NativeVision::BSaveValid() {
	//std::shared_lock<std::shared_mutex> guard(steamsource_mutex);
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
				cv::Mat image;
				image = cv::Mat(pFrame->getImageHeight(pFrame),
					pFrame->getImageWidth(pFrame),
					CV_8U,
					(uint8_t*)((pFrame->getImage(pFrame)))
				);

				if (inBGet && ins < m_thread_nums) {
					ImageInfo image_info;
					image_info.m_image = image.clone();
					image_info.m_id = inId;
					image_info.m_name = inName;
					image_info.m_second = inSecond;
					//m_s_images.at(ins).push_back(image_info);
					{
						std::unique_lock<std::shared_mutex> m_s_images_guard(m_s_images_mutex);
						m_s_images[ins].push_back(image_info);
					}
				}

				if (b_time_to_preview) {
					InsertQueue(image.clone());
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
//		GetResult(m_total_images[i].m_image);
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
	for (int i = 0; i < m_thread_nums; i++)
	{
		sum += m_analyze_progress[i];
	}
	return sum/ m_thread_nums;
}

int NativeVision::GetTotalImageSize() {
	int sum = 0.0;
	for (int i = 0; i< m_thread_nums; i++)
	{
		sum += m_s_images[i].size();
	}
	return sum;
}

std::vector<CellInfo> NativeVision::GetTotalCells() {
	m_total_cells.reserve(10000);
	for (int i = 0; i < m_thread_nums; i++)
	{
		m_total_cells.insert(m_total_cells.end(), m_s_cells[i].begin(), m_s_cells[i].end());
		
	}
	for (int i = 0; i < m_thread_nums; i++)
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

	for (int i = 0; i < m_thread_nums; i++)
	{
		std::vector<CellInfo> temp_cell_infos;
		m_s_cells.at(i).swap(temp_cell_infos);

		std::vector<ImageInfo> temp_images;
		m_s_images[i].swap(temp_images);

		m_analyze_progress[i] = 0.0;
	}

}


//m_thread_nums个不同分析线程调用的函数
void NativeVision::AnalyzeImages0_9(const std::string inSampleName, const std::string inRootPath, const int ins, bool insave_cell, bool save_original_img) {
	m_s_cells.at(ins).reserve(1000);
	int cell_num = 0;                       //代表样本中细胞数目编号
	std::vector<CellInfo> cellInfos;
	CellInfo cell_info;
	std::vector<ImageInfo>* current_images;
	{
		//std::shared_lock<std::shared_mutex> m_s_images_guard(m_s_images_mutex);
		current_images = &(m_s_images[ins]);
	}
	clock_t t_start = clock();
	for (int i = 0; i < current_images->size(); ++i) {
		//m_log_window->AddLog("%d in %d of %d \n", ins,i, current_images.size());

		cell_num = 0;
		m_analyze_progress[ins] = (static_cast<float>(i) + 1) / current_images->size();

		cv::Mat temp_image;
		{
			//std::shared_lock<std::shared_mutex> m_s_images_guard(m_s_images_mutex);
			temp_image = current_images->at(i).m_image;
		}
		cellInfos =GetResult(temp_image);
		temp_image.release();

		for (int j = 0; j < cellInfos.size(); ++j) {
			cell_info = cellInfos[j];
			cell_info.m_second = current_images->at(i).m_second;
			//cell_info.m_class = Classify(cell_info.m_diameter);
			cell_info.m_id = current_images->at(i).m_id;
			cell_info.m_name = inSampleName + "_" + current_images->at(i).m_name +"_"+ std::to_string(cell_num)   + ".bmp";
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
			std::filesystem::path save_path = img_root_path / std::filesystem::path(current_images->at(i).m_name + std::to_string(i) + ".bmp");
			cv::imwrite(save_path.string(), current_images->at(i).m_image);
		}
	}

	int time = clock()- t_start;
	m_log_window->AddLog("%d completed %d - %d -----------------------\n", ins, time, current_images->size());

	std::vector<ImageInfo> temp;
	{
		std::unique_lock<std::shared_mutex> m_s_images_guard(m_s_images_mutex);
		m_s_images[ins].swap(temp);
	}

}

