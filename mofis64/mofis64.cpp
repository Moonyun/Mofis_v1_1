// mofis64.cpp : 定义应用程序的入口点。
//

#include <iostream>
#include <tchar.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <Windows.h>
#include <unordered_map>
#include <tuple>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <Shlobj.h>
#include <opencv2/opencv.hpp>

//gui工具
#include <glad/glad.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot-master/implot.h"
#include <GLFW/glfw3.h>
#include "stbimage/stb_image.h"
#include "stbimage/stb_image_resize.h"

//sdk外部调用
#include "Flowcam\IInterface.h"

//自写文件
#include "framework.h"
#include "mofis64.h"
#include "camera_control.h"        //相机控制
#include "laser_control.h"        //光源控制
#include "NativeVision.h"


//全局变量参数

LogWindows* g_LogWindow = new LogWindows();       // global logger

IInterface* g_flow ;            //液路控制
int g_flow_p0;                        //液路的function type
int g_flow_p1;                        //液路的para1
int g_flow_p2;                        //液路的para2
int g_flow_p3;                        //液路的para3

int g_status;                        //全局状态表示
int g_s = 0;
int g_posID = 0;                     //样本Id
bool g_savecells = TRUE;            //代表是否保存细胞图像
bool g_saveallimgs = false;
bool g_GetImgs = FALSE;              //代表是否保存相机照片，以作细胞切割

bool g_get = true;

std::string g_file_path ="E:\\Data\\CameraImages\\20220610\\test";

int g_cell_total;
int g_img_total;


std::vector<float> g_cell_total_array;     //细胞形态学参数
std::vector<float> g_cell_area_array;
std::vector<float> g_cell_peri_array;
std::vector<float> g_cell_dia_array;
std::vector<float> g_cell_short_array;
std::vector<float> g_cell_long_array;
std::vector<float> g_cell_vol_array;
std::vector<float> g_cell_ell_array;
std::vector<float> g_cell_round_array;

int g_cam_width = 480;
int g_cam_height = 480;
int g_cam_width_offset = 80;
int g_cam_exposure = 171;
int g_cam_fps = 2000;
int g_detect_class_num = 3;                //预览细胞的种类，既行数
int g_detect_preview_num = 8;
//std::vector<std::vector<std::string> > g_detect_img_paths(g_detect_class_num, std::vector<std::string>(g_detect_preview_num, ""));
std::vector<std::vector<GLuint> > g_detect_tex_ids(g_detect_class_num, std::vector<GLuint>(g_detect_preview_num, 0));

std::vector<CellInfo> g_show_cellinfos;

//set filter index
bool set_dia = false;
bool set_peri = false;
bool set_area = false;
bool set_vol = false;
bool set_ell = false;
std::vector<Condition> g_conditions;
int g_set_filter_store = 0;
std::string g_img_filter_path = "";

int g_cell_filter = 0;

std::vector<std::string> g_condition_names = { u8"直径",u8"面积",u8"周长",u8"体积",u8"偏心率",u8"圆度" };
int g_property_int = 0;
int g_logic = 0;
std::vector<CellInfo> g_filter_cellinfos;


//液路参数回调
void UpdateInstateCallback(int in_function_type, int in_para1, int in_para2, int in_para3)
{
    g_flow_p0 = in_function_type;
    g_flow_p1 = in_para1;
    g_flow_p2 = in_para2;
    g_flow_p3 = in_para3;
}


//液路初始化
void FlowInit()
{
    //::OutputDebugStringA("#22 FlowInit.\n");
    IInterface* pShowUI = IInterface::CreateInstance();
    g_flow = pShowUI;
    pShowUI->SetInitStateCallback(&UpdateInstateCallback);   //设置事件响应回调
    pShowUI->Init();   //初始化
}

//清理vector
template <typename T>
void ClearVector(std::vector<T>& inVector) {
    std::vector<T> init_vector;
    inVector.swap(init_vector);
}

float VectorAverage(const std::vector<float>& v) {
    float avg = 0;
    for (const int& e : v) {
        avg += e;
    }
    avg /= v.size();
    return avg;
}

//灰度转3通道
cv::Mat GrayToRGB(cv::Mat in_gray_image) {
    cv::Mat channels[3];
    channels[0] = in_gray_image;
    channels[1] = in_gray_image;
    channels[2] = in_gray_image;
    cv::Mat img;
    cv::merge(channels, 3, img);
    return img;
}

//拉流保存图片
void GetImage(NativeVision* in_camera) {

    int second = 0;
    std::string old_time_str;
    struct tm time_info;
    time_t raw_time;
    int image_id = 0;
    auto time_now = std::chrono::steady_clock::now();
    if (in_camera->Init()) {
        in_camera->SetCamParas(g_cam_width, g_cam_height, g_cam_width_offset, g_cam_exposure, g_cam_fps);
        in_camera->StartCapture();
    }
    while (g_get) {
        {
            if (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_now).count() > 1100) {
                time_now = std::chrono::steady_clock::now();
                time(&raw_time);
                localtime_s(&time_info, &raw_time);
                char time_info_buffer[100];
                strftime(time_info_buffer, 100, "%G_%m_%d_%H%M%S", &time_info);
                if (std::string(time_info_buffer) != old_time_str)
                {
                    old_time_str = std::string(time_info_buffer);
                    second++;
                    image_id = 0;
                }
                else {
                    image_id++;
                }
                std::string image_name = std::string(time_info_buffer) + "_" +std::to_string(image_id);
                if (in_camera->GetImages(g_posID, second, image_name, g_GetImgs , g_s)) {
                    //g_img_per_second++;
                }
            }
        }
    }
}

//加载image到texture，显示图片
bool LoadTextureFromImage(cv::Mat in_image, GLuint* out_texture)
{
    glPixelStorei(GL_UNPACK_ALIGNMENT, (in_image.step & 3) ? 1 : 4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, static_cast<GLint>(in_image.step / in_image.elemSize()));
    GLenum internal_format = GL_RGB;
    if (in_image.channels() == 4) internal_format = GL_RGBA;
    if (in_image.channels() == 3) internal_format = GL_RGB;
    if (in_image.channels() == 2) internal_format = GL_RG;
    if (in_image.channels() == 1) internal_format = GL_RED;

    GLenum external_format = GL_BGR;
    if (in_image.channels() == 1) external_format = GL_RED;

    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    // Upload pixels into texture
    try {
        glTexImage2D(GL_TEXTURE_2D, 0, internal_format, in_image.cols, in_image.rows, 0, external_format, GL_UNSIGNED_BYTE, in_image.ptr());
    }
    catch (std::exception& e) {
        std::cout << "exception : " << e.what() << std::endl;
    }

    glDeleteTextures(1, out_texture);
    *out_texture = image_texture;
    return true;
}

//unicode转string
string unicode2string(LPCWSTR lps) {

    int len = WideCharToMultiByte(CP_ACP, 0, lps, -1, NULL, 0, NULL, NULL);
    if (len <= 0) {
        return "";
    }
    else {
        char* dest = new char[len];
        WideCharToMultiByte(CP_ACP, 0, lps, -1, dest, len, NULL, NULL);
        dest[len - 1] = 0;
        string str(dest);
        delete[] dest;
        return str;
    }
}

//状态更新
int StatusParamUpdata(int inFlow_p0, int inFlow_p1, bool inStart_flow)
{
    if(inFlow_p0 == 0){                //未开机
        return 0;
    }
    else if (inFlow_p0 == 58 && inFlow_p1 == 0) {      //开机中
        return 1;
    }
    else if (inFlow_p0 == 58 && inFlow_p1 == 1) {     //开机完成
        return 2;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 3) {    //正常等待
        return 3;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 4 && !inStart_flow) {    //准备中,进出仓状态
        return 4;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 4 && inStart_flow) {    //准备中,吸样，准备测试
        return 5;
    }
    else if (inFlow_p0 == 60 && inFlow_p1 == 1) {    //测试中
        return 6;
    }
    else if (inFlow_p0 == 60 && inFlow_p1 == 2) {    //测试完成，清洗管道中
        return 7;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 7) {    //维护中，执行维护功能
        return 8;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 5)                   //关机中
    {
        return 9;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 6)                   //已关机，等待关闭电源
    {
        return 10;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 10)                   //正在进入休眠
    {
        return 11;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 11)                   //休眠
    {
        return 12;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 13)                   //故障
    {
        return 13;
    }
    else if (inFlow_p0 == 57 && inFlow_p1 == 14)                   //故障
    {return 14;}
    else{ return 15;}                                              //未知
    
}

void ClearPlot() {
   
    ClearVector(g_cell_total_array);
    ClearVector(g_cell_area_array);
    ClearVector(g_cell_peri_array);
    ClearVector(g_cell_dia_array);
    ClearVector(g_cell_short_array);
    ClearVector(g_cell_long_array);
    ClearVector(g_cell_vol_array);
    ClearVector(g_cell_ell_array);
    ClearVector(g_cell_round_array);
    ClearVector(g_show_cellinfos);
    ClearVector(g_filter_cellinfos);
    g_cell_total = 0;
    g_img_total = 0;
    for (int i = 0; i < g_detect_tex_ids.size(); i++) {
        for (int j = 0; j < g_detect_tex_ids[0].size(); j++) {
            glDeleteTextures(1, &(g_detect_tex_ids[i][j]));
            g_detect_tex_ids[i][j] = 0;
        }
    }
}


//glfw错误回调
static void GlfwErrorCallback(int in_error, const char* in_description)
{
    //fprintf(stderr, "Glfw Error %d: %s\n", in_error, in_description);
}


//ui窗口函数
int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    //液路初始化
    FlowInit();

    // glfw init
    glfwSetErrorCallback(GlfwErrorCallback);
    if (!glfwInit())
        return 1;
    // opengl version
    const char* glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // create window
    int nScreenWidth = GetSystemMetrics(SM_CXSCREEN);
    int nScreenHeight = GetSystemMetrics(SM_CYSCREEN);
    GLFWwindow* window = glfwCreateWindow(nScreenWidth, nScreenHeight, "Mofis", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // glad bind to window
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        g_LogWindow->AddLog("Failed to initialize GLAD! \n");
        return -1;
    }

    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF("Microsoft-YaHei-Regular.ttc", 20, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
    ImGui::StyleColorsLight();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    //windows setup窗口属性参数
    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoScrollbar;

 

    //参数
    int pos = 39;                                                                    //默认采样试管位置
    const char* volume_items[] = { "10  uL","20  uL","30  uL","50  uL","100 uL" };   //吸样体积选取
    int volume_array[5] = { 10,20,30,50,100 };                                       //吸样具体数值
    int volume_current = 1;                                                          //默认选取吸样位置
    enum Speed_Element { Element_1, Element_2, Element_3, Element_COUNT };           //流速选择
    int speed_array[3] = { 10,20,30 };
    int speed_current_elem = Element_1;
    //int save_time = (volume_array[volume_current] / speed_array[speed_current_elem]) * 60 * 1000;
    int save_time = 40*1000;
    char save_time_cstr[128] ="";
    _itoa(save_time/1000, save_time_cstr,10);

    char img_width_cstr[128] = "480";                                              //相机控制的参数，长宽像素，曝光，设置帧率
    char img_offset_x_cstr[128] = "80";
    char img_height_cstr[128] = "480";
    char img_exposure_cstr[128] = "171";
    char img_acquisition_frame_rate_cstr[128] = "2000";

    char laser_frequency_cstr[128] = "5000";                                      //激光频率，脉宽，电流大小，串口
    char laser_width_cstr[128] = "200";
    char laser_intensity_cstr[128] = "100";
    int laser_frequency = atoi(laser_frequency_cstr);
    int laser_width = atoi(laser_width_cstr);
    int laser_intensity = atoi(laser_intensity_cstr);
    std::string serial_name = "COM4";
    char serial_name_cstr[128] = "COM4";                                     //代表输入的serial_name

    int detect_img_width = 100;
    int detect_img_height = 100;

    int px = 8;
    int py = 5;
    clock_t t_start = clock();
    clock_t t_now;                                   //记录时间
    float t_time_clock = 0;                         //代表已测试时间
    bool sample_flowing = FALSE;                    //点击开始测试到结束这段时间为Ture
    bool b_able_analyze = false;                     //样本测量结束后为ture，进入分析后变为false且其余时间为false
    const int thread_nums = 40;
    std::vector<bool> b_able_analyzes = std::vector<bool>(thread_nums,false);
    bool b_in_plot = false;
    float flow_pogress_1 = 0;                            //测试进度
    float analyze_progress_2 = 0;                        //分析进度

    //
    const char* status_items[] = { "0未开机","开机中","开机完成","正常待机","准备样品","吸样中","测试中" ,\
                                    "维护中","关机中","关机完成","已关机","进入休眠","休眠中","故障","处理故障","未知错误"};

    //enum flow_info {px, py, flow_v, flow_s};               //px试管位置,py试管位置,flow_v吸样体积,flow_s流速
    enum Info_Item { Item_tv, Item_s,Item_id, Item_px, Item_py, Item_tt, Item_tc, Item_ci, Item_cc,Item_status, Item_count };              //用于显示用的参数信息表
    std::vector<std::tuple<std::string, float, std::string> > info_v = {
    std::make_tuple(u8"总体积",0.0,"uL"),
    std::make_tuple(u8"流速",0.0,"uL/min"),
    std::make_tuple(u8"样本编号",0.0,""),
    std::make_tuple(u8"试管位置 x",0.0,""),
    std::make_tuple(u8"试管位置 y",0.0,""),
    std::make_tuple(u8"测试总时间",0.0,"s"),
    std::make_tuple(u8"已测试时间",0.0,"s"),
    std::make_tuple(u8"已采集图像",0.0,""),
    std::make_tuple(u8"已采集细胞",0.0,""),
    std::make_tuple(u8"状态参数",0.0,""),
    };
    

    // 进度条color
    ImVec4 orange_color = ImVec4(1.0, 0.43, 0.35, 1.0);                     //橙色，代表进行中
    ImVec4 green_color = ImVec4(0.6, 1.0, 0.6, 0.8);                        //绿色，代表完成
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);                //清空颜色，灰底
    ImVec4 progress_bar_color = orange_color;


    LaserControl* laser = new LaserControl(serial_name, laser_width, laser_frequency, laser_intensity, g_LogWindow);   //激光
    NativeVision* Vision = new NativeVision(g_LogWindow,thread_nums);                                                                      //相机

    GLuint tex_id_preview = 0;

    //连接相机并获取相机照片
    std::thread thread_get_img = std::thread(GetImage, Vision);
    thread_get_img.detach();

    //分析函数的多线程
  /*  std::vector< std::thread > thread_analyzes;
    for (int i = 0; i < 10; i++)
    {
        std::thread tempthread;
        thread_analyzes.push_back(tempthread);
    }*/

    while (!glfwWindowShouldClose(window)) 
    {

        //状态参数更新传递
        g_status = StatusParamUpdata(g_flow_p0,g_flow_p1,sample_flowing);    //状态
        if (g_status == 6 ) {                                       //计时并开始采集存储
            if (t_time_clock == 0) {
                t_start = clock();
                Sleep(1);               //暂停1ms，保证t_now和t_start不相等
            }
            t_now = clock();
            t_time_clock = (t_now-t_start) ;
            g_GetImgs = true;
            progress_bar_color = orange_color;
        }
        else {
            g_GetImgs = false;
        }
        flow_pogress_1 = t_time_clock/ save_time;

        if (flow_pogress_1 > 0.999)
        {
            flow_pogress_1 = 1;
            t_time_clock = save_time;
            progress_bar_color = green_color;
        }
        analyze_progress_2 = Vision->GetAnalyzeProgress();

        g_s = int(flow_pogress_1 * thread_nums);

        px = pos % 8 + 1;
        py = int(pos / 8) + 1;
        save_time = MIN(atoi(save_time_cstr) * 1000, volume_array[volume_current] / speed_array[speed_current_elem] * 60 * 1000); // 设定时间不得超过最大时间
        std::get<1>(info_v[Item_tv]) = volume_array[volume_current];
        std::get<1>(info_v[Item_s]) = speed_array[speed_current_elem];
        std::get<1>(info_v[Item_id]) = g_posID;
        std::get<1>(info_v[Item_py]) = py;
        std::get<1>(info_v[Item_px]) = px;
        std::get<1>(info_v[Item_tt]) = save_time / 1000;                          //ms与s转化
        std::get<1>(info_v[Item_tc]) = t_time_clock / CLOCKS_PER_SEC;
        //std::get<1>(info_v[Item_ci]) = Vision->GetTotalImageSize();
        std::get<1>(info_v[Item_cc]) = g_cell_total;
        std::get<1>(info_v[Item_status]) = g_status;



        glfwPollEvents();      //触发界面状态回调

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // operate window
        {
            ImGui::Begin("operate", NULL, window_flags);
            int current_region_width = ImGui::GetContentRegionAvail().x;
            
            // select pos,8*5布局排列
            ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5f, 0.5f));
            for (int r = 0; r < 5; r++) 
            {
                for (int c = 0; c < 8; c++) {
                    if (c > 0)
                        ImGui::SameLine();
                    ImGui::PushID(r * 8 + c);
                    std::string AtoE[5] = { "A","B","C","D","E" };
                    std::string select_text = AtoE[r] + std::to_string(c + 1);
                    if (ImGui::Selectable(select_text.c_str(), pos == (r * 8 + c), 0, ImVec2(50, 35))) {
                        pos = r * 8 + c;
                    }
                    ImGui::PopID();
                }
            }
            ImGui::PopStyleVar(1);

            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, progress_bar_color);
            ImGui::ProgressBar(flow_pogress_1, ImVec2(current_region_width * 1.0f, 20)); 
            ImGui::ProgressBar(analyze_progress_2, ImVec2(current_region_width * 1.0f, 20));
            ImGui::PopStyleColor(1);

            ImGui::Separator();
            //测试操作
            if (ImGui::Button(u8"进出仓", ImVec2(current_region_width * 0.5f, 40))) {
                g_flow->OpenCloseDoor();
                g_LogWindow->AddLog(u8"进出仓成功! \n");
            }
            ImGui::SameLine();
            if (ImGui::Button(u8"开始", ImVec2(current_region_width * 0.5f, 40)))
            {
                ClearPlot();
                Vision->Clear();
                flow_pogress_1 = 0;
                analyze_progress_2 = 0;
                t_time_clock = 0.0;
                g_flow->CollectStart(volume_array[volume_current], speed_array[speed_current_elem], 0, py, px);
                sample_flowing = TRUE;   //样本开始流动
                b_able_analyze = TRUE;  //可以分析，分析预准备
                g_s = 0;
                for (int i = 0; i < thread_nums; i++) {
                    b_able_analyzes[i] = true;
                }
                b_in_plot = true;

                g_LogWindow->AddLog(u8"开始测试! \n");
                
            }

            if (ImGui::IsItemHovered()) {
                char tip_buffer[100];
                sprintf(tip_buffer, "volume: %d \n speed : %d \n   pos   : %d %d \n", volume_array[volume_current], speed_array[speed_current_elem], py, px);
                ImGui::SetTooltip(tip_buffer);
            }

            if (ImGui::Button(u8"设置路径", ImVec2(current_region_width * 0.5f, 40)))
            {
                BROWSEINFO ofn;
                TCHAR szFile[MAX_PATH];
                ZeroMemory(&ofn, sizeof(BROWSEINFO));
                ofn.hwndOwner = NULL;
                ofn.pszDisplayName = szFile;
                ofn.lpszTitle = _T("选择存储文件夹:");
                ofn.ulFlags = BIF_DONTGOBELOWDOMAIN | BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
                LPITEMIDLIST idl = SHBrowseForFolder(&ofn);
                if (idl != NULL)
                {
                    SHGetPathFromIDList(idl, szFile);
                    g_file_path = unicode2string(szFile);
                }

            }


            ImGui::SameLine();
            if (ImGui::Button(u8"停止", ImVec2(current_region_width * 0.5f, 40)) ||(sample_flowing && t_time_clock > 0.999*save_time)) {
                sample_flowing = false;
                g_GetImgs = false;
                //g_progress = 0.0;
                g_flow->StopCollect();
                /*flow_pogress_1 = 1;
                t_time_clock = save_time;
                progress_bar_color = green_color;*/
            }

            //图像分析
            //if ((b_able_analyze && flow_pogress_1 >0.99))
            //{
            //    std::thread thread_analyze(&NativeVision::AnalyzeImages, Vision, "1", g_file_path, g_savecells, g_saveallimgs);
            //    thread_analyze.detach();
            //    b_able_analyze = false;              //保证只分析一次
            //}
            //std::vector< std::thread >thread_analyzes;
            /*if (g_s > 0) {
                for (int i = 0; i < thread_nums; i++) {

                    if (b_able_analyzes[i] && g_s == i+1)
                    {
                       std::thread th(&NativeVision::AnalyzeImages0_9, Vision, std::to_string(g_posID), g_file_path, i, g_savecells, g_saveallimgs);
                       th.detach();
                       b_able_analyzes[i] = false;
                    }
                }
            }*/
            if (g_s > 0) {
                for (int i = 0; i < thread_nums; i++) {

                    if (b_able_analyzes[i] && flow_pogress_1 == 1)
                    {
                        std::thread th(&NativeVision::AnalyzeImages0_9, Vision, std::to_string(g_posID), g_file_path, i, g_savecells, g_saveallimgs);
                        th.detach();
                        b_able_analyzes[i] = false;
                    }
                }
            }

           

            //分析完成后导入数据
            if (Vision->GetTotalImageSize() == 0 && analyze_progress_2 == 1.0 && g_cell_dia_array.size() == 0 && b_in_plot) {
                b_in_plot = false;
                // clear display tex id 
                for (int i = 0; i < g_detect_tex_ids.size(); i++) {
                    for (int j = 0; j < g_detect_tex_ids[0].size(); j++) {
                        glDeleteTextures(1, &(g_detect_tex_ids[i][j]));
                        g_detect_tex_ids[i][j] = 0;
                    }
                }

            //    //传入形态学参数到数组

                std::vector<CellInfo> total_cells = Vision->GetTotalCells();
                g_cell_total = total_cells.size();

                for (int i = 0; i < total_cells.size(); i++) {
                    g_cell_dia_array.push_back(total_cells[i].m_diameter);
                    g_cell_area_array.push_back(total_cells[i].m_area);
                    g_cell_peri_array.push_back(total_cells[i].m_perimeter);
                    g_cell_short_array.push_back(total_cells[i].m_shortaxis);
                    g_cell_long_array.push_back(total_cells[i].m_longaxis);
                    g_cell_vol_array.push_back(total_cells[i].m_vol);
                    g_cell_ell_array.push_back(total_cells[i].m_eccentricity);
                    g_cell_round_array.push_back(total_cells[i].m_roundness);
                    //if (i > 5000) break;

                    // display total cells image 
                    if (i < g_detect_class_num * g_detect_preview_num) {
                        int y = i % g_detect_preview_num;
                        int x = i / g_detect_preview_num;
                        g_show_cellinfos.push_back(total_cells[i]);
                        LoadTextureFromImage(GrayToRGB(g_show_cellinfos[i].m_image), &(g_detect_tex_ids.at(x).at(y)));
                    }
                }
            }

            if (ImGui::Button(u8"打开图片", ImVec2(current_region_width * 0.5f, 30))) {
                if (std::filesystem::exists(std::filesystem::path(g_file_path))) {
                    system(("start " + std::string(g_file_path)).c_str());
                }
            }
            ImGui::SameLine();
            if (ImGui::Button(u8"清空", ImVec2(current_region_width * 0.5f, 30))) {

                //清空数据
                ClearPlot();
                t_time_clock = 0;
                Vision->Clear();
                flow_pogress_1 = 0;
                //g_get = false;
                //Sleep(20);
                //Vision->StopCapture();
                //delete Vision;
                //Vision =  new NativeVision(g_LogWindow, thread_nums);
                //g_get = true;
                ////重新连接相机并获取相机照片
                //std::thread thread_get_img = std::thread(GetImage, Vision);
                //thread_get_img.detach();
                g_LogWindow->AddLog(u8"清空成功! \n");
            }

            ImGui::Checkbox(u8"保存图片", &g_savecells);
            //ImGui::SameLine();

            ImGui::InputInt(u8"样本编号", &g_posID);
            /*ImGui::SameLine();
            if (ImGui::Button(u8"新建样本", ImVec2(current_region_width * 0.25f, 25)))
            {
                g_posID++;
            }*/
            //ImGui::Checkbox(u8"保存所有图片", &g_saveallimgs);
            //ImGui::SameLine();
            if (std::filesystem::exists(g_file_path)) {
                ImGui::Text(g_file_path.c_str());
            }
            else {
                ImGui::Text(u8"无存储路径");

            }
            ImGui::End();
        }

        // preview window
        {
            ImGui::Begin("Preview", NULL, window_flags);            // 预览窗口
            // load preview image
            cv::Mat image_temp = Vision->OperateImageQueue(cv::Mat(), false);
            //resize(image_temp,image_temp,Size(320,240),0,0,INTER_LINEAR);
            if (!image_temp.empty()) {
                bool ret = LoadTextureFromImage(GrayToRGB(image_temp), &tex_id_preview);
            }
            image_temp.release();
            ImGui::Image((void*)(intptr_t)tex_id_preview, ImVec2(atoi(img_width_cstr), atoi(img_height_cstr)));
            ImGui::End();
        }

         //plot window

        {

            ImGui::Begin("Plot", NULL, window_flags);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)

            // plot data prepare


            //std::lock_guard<std::mutex> guard(g_img_plot_mutex);
            // begin plot
            static ImPlotSubplotFlags flags = ImPlotSubplotFlags_NoTitle;
            static float rratios[] = { 1,1,1 };
            static float cratios[] = { 1,1,1 };
            static int xybins[2] = { 90,30 };
            ImGui::SliderInt2("Bins", xybins, 1, 200);  //滑动条
            ImGui::SameLine();
            // Screen shot
            if (ImGui::Button(u8"截图保存", ImVec2(100, 30))) {

                ImVec2 StatisPos = ImGui::GetWindowPos();        //获取图像窗口位置及大小
                float Statis_width = ImGui::GetWindowWidth();
                float Statis_height = ImGui::GetWindowHeight();


                //cv::Mat img(nScreenHeight, nScreenWidth, CV_8UC3);
                cv::Mat img(Statis_height - 60, Statis_width, CV_8UC3);    //去掉窗口最上面一行约60


                glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
                //glPixelStorei(GL_PACK_ROW_LENGTH, img.step / img.elemSize()); // 这句不加好像也没问题？
                glReadPixels(StatisPos.x, nScreenHeight - StatisPos.y - Statis_height, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);

                cv::Mat flipped;
                cv::flip(img, flipped, 0);
                std::string desktop_path = g_file_path;
                struct tm time_info;
                time_t raw_time;
                time(&raw_time);
                localtime_s(&time_info, &raw_time);
                char time_info_buffer[100];
                strftime(time_info_buffer, 100, "%G_%m_%d_%H%M%S", &time_info);
                std::string res_path = (std::filesystem::path(desktop_path) / std::filesystem::path(std::string(time_info_buffer)+"_"+ std::to_string(g_posID)+".bmp")).string();
                g_LogWindow->AddLog(u8"保存： %s \n", res_path.c_str());
                cv::imwrite(res_path, flipped);

            }


            if (ImPlot::BeginSubplots("My Subplots", 3, 3, ImVec2(-1, -1), flags, rratios, cratios)) {

                ImPlot::PushColormap("Hot");
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"直径(um)", u8"圆度(um)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotHistogram2D("", g_cell_dia_array.data(), g_cell_round_array.data(), g_cell_dia_array.size(), xybins[0], xybins[1], false, ImPlotRect(0, 0, 0, 0));
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"直径(um)", u8"偏心率", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotHistogram2D("", g_cell_dia_array.data(), g_cell_ell_array.data(), g_cell_dia_array.size(), xybins[0], xybins[1], false, ImPlotRect(0, 0, 0, 0));
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"长(um)", u8"短(um2)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotHistogram2D("", g_cell_long_array.data(), g_cell_short_array.data(), g_cell_long_array.size(), xybins[0], xybins[1], false, ImPlotRect(0, 0, 0, 0));
                    ImPlot::EndPlot();
                }
                ImPlot::PopColormap();

                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"直径(um)", u8"数量", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_SouthEast, ImPlotLegendFlags_None);
                    ImPlot::PlotHistogram("", g_cell_dia_array.data(), g_cell_dia_array.size(), 100);
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"周长(um)", u8"数量", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_SouthEast, ImPlotLegendFlags_None);
                    ImPlot::PlotHistogram("", g_cell_peri_array.data(), g_cell_peri_array.size(), 100);
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"面积(um2)", u8"数量", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_SouthEast, ImPlotLegendFlags_None);
                    ImPlot::PlotHistogram("", g_cell_area_array.data(), g_cell_area_array.size(), 100);
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"体积(um3)", u8"数量", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_SouthEast, ImPlotLegendFlags_None);
                    ImPlot::PlotHistogram("", g_cell_vol_array.data(), g_cell_vol_array.size(), 100);
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"偏心率", u8"数量", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_SouthEast, ImPlotLegendFlags_None);
                    ImPlot::PlotHistogram("", g_cell_ell_array.data(), g_cell_ell_array.size(), 50);
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("")) {
                    ImPlot::SetupAxes(u8"圆度", u8"数量", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_SouthEast, ImPlotLegendFlags_None);
                    ImPlot::PlotHistogram("", g_cell_round_array.data(), g_cell_round_array.size(), 50);
                    ImPlot::EndPlot();
                }


                

                ImPlot::EndSubplots();
            }

            ImGui::End();
        }

        //filter window,用户自定义设定参数过滤
        {
        ImGui::Begin("filter", NULL, window_flags);
        ImGui::Text(u8"筛选自定义设定");
        ImGui::Separator();

        //IMGUI_DEMO_MARKER("Help");
        if (ImGui::CollapsingHeader(u8"条件添加"))
        {
            Condition m_condition;

            float c_min;
            float c_max;


            ImGui::Combo(u8"条件选择", &g_property_int, u8"直径\0面积\0周长\0体积\0偏心率\0圆度\0");
            m_condition.m_property = (Enum_property)g_property_int;

            static float set_min = 0.001f;
            static float set_max = 0.001f;

            ImGui::InputFloat(u8"最小", &set_min, 0.01f, 1.0f, "%.3f");
            ImGui::InputFloat(u8"最大", &set_max, 0.01f, 1.0f, "%.3f");

            ImGui::Combo(u8"条件", &g_logic, u8"与\0或\0");
            m_condition.m_min = set_min;
            m_condition.m_max = set_max;
            m_condition.m_b_and = g_logic;

            if (ImGui::Button(u8"确认添加", ImVec2(90, 25))) {
                g_conditions.push_back(m_condition);
                g_LogWindow->AddLog(u8"条件添加成功！\n");
            }
            ImGui::SameLine();
            if (ImGui::Button(u8"清空条件", ImVec2(90, 25))) {
                ClearVector(g_conditions);
                g_LogWindow->AddLog(u8"条件清空成功！\n");
            }

        }

        ImGui::Separator();
        ImGui::Combo(u8"是否保存", &g_set_filter_store, u8"不保存\0保存\0");
        if (g_set_filter_store == 1)
        {
            if (ImGui::Button(u8"选择路径", ImVec2(80, 40)))
            {
                BROWSEINFO ofn;
                TCHAR szFile[MAX_PATH];

                ZeroMemory(&ofn, sizeof(BROWSEINFO));
                ofn.hwndOwner = NULL;
                ofn.pszDisplayName = szFile;
                ofn.lpszTitle = _T("选择文件夹:");
                ofn.ulFlags = BIF_DONTGOBELOWDOMAIN | BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE; //包含新建文件夹功能
                LPITEMIDLIST idl = SHBrowseForFolder(&ofn);
                if (idl != NULL)
                {
                    SHGetPathFromIDList(idl, szFile);
                    /*CHAR szFile_char[MAX_PATH];
                    Tchar2Char(szFile,szFile_char);*/
                    g_img_filter_path = unicode2string(szFile);
                }

            }
            ImGui::Text(u8"保存路径:");
            ImGui::SameLine;
            ImGui::Text(g_img_filter_path.c_str());
        }

        ImGui::Separator();
        // 执行筛选函数
        if (ImGui::Button(u8"执行筛选", ImVec2(150, 40))) {
            ClearVector(g_filter_cellinfos);
            ClearVector(g_show_cellinfos);
            for (int i = 0; i < g_detect_tex_ids.size(); i++) {
                for (int j = 0; j < g_detect_tex_ids[0].size(); j++) {
                    glDeleteTextures(1, &(g_detect_tex_ids[i][j]));
                    g_detect_tex_ids[i][j] = 0;
                }
            }

            //传入数据
            std::vector<CellInfo> cell_infos = Vision->GetTotalCells();
            int filter_cell_num = 0;
            for (int i = 0; i < cell_infos.size(); i++) {
                bool b_cell_filter_total = false;
                for (int j = 0; j < g_conditions.size(); j++) {
                    bool b_cell_filter_single = true;
                    switch (g_conditions[j].m_property) {
                    case Enum_property::Enum_diameter:
                        b_cell_filter_single = (cell_infos[i].m_diameter >= g_conditions[j].m_min && cell_infos[i].m_diameter <= g_conditions[j].m_max);
                        break;
                    case Enum_property::Enum_area:
                        b_cell_filter_single = (cell_infos[i].m_area >= g_conditions[j].m_min && cell_infos[i].m_area <= g_conditions[j].m_max);
                        break;
                    case Enum_property::Enum_perimeter:
                        b_cell_filter_single = (cell_infos[i].m_perimeter >= g_conditions[j].m_min && cell_infos[i].m_perimeter <= g_conditions[j].m_max);
                        break;
                    case Enum_property::Enum_volume:
                        b_cell_filter_single = (cell_infos[i].m_vol >= g_conditions[j].m_min && cell_infos[i].m_vol <= g_conditions[j].m_max);
                        break;
                    case Enum_property::Enum_eccentricity:
                        b_cell_filter_single = (cell_infos[i].m_eccentricity >= g_conditions[j].m_min && cell_infos[i].m_eccentricity <= g_conditions[j].m_max);
                        break;
                    case Enum_property::Enum_roundness:
                        b_cell_filter_single = (cell_infos[i].m_roundness >= g_conditions[j].m_min && cell_infos[i].m_roundness <= g_conditions[j].m_max);
                        break;

                    }
                    if (j == 0) {
                        if (g_conditions[j].m_b_and == 0) {
                            b_cell_filter_total = true;
                        }
                        else if (g_conditions[j].m_b_and == 1) {
                            b_cell_filter_total = false;
                        }
                    }
                    if (g_conditions[j].m_b_and == 0) {
                        // and
                        b_cell_filter_total = b_cell_filter_total && b_cell_filter_single;
                    }
                    else if (g_conditions[j].m_b_and == 1) {
                        // or
                        b_cell_filter_total = b_cell_filter_total || b_cell_filter_single;
                    }
                }
                if (b_cell_filter_total) {
                    g_filter_cellinfos.push_back(cell_infos[i]);
                    if (filter_cell_num < g_detect_class_num * g_detect_preview_num) {
                        int y = filter_cell_num % g_detect_preview_num;
                        int x = filter_cell_num / g_detect_preview_num;
                        LoadTextureFromImage(GrayToRGB(cell_infos[i].m_image), &(g_detect_tex_ids.at(x).at(y)));
                    }
                    filter_cell_num++;
                    //保存筛选图片
                    if (g_set_filter_store) {
                        std::string save_filter_cell_path = (std::filesystem::path(g_img_filter_path) / std::filesystem::path(cell_infos[i].m_name)).string();
                        cv::imwrite(save_filter_cell_path.c_str(), cell_infos[i].m_image);
                    }
                }
            }
        }

        ImGui::Separator();
        ImGui::End();
        }
        // detect window
        {
            ImGui::Begin("detect", NULL, window_flags);
            ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (int i = 0; i < g_conditions.size(); i++)
            {
                std::string bottonname = std::to_string(g_conditions[i].m_min) + "<" + g_condition_names.at(g_conditions[i].m_property) + "<" + std::to_string(g_conditions[i].m_max);
                if (ImGui::Button(bottonname.c_str()))
                {
                    g_conditions.erase(g_conditions.begin() + i);
                    g_LogWindow->AddLog(u8"删除条件：%s\n", bottonname.c_str());
                }
                if (i < g_conditions.size() - 1) {
                    ImGui::SameLine();
                }
                else {
                    ImGui::Separator();
                }
            }
            //添加属性值
            for (int i = 0; i < g_detect_class_num; i++) {
                for (int j = 0; j < g_detect_preview_num; j++) {
                    ImGui::Image((void*)(intptr_t)g_detect_tex_ids[i][j], ImVec2(detect_img_width, detect_img_height));

                    if (g_show_cellinfos.size() != 0 && g_filter_cellinfos.size() == 0) {
                        if ((i * g_detect_preview_num + j) < g_show_cellinfos.size()) {
                            if (ImGui::IsItemHovered()) {
                                char tip_buffer[100];
                                sprintf(tip_buffer, u8"直径: %.3f \n面积: %.3f \n周长: %.3f\n体积：%.3f\n偏心率： %.3f \n圆度：%.3f \n", \
                                g_show_cellinfos[(i * g_detect_preview_num + j)].m_diameter, g_show_cellinfos[(i * g_detect_preview_num + j)].m_perimeter, g_show_cellinfos[(i * g_detect_preview_num + j)].m_area, g_show_cellinfos[(i * g_detect_preview_num + j)].m_vol, g_show_cellinfos[(i * g_detect_preview_num + j)].m_eccentricity, g_show_cellinfos[(i * g_detect_preview_num + j)].m_roundness);
                                ImGui::SetTooltip(tip_buffer);
                            }
                        }
                    }


                    else if ((i * g_detect_preview_num + j) < g_filter_cellinfos.size()) {
                        if (ImGui::IsItemHovered()) {
                            char tip_buffer[100];
                            sprintf(tip_buffer, u8"直径: %.3f \n面积: %.3f \n周长: %.3f\n体积：%.3f\n偏心率： %.3f \n圆度：%.3f \n", \
                            g_filter_cellinfos[(i * g_detect_preview_num + j)].m_diameter, g_filter_cellinfos[(i * g_detect_preview_num + j)].m_perimeter, g_filter_cellinfos[(i * g_detect_preview_num + j)].m_area, g_filter_cellinfos[(i * g_detect_preview_num + j)].m_vol, g_filter_cellinfos[(i * g_detect_preview_num + j)].m_eccentricity, g_filter_cellinfos[(i * g_detect_preview_num + j)].m_roundness);
                            ImGui::SetTooltip(tip_buffer);
                        }
                    }

                    ImGui::SameLine();
                }
                ImGui::NewLine();
            }

            ImGui::Separator();
            std::string btn_text = u8"打开筛选细胞图，一共" + std::to_string(g_filter_cellinfos.size()) + u8"个";
            if (ImGui::Button(btn_text.c_str())) {
                if (std::filesystem::exists(std::filesystem::path(g_img_filter_path))) {
                    system(("start " + std::string(g_img_filter_path)).c_str());
                }
                else {
                    g_LogWindow->AddLog("file : %s not exists !! \n", g_img_filter_path.c_str());
                }
            }
            ImGui::EndChild();
            ImGui::End();
        }

        // info window
        {
            ImGui::Begin("info", NULL, window_flags);
            ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
            if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {
                if (ImGui::BeginTabItem(u8"信息"))
                {
                    static ImGuiTableFlags flags = ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV;
                    if (ImGui::BeginTable(u8"信息", 2, flags))
                    {
                        ImGui::TableSetupColumn(u8"属性");
                        ImGui::TableSetupColumn(u8"值");
                        ImGui::TableHeadersRow();
                        for (int row = 0; row < Item_count; row++)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::Text(std::get<0>(info_v[row]).c_str(), 0, row);
                            ImGui::TableSetColumnIndex(1);
                            ImGui::Text((std::to_string(static_cast<int>(std::get<1>(info_v[row]))) + " " + std::get<2>(info_v[row])).c_str(), 1, row);
                        }
                        ImGui::EndTable();
                    }
                    //ImGui::Text(u8"当前状态：%s",status_items[g_status]);
                    //ImGui::Text(u8"当前进度：%d", g_s);
                   
                    ImGui::Text(" %.1f ms/frame (%.0f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                    ImGui::EndTabItem();
                }

                // properties tab
                if (ImGui::BeginTabItem(u8"属性"))
                {
                    //flow setting
                    ImGui::Text(u8"液流");
                    ImGui::Combo(u8"选择体积", &volume_current, volume_items, IM_ARRAYSIZE(volume_items));
                    // speed slider
                    const char* elems_names[Element_COUNT] = { "10 uL/min","20 uL/min","30 uL/min" };
                    const char* elem_name = (speed_current_elem >= 0 && speed_current_elem < Element_COUNT) ? elems_names[speed_current_elem] : "Unknown";
                    ImGui::SliderInt(u8"选择流速", &speed_current_elem, 0, Element_COUNT - 1, elem_name);
                    ImGui::InputText(u8"测试时长(s)", save_time_cstr, IM_ARRAYSIZE(save_time_cstr));

                    // image setting
                    ImGui::Separator();
                    ImGui::Text(u8"相机");
                    ImGui::InputText(u8"宽度", img_width_cstr, IM_ARRAYSIZE(img_width_cstr));
                    ImGui::InputText(u8"高度", img_height_cstr, IM_ARRAYSIZE(img_height_cstr));
                    ImGui::InputText(u8"偏移", img_offset_x_cstr, IM_ARRAYSIZE(img_offset_x_cstr));
                    ImGui::InputText(u8"同步", img_exposure_cstr, IM_ARRAYSIZE(img_exposure_cstr));
                    ImGui::InputText(u8"FPS", img_acquisition_frame_rate_cstr, IM_ARRAYSIZE(img_acquisition_frame_rate_cstr));
                    if (ImGui::Button(u8"更新相机设置")) 
                    {
                        if (Vision->SetCamParas(atoi(img_width_cstr), atoi(img_height_cstr), atoi(img_offset_x_cstr), stod(img_exposure_cstr), stod(img_acquisition_frame_rate_cstr))) {
                            g_LogWindow->AddLog(u8"更新相机参数成功! \n");
                        }
                        else {
                            g_LogWindow->AddLog(u8"更新相机参数失败! \n");
                        }
                    }

                    // laser setting
                    ImGui::Separator();
                    ImGui::Text(u8"激光");
                    if (ImGui::Button(u8"重连")) {
                        delete laser;
                        laser = new LaserControl(serial_name, laser_width, laser_frequency, laser_intensity, g_LogWindow);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(u8"打开")) {
                        laser->OpenLaser();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(u8"关闭")) {
                        laser->CloseLaser();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(u8"更新")) {
                        laser_width = atoi(laser_width_cstr);
                        laser_frequency = atoi(laser_frequency_cstr);
                        laser_intensity = atoi(laser_intensity_cstr);
                        serial_name = serial_name_cstr;
                        laser->SetParas(laser_width, laser_frequency, laser_intensity, serial_name);
                        delete laser;
                        laser = new LaserControl(serial_name, laser_width, laser_frequency, laser_intensity, g_LogWindow);
                    }
                    ImGui::InputText(u8"脉冲", laser_frequency_cstr, IM_ARRAYSIZE(laser_frequency_cstr));
                    ImGui::InputText(u8"脉宽(ns)", laser_width_cstr, IM_ARRAYSIZE(laser_width_cstr));
                    ImGui::InputText(u8"强度(0-200）", laser_intensity_cstr, IM_ARRAYSIZE(laser_intensity_cstr));
                    ImGui::InputText(u8"端口(COM) ", serial_name_cstr, IM_ARRAYSIZE(serial_name_cstr));

                    ImGui::EndTabItem();
                }

                //maintain
                if (ImGui::BeginTabItem(u8"操作")) {
                    ImGui::Text(u8"维护");
                    if (ImGui::Button(u8"过滤器充灌", ImVec2(200, 25))) {
                        g_flow->Maintain((int)3);
                    }
                    if (ImGui::Button(u8"柱塞器充灌", ImVec2(200, 25))) {
                        g_flow->Maintain((int)4);
                    }
                    if (ImGui::Button(u8"整流腔充灌", ImVec2(200, 25))) {
                        g_flow->Maintain((int)5);
                    }
                    if (ImGui::Button(u8"样本通道清洗", ImVec2(200, 25))) {
                        g_flow->Maintain((int)6);
                    }
                    if (ImGui::Button(u8"采用通道排堵", ImVec2(200, 25))) {
                        g_flow->Maintain((int)7);
                    }
                    if (ImGui::Button(u8"液路初始化", ImVec2(200, 25))) {
                        g_flow->Maintain((int)8);
                    }
                    if (ImGui::Button(u8"整机排空", ImVec2(200, 25))) {
                        g_flow->Maintain((int)11);
                    }
                    if (ImGui::Button(u8"整机充灌", ImVec2(200, 25))) {
                        g_flow->Maintain((int)12);
                    }
                    if (ImGui::Button(u8"更换保护液", ImVec2(200, 25))) {
                        g_flow->Maintain((int)13);
                    }
                    if (ImGui::Button(u8"更换清洗液", ImVec2(200, 25))) {
                        g_flow->Maintain((int)14);
                    }
                    ImGui::Separator();
                    ImGui::Text(u8"电源");
                    if (ImGui::Button(u8"关机", ImVec2(100, 25))) {
                        g_flow->ShutDown();
                    }

                    ImGui::EndTabItem();
                }
            ImGui::EndTabBar();
            }

            ImGui::End();
        }

        // Log window
        {
            g_LogWindow->Draw("Log", NULL, window_flags);
        }





    //渲染显示
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);              //设置清理的颜色，即背景颜色
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
    }
}




