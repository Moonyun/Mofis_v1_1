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


//全局变量参数

LogWindows* g_LogWindow = new LogWindows();       // global logger

IInterface* g_flow ;            //液路控制
int g_flow_p0;                        //液路的function type
int g_flow_p1;                        //液路的para1
int g_flow_p2;                        //液路的para2
int g_flow_p3;                        //液路的para3

int g_status;                        //全局状态表示
bool g_savecells = FALSE;

std::vector<float> g_cell_total_array;     //细胞形态学参数
std::vector<float> g_cell_area_array;
std::vector<float> g_cell_peri_array;
std::vector<float> g_cell_dia_array;
std::vector<float> g_cell_short_array;
std::vector<float> g_cell_long_array;
std::vector<float> g_cell_vol_array;
std::vector<float> g_cell_ell_array;
std::vector<float> g_cell_round_array;




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

//状态更新
int StatusParamUpdata()
{
    int status = 0;
    return status;
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
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF("Microsoft-YaHei-Regular.ttc", 20, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
    ImGui::StyleColorsLight();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    //windows setup窗口属性参数
    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoScrollbar;

    //液路初始化
    FlowInit();

    //参数
    int pos = 39;                                                                    //默认采样试管位置
    const char* volume_items[] = { "10  uL","20  uL","30  uL","50  uL","100 uL" };   //吸样体积选取
    int volume_array[5] = { 10,20,30,50,100 };                                       //吸样具体数值
    int volume_current = 1;                                                          //默认选取吸样位置
    enum Speed_Element { Element_1, Element_2, Element_3, Element_COUNT };           //流速选择
    int speed_array[3] = { 10,20,30 };
    int speed_current_elem = Element_1;

    char img_width_cstr[128] = "320";                                              //相机控制的参数，长宽像素，曝光，设置帧率
    char img_offset_x_cstr[128] = "160";
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

    int px = 8;
    int py = 5;

    int save_time = 100000;
    //enum flow_info {px, py, flow_v, flow_s};               //px试管位置,py试管位置,flow_v吸样体积,flow_s流速
    

    // 进度条color
    ImVec4 orange_color = ImVec4(1.0, 0.43, 0.35, 1.0);                     //橙色，代表进行中
    ImVec4 green_color = ImVec4(0.6, 1.0, 0.6, 0.8);                        //绿色，代表完成
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);                //清空颜色，灰底
    ImVec4 progress_bar_color = orange_color;


    LaserControl* laser = new LaserControl(serial_name.c_str(), laser_width, laser_frequency, laser_intensity, g_LogWindow);

    while (!glfwWindowShouldClose(window)) 
    {
        glfwPollEvents();      //触发界面状态回调

        //状态参数传递
        px = pos % 8 + 1;
        py = int(pos / 8) + 1;

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
                    if (ImGui::Selectable(select_text.c_str(), pos == (r * 8 + c), 0, ImVec2(35, 35))) {
                        pos = r * 8 + c;
                    }
                    ImGui::PopID();
                }
            }
            ImGui::PopStyleVar(1);

            ImGui::Separator();
            //测试操作
            if (ImGui::Button(u8"进出仓", ImVec2(current_region_width * 0.5f, 40))) {
                g_flow->OpenCloseDoor();
                g_LogWindow->AddLog(u8"进出仓成功! \n");
            }

            if (ImGui::Button(u8"开始", ImVec2(current_region_width * 0.5f, 40))) 
            {
                g_flow->CollectStart(volume_array[volume_current],speed_array[speed_current_elem], 0, py, px);

                g_LogWindow->AddLog(u8"开始测试! \n");
            }

            ImGui::End();
        }


        // info window
        {
            ImGui::Begin("info", NULL, window_flags);
            ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
            if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {

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
                    // image setting
                    ImGui::Separator();

                    ImGui::Text(u8"相机");
                    //if (ImGui::Button(u8"更新相机设置")) {
                    //    if (camera->SetParas(atoi(img_width_cstr), atoi(img_height_cstr), atoi(img_offset_x_cstr), atoi(img_exposure_cstr), atoi(img_acquisition_frame_rate_cstr))) {
                    //        std::cout << "set paras success " << std::endl;
                    //    }
                    //    else {
                    //        std::cout << "set paras failed " << std::endl;
                    //    }
                    //}
                    ImGui::InputText(u8"宽度", img_width_cstr, IM_ARRAYSIZE(img_width_cstr));
                    ImGui::InputText(u8"高度", img_height_cstr, IM_ARRAYSIZE(img_height_cstr));
                    ImGui::InputText(u8"偏移", img_offset_x_cstr, IM_ARRAYSIZE(img_offset_x_cstr));
                    ImGui::InputText(u8"同步", img_exposure_cstr, IM_ARRAYSIZE(img_exposure_cstr));
                    ImGui::InputText(u8"FPS", img_acquisition_frame_rate_cstr, IM_ARRAYSIZE(img_acquisition_frame_rate_cstr));
                    // laser setting
                    ImGui::Separator();
                    ImGui::Text(u8"激光");
                    if (ImGui::Button(u8"重连")) {
                        delete laser;
                        laser = new LaserControl(serial_name.c_str(), laser_width, laser_frequency, laser_intensity, g_LogWindow);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(u8"打开")) {
                        laser->OpenLaser();
                        Sleep(10);
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
                        laser->SetParas(laser_width, laser_frequency, laser_intensity, serial_name.c_str());
                    }
                    ImGui::InputText(u8"脉冲", laser_frequency_cstr, IM_ARRAYSIZE(laser_frequency_cstr));
                    ImGui::InputText(u8"脉宽(ns)", laser_width_cstr, IM_ARRAYSIZE(laser_width_cstr));
                    ImGui::InputText(u8"强度(0-200）", laser_intensity_cstr, IM_ARRAYSIZE(laser_intensity_cstr));
                    ImGui::InputText(u8"端口(COM) ", serial_name.data(), IM_ARRAYSIZE(serial_name.c_str()));

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




