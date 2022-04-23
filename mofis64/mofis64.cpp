// mofis64.cpp : 定义应用程序的入口点。
//

#include "framework.h"
#include "mofis64.h"
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

//gui工具
#include <glad/glad.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include "stbimage/stb_image.h"
#include "stbimage/stb_image_resize.h"

//sdk外部调用
#include "FlowCam\IInterface.h"




//全局变量参数
IInterface* g_flow = NULL;            //液路控制
int g_flow_p0;                        //液路的function type
int g_flow_p1;                        //液路的para1
int g_flow_p2;                        //液路的para2
int g_flow_p3;                        //液路的para3
int g_pos;                            //测量试管位置,1维数据


//液路控制回调
void UpdateInstateCallback(int in_function_type, int in_para1, int in_para2, int in_para3)
{
    g_flow_p0 = in_function_type;
    g_flow_p1 = in_para1;
    g_flow_p2 = in_para2;
    g_flow_p3 = in_para3;
}

void FlowInit()
{
    ::OutputDebugStringA("#22 FlowInit.\n");

    IInterface* pShowUI = IInterface::CreateInstance();
    g_flow = pShowUI;

    pShowUI->SetInitStateCallback(&UpdateInstateCallback);   //设置事件响应回调
    pShowUI->Init();   //初始化
}


static void GlfwErrorCallback(int in_error, const char* in_description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", in_error, in_description);
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    // open console
    //AllocConsole();
    //HANDLE  g_hOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    //freopen("CONIN$", "r", stdin);
    //freopen("CONOUT$", "w", stdout);
    //freopen("CONOUT$", "w", stderr);

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
    GLFWwindow* window = glfwCreateWindow(nScreenWidth, nScreenHeight, "LSKJ-VS1000", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // glad bind to window
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
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


    FlowInit();

    //参数


    while (!glfwWindowShouldClose(window)) 
    {
        glfwPollEvents();      //触发状态回调

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
                    if (ImGui::Selectable(select_text.c_str(), g_pos == (r * 8 + c), 0, ImVec2(35, 35))) {
                        g_pos = r * 8 + c;
                    }
                    ImGui::PopID();
                }
            }
            ImGui::PopStyleVar(1);

            ImGui::Separator();
            //测试操作
            if (ImGui::Button(u8"进出仓", ImVec2(current_region_width * 0.5f, 40))) {
                g_flow->OpenCloseDoor();
                //g_LogWindow->AddLog(u8"进出仓成功! \n");
            }
            if (ImGui::Button(u8"开始", ImVec2(current_region_width * 0.5f, 40))) 
            {
                
            }
            ImGui::End();
        }


        // debug window
        {
            ImGui::Begin("Debug", NULL, window_flags);
            ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
            if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {
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
                    if (ImGui::Button(u8"更换PBS", ImVec2(200, 25))) {
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




