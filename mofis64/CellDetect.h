#pragma once

#include<cmath>
#include<direct.h> 
#include<io.h> 
#include<math.h>

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
//#include<opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
//#include "opencv2/ml.hpp"
//#include "opencv2/objdetect.hpp"
//#include <opencv2/core/utils/logger.hpp>
#include<iostream>
#include<algorithm>
#include <stdio.h>
#include <ctime>
#include <vector>


#define UM_PER_PIXEL 0.13824
#define UM_2_PER_PIXEL 0.01911
#define pi 3.1415926

using namespace std;
using namespace cv;

struct CellInfo {
    cv::Mat m_image = cv::Mat::zeros(1,1,0);
    double m_area;
    double m_perimeter;
    double m_diameter;
    double m_shortaxis;
    double m_longaxis;
    double m_vol;
    double m_eccentricity;
    double m_roundness;
    int m_id;
    int m_second;
    std::string m_name = "";
    std::string m_path = "";
    int m_class;
};

struct ImageInfo {
    cv::Mat m_image = cv::Mat::zeros(1, 1, 0);;
    int m_id;
    int m_second;
    std::string m_name;
};





