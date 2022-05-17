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
    cv::Mat m_image;
    int m_id;
    int m_second;
    std::string m_name;
};

class CellDetect
{
private:
    int width = 200, height = 200;
    int img_w = 320, img_h = 480;
    float beta = 0.5;
public:
    std::vector<CellInfo> GetResult(cv::Mat src) {
        std::vector<CellInfo> res;
        CellInfo cellinfo;
        if (!src.data)
            std::cerr << "Problem loading image!!!" << std::endl;
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
            float cell_peri = arcLength(contours_big_img[i], true) * UM_PER_PIXEL;     //求细胞周长

            if (cell_peri > 50.0 || cell_peri < 6)
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
            //ell = fitEllipse(contours_big_img[i]);  //椭圆拟合
            cell_short_axis = MIN(ell.size.height, ell.size.width);
            cell_long_axis = MAX(ell.size.height, ell.size.width);
            cell_dia = 2 * sqrt(cell_area / pi);                //等效直径
            //cell_dia = UM_PER_PIXEL*(cell_short_axis+ cell_long_axis)/2;               
            cell_vol = pow(cell_dia, 3) * pi / 6;    //等效体积
            cell_eccentricity = (cell_long_axis - cell_short_axis) / cell_long_axis;   //根据长短轴求偏心率

            cell_roundness = cell_area / pow(cell_peri, 2) * 4 * pi;

            cellinfo = { roiImg, cell_area, cell_peri, cell_dia ,cell_short_axis * UM_PER_PIXEL,cell_long_axis * UM_PER_PIXEL,cell_vol,cell_eccentricity, cell_roundness };
            res.push_back(cellinfo);
        }
        return res;


    }




};





