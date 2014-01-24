/*
 * hsvfilter.cpp
 *
 *  Created on: Dec 16, 2013
 *      Author: Mafioso
 */

#include "hsvfilter.hpp"

HsvFilter::HsvFilter() :
    min(0,0,0), max(255, 255, 255)
{

}

HsvFilter::~HsvFilter()
{

}

void HsvFilter::SetLimits(const cv::Scalar& _min, const cv::Scalar& _max)
{
    min = _min;
    max = _max;
}

void HsvFilter::Evaluate(const cv::Mat& imgIn, cv::Mat& imgOut) const
{
    cv::Mat hsv;
    cv::cvtColor(imgIn, hsv, CV_RGB2HSV);
    //GaussianBlur(hsv, hsv, Size(9, 9), 2, 2);
    cv::inRange(hsv, min, max, imgOut);
    cv::erode(imgOut, imgOut, cv::Mat());
    cv::dilate(imgOut, imgOut,  cv::Mat());
}

bool HsvFilter::IsInRage(uint8_t* pixel) const
{
    float R = pixel[0] / 255.f;
    float G = pixel[1] / 255.f;
    float B = pixel[2] / 255.f;

    float Cmin = fminf(fminf(R, G), B);
    float Cmax = fmaxf(fmaxf(R, G), B);

    float delta = Cmax-Cmin;

    int H;
    int S = 255 * delta / Cmax;
    int V = Cmax * 255;

    if (Cmax == R) {
        H = 30.f * (G-B)/delta;
    } else if (Cmax == G) {
        H = 60.f + 30.f * (B-R)/delta;
    } else {
        H = 120 + 30.f * (R-G)/delta;
    }

    bool inRange = true;

    if (H < min.val[0]) {
        inRange = false;
    } else if (H > max.val[0]) {
        inRange = false;
    } else if (S < min.val[1]) {
        inRange = false;
    } else if (S > max.val[1]) {
        inRange = false;
    } else if (V < min.val[2]) {
        inRange = false;
    } else if (V > max.val[2]) {
        inRange = false;
    }

    return inRange;
}

