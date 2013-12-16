/*
 * RgbFilter.cpp
 *
 *  Created on: Dec 16, 2013
 *      Author: Mafioso
 */

#include "rgbfilter.hpp"

RgbFilter::RgbFilter() :
    min(0,0,0), max(255, 255, 255)
{

}

RgbFilter::~RgbFilter()
{

}

void RgbFilter::SetLimits(const cv::Scalar& _min, const cv::Scalar& _max)
{
    min = _min;
    max = _max;
}

void RgbFilter::Evaluate(const cv::Mat& imgIn, cv::Mat& imgOut) const
{
    //GaussianBlur(hsv, hsv, Size(9, 9), 2, 2);
    cv::inRange(imgIn, min, max, imgOut);
    //cv::erode(imgOut, imgOut, cv::Mat());
    //cv::dilate(imgOut, imgOut,  cv::Mat());
}
