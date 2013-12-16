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
    //cv::inRange(hsv, cv::Scalar(109,167,46), cv::Scalar(132, 255, 158), red);
    cv::inRange(hsv, min, max, imgOut);
    cv::erode(imgOut, imgOut, cv::Mat());
    cv::dilate(imgOut, imgOut,  cv::Mat());
}
