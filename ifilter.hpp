/*
 * filter.hpp
 *
 *  Created on: Dec 16, 2013
 *      Author: Mafioso
 */

#ifndef FILTER_HPP_
#define FILTER_HPP_

#include "opencv2/opencv.hpp"

class IFilter
{
public:
    virtual void Evaluate(const cv::Mat& ImgIn, cv::Mat& imgOut) const = 0;
};



#endif /* FILTER_HPP_ */
