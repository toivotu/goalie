/*
 * RgbFilter.h
 *
 *  Created on: Dec 16, 2013
 *      Author: Mafioso
 */

#ifndef RGBFILTER_H_
#define RGBFILTER_H_

#include "opencv2/opencv.hpp"

#include "ifilter.hpp"

class RgbFilter : public IFilter {
public:
    RgbFilter();
    virtual ~RgbFilter();

    virtual void Evaluate(const cv::Mat& imgIn, cv::Mat& imgOut) const;

    void SetLimits(const cv::Scalar& min, const cv::Scalar& max);

private:
    cv::Scalar min;
    cv::Scalar max;
};


#endif /* RGBFILTER_H_ */
