/*
 * hsvfilter.h
 *
 *  Created on: Dec 16, 2013
 *      Author: Mafioso
 */

#ifndef HSVFILTER_H_
#define HSVFILTER_H_

#include "opencv2/opencv.hpp"

#include "ifilter.hpp"

class HsvFilter : public IFilter {
public:
    HsvFilter();
    virtual ~HsvFilter();

    virtual void Evaluate(const cv::Mat& imgIn, cv::Mat& imgOut) const;
    virtual bool IsInRage(uint8_t* pixel) const;

    void SetLimits(const cv::Scalar& min, const cv::Scalar& max);

private:
    cv::Scalar min;
    cv::Scalar max;
};


#endif /* HSVFILTER_H_ */
