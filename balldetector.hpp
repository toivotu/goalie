/*
 * balldetector.h
 *
 *  Created on: Dec 3, 2013
 *      Author: Mafioso
 */

#ifndef BALLDETECTOR_H_
#define BALLDETECTOR_H_

#include "opencv2/opencv.hpp"

#include "ifilter.hpp"

class Balldetector {
public:
    typedef struct {
        int x;
        int y;
        int radius;
    } Ball;


    Balldetector(const IFilter* filter, int minDiameter, int maxDiameter, double tolerance, int step = 5);
    Ball Detect(cv::Mat& image);

    void SetTolerance(double tolerance);

private:
    const IFilter* filter;
    double tolerance;
    int step;

    typedef enum {
        AXIS_X,
        AXIS_Y
    } Axis;

    typedef struct {
        int x;
        int y;
        int xWidth;
        int yWidth;
        int radius;
    } Circle;

    void Recurse(Circle& circle, cv::Mat& image, cv::Mat& orig, Axis axis, int x, int y, int xMax, int yMax);
};


#endif /* BALLDETECTOR_H_ */
