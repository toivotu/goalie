/*
 * balldetector.cpp
 *
 *  Created on: Dec 3, 2013
 *      Author: Mafioso
 */

#ifndef BALLDETECTOR_CPP_
#define BALLDETECTOR_CPP_

#include "balldetector.hpp"

namespace {

typedef std::pair<cv::Point, cv::Point> MyLine;

class Line {
public:
    Line(int point1, int point2): _p1(point1), _p2(point2)
    {

    }

    int Center()
    {
        return (_p1 + _p2) / 2;
    }

    int Length()
    {
        return _p2 - _p1;
    }

private:
    int _p1;
    int _p2;

};

Line HLine(cv::Mat& image, cv::Mat& orig, int x, int y)
{
    int xMin = x;
    int xMax = x;

    while ((xMin > 1) && image.data[xMin - 1 + y * image.cols] == 255u) {
        orig.data[(xMin + orig.cols * y) * 3] = 255;
        xMin--;
    }

    while ((xMax < image.cols - 1) && image.data[xMax - 1 + y * image.cols] == 255u) {
        orig.data[(xMax + orig.cols * y) * 3] = 255;
        xMax++;
    }

    return Line(xMin, xMax);
}

Line VLine(cv::Mat& image, cv::Mat& orig, int x, int y)
{
    int yMin = y;
    int yMax = y;

    while ((yMin > 1) && image.data[x + yMin * image.cols] == 255u) {
        orig.data[(x + orig.cols * yMin) * 3] = 255;
        yMin--;
    }

    while ((yMax < image.rows - 1) && image.data[x - 1 + yMax * image.cols] == 255u) {
        orig.data[(x + orig.cols * yMax) * 3] = 255;
        yMax++;
    }

    return Line(yMin, yMax);
}

static MyLine Diagonal(cv::Mat& image, cv::Mat& orig, int x, int y)
{
    int xMin = x;
    int xMax = x;
    int yMin = y;
    int yMax = y;

    while ((xMin > 1) && (yMin > 1) && image.data[xMin + yMin * image.cols] == 255u) {
        orig.data[(xMin + orig.cols * yMin) * 3] = 255;
        yMin--;
        yMin--;
    }

    while ((xMax < image.cols - 1) && (yMax < image.rows - 1) && image.data[xMax + yMax * image.cols] == 255u) {
        orig.data[(xMax + orig.cols * yMax) * 3] = 255;
        xMax++;
        yMax++;
    }

    return std::pair<cv::Point, cv::Point>(cv::Point(xMin,yMin),cv::Point(xMax, yMax));
}
}

void Balldetector::Recurse(Circle& circle, cv::Mat& image, cv::Mat& orig, Balldetector::Axis axis, int x, int y, int xMax, int yMax)
{
    if (axis == AXIS_X) {

        Line xLine = HLine(image, orig, x, y);

        circle.x = x;
        circle.y = y;
        if (xLine.Length() > xMax) {

            circle.xWidth = xLine.Length();

            Recurse(circle, image, orig, AXIS_Y, xLine.Center(), y, xLine.Length(), yMax);
        }

    } else {

        Line yLine = VLine(image, orig, x, y);

        circle.x = x;
        circle.y = y;

        if (yLine.Length() > yMax) {

            circle.yWidth = yLine.Length();

            Recurse(circle, image, orig, AXIS_X, x, yLine.Center(), xMax, yLine.Length());
        }
    }
}



Balldetector::Ball Balldetector::Detect(cv::Mat& image)
{
    Circle ball = {0,0,0};

    cv::Mat hsv;
    cv::Mat red;

    cv::cvtColor(image, hsv, CV_RGB2HSV);
    //GaussianBlur(hsv, hsv, Size(9, 9), 2, 2);
    cv::inRange(hsv, cv::Scalar(109,167,46), cv::Scalar(132, 255, 158), red);
    //imshow("blur", red);
    imshow("bw", red);

   // std::cout << image.rows << " " <<image.cols << std::endl;

    for (int i = 0; i < image.cols; i += 5) {
        for (int j = 0; j < image.rows; j +=5) {
             if (red.data[i + image.cols * j] == 255u) {
                image.data[(i + image.cols * j) * 3] = 255;

                Line line = HLine(red, image, i, j);
                Circle detected = {0,0,0,0,0};

                detected.radius = 0;
                Recurse(detected, red, image, AXIS_X, i, j, 0, 0);
                MyLine diag = Diagonal(red, image, detected.x, detected.y);

                if (abs(1.f - (double)detected.xWidth / (double)detected.yWidth) < 0.3) {
                    detected.radius = (detected.xWidth + detected.yWidth) / 2;
                }

                if (detected.radius > 0  && abs(1.f - (double)detected.radius / (double)(diag.second.x - diag.first.x) * 0.707f) > 0.3) {
                    detected.radius = 0;
                }

                if (detected.radius > 20) {

                    circle( image, cv::Point(detected.x, detected.y), detected.radius / 2, cv::Scalar(0,255,0), 3, 8, 0 );
                    ball = detected;
                    j += ball.radius;
                    i += ball.radius;
                    return ball;
                }
            }
        }
    }

    //std::cout << (int)red.at<unsigned char>(1, 1) << std::endl;
    imshow("bw", red);

    return ball;
}



#endif /* BALLDETECTOR_CPP_ */
