/*
 * balldetector.cpp
 *
 *  Created on: Dec 3, 2013
 *      Author: Mafioso
 */

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

const int dashTolerance = 5;

bool LineValid(int& count, bool pixelValid)
{
    if (pixelValid) {
        count = dashTolerance;
    } else {
        count--;
    }

    return count > 0;
}

Line HLine(const IFilter* filter, cv::Mat& image, int x, int y)
{
    int xMin = x;
    int xMax = x;

    int dashCount = dashTolerance;

    while ((xMin > 1) && LineValid(dashCount, filter->IsInRage(&image.data[(xMin - 1 + y * image.cols) * 3]))) {
        xMin--;
    }

    dashCount = dashTolerance;

    while ((xMax < image.cols - 1) && LineValid(dashCount, filter->IsInRage(&image.data[(xMax - 1 + y * image.cols) * 3]))) {
        xMax++;
    }

    return Line(xMin, xMax);
}

Line VLine(const IFilter* filter, cv::Mat& image, int x, int y)
{
    int yMin = y;
    int yMax = y;

    int dashCount = dashTolerance;

    while ((yMin > 1) && LineValid(dashCount, filter->IsInRage(&image.data[(x + yMin * image.cols) * 3]))) {
        yMin--;
    }

    dashCount = dashTolerance;

    while ((yMax < image.rows - 1) && LineValid(dashCount, filter->IsInRage(&image.data[(x - 1 + yMax * image.cols) * 3]))) {
        yMax++;
    }

    return Line(yMin, yMax);
}

MyLine Diagonal(const IFilter* filter, cv::Mat& image, int x, int y)
{
    int xMin = x;
    int xMax = x + 1;
    int yMin = y;
    int yMax = y + 1;

    int dashCount = dashTolerance;

    while ((xMin > 0) && (yMin > 0) && LineValid(dashCount, filter->IsInRage(&image.data[(xMin + yMin * image.cols) *3]))) {
        xMin--;
        yMin--;
    }

    dashCount = dashTolerance;

    while ((xMax < image.cols - 1) && (yMax < image.rows - 1) && LineValid(dashCount, filter->IsInRage(&image.data[(xMax + yMax * image.cols) * 3]))) {
        xMax++;
        yMax++;
    }

    return std::pair<cv::Point, cv::Point>(cv::Point(xMin + dashTolerance ,yMin + dashTolerance),cv::Point(xMax - dashTolerance, yMax + dashTolerance));
}
}

Balldetector::Balldetector(const IFilter* _filter, int _minDiameter, int _maxDiameter, double _tolerance, int _step):
    filter(_filter), tolerance(_tolerance), step(_step), dashLineStep(15)
{

}

void Balldetector::Recurse(Circle& circle, cv::Mat& image, Balldetector::Axis axis, int x, int y, int xMax, int yMax)
{
    if (axis == AXIS_X) {

        Line xLine = HLine(filter, image, x, y);

        circle.x = x;
        circle.y = y;
        if (xLine.Length() > xMax) {

            circle.xWidth = xLine.Length();

            Recurse(circle, image, AXIS_Y, xLine.Center(), y, xLine.Length(), yMax);
        }

    } else {

        Line yLine = VLine(filter, image, x, y);

        circle.x = x;
        circle.y = y;

        if (yLine.Length() > yMax) {

            circle.yWidth = yLine.Length();

            Recurse(circle, image, AXIS_X, x, yLine.Center(), xMax, yLine.Length());
        }
    }
}



Balldetector::Ball Balldetector::Detect(cv::Mat& image)
{
    Ball ball = {0,0,0};

    for (int i = 0; i < image.cols; i += step) {
        for (int j = 0; j < image.rows; j += step ) {
             if (filter->IsInRage(&image.data[(i + image.cols * j) * 3])) {
                image.data[(i + image.cols * j) * 3] = 255;
                Circle detected = {0,0,0,0,0};

                detected.radius = 0;
                Recurse(detected, image, AXIS_X, i, j, 0, 0);
                detected.xWidth -= 2* dashTolerance;
                detected.yWidth -= 2* dashTolerance;

                if (abs(1.f - (double)detected.xWidth / (double)detected.yWidth) < tolerance) {
                    detected.radius = (detected.xWidth + detected.yWidth) / 2;

                    MyLine diag = Diagonal(filter, image, detected.x, detected.y);

                    if (detected.radius > 0  && abs(1.f - (double)detected.radius / (double)(diag.second.x - diag.first.x) * 0.707f) > tolerance) {
                        detected.radius = 0;
                    }

                    if (detected.radius > 20) {

                        circle( image, cv::Point(detected.x, detected.y), detected.radius / 2, cv::Scalar(0,255,0), 3, 8, 0 );
                        ball.radius = detected.radius;
                        ball.x = detected.x;
                        ball.y = detected.y;
                        j += ball.radius;
                        i += ball.radius;

                        return ball;
                    }
                }
            }
        }
    }

    return ball;
}

