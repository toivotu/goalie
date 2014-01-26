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
    orig.data[(x + orig.cols * y) * 3 + 2] = 255;
    int xMin = x;
    int xMax = x + 1;
    int yMin = y;
    int yMax = y + 1;

    while ((xMin > 0) && (yMin > 0) && image.data[xMin + yMin * image.cols] == 255u) {
        orig.data[(xMin + orig.cols * yMin) * 3] = 255;
        xMin--;
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

Balldetector::Balldetector(const IFilter* _filter, int _minDiameter, int _maxDiameter, double _tolerance, int _step):
    filter(_filter), tolerance(_tolerance), step(_step)
{

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
    Ball ball = {0,0,0};

    cv::Mat hsv;
    cv::Mat red;

    filter->Evaluate(image, red);

    imshow("bw", red);

    for (int i = 0; i < image.cols; i += step) {
        for (int j = 0; j < image.rows; j += step) {
             if (red.data[i + image.cols * j] == 255u) {
                image.data[(i + image.cols * j) * 3] = 255;

                Line line = HLine(red, image, i, j);
                Circle detected = {0,0,0,0,0};

                detected.radius = 0;
                Recurse(detected, red, image, AXIS_X, i, j, 0, 0);

                if (abs(1.f - (double)detected.xWidth / (double)detected.yWidth) < tolerance) {
                    detected.radius = (detected.xWidth + detected.yWidth) / 2;

                    MyLine diag = Diagonal(red, image, detected.x, detected.y);


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

