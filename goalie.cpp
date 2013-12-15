/*
 * opencv-test.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Mafioso
 */

#include <iostream>
#include <vector>
#include <ctime>

#include <unistd.h>
#include <stdlib.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "balldetector.hpp"

using std::vector;
using namespace cv;

class Median {
public:
    Median(int _numSamples) : numSamples(_numSamples), sum(0)
    {
        values.clear();
    }

    double Value()
    {
        return sum / values.size();
    }

    double Evaluate(double value)
    {
        sum += value;
        values.push_back(value);

        if (values.size() > numSamples) {
            sum -= values.front();
            values.pop_front();
        }

        return Value();
    }

private:
    std::list<int> values;
    double sum;
    int numSamples;

};


int main (int argc, char** argv)
{
    cv::VideoCapture video;

    std::cout << "Starting" << std::endl;

    //if (video.open("rtsp://doorguard:8554/")) {
    if (video.open("rtsp://192.168.0.232:8554/")) {

        cv::Mat frame;
        Median filterTime(10);

        Balldetector detector(5, 100, 0.2f);

        namedWindow("bw");
        namedWindow("output");

        while (video.read(frame)) {
            clock_t start = clock();

            Balldetector::Ball ball = detector.Detect(frame);

            clock_t end = clock();
            double elapsed_secs = double(end - start); /* ms */

            std::stringstream value;
            value <<  ball.x << "," << ball.y << ":"<< ball.radius;
            putText(frame, value.str(), Point(0, 50), FONT_HERSHEY_PLAIN, 1.7, Scalar(0, 255, 0), 2);

            std::stringstream strs;
            strs << filterTime.Evaluate(elapsed_secs) << " ms";
            putText(frame, strs.str(), Point(0,25), FONT_HERSHEY_PLAIN, 1.7, Scalar(0, 255, 0), 2);

            imshow("output", frame);

            if (cv::waitKey(10) != -1) {
                break;

            }
        }
        video.release();
    } else {
        std::cerr << "Failed to open video" << std::endl;
    }

    std::cout << "Finished" << std::endl;
    return 0;
}

