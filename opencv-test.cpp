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

void FindContours(cv::Mat& src)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat dst = Mat::zeros(src.rows, src.cols, CV_8UC3);
    findContours( src, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
    }

    namedWindow( "Components", 1 );
    imshow( "Components", dst );

}

void HighlightCircles(cv::Mat& src, cv::Mat& result)
{
   /* Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);


    namedWindow( "gray", 1 );
    imshow( "gray", gray );


    // smooth it, otherwise a lot of false circles may be detected
    GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

*/
    //namedWindow( "blurr", 1 );
    //imshow( "blurr", src );


    vector<Vec3f> circles;
    HoughCircles(src, circles, CV_HOUGH_GRADIENT,
                 2, src.rows, 100, 50, 10, 75 );
    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         //circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( result, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }
}


void DetectHoughCircles(cv::Mat& img)
{
    Mat gray;

    cvtColor(img, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;

    HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
                 2, gray.rows/4, 1000, 300, 0, 100 );

    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
}

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


void OpenVideo2()
{
    cv::VideoCapture video;

    std::cout << "Opening video" << std::endl;
    if (video.open("rtsp://doorguard:8554/")) {


        namedWindow("video");
        namedWindow("orig");
        //namedWindow("blur");

        int minH = 109;
        int maxH = 132;
        int minS = 167;
        int maxS = 255;
        int minV = 46;
        int maxV = 158;
        createTrackbar("H min", "video", &minH, 255, 0, 0);
        createTrackbar("H max", "video", &maxH, 255, 0, 0);
        createTrackbar("S min", "video", &minS, 255, 0, 0);
        createTrackbar("S max", "video", &maxS, 255, 0, 0);
        createTrackbar("V min", "video", &minV, 255, 0, 0);
        createTrackbar("V max", "video", &maxV, 255, 0, 0);

        Mat frame;
        Mat hsv;
        Mat red;

        Median filterTime(10);
        while (video.read(frame)) {
            clock_t start = clock();
            cvtColor(frame,hsv, CV_RGB2HSV);
            //GaussianBlur(red, red, Size(9, 9), 2, 2);
            //imshow("blur", red);
            inRange(hsv, Scalar(minH,minS,minV), Scalar(maxH, maxS, maxV), red);


            //GaussianBlur(gray, gray, Size(15, 15), (sigma - 50) / 10.f, (sigma - 50) / 10.f);
            //adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 25, (C - 50) / 10.f);
            //inRange(frame, Scalar(0, 0, 0), Scalar(20, 20, 255) , red);

            //clock_t start = clock();
            HighlightCircles(red, frame);
            clock_t end = clock();
            double elapsed_secs = double(end - start); // / CLOCKS_PER_SEC * 1000;

            std::stringstream strs;
            strs << filterTime.Evaluate(elapsed_secs) << " ms";

            putText(frame, strs.str(), Point(0,25), FONT_HERSHEY_PLAIN, 1.7, Scalar(0, 255, 0), 2);
            imshow("video", red);
            imshow("orig", frame);
            if (cv::waitKey(10) != -1) {
                break;

            }
        }
        video.release();

        std::cout << frame.cols << " x " << frame.rows << std::endl;
        std::cout << "Video finished" << std::endl;
    } else {
        std::cout << "Failed to open video" << std::endl;
    }
}


void HaarCascade()
{
    cv::VideoCapture video;

    if (video.open("rtsp://doorguard:8554/")) {

        Mat frame;
        Median filterTime(10);

        namedWindow("orig");

        CascadeClassifier classifier;
        //if (!classifier.load("lbpcascade_frontalface.xml")) {
        if (!classifier.load("haar_training/cascade.xml")) {
            std::cout << "Failed to load classifier" << std::endl;
            return;
        }


        while (video.read(frame)) {
            clock_t start = clock();

            std::vector<cv::Rect> objs;

            classifier.detectMultiScale(
                frame,
                objs,
                1.2f, 3, 0, Size(10,10), Size(40,40));

            std::stringstream value;
            value << objs.size();
            putText(frame, value.str(), Point(0, 50), FONT_HERSHEY_PLAIN, 1.7, Scalar(0, 255, 0), 2);
            clock_t end = clock();
            double elapsed_secs = double(end - start); // / CLOCKS_PER_SEC * 1000;

            std::stringstream strs;
            strs << filterTime.Evaluate(elapsed_secs) << " ms";

            putText(frame, strs.str(), Point(0,25), FONT_HERSHEY_PLAIN, 1.7, Scalar(0, 255, 0), 2);
            imshow("orig", frame);
            if (cv::waitKey(10) != -1) {
                break;

            }
        }


        video.release();
    }
}




/*
Circle DetectCircle(cv::Mat& image, cv::Mat& orig, int x, int y, int tolerance)
{
    int xMax = -1;
    int yMax = -1;

    int y_xMax = y;
    int x_yMax = x;

    int xWidth = 0;
    int yWidth = 0;
    do {
        xMax = xWidth;
        yMax = yWidth;

        Point px = XCenter(image, orig, x_yMax, y_xMax);
        xMax = px.y - px.y;
        x_yMax = (px.y + px.y) / 2;

        Point py = YCenter(image, orig, x_yMax, y_xMax);
        yMax = py.y - py.y;
        y_xMax = (py.y + py.y) / 2;

    } while ((xMax < xWidth) && (yMax < yWidth));
}
*/

int main (int argc, char** argv)
{
    cv::VideoCapture video;

    namedWindow("bw");
    namedWindow("output");

    if (video.open("rtsp://doorguard:8554/")) {

        cv::Mat frame;
        Median filterTime(10);

        while (video.read(frame)) {
            clock_t start = clock();

            Circle ball = DetectCircles(frame);

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
    }

    std::cout << "Finished" << std::endl;
    return 0;
}

