/*
 * opencv-test.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Mafioso
 */


#include "serial/serial.h"
#include "raspicamcv/RaspiCamCV.h"
#include "balldetector.hpp"
#include "hsvfilter.hpp"

#include <iostream>
#include <sstream>
#include <ctime>

#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//#define DebugOut(x) {std::cout << x << std::endl;}
//#define DebugOut(x) {std::stringstream  _foo; _foo << x; mvprintw(23, 0, _foo.str().c_str());}
#define DebugOut(x)

using std::vector;
using namespace cv;

static bool f_running = true;

class Median {
public:
    Median(int _numSamples) : numSamples(_numSamples)
    {

    }

    float Evaluate(float value)
    {
        sum += value;
        values.push_back(value);

        if (values.size() > numSamples) {
            sum -= values.front();
            values.pop_front();
        }

        return sum / values.size();
    }

private:
    std::list<int> values;
    float sum;
    int numSamples;

};

class Timer {
public:
    void Start()
    {
        begin = clock();
    }
    
    void Stop()
    {
        end = clock();
    }
    
    double Duration()
    {
        return double (end - begin) / CLOCKS_PER_SEC;
    }
    
private:
    clock_t begin;
    clock_t end;
};

static void UpdateAxis(const char* channel, float value, serial::Serial& serialPort)
{
    std::stringstream message;
    message << channel << value << '!';
    serialPort.write(message.str());
}

static float Limit(float val, float min, float max)
{
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

static float posX = 0;
static float posY = 0;

static void ResetPosition(serial::Serial& serialPort)
{
    posX = 0;
    posY = 0;
    
    UpdateAxis("a", posX, serialPort);
    UpdateAxis("b", posY, serialPort);
}
    
static void Follow(serial::Serial& serialPort, int width, int height, int x, int y)
{
    //static float posX = 0;
    //static float posY = 0;
    const float rate = 0.1f;
    
    float centerX = width / 2;
    float centerY = height / 2;
    
    /* Normalized diff */
    float diffX = (x - centerX) * 2.f / width;
    float diffY = (y - centerY) * 2.f / height;
    
    posX += diffX * rate;
    posY += diffY * rate;

    posX = Limit(posX, -1.f, 1.f);
    posY = Limit(posY, -1.f, 1.f);
    
    DebugOut(diffX << " " << diffY << " servo x: " << posX << " servo y: " << posY );
    
    UpdateAxis("a", posX, serialPort);
    UpdateAxis("b", posY, serialPort);
}

void HighlightCircles(cv::Mat& src, cv::Mat& result, serial::Serial& port)
{
    vector<Vec3f> circles;
    HoughCircles(src, circles, CV_HOUGH_GRADIENT,
                 2, src.rows, 100, 50, 10, 75);
    
    if (circles.size() > 0) {
        Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
        int radius = cvRound(circles[0][2]);
        // draw the circle center
        //circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( result, center, radius, Scalar(255,0,0), 3, 8, 0 );
        DebugOut("x: " << center.x << " y: " << center.y);
        Follow(port, src.cols, src.rows, center.x, center.y);
    }
    
}

#define TIMED_BLOCK_START() {Timer _timer; _timer.Start();

#define TIMED_BLOCK_END(x, y) _timer.Stop(); std::stringstream _str; _str << _timer.Duration(); mvprintw(x, y, _str.str().c_str());}


static void PrintReadTime(float time)
{
    std::stringstream value;
    value << time;
    mvprintw(0, 6, value.str().c_str());
}

void OpenVideo2()
{
    serial::Serial serialPort("/dev/ttyAMA0", 38400);
    
    UpdateAxis("a", 0, serialPort);
    UpdateAxis("b", 0, serialPort);
    
    RaspiCamCvCapture * capture = raspiCamCvCreateCameraCapture(0); // Index doesn't really matter
    
    DebugOut("Opening video");

    if (capture != 0) {
  /*  
        int minH = 109;
        int maxH = 132;
        int minS = 167;
        int maxS = 255;
        int minV = 46;
        int maxV = 158;
    */    
        HsvFilter filter;
        filter.SetLimits(cv::Scalar(109,167,46), cv::Scalar(132, 255, 158));

        Balldetector detector(&filter, 5, 100, 0.3f);
        
        
        //Mat frame;
      /*  Mat hsv;
        Mat red;*/
        int i = 0;
        IplImage* image = raspiCamCvQueryFrame(capture);
        //bool readOk = video.read(frame); 
        bool readOk = image != 0;
        Mat frame(image);

        int timeout = 0;
        while (f_running && readOk) {
/*
            TIMED_BLOCK_START();
            cvtColor(frame, hsv, CV_RGB2HSV);
            TIMED_BLOCK_END(1, 8);

            TIMED_BLOCK_START();
            inRange(hsv, Scalar(minH,minS,minV), Scalar(maxH, maxS, maxV), red);
            TIMED_BLOCK_END(2, 9);
            
            TIMED_BLOCK_START()
            GaussianBlur(red, red, Size(9, 9), 2, 2);
            TIMED_BLOCK_END(3, 14);
*/          
            Balldetector::Ball ball;
            TIMED_BLOCK_START();
            //HighlightCircles(red, frame, serialPort);
            ball = detector.Detect(frame);
            TIMED_BLOCK_END(4, 18);
            
            TIMED_BLOCK_START();
            //readOk = video.read(frame); 
            image = raspiCamCvQueryFrame(capture);
            frame = image;
            TIMED_BLOCK_END(0, 7);
           
            std::stringstream asdf;
            asdf << i++;
            mvprintw(6, 12, asdf.str().c_str());
            refresh();
            if (ball.radius > 0) {
                Follow(serialPort, frame.cols, frame.rows, ball.x, ball.y);
                mvprintw(7, 11, "%i,%i      ", ball.x, ball.y);
            } else {
                timeout++;
                if (timeout == 50) {
                    ResetPosition(serialPort);
                }
            }
        }
        imwrite("/var/www/opencv.jpg", frame);
        
        raspiCamCvReleaseCapture(&capture);
        DebugOut("Video finished");
    } else {
        ("Failed to open video");
    }
}

static void HandleSigInt(int signum)
{
    f_running = false;
}

int main (int argc, char** argv)
{
    signal(SIGINT, HandleSigInt);
    initscr(); 
    mvprintw(0, 0, "Read: ");
    mvprintw(1, 0, "cvtColor: ");
    mvprintw(2, 0, "inRange: ");
    mvprintw(3, 0, "GaussianBlur: ");
    mvprintw(4, 0, "HighlightCircles: ");
    mvprintw(5, 0, "Total: ");
    mvprintw(6, 0, "Frame count: ");
    mvprintw(7, 0, "Position: ");
    refresh();
    OpenVideo2();
    
    endwin();
    
    //std::cout << "Finished" << std::endl;
    return 0;
}

