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
#include "median.hpp"
#include "timer.hpp"

#include <iostream>
#include <sstream>

#include <ncurses.h>
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

    HsvFilter filter;
        filter.SetLimits(cv::Scalar(109,167,46), cv::Scalar(132, 255, 158));

        Balldetector detector(&filter, 5, 100, 0.3f);
        
        int frameCount = 0;
        IplImage* image = raspiCamCvQueryFrame(capture);
        bool readOk = image != 0;
        Mat frame(image);

        int timeout = 0;
        while (f_running && readOk) {
            Balldetector::Ball ball;

            TIMED_BLOCK_START();
            ball = detector.Detect(frame);
            TIMED_BLOCK_END(1, 8);
            
            TIMED_BLOCK_START();
            image = raspiCamCvQueryFrame(capture);
            frame = image;
            TIMED_BLOCK_END(0, 7);
           
            std::stringstream asdf;
            asdf << frameCount++;
            mvprintw(2, 12, asdf.str().c_str());
            refresh();
            
            if (ball.radius > 0) {
                Follow(serialPort, frame.cols, frame.rows, ball.x, ball.y);
                mvprintw(3, 11, "%i,%i      ", ball.x, ball.y);
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
        DebugOut("Failed to open video");
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
    mvprintw(1, 0, "Detect: ");
    mvprintw(2, 0, "Frame count: ");
    mvprintw(3, 0, "Position: ");
    refresh();
    OpenVideo2();
    
    endwin();
    
    //std::cout << "Finished" << std::endl;
    return 0;
}

