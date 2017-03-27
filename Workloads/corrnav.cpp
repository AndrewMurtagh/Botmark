/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       corrnav.cpp
*
*
*   DESCRIPTION:    program to visually navigate a corrider.
*
*
*   DATE:           14/03/2017
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -uses threhold to segment image into 'floor' and
*                   'wall' region.
*                   -scans the bottom half to find boundary and fits
*                   a line down the center of the corrider.
*                   -the turning command will be a scaled multiple of the
*                   slope of this line. 
*                   -metric is frame rate [Hz]
*
*
*   VERSION:        v1
*
*
*******************************************************************
******************************************************************/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "botmark.h"


#define NUMFRAMES 20
#define THRESHOLD 62
#define STARTPOINT 0.6f

using namespace std;
using namespace cv;




struct timespec starttime, endtime;
double time_accumulator = 0.0f;




void processFrame(Mat frame);
int sumX(int array[][2], int lenght);
int sumY(int array[][2], int lenght);
int sumXX(int array[][2], int lenght);
int sumXY(int array[][2], int lenght);
int getCardinality(int array[][2], int lenght);
float rad2deg(float rads);



int main(int argc, const char** argv) {


    srand(time(0));
    Mat frame;


    Mat frame_gray, binary_frame;

    for(int i=0; i<NUMFRAMES; i++) {


        if(DEBUG) { printf("Processing image: %i \n", i+1); }
        std::stringstream ss;
        ss << std::setw(1) << std::setfill('0') << i;
        std::string s = ss.str();
        s = "../Data/corrider_images/" + s + ".png";
        frame = imread(s);


        // start timer
        if(!DEBUG) {
            clock_gettime(CLOCK_MONOTONIC_RAW, &starttime);
        }


        if(!frame.empty()) {
            processFrame(frame);
        } else {
            printf("Error: no frame detected\n");
            break;
        }


        // stop timer
        if(!DEBUG) {
            clock_gettime(CLOCK_MONOTONIC_RAW, &endtime);
            double microseconds = (endtime.tv_sec - starttime.tv_sec) * 1000000 + (endtime.tv_nsec - starttime.tv_nsec) / 1000;
            double ms = microseconds/1000;
            time_accumulator += ms;
        }
    }

    if(!DEBUG) {
        printf ("Total time in milliseconds: %f \n", time_accumulator);
        double mean = time_accumulator/NUMFRAMES;
        printf ("Mean time per frame: %f \n", mean);
        float framerate = NUMFRAMES/(time_accumulator/1000.0f);
        printf ("Mean frame rate: %f \n", framerate);

    }

    return 0;
 }


void processFrame(Mat frame) {
    Mat frame_gray, binary_frame;

    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    if(DEBUG) {
        imshow("Display window", frame_gray);             
        waitKey(0);
    }


    threshold(frame_gray, binary_frame, THRESHOLD, 255, 0);
    if(DEBUG) {
        imshow("Display window", binary_frame);             
        waitKey(0);
    }
    int rows = frame.rows;
    int cols = frame.cols;


    int startrow = round((float)rows*STARTPOINT);
    
    int numrows = rows - startrow;

    int leftPositions[numrows][2]; //x, y
    int rightPositions[numrows][2]; //x,y
    int centerPositions[numrows][2];

    for(int i=0; i<numrows; i++) {
        leftPositions[i][0] = -1;
        leftPositions[i][1] = -1;
        rightPositions[i][0] = -1;
        rightPositions[i][1] = -1;
    }
    
    
    for(int i=0; i<numrows; i++) {
        int centerPoint = cols/2;
        bool leftFound = false;
        int leftPoint = centerPoint;
        
        while (!leftFound && leftPoint>0) {
            int leftPix = (int)binary_frame.at<uchar>(i+startrow, leftPoint);
            if(leftPix != 255) {
                leftFound = true;
            } else {
                leftPoint--;
            }
        }
        if(DEBUG) {
            circle(frame, Point(leftPoint, startrow+i), 1, CV_RGB(255,255,0), 1, 8, 0);
        }
        

        bool rightFound = false;
        int rightPoint = centerPoint;
        
        while (!rightFound && rightPoint<cols) {
            int rightPix = (int)binary_frame.at<uchar>(i+startrow, rightPoint);
           // cout << leftPix << endl;
            if(rightPix != 255) {
                rightFound = true;
            } else {
                rightPoint++;
            }
        }
        if(DEBUG) {
            circle(frame, Point(rightPoint, startrow+i), 1, CV_RGB(255,255,0), 1, 8, 0);
        }

        if(leftPoint==0 && rightPoint==cols) {
            centerPositions[i][0] = -1;
            centerPositions[i][1] = -1;
        } else {
            centerPositions[i][0] = (int)(0.5f*(leftPoint + rightPoint));
            centerPositions[i][1] = i+startrow;
        }
    
        if(DEBUG) {
            circle(frame, Point(centerPositions[i][0], centerPositions[i][1]), 1, CV_RGB(0,255,0), 1, 8, 0);
        }
    }



    float n = (float)getCardinality(centerPositions, numrows);
    float sumx = (float)sumX(centerPositions, numrows);
    float sumy = (float)sumY(centerPositions, numrows);
    float sumxx = (float)sumXX(centerPositions, numrows);
    float sumxy = (float)sumXY(centerPositions, numrows);

    float m = (n*sumxy - sumx*sumy) / (n*sumxx - sumx*sumx);

    
    if(DEBUG) {
    float c = (sumy*sumxx - sumx*sumxy) / (n*sumxx - sumx*sumx);
        int plotY1 = 0; 
        int plotX1 = (plotY1-c)/m;
        int plotY2 = 512;
        int plotX2 = (plotY2-c)/m;
      
        line(frame, Point(plotX1, plotY1), Point(plotX2, plotY2), CV_RGB(0,0,255), 1, 8, 0);
    }

    
    float angleToTurn;
    if(m < 0) {
        angleToTurn = 90.0f  + rad2deg(atan(m));
    } else {
        angleToTurn = rad2deg(atan(m)) - 90.0f;
    }


    if(DEBUG) {
        cout << "Turn by: " << angleToTurn << endl;
    }

    if(DEBUG) {
        imshow("Display window", frame);             
        waitKey(0);
    }

}




int getCardinality(int array[][2], int lenght) {
    int sum = 0;
    for(int i=0; i<lenght; i++) {
        if(array[i][0] != -1 && array[i][1] != -1) {
            sum += 1;
        }
    }
    return sum;
}

int sumX(int array[][2], int lenght) {
    int sum = 0;
    for(int i=0; i<lenght; i++) {
        if(array[i][0] != -1 && array[i][1] != -1) {
            sum += array[i][0];
        }
    }
    return sum;
}

int sumY(int array[][2], int lenght) {
    int sum = 0;
    for(int i=0; i<lenght; i++) {
        if(array[i][0] != -1 && array[i][1] != -1) {
            sum += array[i][1];
        }
    }
    return sum;
}



int sumXY(int array[][2], int lenght) {
    int sum = 0;
    for(int i=0; i<lenght; i++) {
        if(array[i][0] != -1 && array[i][1] != -1) {
            sum += array[i][0]*array[i][1];
        }
    }
    return sum;
}
int sumXX(int array[][2], int lenght) {
    int sum = 0;
    for(int i=0; i<lenght; i++) {
        if(array[i][0] != -1 && array[i][1] != -1) {
            sum += array[i][0]*array[i][0];
        }
    }
    return sum;
}


float rad2deg(float rads) {
    return (rads)*(180.0f/PI);
}