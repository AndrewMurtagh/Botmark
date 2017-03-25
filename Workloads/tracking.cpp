/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       tracking.cpp
*
*
*   DESCRIPTION:    program to track object.
*
*
*   DATE:           25/03/2017
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -splits rgb into each plane, applies threshold on each,
*		    -applies bitwise OR, median blur, image moments, blob centroid.
*                   -has opencv3.2.0 as a required dependency.
*                   -images taken from camear phone
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


#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "botmark.h"


#define NUMFRAMES 38
#define REDTHRESHOLD  217
#define GREENTHRESHOLD 191
#define BLUETHRESHOLD 204


using namespace std;
using namespace cv;







struct timespec starttime, endtime;
double time_accumulator = 0.0f;



void proessFrame(Mat frame);



int main(int argc, const char** argv) {


    Mat frame;


    for(int i=0; i<NUMFRAMES; i++) {
        // start timer
        if(!DEBUG) {
            clock_gettime(CLOCK_MONOTONIC_RAW, &starttime);
        }


        if(DEBUG) { printf("Processing image: %i \n", i+1); }
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string s = ss.str();
        s = "../Data/object_tracking_images/" + s + ".png";
        frame = imread(s);
        if(!frame.empty()) {
            proessFrame(frame);
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


void proessFrame( Mat frame ) {
	Mat colour_planes[3]; //OpenCV uses BGR color order
	Mat redbinary, greenbinary, bluebinary;
	split(frame, colour_planes);

	int rows = frame.rows;
    int cols = frame.cols;

    threshold(colour_planes[2], redbinary, REDTHRESHOLD, 255, 0);
    

    threshold(colour_planes[1], greenbinary, GREENTHRESHOLD, 1, 0);
   

    threshold(colour_planes[0], bluebinary, BLUETHRESHOLD, 1, 0);
   


     Mat combinedBinary = Mat::zeros(rows, cols, CV_8U);
    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
        	int thisvalue = (int)redbinary.at<uchar>(i, j) || (int)greenbinary.at<uchar>(i, j) || (int)bluebinary.at<uchar>(i, j);
            combinedBinary.at<uchar>(i, j) = thisvalue*1;
        }
        
    }
    



    Mat blurredImage;
    medianBlur(combinedBinary, blurredImage, 9);

    int blobarea = 0;
    int sumi = 0;
    int sumj = 0;
     for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            int thisvalue = (int)blurredImage.at<uchar>(i, j);
            blobarea += thisvalue;
            sumi += i*thisvalue;
            sumj += j*thisvalue;
        }
        
    }
    
    int centroidi = sumi/blobarea;
    int centroidj = sumj/blobarea;
    
    circle(frame, Point(centroidj, centroidi), 6, CV_RGB(0,0,255), 1, 8, 0);   
    if(DEBUG) {
		imshow("Display window", frame);          
	    waitKey(0);
	}

    

}



