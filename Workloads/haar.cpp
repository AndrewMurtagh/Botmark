/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       haar.cpp
*
*
*   DESCRIPTION:    program to detect faces.
*
*
*   DATE:           12/03/2017
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -uses Viola-Jones algorithm to detect faces.
*                   -has opencv3.2.0 as a required dependency.
*                   -images taken from ImageNet.
*                   -uses haarcascade_frontalface_alt.xml as detector.
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


#define NUMFRAMES 100

using namespace std;
using namespace cv;




String face_cascade_name = "../Data/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;



struct timespec starttime, endtime;
double time_accumulator = 0.0f;



void detectAndDisplay( Mat frame );



int main(int argc, const char** argv) {


    Mat frame;

    if(!face_cascade.load(face_cascade_name)) { 
        printf("Error: can't load cascade\n"); 
        return -1;
    };

    for(int i=0; i<NUMFRAMES; i++) {
        // start timer
        if(!DEBUG) {
            clock_gettime(CLOCK_MONOTONIC_RAW, &starttime);
        }


        if(DEBUG) { printf("Processing image: %i \n", i+1); }
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string s = ss.str();
        s = "../Data/facial_detection_images/" + s + ".jpg";
        frame = imread(s);
        if(!frame.empty()) {
            detectAndDisplay(frame);
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


void detectAndDisplay( Mat frame ) {
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );


    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );


    if(DEBUG) {
        namedWindow("Display window", WINDOW_AUTOSIZE );
        for( size_t i = 0; i < faces.size(); i++ ) {
            Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
            ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

            Mat faceROI = frame_gray( faces[i] );
        }
        imshow( "Display window", frame );             
        waitKey(0);
    }

}



