/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       ogm.cpp
*
*
*   DESCRIPTION:    program to make map using occupancy grid mapping.
*
*
*   DATE:           21/02/2017
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -data is from laser rangefinder from RAWSEEDS.
*                   -ground truth is known.
*                   -grid is declared on heap because of size
*                   -writes map to 'map_validation.txt' file for
*                    MATLAB validation.
*                   -metric is millisecond.
*
*
*   VERSION:        v1
*
*
*******************************************************************
******************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <sstream>
#include <iterator>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <time.h>


#include "botmark.h"



#define GRIDSIZE 1500
#define NUMSCANS 682


using namespace std;



/*************************************************************************/
/** define parameters **/
/*************************************************************************/

float minX = -35;
float maxX = 82;
float minY = -60;
float maxY = 77;

float maxdist = 5.6;
int ind_data= 2;
float angles[NUMSCANS];
float timestep, x, y, theta;
float angle_min = -2.0944f;
float angle_max = 2.0944f;
float angle_inc =(360.0f/1024.0f)*(PI/180.0f);
float range_min = 0;
float range_max = 5.6f;
float ranges[NUMSCANS];

float alpha = 0.2f;
float prob_prior = 0;
float prob_occ = 2;
float prob_free = -2;
float grid_inc_size = 0.1f;
float mapx[GRIDSIZE];
float mapy[GRIDSIZE];

struct timespec starttime, endtime;


/*************************************************************************/
/** declare functions **/
/*************************************************************************/
float returnMin(float a, float b);





/*************************************************************************/
/** main function **/
/*************************************************************************/
int main() {



    /*********************************************************************/
    /** initiliase parameters **/
    /*********************************************************************/
    float *grid = new float[GRIDSIZE*GRIDSIZE];

    for(int i=0; i < GRIDSIZE; i++) {
        mapx[i] = minX + i*grid_inc_size;
        mapy[i] = minY + i*grid_inc_size;
    }
    for(int i=0; i < NUMSCANS; i++) {
        angles[i] = -2.0944f + i*angle_inc;
    }

    /** start timer **/
    if(!DEBUG) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &starttime);
    }









    /*********************************************************************/
    /** occupancy grid mapping **/
    /*********************************************************************/
    ifstream infile("../Data/ogm_data.txt");
    int inc=1;
    for(string line; getline(infile, line);) {
        if(DEBUG) {
            cout << "line " << inc << endl;
        }
        if(inc == 100) break; //only read in 100 lines
        inc++;

        std::stringstream stream(line);
        stream >> timestep;
        stream >> x;
        stream >> y;
        stream >> theta;
        

        float ranges[NUMSCANS];
        for(int i=0; i<NUMSCANS; i++) {
            stream >> ranges[i];
        }

        float beta = angle_inc;
        float min_angle_range = theta + angle_min;
        float max_angle_range = theta + angle_max;
        for(int i=0; i<GRIDSIZE; i++) {
            for(int j=0;j<GRIDSIZE; j++) {

                float angle2 = atan2(mapy[i] - y, mapx[j] - x);

                if (angle2 > min_angle_range && angle2 < max_angle_range) {

                    float r = sqrt(pow((x - mapx[j]),2.0f) + pow((y - mapy[i]),2.0f));
                    float phi = angle2 - theta;

                    int k = (int)( abs( round( ((phi - angle_min)/angle_inc)+0.5f ) ));
                    
                    if (ranges[k] > range_max || ranges[k] < range_min) { //TODO: CLEANUP
                        grid[i*GRIDSIZE+j] = grid[i*GRIDSIZE+j];

                    } else if(r > range_max || r < range_min) {
                        grid[i*GRIDSIZE+j] = grid[i*GRIDSIZE+j];


                    } else if (r > returnMin(range_max, ranges[k] + alpha/2.0f) || abs(phi - k*angle_inc - angle_min) > beta/2.0f) {
                        grid[i*GRIDSIZE+j] = grid[i*GRIDSIZE+j];

                    } else if (ranges[k] < range_max && abs(r - ranges[k]) < alpha/2.0f) {
                   //     cout << "here noe" << endl;
                        grid[i*GRIDSIZE+j] = grid[i*GRIDSIZE+j] + prob_occ - prob_prior;

                    } else if (r <= ranges[k]) {
                    //    cout << "here noe" << endl;
                        grid[i*GRIDSIZE+j] = grid[i*GRIDSIZE+j] + prob_free - prob_prior;

                    }



                }
                //in percpetual range

                
            }
        }
        

    }
    infile.close();


    /** convert from log-odds to probability **/
    for(int i=0; i<GRIDSIZE; i++) {
        for(int j=0;j<GRIDSIZE; j++) {
            float temp = grid[i*GRIDSIZE+j];
            grid[i*GRIDSIZE+j] = 1.0f - (1.0f/(1.0f + exp(temp)));
        }
    }



    /*********************************************************************/
    /** write data for MATLAB validation **/
    /*********************************************************************/

   if(DEBUG) {
        cout << "outputting to file" << endl;
        std::ofstream outfile ("../Visualisation/ogm_validation.txt");
        for(int i=0; i<GRIDSIZE; i++) {
            for(int j=0;j<GRIDSIZE; j++) {

                outfile << grid[i*GRIDSIZE+j] << ' ';
            }
            outfile << '\n';

        }
        outfile.close();


    }







    /*********************************************************************/
    /** stop time **/
    /*********************************************************************/
    if(!DEBUG) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &endtime);
        double microseconds = (endtime.tv_sec - starttime.tv_sec) * 1000000 + (endtime.tv_nsec - starttime.tv_nsec) / 1000;
        double ms = microseconds/1000;
        printf ("Execution time in milliseconds: ");
        printf ("%f \n", ms);
    }


    /** deallocate heap variables **/
    delete [] grid;


    return 0;
}




/*************************************************************************/
/** define functions **/
/*************************************************************************/
float returnMin(float a, float b) {
    if(a < b) {
        return a;
    } else {
        return b;
    }
}
