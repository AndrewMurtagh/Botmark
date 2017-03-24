/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       invkn.c
*
*
*   DATE:           21/02/2017
*
*
*   DESCRIPTION:    program to compute the inverse kinematics of a 3
*   		    	DOF RRR manipulator.
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -Jacobian transposition.
*                   -orientation is not considered, only position.
*                   -terminates once threshold distance to goal
*                    is reached.
*                   -forward kinematics are also computed for visualising
*                    manipulator.
*                   -lengths of linkages are hardcoded.
*                   -metric is millisecond.
*
*
*   VERSION:        v1
*
*
*******************************************************************
******************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "botmark.h"


float pos_vector[4] = {0,0,0,1};
float O[4][4] =  {{1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};



/*************************************************************************/
/** Define parameters **/
/*************************************************************************/
/** DH parameters **/
float a_1 = 0;
float alpha_1 = -90;
float d_1 = 0;
float THETA_1 = 45;

float a_2 = 0.1;
float alpha_2 = 0;
float d_2 = 0;
float THETA_2 = -45;

float a_3 = 0.1;
float alpha_3 = 0;
float d_3 = 0;
float THETA_3 = 45;

/** position vectors **/
float goal_vector[3] = {0.08f, -0.05f, 0.02f};
float curr_vector[3] = {0.0f, 0.0f, 0.0f};


/**algorithm parameters **/
float alpha = 15.0f;
float distance_threshold = 0.01f;





/*************************************************************************/
/** declare functions **/
/*************************************************************************/
float getDistance(float goal[3], float curr[3]);
void matrixVectorMult(float mat[3][3], float vec[3], float out[3]);
void transposeMatrix(float mat[3][3], float out[3][3]);
void constantTimesMatrix(float mat[3][3], float out[3][3], float constant);


struct timespec start, end;





/*************************************************************************/
/** main function **/
/*************************************************************************/
int main() {


    /** start timer **/
    if(!DEBUG) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    }


    do {
        //rename joint position variables for convenience and convert to radians
        float q1 = THETA_1*(PI/180.0);
        float q2 = THETA_2*(PI/180.0);
        float q3 = THETA_3*(PI/180.0);

        //compute current position according to overall transformation matrix
        curr_vector[0] = 0.1f*cos(q1)*cos(q2)*cos(q3) - 0.1f*cos(q1)*sin(q2)*sin(q3) + 0.1f*cos(q1)*cos(q2);
        curr_vector[1] = 0.1f*sin(q1)*cos(q2)*cos(q3) - 0.1f*sin(q1)*sin(q2)*sin(q3) + 0.1f*sin(q1)*cos(q2);
        curr_vector[2] = 0.1f -0.1f*cos(q3)*sin(q2) - 0.1f*cos(q2)*sin(q3) - 0.1f*sin(q2);

        //compute jacobian
        float J[3][3] = {};

        J[0][0] = -0.1f*sin(q1)*cos(q2)*cos(q3) + 0.1f*sin(q1)*sin(q2)*sin(q3) - 0.1f*sin(q1)*cos(q2);
        J[0][1] = -0.1f*cos(q1)*sin(q2)*cos(q3) - 0.1f*cos(q1)*cos(q2)*sin(q3) - 0.1f*cos(q1)*sin(q2);
        J[0][2] = -0.1f*cos(q1)*cos(q2)*sin(q3) - 0.1f*cos(q1)*sin(q2)*cos(q3);

        J[1][0] = 0.1f*cos(q1)*cos(q2)*cos(q3) - 0.1f*cos(q1)*sin(q2)*sin(q3) + 0.1f*cos(q1)*cos(q2);
        J[1][1] = -0.1f*sin(q1)*sin(q2)*cos(q3) - 0.1f*cos(q1)*sin(q2)*sin(q3) + 0.1f*cos(q1)*cos(q2);
        J[1][2] = -0.1f*sin(q1)*cos(q2)*sin(q3) - 0.1f*sin(q1)*sin(q2)*cos(q3);

        J[2][0] = 0.0f;
        J[2][1] = -0.1f*cos(q3)*cos(q2) + 0.1f*sin(q2)*sin(q3) - 0.1f*cos(q2);
        J[2][2] = 0.1f*sin(q3)*sin(q2) - 0.1f*cos(q2)*cos(q3);

        //Get delta e
        float delta_e[3] = {};
        delta_e[0] = goal_vector[0] - curr_vector[0];
        delta_e[1] = goal_vector[1] - curr_vector[1];
        delta_e[2] = goal_vector[2] - curr_vector[2];


        float Jtranspose[3][3] = {};
        transposeMatrix(J, Jtranspose);
        float JtransposeTimesConstant[3][3] = {};
        constantTimesMatrix(Jtranspose, JtransposeTimesConstant, alpha);
        float delta_q[3] = {};
        matrixVectorMult(JtransposeTimesConstant, delta_e, delta_q);

        //update joint postions
        THETA_1 = THETA_1 + delta_q[0];
        THETA_2 = THETA_2 + delta_q[1];
        THETA_3 = THETA_3 + delta_q[2];


    } while(getDistance(goal_vector, curr_vector) > distance_threshold);



    /** stop timer **/
    if(!DEBUG) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        double microseconds = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
        double ms = microseconds/1000;
        printf ("Execution time in milliseconds: ");
        printf ("%f \n", ms);
	}



    return 0;
}





/*************************************************************************/
/** define functions **/
/*************************************************************************/
float getDistance(float goal[3], float curr[3]) {
    return sqrt(pow((goal[0] - curr[0]),2) + pow((goal[1] - curr[1]),2) + pow((goal[2] - curr[2]),2));

}

void matrixVectorMult(float mat[3][3], float vec[3], float out[3]) {
	out[0] = mat[0][0]*vec[0] + mat[0][1]*vec[1] + mat[0][2]*vec[2];
	out[1] = mat[1][0]*vec[0] + mat[1][1]*vec[1] + mat[1][2]*vec[2];
	out[2] = mat[2][0]*vec[0] + mat[2][1]*vec[1] + mat[2][2]*vec[2];
}

void transposeMatrix(float mat[3][3], float out[3][3]) {
    out[0][0] = mat[0][0];
    out[0][1] = mat[1][0];
    out[0][2] = mat[2][0];

    out[1][0] = mat[0][1];
    out[1][1] = mat[1][1];
    out[1][2] = mat[2][1];

    out[2][0] = mat[0][2];
    out[2][1] = mat[1][2];
    out[2][2] = mat[2][2];
}

void constantTimesMatrix(float mat[3][3], float out[3][3], float constant) {
    out[0][0] = constant*mat[0][0];
    out[0][1] = constant*mat[0][1];
    out[0][2] = constant*mat[0][2];

    out[1][0] = constant*mat[1][0];
    out[1][1] = constant*mat[1][1];
    out[1][2] = constant*mat[1][2];

    out[2][0] = constant*mat[2][0];
    out[2][1] = constant*mat[2][1];
    out[2][2] = constant*mat[2][2];

}



