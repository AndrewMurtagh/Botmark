/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       forkin.c
*
*
*   DESCRIPTION:    program to compute the forward kinematics of a
*   		    	      humanoid robot.
*
*
*   DATE:           19/02/2017
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -DH parameters in BOLD represent joints that can be moved.
*                   -updates several joint positions over 20 iterations.
*                   -yaw is about z, pitch is about y and roll is about x.
*                   -right and left are from robot's point of view.
*                   -movements are reversed between left and right limbs.
*                   -see MATLAB file for explanation.
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
#include <string.h>

#include "botmark.h"


float pos_vector[4] = {0,0,0,1};
float O[4][4] =  {{1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};

/*************************************************************************/
/** Define DH parameters **/
/*************************************************************************/
//head
float head_a_1 = 0;
float head_alpha_1 = -90;
float head_d_1 = 0;
float HEAD_THETA_1 = -20;

float head_a_2 = 0;
float head_alpha_2 = 90;
float head_d_2 = 0;
float HEAD_THETA_2 = 10;

float NeckOffset[4][4] ={{1, 0, 0, 0},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0.126},
                         {0, 0, 0, 1}};



//right arm
float ra_a_1 = 0;
float ra_alpha_1 = -90;
float ra_d_1 = 0;
float RA_THETA_1 = -120;

float ra_a_2 = 0;
float ra_alpha_2 = 90;
float ra_d_2 = 0;
float RA_THETA_2 = 90;

float ra_a_3 = 0;
float ra_alpha_3 = 90;
float ra_d_3 = 0.11;
float RA_THETA_3 = -10;

float ra_a_4 = 0;
float RA_ALPHA_4 = -120;
float ra_d_4 = 0;
float ra_theta_4 = 0;

float ra_a_5 = 0;
float ra_alpha_5 = 90;
float ra_d_5 = 0.09;
float ra_theta_5 = 0;

float RightArmOffset[4][4] ={{1, 0, 0, 0},
                             {0, 1, 0, -0.098},
                             {0, 0, 1, 0.1},
                             {0, 0, 0, 1}};



//left arm
float la_a_1 = 0;
float la_alpha_1 = -90;
float la_d_1 = 0;
float LA_THETA_1 = -90;

float la_a_2 = 0;
float la_alpha_2 = 90;
float la_d_2 = 0;
float LA_THETA_2 =90;

float la_a_3 = 0;
float la_alpha_3 = 90;
float la_d_3 = 0.11;
float LA_THETA_3 = -10;

float la_a_4 = 0;
float LA_ALPHA_4 = -90;
float la_d_4 = 0;
float la_theta_4 = 0;

float la_a_5 = 0;
float la_alpha_5 = 90;
float la_d_5 = 0.09;
float la_theta_5 = 0;


float LeftArmOffset[4][4] ={{1, 0, 0, 0},
                             {0, 1, 0, 0.098},
                             {0, 0, 1, 0.1},
                             {0, 0, 0, 1}};





//left leg
float ll_a_1 = 0;
float LL_ALPHA_1 = 0;
float ll_d_1 = -0.0;
float ll_theta_1 = 0;

float ll_a_2 = -0.0;
float LL_ALPHA_2 = 0;
float ll_d_2 = 0;
float ll_theta_2 = -90;

float ll_a_3 = 0;
float LL_ALPHA_3 = 0;
float ll_d_3 = -0.1;
float ll_theta_3 = 0;

float ll_a_4 = 0;
float LL_ALPHA_4 = 0;
float ll_d_4 = -0.12;
float ll_theta_4 = 0;

float ll_a_5 = 0;
float LL_ALPHA_5 = 0;
float ll_d_5 = -0.0;
float ll_theta_5 = 90;

float ll_a_6 = 0;
float ll_alpha_6 = 0;
float ll_d_6 = -0.02;
float ll_theta_6 = 0;

float LeftLegOffset[4][4] ={{1, 0, 0, 0},
                             {0, 1, 0, 0.05},
                             {0, 0, 1, -0.085},
                             {0, 0, 0, 1}};



//right leg
float rl_a_1 = 0;
float RL_ALPHA_1 = -10;
float rl_d_1 = -0.0;
float rl_theta_1 = 0;

float rl_a_2 = -0.0;
float RL_ALPHA_2 = -10;
float rl_d_2 = 0;
float rl_theta_2 = -90;

float rl_a_3 = 0;
float RL_ALPHA_3 = -10;
float rl_d_3 = -0.1;
float rl_theta_3 = 0;

float rl_a_4 = 0;
float RL_ALPHA_4 = 0;
float rl_d_4 = -0.12;
float rl_theta_4 = 0;

float rl_a_5 = 0;
float RL_ALPHA_5 = 10;
float rl_d_5 = -0.0;
float rl_theta_5 = 90;

float rl_a_6 = 0;
float rl_alpha_6 = 0;
float rl_d_6 = -0.02;
float rl_theta_6 = 0;

float RightLegOffset[4][4] ={{1, 0, 0, 0},
                             {0, 1, 0, -0.05},
                             {0, 0, 1, -0.085},
                             {0, 0, 0, 1}};






/*************************************************************************/
/** declare functions **/
/*************************************************************************/
void matrixMatrixMult(float mat1[4][4], float mat2[4][4], float out[4][4]);
void Rx(float theta, float out[4][4]);
void calculateDH(float a, float alpha, float d, float theta, float mat[4][4]);
void matrixVectorMult(float mat[4][4], float vec[4], float out[4]);
//DEBUG
void displayMatrix(float mat[4][4]);
void displayVector(float vec[4]);



struct timespec start, end;




/*************************************************************************/
/** main function **/
/*************************************************************************/
int main() {

    /** setup origins **/
    //head
    float NeckO[4][4];
    matrixMatrixMult(O, NeckOffset, NeckO);

    //right arm origin
    float tempRightArmOrigin[4][4];
    matrixMatrixMult(O, RightArmOffset, tempRightArmOrigin);
    float RightArmOrigin[4][4];
    float rotationmatrix[4][4];
    Rx(90, rotationmatrix);
    matrixMatrixMult(tempRightArmOrigin, rotationmatrix, RightArmOrigin);

    //left arm origin
    float tempLeftArmOrigin[4][4];
    matrixMatrixMult(O, LeftArmOffset, tempLeftArmOrigin);
    float LeftArmOrigin[4][4];
    matrixMatrixMult(tempLeftArmOrigin, rotationmatrix, LeftArmOrigin);

    
    //left leg
    float LeftLegOrigin[4][4];
    matrixMatrixMult(O, LeftLegOffset, LeftLegOrigin);

    //right leg
    float RightLegOrigin[4][4];
    matrixMatrixMult(O, RightLegOffset, RightLegOrigin);





    /** start timer **/
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);







    /** main loop **/
    int i;
    for(i=0; i<20; i++) {


        /** compute transformation matrices using DH parameters **/
        //head
        float  head_A1[4][4];
        float  head_A2[4][4];
        calculateDH(head_a_1, head_alpha_1, head_d_1, HEAD_THETA_1, head_A1);
        calculateDH(head_a_2, head_alpha_2, head_d_2, HEAD_THETA_2, head_A2);

        //right arm
        float  ra_A1[4][4];
        float  ra_A2[4][4];
        float  ra_A3[4][4];
        float  ra_A4[4][4];
        float  ra_A5[4][4];

        calculateDH(ra_a_1, ra_alpha_1, ra_d_1, RA_THETA_1, ra_A1);
        calculateDH(ra_a_2, ra_alpha_2, ra_d_2, RA_THETA_2, ra_A2);
        calculateDH(ra_a_3, ra_alpha_3, ra_d_3, RA_THETA_3, ra_A3);
        calculateDH(ra_a_4, RA_ALPHA_4, ra_d_4, ra_theta_4, ra_A4);
        calculateDH(ra_a_5, ra_alpha_5, ra_d_5, ra_theta_5, ra_A5);

        //left arm
        float  la_A1[4][4];
        float  la_A2[4][4];
        float  la_A3[4][4];
        float  la_A4[4][4];
        float  la_A5[4][4];

        calculateDH(la_a_1, la_alpha_1, la_d_1, LA_THETA_1, la_A1);
        calculateDH(la_a_2, la_alpha_2, la_d_2, LA_THETA_2, la_A2);
        calculateDH(la_a_3, la_alpha_3, la_d_3, LA_THETA_3, la_A3);
        calculateDH(la_a_4, LA_ALPHA_4, la_d_4, la_theta_4, la_A4);
        calculateDH(la_a_5, la_alpha_5, la_d_5, la_theta_5, la_A5);


        //left leg
        float  ll_A1[4][4];
        float  ll_A2[4][4];
        float  ll_A3[4][4];
        float  ll_A4[4][4];
        float  ll_A5[4][4];
        float  ll_A6[4][4];

        calculateDH(ll_a_1, LL_ALPHA_1, ll_d_1, ll_theta_1, ll_A1);
        calculateDH(ll_a_2, LL_ALPHA_2, ll_d_2, ll_theta_2, ll_A2);
        calculateDH(ll_a_3, LL_ALPHA_3, ll_d_3, ll_theta_3, ll_A3);
        calculateDH(ll_a_4, LL_ALPHA_4, ll_d_4, ll_theta_4, ll_A4);
        calculateDH(ll_a_5, LL_ALPHA_5, ll_d_5, ll_theta_5, ll_A5);
        calculateDH(ll_a_6, ll_alpha_6, ll_d_6, ll_theta_6, ll_A6);


        //right leg
        float  rl_A1[4][4];
        float  rl_A2[4][4];
        float  rl_A3[4][4];
        float  rl_A4[4][4];
        float  rl_A5[4][4];
        float  rl_A6[4][4];

        calculateDH(rl_a_1, RL_ALPHA_1, rl_d_1, rl_theta_1, rl_A1);
        calculateDH(rl_a_2, RL_ALPHA_2, rl_d_2, rl_theta_2, rl_A2);
        calculateDH(rl_a_3, RL_ALPHA_3, rl_d_3, rl_theta_3, rl_A3);
        calculateDH(rl_a_4, RL_ALPHA_4, rl_d_4, rl_theta_4, rl_A4);
        calculateDH(rl_a_5, RL_ALPHA_5, rl_d_5, rl_theta_5, rl_A5);
        calculateDH(rl_a_6, rl_alpha_6, rl_d_6, rl_theta_6, rl_A6);

        /*
        DEBUG
        displayMatrix(rl_A1);
        displayMatrix(rl_A2);
        displayMatrix(rl_A3);
        displayMatrix(rl_A4);
        displayMatrix(rl_A5);
        displayMatrix(rl_A6);
        */







        /** compute link positions **/
        float temp[4][4];

        //right arm
        float ra_0[4];
        matrixVectorMult(RightArmOrigin, pos_vector, ra_0);

        float ra_1[4];
        matrixMatrixMult(RightArmOrigin, ra_A1, temp);
        matrixVectorMult(temp, pos_vector, ra_1);

        float ra_2[4];
        matrixMatrixMult(temp, ra_A2, temp);
        matrixVectorMult(temp, pos_vector, ra_2);

        float ra_3[4];
        matrixMatrixMult(temp, ra_A3, temp);
        matrixVectorMult(temp, pos_vector, ra_3);

        float ra_4[4];
        matrixMatrixMult(temp, ra_A4, temp);
        matrixVectorMult(temp, pos_vector, ra_4);

        float ra_5[4];
        matrixMatrixMult(temp, ra_A5, temp);
        matrixVectorMult(temp, pos_vector, ra_5);




        //left arm
        float la_0[4];
        matrixVectorMult(LeftArmOrigin, pos_vector, la_0);

        float la_1[4];
        matrixMatrixMult(LeftArmOrigin, la_A1, temp);
        matrixVectorMult(temp, pos_vector, la_1);

        float la_2[4];
        matrixMatrixMult(temp, la_A2, temp);
        matrixVectorMult(temp, pos_vector, la_2);

        float la_3[4];
        matrixMatrixMult(temp, la_A3, temp);
        matrixVectorMult(temp, pos_vector, la_3);

        float la_4[4];
        matrixMatrixMult(temp, la_A4, temp);
        matrixVectorMult(temp, pos_vector, la_4);

        float la_5[4];
        matrixMatrixMult(temp, la_A5, temp);
        matrixVectorMult(temp, pos_vector, la_5);




        //left leg
        float ll_0[4];
        matrixVectorMult(LeftLegOrigin, pos_vector, ll_0);

        float ll_1[4];
        matrixMatrixMult(LeftLegOrigin, ll_A1, temp);
        matrixVectorMult(temp, pos_vector, ll_1);

        float ll_2[4];
        matrixMatrixMult(temp, ll_A2, temp);
        matrixVectorMult(temp, pos_vector, ll_2);

        float ll_3[4];
        matrixMatrixMult(temp, ll_A3, temp);
        matrixVectorMult(temp, pos_vector, ll_3);

        float ll_4[4];
        matrixMatrixMult(temp, ll_A4, temp);
        matrixVectorMult(temp, pos_vector, ll_4);

        float ll_5[4];
        matrixMatrixMult(temp, ll_A5, temp);
        matrixVectorMult(temp, pos_vector, ll_5);

        float ll_6[4];
        matrixMatrixMult(temp, ll_A6, temp);
        matrixVectorMult(temp, pos_vector, ll_6);





        //left leg
        float rl_0[4];
        matrixVectorMult(RightLegOrigin, pos_vector, rl_0);

        float rl_1[4];
        matrixMatrixMult(RightLegOrigin, rl_A1, temp);
        matrixVectorMult(temp, pos_vector, rl_1);

        float rl_2[4];
        matrixMatrixMult(temp, rl_A2, temp);
        matrixVectorMult(temp, pos_vector, rl_2);

        float rl_3[4];
        matrixMatrixMult(temp, rl_A3, temp);
        matrixVectorMult(temp, pos_vector, rl_3);

        float rl_4[4];
        matrixMatrixMult(temp, rl_A4, temp);
        matrixVectorMult(temp, pos_vector, rl_4);

        float rl_5[4];
        matrixMatrixMult(temp, rl_A5, temp);
        matrixVectorMult(temp, pos_vector, rl_5);

        float rl_6[4];
        matrixMatrixMult(temp, rl_A6, temp);
        matrixVectorMult(temp, pos_vector, rl_6);

        /*
        DEBUG

        displayVector(rl_0);
        displayVector(rl_1);
        displayVector(rl_2);
        displayVector(rl_3);
        displayVector(rl_4);
        displayVector(rl_5);
        displayVector(rl_6);
        */





        /** update joint positions **/
        //right arm
        RA_THETA_1 = RA_THETA_1 + 2;
        RA_ALPHA_4 = RA_ALPHA_4-2;

        //head
        HEAD_THETA_1 = HEAD_THETA_1  + 2;
        HEAD_THETA_2 = HEAD_THETA_2 - 1;

        //left arm
        LA_THETA_2 = LA_THETA_2+2;

        //left leg
        LL_ALPHA_2 = LL_ALPHA_2 +2;
        LL_ALPHA_3 = LL_ALPHA_3 - 2;
    }


    /** stop timer **/
    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    double microseconds = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
    double ms = microseconds/1000;
	printf ("Execution time in milliseconds: ");
	printf ("%f \n", ms);



    return 0;
}







/*************************************************************************/
/** Define Functions **/
/*************************************************************************/
void calculateDH(float a, float alpha, float d, float theta, float mat[4][4]) {
	double theta_rads = theta*(PI/180.0);
	double alpha_rads = alpha*(PI/180.0);

	mat[0][0] = cos(theta_rads);
	mat[0][1] = -sin(theta_rads)*cos(alpha_rads);
	mat[0][2] = sin(theta_rads)*sin(alpha_rads);
	mat[0][3] = a*cos(theta_rads);

	mat[1][0] = sin(theta_rads);
	mat[1][1] = cos(theta_rads)*cos(alpha_rads);
	mat[1][2] = -cos(theta_rads)*sin(alpha_rads);
	mat[1][3] = a*sin(theta_rads);

	mat[2][0] = 0;
	mat[2][1] = sin(alpha_rads);
	mat[2][2] = cos(alpha_rads);
	mat[2][3] = d;

	mat[3][0] = 0;
	mat[3][1] = 0;
	mat[3][2] = 0;
	mat[3][3] = 1;

	//TODO: make nearly zero elements exactly zero
}


void matrixVectorMult(float mat[4][4], float vec[4], float out[4]) {
	out[0] = mat[0][0]*vec[0] + mat[0][1]*vec[1] + mat[0][2]*vec[2] + mat[0][3]*vec[3];
	out[1] = mat[1][0]*vec[0] + mat[1][1]*vec[1] + mat[1][2]*vec[2] + mat[1][3]*vec[3];
	out[2] = mat[2][0]*vec[0] + mat[2][1]*vec[1] + mat[2][2]*vec[2] + mat[2][3]*vec[3];
	out[3] = mat[3][0]*vec[0] + mat[3][1]*vec[1] + mat[3][2]*vec[2] + mat[3][3]*vec[3];
}



void matrixMatrixMult(float mat1[4][4], float mat2[4][4], float out[4][4]) {
    float temp[4][4];
	temp[0][0] = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0] + mat1[0][2]*mat2[2][0] + mat1[0][3]*mat2[3][0];
	temp[0][1] = mat1[0][0]*mat2[0][1] + mat1[0][1]*mat2[1][1] + mat1[0][2]*mat2[2][1] + mat1[0][3]*mat2[3][1];
	temp[0][2] = mat1[0][0]*mat2[0][2] + mat1[0][1]*mat2[1][2] + mat1[0][2]*mat2[2][2] + mat1[0][3]*mat2[3][2];
	temp[0][3] = mat1[0][0]*mat2[0][3] + mat1[0][1]*mat2[1][3] + mat1[0][2]*mat2[2][3] + mat1[0][3]*mat2[3][3];

	temp[1][0] = mat1[1][0]*mat2[0][0] + mat1[1][1]*mat2[1][0] + mat1[1][2]*mat2[2][0] + mat1[1][3]*mat2[3][0];
	temp[1][1] = mat1[1][0]*mat2[0][1] + mat1[1][1]*mat2[1][1] + mat1[1][2]*mat2[2][1] + mat1[1][3]*mat2[3][1];
	temp[1][2] = mat1[1][0]*mat2[0][2] + mat1[1][1]*mat2[1][2] + mat1[1][2]*mat2[2][2] + mat1[1][3]*mat2[3][2];
	temp[1][3] = mat1[1][0]*mat2[0][3] + mat1[1][1]*mat2[1][3] + mat1[1][2]*mat2[2][3] + mat1[1][3]*mat2[3][3];

	temp[2][0] = mat1[2][0]*mat2[0][0] + mat1[2][1]*mat2[1][0] + mat1[2][2]*mat2[2][0] + mat1[2][3]*mat2[3][0];
	temp[2][1] = mat1[2][0]*mat2[0][1] + mat1[2][1]*mat2[1][1] + mat1[2][2]*mat2[2][1] + mat1[2][3]*mat2[3][1];
	temp[2][2] = mat1[2][0]*mat2[0][2] + mat1[2][1]*mat2[1][2] + mat1[2][2]*mat2[2][2] + mat1[2][3]*mat2[3][2];
	temp[2][3] = mat1[2][0]*mat2[0][3] + mat1[2][1]*mat2[1][3] + mat1[2][2]*mat2[2][3] + mat1[2][3]*mat2[3][3];

	temp[3][0] = mat1[3][0]*mat2[0][0] + mat1[3][1]*mat2[1][0] + mat1[3][2]*mat2[2][0] + mat1[3][3]*mat2[3][0];
	temp[3][1] = mat1[3][0]*mat2[0][1] + mat1[3][1]*mat2[1][1] + mat1[3][2]*mat2[2][1] + mat1[3][3]*mat2[3][1];
	temp[3][2] = mat1[3][0]*mat2[0][2] + mat1[3][1]*mat2[1][2] + mat1[3][2]*mat2[2][2] + mat1[3][3]*mat2[3][2];
	temp[3][3] = mat1[3][0]*mat2[0][3] + mat1[3][1]*mat2[1][3] + mat1[3][2]*mat2[2][3] + mat1[3][3]*mat2[3][3];

    memcpy(out, temp, 4*4*sizeof(float));
}

void Rx(float theta, float out[4][4]) {
	double theta_rads = theta*(PI/180.0);
	out[0][0] = 1;
	out[0][1] = 0;
	out[0][2] = 0;
	out[0][3] = 0;

	out[1][0] = 0;
	out[1][1] = cos(theta_rads);
	out[1][2] = -sin(theta_rads);
	out[1][3] = 0;

	out[2][0] = 0;
	out[2][1] = sin(theta_rads);
	out[2][2] = cos(theta_rads);
	out[2][3] = 0;

	out[3][0] = 0;
	out[3][1] = 0;
	out[3][2] = 0;
	out[3][3] = 1;

 }




void displayMatrix(float mat[4][4]) {
	int j,k;
	for(j=0;j<4;j++) {
		for(k=0;k<4;k++) {
			printf("%.4f",mat[j][k]);
			printf("\t");
		}
		printf("\n");
	}
    printf("\n");
}


void displayVector(float vec[4]) {
	int j;
	for(j=0;j<4;j++) {
		printf("%.4f",vec[j]);
		printf("\t");
	}
	printf("\n");
}
