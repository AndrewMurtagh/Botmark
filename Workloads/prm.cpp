/*****************************************************************
******************************************************************
*
*   Copyright (C) 2017 Trinity Robotics Research Group.
*
*
*   FILENAME:       prm.cpp
*
*
*   DESCRIPTION:    program to plan a path using probabilistic
*                   roadmap method.
*
*
*   DATE:           11/03/2017
*
*
*   AUTHOR:         Andrew Murtagh,
*                   Trinity Robotics Research Group, Trinity College Dublin.
*
*
*   NOTES:          -K is number of free points.
*                   -map representation is 500x500 b & w image.
*                   -graph is searched using A*.
*                   -openCv is a required dependency.
*		    -metric is millisecond.
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


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctime>
#include <string.h>
#include <string>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <time.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <typeinfo>


#include "botmark.h"

#define K 150
#define DRAWCOLOR 190
#define PATHCOLOR 70
#define GRIDSIZE 500


using namespace std;



/*************************************************************************/
/** define parameters **/
/*************************************************************************/
struct graphNode {
	int index;
	float historicCost;
	float heuristicCost;
	float totalCost;
	int parentIndex;
};
struct timespec starttime, endtime;
float grid[GRIDSIZE][GRIDSIZE];


/*************************************************************************/
/** declare functions **/
/*************************************************************************/
bool collisionFreePath(int basex, int basey, int pointx, int pointy, float (&grid)[GRIDSIZE][GRIDSIZE]);
void generateRandomPoint(int (&arr)[2], float (&grid)[GRIDSIZE][GRIDSIZE]);
bool collisionFreePoint(int (&arr)[2], float (&grid)[GRIDSIZE][GRIDSIZE]);
float calcHeuristicCost(int point[2], int goal[2]);
float calcHistoricCost(int point1[2], int point2[2]);
int findLowestIndex(vector<graphNode> openList);
bool isInClose(vector<graphNode> closedList, int index);
int findIndexOf(vector<graphNode> nodeList, graphNode node);

//void showVertices(int v[K+2][2], Mat map);
string toString(int i);
void displayList(vector<graphNode> lista);






/*************************************************************************/
/** main function **/
/*************************************************************************/
int main(){



    /*********************************************************************/
    /** initiliase parameters **/
    /*********************************************************************/
    srand(time(NULL));
    ifstream infile("../Data/officemap.txt");

    int lineinc = 0;
    for(string line; getline(infile, line);) {

        std::stringstream stream(line);


        for(int i=0; i<GRIDSIZE; i++) {
           // stream >> ranges[i];
           stream >> grid[lineinc][i];
        }
        lineinc++;
    }
    infile.close();




    /** start timer **/
    if(!DEBUG) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &starttime);
    }




    /*********************************************************************/
    /** add start and end configurations **/
    /*********************************************************************/
	int start[2] = {140, 45};
	int goal[2] = {460, 460};

	if(collisionFreePoint(start, grid) && collisionFreePoint(goal, grid)) {
		if(DEBUG) {
            cout << "Start & End Points Are Good" << endl << endl;
        }

	} else {
		if(DEBUG) {
            cout << "Start & End Points Are Bad" << endl << endl;
		}
        return -1;
	}


	int vertices[K+2][2] = {};
	vertices[0][0] = start[0];
	vertices[0][1] = start[1];
	vertices[1][0] = goal[0];
	vertices[1][1] = goal[1];






    /*********************************************************************/
    /** add K random points **/
    /*********************************************************************/
    for(int i=0; i<K; i++) {
		int newpoint[2];
		generateRandomPoint(newpoint, grid);
		vertices[i+2][0] = newpoint[0];
		vertices[i+2][1] = newpoint[1];
	}
	if(DEBUG) {
		cout << "Random Points Added" << endl << endl;
	}


    /*********************************************************************/
    /** add edges **/
    /*********************************************************************/
    vector< vector<int> > edges(K+2);
    for(int i=0; i<K+2; i++) {
		for(int j=i+1; j<K+2; j++) {
			if(collisionFreePath(vertices[i][0], vertices[i][1], vertices[j][0], vertices[j][1], grid)) {
				edges[i].push_back(j);
				edges[j].push_back(i);
			}
		}
	}

	if(DEBUG) {
		for(int i = 0; i < K+2; i++) {
			cout << i << ": ";
			for(int j=0; j<edges[i].size(); j++) {
				cout << edges[i][j] << ", ";
			}
			cout << endl;
		}
		cout << "Edges Added" << endl << endl;
	}







    /*********************************************************************/
    /** A* graph search **/
    /*********************************************************************/
	vector<graphNode> openList;
	vector<graphNode> closedList;


	graphNode startNode;
	startNode.index = 0;
	startNode.historicCost = 0;
	startNode.heuristicCost = calcHeuristicCost(vertices[0], goal);
	startNode.totalCost = startNode.historicCost + startNode.heuristicCost;
	startNode.parentIndex = -1;
	openList.push_back(startNode);



	bool pathFound = false;
    graphNode lowestCostNode;

	while(openList.size() > 0) {

        int lowestIndex = findLowestIndex(openList);

		lowestCostNode.index = openList.at(lowestIndex).index;
		lowestCostNode.historicCost = openList.at(lowestIndex).historicCost;
		lowestCostNode.heuristicCost = openList.at(lowestIndex).heuristicCost;
		lowestCostNode.totalCost = openList.at(lowestIndex).totalCost;
		lowestCostNode.parentIndex = openList.at(lowestIndex).parentIndex;
        openList.erase(openList.begin() + lowestIndex);

        //if lowestCostNode is goal return
        if(lowestCostNode.index == 1) {
            pathFound = true;
            break;
        }

        /** cycle through neighbours **/
        for(int neighbourIterator=0; neighbourIterator < edges[lowestCostNode.index].size(); neighbourIterator++) {
            int currNeighbourVertexIndex = edges[lowestCostNode.index][neighbourIterator];
            bool inClosed = false;
            for(int i=0; i<closedList.size(); i++) {
                if(closedList.at(i).index == lowestCostNode.index) {
                    inClosed = true;
                }
            }
            if (closedList.size() ==0 || !inClosed) {
                float neighbourHistoricCost = lowestCostNode.historicCost + calcHistoricCost(vertices[currNeighbourVertexIndex], vertices[lowestCostNode.index]);
                float neghibourHeuristicCost = calcHeuristicCost(vertices[currNeighbourVertexIndex], goal);
                float neighbourTotalCost = neighbourHistoricCost + neghibourHeuristicCost;


                bool addToOpen = true;
                int alreadyInOpenListIndex = -1;
                for(int i=0; i<openList.size(); i++) {
                    if(openList.at(i).index == currNeighbourVertexIndex) {
                        alreadyInOpenListIndex = i;
                    }
                }
                if(alreadyInOpenListIndex != -1) { //is in openlist
                    if(openList.at(alreadyInOpenListIndex).totalCost < neighbourTotalCost) {
                       addToOpen = false;
                    } else {
                        openList.erase(openList.begin() + alreadyInOpenListIndex);
                       addToOpen = true;
                    }
                }
                if(addToOpen) {
                    graphNode neighbourNode;
                    neighbourNode.index = currNeighbourVertexIndex;
                    neighbourNode.historicCost = neighbourHistoricCost;
                    neighbourNode.heuristicCost = neghibourHeuristicCost;
                    neighbourNode.totalCost = neighbourTotalCost;
                    neighbourNode.parentIndex = closedList.size();
                    openList.push_back(neighbourNode);

                }


            }

        }
        closedList.push_back(lowestCostNode);

	}



    if (DEBUG) { cout << "pathFound: " << pathFound << endl; }



    if(pathFound) {

        vector< vector<int> > path;
        vector<int> this_vertex;
        this_vertex.push_back(vertices[lowestCostNode.index][0]);
        this_vertex.push_back(vertices[lowestCostNode.index][1]);
        path.push_back(this_vertex);
        int parentIndexInClosed = lowestCostNode.parentIndex;
        int parentIndexInVertices = closedList.at(parentIndexInClosed).index;

        while (parentIndexInClosed != -1) {
            int parentIndexInVertices = closedList.at(parentIndexInClosed).index;
            vector<int> this_vertex;
            this_vertex.push_back(vertices[parentIndexInVertices][0]);
            this_vertex.push_back(vertices[parentIndexInVertices][1]);
            path.push_back(this_vertex);
            parentIndexInClosed = closedList.at(parentIndexInClosed).parentIndex;

        }


    }

    /** stop timer **/
    if(!DEBUG) {
        if(pathFound) {
            clock_gettime(CLOCK_MONOTONIC_RAW, &endtime);
            double microseconds = (endtime.tv_sec - starttime.tv_sec) * 1000000 + (endtime.tv_nsec - starttime.tv_nsec) / 1000;
            double ms = microseconds/1000;
            printf ("Execution time in milliseconds: ");
            printf ("%f \n", ms);
        } else {
            printf ("Path not found, time not recorded.");
        }
    }



    return 0;
}







/*************************************************************************/
/** define functions **/
/*************************************************************************/
int findIndexOf(vector<graphNode> nodeList, graphNode node) {
    int index = -1;
    for(int i=0; i<nodeList.size(); i++) {
        if(nodeList.at(i).index == node.index) {
            index = i;
        }
    }
    return index;
}


bool isInClose(vector<graphNode> closedList, int thisindex) {
    bool inClosed = false;
    for(int i=0; i<closedList.size(); i++) {
        if(closedList.at(i).index == thisindex) {
            inClosed = true;
        }
    }
    return inClosed;
}

int findLowestIndex(vector<graphNode> openList) {
    int lowestIndex = 0;
    for(int i=0; i<openList.size(); i++) {
        if(openList.at(i).totalCost < openList.at(lowestIndex).totalCost) {
            lowestIndex = i;
        }
    }
    return lowestIndex;
}

float calcHeuristicCost(int point[2], int goal[2]) {
	return sqrt(pow(point[0]-goal[0],2) + pow(point[1]-goal[1],2));
}

float calcHistoricCost(int point1[2], int point2[2]) {
	return sqrt(pow(point1[0]-point2[0],2) + pow(point1[1]-point2[1],2));
}

bool collisionFreePath(int basex, int basey, int pointx, int pointy, float (&grid)[GRIDSIZE][GRIDSIZE]) {
	bool feasible = true;
	double dist = sqrt(pow(pointx-basex,2) + pow(pointy-basey,2));
	double angle = atan2((pointy-basey), (pointx-basex));
	for(double l=0; l<dist; l=l+2) { //TODO: decrease size to 0.5
		int temppoint[2];
		temppoint[0] = basex + l*cos(angle);
		temppoint[1] = basey + l*sin(angle);
		if(!collisionFreePoint(temppoint, grid)) {
			feasible = false;
			break;
		} else {
		}
	}
	int temppoint[2];
	temppoint[0] = pointx;
	temppoint[1] = pointy;
	if(!collisionFreePoint(temppoint, grid)) {
		feasible = false;
	}



	return feasible;
}


bool collisionFreePoint(int (&point)[2], float (&grid)[GRIDSIZE][GRIDSIZE]) {
	if (point[1] >=1 && point[1] <= GRIDSIZE && point[0] >= 1 && point[0] <= GRIDSIZE && grid[point[0]][point[1]] == 1) {
		return true;
	} else {
		return false;
	}
}




void generateRandomPoint(int (&arr)[2], float (&grid)[GRIDSIZE][GRIDSIZE]) {
	int temp[2];
	temp[0] = 1 + (std::rand() % (GRIDSIZE - 1 + 1 ));
	temp[1] = 1 + (std::rand() % (GRIDSIZE - 1 + 1 ));

	if(collisionFreePoint(temp, grid)) {
		memcpy(arr,temp,sizeof(arr));
		return;
	} else {
		return generateRandomPoint(arr, grid);
	}

}
