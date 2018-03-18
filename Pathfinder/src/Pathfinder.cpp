//============================================================================
// Name        : Pathfinder.cpp
// Author      : RocketRedNeck
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "pathfinder.h"
#include <stdio.h>

#define DIM(x) (sizeof(x)/sizeof(x[0]))

int main() {

    // ******************************************************
    // SWITCH
    // ******************************************************

    Waypoint centerStartRightSwitchPath[] =
 {
         {0,                  0, d2r(0)},
         {0.8382,       -0.6096, d2r(-40)},
         {2.6138,       -1.0668, d2r(0)}
 };

     Waypoint centerStartLeftSwitchPath[] =
 {
         {0,              0,   d2r(0)},
         {0.8382,      0.9096, d2r(40)},    // Includes slight y-bias
         {2.6138,      1.3668, d2r(0)}
 };

     Waypoint rightStartRightSwitchPath[] =
 {
         {0,                     0,     d2r(0)},
         {0.864,     -0.699-0.3048,     d2r(-45)},
         {1.817+0.5, -1.245-0.3048,     d2r(0)},
         {2.922+0.3, -1.245+0.3-0.3048, d2r(45)},
         {3.405,          0-0.3048,     d2r(100)}

 };

 // basically a left/right (y) mirror of the above
 // Rather than loading programmatically, we place the value explicitly
 // just in case we need to make slight adjustments
     Waypoint leftStartLeftSwitchPath[] =
 {
         {0,         0,                d2r(0)},
         {0.864,     0.699-0.3048,     d2r(45)},
         {1.817+0.5, 1.245-0.3048,     d2r(0)},
         {2.922+0.3, 1.245-0.3-0.3048, d2r(-45)},
         {3.405,         0,            d2r(-100)}
 };

     Waypoint rightStartLeftSwitchPath[] =
 {
         {0,              0, d2r(0)},
         {0.864,     -0.699-0.3048, d2r(-45)},
         {1.817+0.5, -1.245-0.3048, d2r(0)},

         {4.318,         -1.245-0.3048,     d2r(0)},
         {5.182,          0.254-0.3048,     d2r(90)},
         {5.182,          3.937-0.3048,     d2r(90)},
         {4.318+0.3048,   5.436-3*0.3048,   d2r(180)},
         {3.600+0.3048,   5.136-3*0.3048,   d2r(225)},
         {3.405+0.3048,   4.191-3*0.3048,   d2r(310)}

 };

 // Again, a mirror but nor programmatically just in case

     Waypoint leftStartRightSwitchPath[] =
 {
         {0,              0,                d2r(0)},
         {0.864,         0.699,          d2r(45)},      // NOTE: Inlcude y-bias
         {1.817+0.5,     1.245,          d2r(0)},

         {4.318,          1.245, 	      d2r(0)},
         {5.182,         -0.254,    		   d2r(-90)},
         {5.182,         -3.937,           d2r(-90)},
         {4.318+0.3048,  -5.436+2*0.3048,  d2r(-180)},
         {3.600+0.3048,  -5.136+2*0.3048,  d2r(-225)},
         {3.405+0.3048,  -4.191+2*0.3048,  d2r(-310)}

 };

     // ******************************************************
     // SCALE
     // ******************************************************
     Waypoint rightStartRightScalePath[] =
 {
         {0,                     0,     d2r(0)},
         {0.864,            -1.004,     d2r(-45)},
         {2.317,            -1.550,     d2r(0)},
         {5.508,            -1.550,     d2r(0)},
         {6.682,            -0.376,     d2r(45)}

 };

     Waypoint leftStartLeftScalePath[] =
 {
         {0,                     0,     d2r(0)},
         {0.864,             1.004,     d2r(45)},
         {2.317,             1.550,     d2r(0)},
         {5.508,             1.550,     d2r(0)},
         {6.682,             0.376,     d2r(-45)}

 };

     Waypoint rightStartLeftScalePath[] =
 {
         {0,                     0,      d2r(0)},
         {0.864,            -1.004,      d2r(-45)},
         {2.317,            -1.550,      d2r(0)},

         {4.318,            -1.550,      d2r(0)},
         {5.182,            -0.051,      d2r(90)},
         {5.182,             3.632,      d2r(90)},
         {4.623,             4.522,      d2r(90)},

         {5.062,              5.088,     d2r(0)},
         {5.812,              5.088,     d2r(0)},
         {6.790,              4.110,     d2r(-45)}

 };

     Waypoint leftStartRightScalePath[] =
 {
         {0,                     0,      d2r(0)},
         {0.864,             1.004,      d2r(45)},
         {2.317,             1.550,      d2r(0)},

         {4.318,             1.550,      d2r(0)},
         {5.182,             0.051,      d2r(-90)},
         {5.182,            -3.632,      d2r(-90)},
         {4.623,            -4.522,      d2r(-90)},

         {5.062,            -5.088,     d2r(0)},
         {5.812,            -5.088,     d2r(0)},
         {6.790,            -4.110,     d2r(45)}

 };

    TrajectoryCandidate candidate;

#define CREATE_INTERFACE_AND_FILE(aname) \
    Waypoint *points = aname; \
    int numPoints = DIM(aname); \
    FILE *file = fopen(#aname ".py","w+"); \
    const char name[] = #aname

    CREATE_INTERFACE_AND_FILE(rightStartLeftScalePath);

    // Write out python code to file
    fprintf(file,"import matplotlib.pyplot as pl\n\n");     // Import packages

#define PRINT_XY(trajName, trajObject, trajLength)\
        fprintf(file,#trajName "x = [\n");                        \
        for (int i = 0; i < trajLength-1; ++i)                    \
        {                                                         \
            fprintf(file,"%f,\n",trajObject[i].x);                \
        }                                                         \
        fprintf(file,"%f\n]\n",trajObject[trajLength-1].x);       \
        fprintf(file, #trajName "y = [\n");                       \
        for (int i = 0; i < trajLength-1; ++i)                    \
        {                                                         \
            fprintf(file,"%f,\n",trajObject[i].y);                \
        }                                                         \
        fprintf(file,"%f\n]\n",trajObject[trajLength-1].y)

    // Print x and y values of waypoints
    PRINT_XY(w,points,numPoints);

    pathfinder_prepare(points,
                       numPoints,
                       FIT_HERMITE_CUBIC,
                       PATHFINDER_SAMPLES_HIGH,
                       0.050,
                       6*0.3048,
                       2.0,
                       60.0,
                       &candidate);

    int length = candidate.length;
    Segment *trajectory = reinterpret_cast<Segment *>(malloc(length * sizeof(Segment)));

    pathfinder_generate(&candidate, trajectory);

    Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
    Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

    double wheelbase_width = 24.25 * 0.0254;

    pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

    // Do something with the trajectories...
    // Loop through each to produce center, left, and right trajectories

    PRINT_XY(c,trajectory,length);
    PRINT_XY(l,leftTrajectory,length);
    PRINT_XY(r,rightTrajectory,length);

    // Now write out plotting code
    fprintf(file,"pl.axis('equal')\n");
    fprintf(file,"pl.plot(wx,wy,'rx',cx,cy,'b-',lx,ly,'g.',rx,ry,'g.')\n");
    fprintf(file,"pl.text(1.5,-0.5,'Num Path = ' + str(len(wx)))\n");
    fprintf(file,"pl.text(1.5, 0.0,'Num Traj = ' + str(len(cx)))\n");
    fprintf(file,"pl.text(1.5, 0.5,'Time (s) = ' + str(len(cx)*0.050))\n");
    fprintf(file,"pl.savefig('%s.png')\n",name);

    char cmd[1024];
    sprintf(cmd,"python ./%s.py",name);
    printf(cmd);
    printf("\n");
    system(cmd);

    system("pwd");

    free(trajectory);
    free(leftTrajectory);
    free(rightTrajectory);

    printf("DONE.\n");
    return 0;
}

