//============================================================================
// Name        : Pathfinder.cpp
// Author      : RocketRedNeck
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "pathfinder.h"

#define DIM(x) (sizeof(x)/sizeof(x[0]))

int main() {

    Waypoint points[] =
    {
            {0,      0,      d2r(0)},

            {1.5240, 0,      d2r(35)},
            {2.1082, 0.5080, d2r(70)},
            {2.6924, 2.2098, d2r(90)},
            {2.1082, 5.1054, d2r(115)},
            {2.7686, 6.5024, d2r(0)}
    };

    TrajectoryCandidate candidate;
    pathfinder_prepare(points,
                       DIM(points),
                       FIT_HERMITE_CUBIC,
                       PATHFINDER_SAMPLES_HIGH,
                       0.050,
                       5*0.3048,
                       2.0,
                       60.0,
                       &candidate);

    int length = candidate.length;
    Segment *trajectory = reinterpret_cast<Segment *>(malloc(length * sizeof(Segment)));

    pathfinder_generate(&candidate, trajectory);

    Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
    Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

    double wheelbase_width = 24.0 * 0.0254;

    pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

    // Do something with the trajectories...
    int i;
    for (i = 0; i < length; i++) {
        Segment s = trajectory[i];
        Segment sl = leftTrajectory[i];
        Segment sr = rightTrajectory[i];
        if (0 == (i % 10))
        {
            printf("  i) dt    t    (x    , y    ) =  p    (l    , r    )  v    (l    , r    )  a    (l    , r    )    j    (l      , r      )  h    (l    , r    )\n");
        }
        printf("%3i) %4.2f %5.2f (%5.2f, %5.2f) = %5.2f (%5.2f, %5.2f) %5.2f (%5.2f, %5.2f) %5.2f (%5.2f, %5.2f) %7.2f (%7.2f, %7.2f) %5.2f (%5.2f, %5.2f)\n",
               i,
               s.dt,
               s.dt*i,
               s.x,
               s.y,
               s.position,
               sl.position,
               sr.position,
               s.velocity,
               sl.velocity,
               sr.velocity,
               s.acceleration,
               sl.acceleration,
               sr.acceleration,
               s.jerk,
               sl.jerk,
               sr.jerk,
               s.heading,
               sl.heading,
               sr.heading);
    }
    free(trajectory);
    return 0;
}

