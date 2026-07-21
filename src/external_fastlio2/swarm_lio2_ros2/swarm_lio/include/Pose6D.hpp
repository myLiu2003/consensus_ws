#ifndef POSE6D_HPP
#define POSE6D_HPP

struct Pose6D {
    double offset_time = 0.0;
    double acc[3] = {0, 0, 0};
    double gyr[3] = {0, 0, 0};
    double vel[3] = {0, 0, 0};
    double pos[3] = {0, 0, 0};
    double rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

#endif