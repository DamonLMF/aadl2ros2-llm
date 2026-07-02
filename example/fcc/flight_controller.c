#include <stdio.h>
#include <math.h>


float val(double Feedback_roll, double Feedback_pitch, double Expected_az) 
{ 
    // return a float value
    double max_u; // declare variable max_u and u1
    float kt = 0.00000584;
    // calculate max_u and u1
    max_u = kt * 1897 * 1897 * cos(Feedback_roll) * cos(Feedback_pitch);
    float u1 = 1.5 * (9.81 - Expected_az) / max_u;
    return u1; // return u1
}

float motions(double roll, double pitch, double yaw, double p_in, double q_in, double r_in, double vx, double vy, double vz, double f1, double f2, double f3, double f4)
{
    // return a float value
    double sum_f, ax1, ay1, az1, ax, ay, az, p, q, r; // declare variable sum_f
    float kt = 0.00000584,kd = 0.06, m = 1.5, g = 9.81, s=0.22, ix = 0.029125, iy = 0.029125, iz = 0.055225; // declare constants kt, m, g, iy, iz, s, ix, kd
    // calculate sum_f
    sum_f = f1 + f2 + f3 + f4;
    // calculate ax1, ay1, az1
    ax1 = sin(roll) * sin(yaw) + sin(pitch) * cos(roll) * cos(yaw);
    ay1 = sin(pitch) * sin(yaw) * cos(roll) - sin(roll) * cos(yaw);
    az1 = cos(pitch) * cos(roll);
    // calculate ax, ay, az
    ax = (ax1 * sum_f - kd * vx) / m;
    ay = (ay1 * sum_f - kd * vy) / m;
    az = (az1 * sum_f - kd * vz - m * g) / m;
    // calculate p, q, r
    p = (q_in * r_in * (iy - iz) + s * (f4 - f2)) / ix;
    q = (p_in * r_in * (iz - ix) + s * (f3 - f1)) / iy;
    r = q_in * (ix - iy)/iz + kt * sum_f / kd * iz;
    // return ax, ay, az, p, q, r
    return ax, ay, az, p, q, r;
}