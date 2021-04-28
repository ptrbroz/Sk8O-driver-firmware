#ifndef MATH_OPS_H
#define MATH_OPS_H

#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#include "math.h"

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
float roundf(float x);
void limit_norm(float *x, float *y, float limit);
void limit(float *x, float min, float max);

inline float my_fmod(float a, float b) {
    return (a - b * floor(a / b));
}

template<unsigned int bits>
int float_to_uint(float x, const float x_max, const bool symmetric=true)
{
    float span = x_max + symmetric*x_max;
    float quantum = span/((1<<bits)-1);

    float offset = 0;
    if (symmetric) {
        offset = -x_max - quantum/2;
    }
    
    return (int) ((x-offset)/quantum + 0.5);
}

template<unsigned int bits>
float uint_to_float(int x, const float x_max, const bool symmetric=true)
{
    float span = x_max + symmetric*x_max;
    float quantum = span/((1<<bits)-1);
    
    float offset = 0;
    if (symmetric) {
        offset = -x_max - quantum/2;
    }
    
    return ((float)x)*quantum + offset;
}

template<unsigned int bits>
int float_to_uint_symmetric(float x, const float x_lim)
{    
    return float_to_uint<bits>(x, x_lim, true);
}

template<unsigned int bits>
float uint_to_float_symmetric(int x, const float x_lim)
{   
    return uint_to_float<bits>(x, x_lim, true);
}

template<unsigned int bits>
int float_to_uint_positive(float x, const float x_lim)
{    
    return float_to_uint<bits>(x, x_lim, false);
}

template<unsigned int bits>
float uint_to_float_positive(int x, const float x_lim)
{   
    return uint_to_float<bits>(x, x_lim, false);
}

#endif
