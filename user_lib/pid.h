#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct
{
    float Target;
    float Measure;
    float LastMeasure;
    float Error;
    float LastError;
    float LastLastError;

    float Kp;
    float Ki;
    float Kd;

    float P_out;
    float I_out;
    float D_out;
    float Output;
} PID_Struct;

float PID_Calculate(PID_Struct *PID, float Measure, float Target, float max_output);
void PID_Set(PID_Struct *PID, float Kp, float Ki, float Kd);
void PID_Reset(PID_Struct *PID);

#endif /* PID_H */
