#ifndef OWN_LIB_H
#define OWN_LIB_H

#include "main.h"

#ifndef PI 
#define PI 3.14159265f
#endif




float deg2rad(float deg);
float rad2deg(float rad);

int16_t abs_int16(int16_t value);
int32_t abs_int32(int32_t value);
float abs_float(float value);

#endif /* OWN_LIB_H */
