#ifndef __CALIBRATE_H
#define __CALIBRATE_H
#include <stdint.h>
#define GRAY_FRONT 0
#define GRAY_REAR  1
typedef struct { uint16_t front_edge; uint16_t rear_edge; uint16_t ground[2]; } GrayBaseline_t;
#endif
