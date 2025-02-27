//
// Created by RUPC on 2022/9/20.
//
#ifndef SMART_CAR_CAMERA_REV_PERSPECTIVE_H
#define SMART_CAR_CAMERA_REV_PERSPECTIVE_H
#include "cstdint"
#define RESULT_H 60
#define RESULT_W 40
extern uint8_t gray_pers_image[RESULT_H][RESULT_W];
extern uint8_t contrast_pers_image[RESULT_H][RESULT_W];
extern uint8_t binary_pers_image[RESULT_H][RESULT_W];
extern uint8_t gray_binary_pers_image[RESULT_H][RESULT_W];

void InitLookupTable(void);

void ImagePerspective(void);

#endif //SMART_CAR_CAMERA_REV_PERSPECTIVE_H
