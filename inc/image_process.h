//
// Created by ashkore on 2023/9/23.
//

#ifndef SMART_CAR_CAMERA_IMAGE_PROCESS_H
#define SMART_CAR_CAMERA_IMAGE_PROCESS_H
#include "cstdint"
#include "opencv2/opencv.hpp"
typedef struct{
    int left_x;
    int left_height;
    int right_x;
    int right_height;
    int start_y;
    int end_y;
} Max_White_Column;

//typedef struct{
//    int top_y;
//    int top_width;
//    int bottom_y;
//    int bottom_width;
//    int start_x;
//    int end_x;
//} Max_White_Row;
#define MT9V03X_WS               (188)
#define MT9V03X_HS               (120)
#define MT9V03X_W               (188)
#define MT9V03X_H               (90)
#define MT9V03X_IMAGE_SIZE      (MT9V03X_WS * MT9V03X_HS)
extern Max_White_Column max_white_column;
extern Max_White_Column max_white_column_pers;
//extern Max_White_Row max_white_row;
extern int max_middle_line_height;
extern int bottom_start_x;
extern int bottom_end_x;
extern int bottom_start_x_pers;
extern int bottom_end_x_pers;
extern int middle_line_index;
extern int middle_line_index_pers;

extern uint8_t left_border[100][2];  // ��߽�
extern uint8_t right_border[100][2];  // �ұ߽�
extern uint8_t middle_line[100][2];  // ����
extern uint8_t middle_line_single[100][2];  // ���ߵ�Y
extern uint8_t left_border_pers[100][2];  // ��߽�
extern uint8_t right_border_pers[100][2];  // �ұ߽�
extern uint8_t middle_line_pers[100][2];  // ����
extern uint8_t middle_line_single_pers[100][2];  // ���ߵ�Y
extern uint8_t left_distance[100][2];
extern uint8_t right_distance[100][2];
extern uint8_t left_distance_pers[100][2];
extern uint8_t right_distance_pers[100][2];
extern uint8_t left_distance_line[100][2];
extern uint8_t right_distance_line[100][2];
extern uint8_t left_distance_line_pers[100][2];
extern uint8_t right_distance_line_pers[100][2];
extern uint8_t distances[100];
extern uint8_t distances_pers[100];
extern uint8_t distance_middle_line[100][2];
extern uint8_t distance_middle_line_pers[100][2];
extern uint8_t narrow_line[100][2];
extern uint8_t    mt9v03x_image[MT9V03X_HS][MT9V03X_WS];
extern uint8_t    gray_image[60][80];
extern uint8_t    binary_image[60][80];
extern uint8_t    binary_image_bak[60][80];
extern uint8_t    contrast_image[60][80];
extern uint8_t    gray_binary_image[60][80];
void bottom_start_end_x_get();

void bottom_start_end_x_get_pers();

void max_white_column_get(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

void max_white_column_get_pers(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

//void max_white_row_get(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

void get_distance_line();

void get_distance_line_pers();

void get_narrow_line();

void get_lost_count();

int check_crossroad();

int check_roundabout();

int check_ramp();

int check_garage_and_obstacle();

void erase_top_left_road(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void erase_top_right_road(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void fix_left_break(uint16_t y1, uint16_t y2);

void fix_right_break(uint16_t y1, uint16_t y2);

void compress_image(uint8_t *Dst, const uint8_t *Src);

uint8_t get_otsu_threshold(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint8_t *image);

void draw_rectan();

void cover_car_head();

int get_border_line(int detect_count_max);

int get_border_line_pers(int detect_count_max);

void get_max_middle_line_height();

void calculate_contrast(uint8_t *dst_image, const uint8_t *src_image, int16_t image_w, int16_t image_h);

void calculate_contrast_x8(uint8_t *dst_image, const uint8_t *src_image, int16_t image_w, int16_t image_h);

void tft180_draw_border_line(cv::Mat& dst_image,uint16_t x, uint16_t y, const uint8_t line[][2], uint16_t color);

float distance_to_line(float Ax, float Ay, float Bx, float By, float Cx, float Cy);

float get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy);

uint8_t is_left_corner(float left_Ax, float left_Ay, float left_Bx, float left_By, float left_Cx, float left_Cy,
                     float right_Ax, float right_Ay, float right_Bx, float right_By, float right_Cx, float right_Cy);

uint8_t is_right_corner(float right_Ax, float right_Ay, float right_Bx, float right_By, float right_Cx, float right_Cy,
                      float left_Ax, float left_Ay, float left_Bx, float left_By, float left_Cx, float left_Cy);

#endif // SMART_CAR_CAMERA_IMAGE_PROCESS_H
