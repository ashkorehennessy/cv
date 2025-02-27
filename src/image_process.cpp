//
// Created by ashkore on 2023/9/23.
//

#include "image_process.h"
#include <cstring>
#include "road.h"
#include "flag.h"
#include "rev_perspective.h"
#include "counter.h"
#include <ctgmath>
#include "config.h"
#include "opencv2/opencv.hpp"

uint8_t    mt9v03x_image[MT9V03X_HS][MT9V03X_WS];
uint8_t    gray_image[60][80];
uint8_t    binary_image[60][80];
uint8_t    binary_image_bak[60][80];
uint8_t    contrast_image[60][80];
uint8_t    gray_binary_image[60][80];
float left_corner_angle;  // ��սǽǶȣ�����extern�������ļ�
float right_corner_angle;  // �ҹսǽǶȣ�����extern�������ļ�
uint8_t left_border[100][2];  // ��߽�
uint8_t right_border[100][2];  // �ұ߽�
int left_dirs[100];  // �����߷���
int right_dirs[100];  // �����߷���
uint8_t left_border_pers[100][2];  // ��߽�
uint8_t right_border_pers[100][2];  // �ұ߽�
int left_dirs_pers[100];  // �����߷���
int right_dirs_pers[100];  // �����߷���
uint8_t middle_line[100][2];  // ����
uint8_t middle_line_single[100][2];  // ���ߵ�Y
uint8_t middle_line_pers[100][2];  // ����
uint8_t middle_line_single_pers[100][2];  // ���ߵ�Y
uint8_t distance_middle_line[100][2];  // ���������
uint8_t distance_middle_line_pers[100][2];  // ͸�����������
uint8_t left_distance[100][2];  // �����
uint8_t right_distance[100][2];  // �Ҿ���
uint8_t left_distance_line[100][2];  // �������
uint8_t right_distance_line[100][2];  // �Ҿ�����
uint8_t left_distance_pers[100][2];  // ͸�������
uint8_t right_distance_pers[100][2];  // ͸���Ҿ���
uint8_t left_distance_line_pers[100][2];  // ͸���������
uint8_t right_distance_line_pers[100][2];  // ͸���Ҿ�����
uint8_t distances[100];  // ����
uint8_t distances_pers[100];  // ͸�Ӿ���
uint8_t narrow_line[100][2];  // խ��(�ϰ���)

int max_middle_line_height = 0;
int left_border_index = 0;
int left_border_index_pers = 0;
int right_border_index = 0;
int right_border_index_pers = 0;
int left_skip_index = 0;
int right_skip_index = 0;
int middle_line_index = 0;
int middle_line_index_pers = 0;
int middle_line_single_index = 0;
int middle_line_single_index_pers = 0;
int left_skip_index_pers = 0;
int right_skip_index_pers = 0;
int distance_index = 0;
int distance_index_pers = 0;
int distance_middle_line_index = 0;
int distance_middle_line_index_pers = 0;
int narrow_line_index = 0;

int bottom_start_x;
int bottom_end_x;
int bottom_start_x_pers;
int bottom_end_x_pers;

int t1,t2;

uint8_t road_color = 0;  // ��·��ɫ
uint8_t border_color = 1;  // �߽���ɫ

Max_White_Column max_white_column;  // ����ɫ��
Max_White_Column max_white_column_pers;
//Max_White_Row max_white_row;  // ����ɫ��

int lost_x1;
int lost_x2;
int lost_y1;
int lost_y2;
int left_lost_count;
int right_lost_count;
int left_lost_dir;
int right_lost_dir;

extern int cornering;
extern int image_diff;
extern int force_roundabout;

void bottom_start_end_x_get(){
    int maxStartIndex = -1;
    int maxEndIndex = -1;
    int currentStartIndex = -1;
    int currentEndIndex = -1;
    int maxLength = 0;
    int currentLength = 0;
    for (int i = 0; i < 80; i++) {
        if (gray_binary_image[43][i] == 1) {
            if (currentStartIndex == -1) {
                currentStartIndex = i;
            }
            currentEndIndex = i;
            currentLength = currentEndIndex - currentStartIndex;
            if (currentLength > maxLength) {
                maxLength = currentLength;
                maxStartIndex = currentStartIndex;
                maxEndIndex = currentEndIndex;
            }
        } else {
            currentStartIndex = -1;
        }
    }
    bottom_start_x = maxStartIndex;
    bottom_end_x = maxEndIndex;
}

void bottom_start_end_x_get_pers(){
    int maxStartIndex = -1;
    int maxEndIndex = -1;
    int currentStartIndex = -1;
    int currentEndIndex = -1;
    int maxLength = 0;
    int currentLength = 0;
    for (int i = 0; i < 47; i++) {
        if (gray_binary_pers_image[43][i] == 1) {
            if (currentStartIndex == -1) {
                currentStartIndex = i;
            }
            currentEndIndex = i;
            currentLength = currentEndIndex - currentStartIndex;
            if (currentLength > maxLength) {
                maxLength = currentLength;
                maxStartIndex = currentStartIndex;
                maxEndIndex = currentEndIndex;
            }
        } else {
            currentStartIndex = -1;
        }
    }
    bottom_start_x_pers = maxStartIndex;
    bottom_end_x_pers = maxEndIndex;
}

void max_white_column_get(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
    int16_t i;
    int16_t j;
    int16_t left_max_white_count = 0;
    int16_t right_max_white_count = 0;
    x1 += 2;
    x2 -= 2;
    if(flag.found_garage == false) {
        for (i = x1; i < x2; i++) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (binary_image_bak[j][i] == 0) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > left_max_white_count) {
                left_max_white_count = white_count;
                max_white_column.start_y = y2;
                max_white_column.end_y = y2 - white_count + 1;
                max_white_column.left_x = i;
                max_white_column.left_height = left_max_white_count;
            }
        }
        for (i = x2; i > x1; i--) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (binary_image_bak[j][i] == 0) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > right_max_white_count) {
                right_max_white_count = white_count;
                max_white_column.right_x = i;
                max_white_column.right_height = right_max_white_count;
            }
        }
    } else {
        for (i = x1; i < x2; i++) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (gray_binary_image[j][i] == 1) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > left_max_white_count) {
                left_max_white_count = white_count;
                max_white_column.start_y = y2;
                max_white_column.end_y = y2 - white_count + 1;
                max_white_column.left_x = i;
                max_white_column.left_height = left_max_white_count;
            }
        }
        for (i = x2; i > x1; i--) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (gray_binary_image[j][i] == 1) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > right_max_white_count) {
                right_max_white_count = white_count;
                max_white_column.right_x = i;
                max_white_column.right_height = right_max_white_count;
            }
        }
    }
}

void max_white_column_get_pers(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
    int16_t i;
    int16_t j;
    int16_t left_max_white_count = 0;
    int16_t right_max_white_count = 0;
    x1 += 2;
    x2 -= 2;
    for (i = x1; i < x2; i++) {
        int16_t white_count = 0;
        for (j = y2; j >= y1; j--) {
            if (binary_pers_image[j][i] == 0) {
                white_count++;
            } else {
                break;
            }
        }
        if (white_count > left_max_white_count) {
            left_max_white_count = white_count;
            max_white_column_pers.start_y = y2;
            max_white_column_pers.end_y = y2 - white_count + 1;
            max_white_column_pers.left_x = i;
            max_white_column_pers.left_height = left_max_white_count;
        }
    }
    for (i = x2; i > x1; i--) {
        int16_t white_count = 0;
        for (j = y2; j >= y1; j--) {
            if (binary_pers_image[j][i] == 0) {
                white_count++;
            } else {
                break;
            }
        }
        if (white_count > right_max_white_count) {
            right_max_white_count = white_count;
            max_white_column_pers.right_x = i;
            max_white_column_pers.right_height = right_max_white_count;
        }
    }
}

//void max_white_row_get(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
//    int16_t i;
//    int16_t j;
//    int16_t top_max_white_count = 0;
//    int16_t bottom_max_white_count = 0;
//    for(i = y1; i < y2; i++){
//        int16_t white_count = 0;
//        int16_t start_x = x1;
//        while(gray_binary_image[i][start_x] == 0) start_x++;
//        for(j = start_x; j < x2; j++){
//            if(gray_binary_image[i][j] == 1){
//                white_count++;
//            } else {
//                break;
//            }
//        }
//        if(white_count > top_max_white_count){
//            top_max_white_count = white_count;
//            max_white_row.start_x = start_x;
//            max_white_row.end_x = j;
//            max_white_row.top_y = i;
//            max_white_row.top_width = top_max_white_count;
//        }
//    }
//    for(i = y2; i > y1; i--){
//        int16_t white_count = 0;
//        j = x1;
//        while(gray_binary_image[i][j] == 0) j++;
//        for(; j < x2; j++){
//            if(gray_binary_image[i][j] == 1){
//                white_count++;
//            } else {
//                break;
//            }
//        }
//        if(white_count > bottom_max_white_count){
//            bottom_max_white_count = white_count;
//            max_white_row.bottom_y = i;
//            max_white_row.bottom_width = bottom_max_white_count;
//        }
//    }
//}

int check_crossroad(){
    flag.found_crossroad = false;
    if(left_lost_count > 2 && right_lost_count > 2 && left_lost_count + right_lost_count > 12 && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0){
        flag.found_crossroad = true;
        counter.drive_in_left_roundabout = 0;
        counter.drive_in_right_roundabout = 0;
        return true;
    } else {
        return false;
    }
}

int check_roundabout(){
    flag.found_left_roundabout = false;
    flag.found_right_roundabout = false;
    int target_left_lost_count;
    int target_right_lost_count;
    if(force_roundabout == 0){
        target_left_lost_count = 3;
        target_right_lost_count = 3;
    } else {
        target_left_lost_count = 3;
        target_right_lost_count = 3;
    }
    if(left_lost_count > target_left_lost_count && right_lost_count == 0 && (right_lost_dir == 0 || force_roundabout == 1)){
        flag.found_left_roundabout = true;
    }
    if(right_lost_count > target_right_lost_count && left_lost_count == 0 && (left_lost_dir == 0 || force_roundabout == 1)){
        flag.found_right_roundabout = true;
    }
    return flag.found_left_roundabout || flag.found_right_roundabout;
}

int check_ramp(){
    flag.found_ramp = false;
    if((max_white_column.left_x > 30 && max_white_column.right_x < 64 && max_white_column.left_height == 44) &&
           distances[40] > 11 && abs(distance_middle_line[0][0] - distance_middle_line[40][0]) < 8){
        flag.found_ramp = true;
    }
    return flag.found_ramp;
}

int check_garage_and_obstacle(){
    int count = 0;
    int flag_next = 0;
    int distance = 0;
    int start_y;
    int end_y;
    flag.found_garage = false;
    flag.found_obstacle = false;
    if(narrow_line[0][1] > 42 || narrow_line[0][1] == 0){
        start_y = 43;
        end_y = 28;
    } else {
        start_y = narrow_line[0][1];
        end_y = narrow_line[narrow_line_index - 1][1];
    }
    int _bottom_start_x = bottom_start_x;
    int _bottom_end_x = bottom_end_x;
    if(_bottom_end_x - _bottom_start_x < 35){
        _bottom_start_x = 17;
        _bottom_end_x = 77;
    }
    for(int i = start_y; i > end_y; i--) {
        for (int j = _bottom_start_x; j < _bottom_end_x; j++) {
            if (gray_binary_image[i][j] == 0) {
                if (flag_next == 0 && distance > 0 && distance < 5) {
                    count++;
                    flag_next = 1;
                }
                distance = 0;
            } else {
                flag_next = 0;
                distance++;
            }
        }
    }
    if(count > 25){
        flag.found_garage = true;
    }
    if(narrow_line_index > 5){
        flag.found_obstacle = true;
        float Ax;
        float Ay;
        float Bx;
        float By;
        float Cx;
        float Cy;
        uint8_t Aindex = 60 - narrow_line[0][1] - 4;
        uint8_t Bindex = 60 - narrow_line[narrow_line_index/2][1] - 2;
        uint8_t Cindex = 60 - narrow_line[narrow_line_index - 1][1] + 4;
        Bx = left_distance_line_pers[Bindex][0];
        By = left_distance_line_pers[Bindex][1];
        Ax = left_distance_line_pers[Aindex][0];
        Ay = left_distance_line_pers[Aindex][1];
        Cx = left_distance_line_pers[Cindex][0];
        Cy = left_distance_line_pers[Cindex][1];
//        narrow_line[narrow_line_index][0] = Ax;
//        narrow_line[narrow_line_index++][1] = Ay;
//        narrow_line[narrow_line_index][0] = Bx;
//        narrow_line[narrow_line_index++][1] = By;
//        narrow_line[narrow_line_index][0] = Cx;
//        narrow_line[narrow_line_index++][1] = Cy;
        float left_dis = distance_to_line(Ax, Ay, Bx, By, Cx, Cy);
        float left_ang = get_angle(Ax, Ay, Bx, By, Cx, Cy);
        Bx = right_distance_line_pers[Bindex][0];
        By = right_distance_line_pers[Bindex][1];
        Ax = right_distance_line_pers[Aindex][0];
        Ay = right_distance_line_pers[Aindex][1];
        Cx = right_distance_line_pers[Cindex][0];
        Cy = right_distance_line_pers[Cindex][1];
//        narrow_line[narrow_line_index][0] = Ax;
//        narrow_line[narrow_line_index++][1] = Ay;
//        narrow_line[narrow_line_index][0] = Bx;
//        narrow_line[narrow_line_index++][1] = By;
//        narrow_line[narrow_line_index][0] = Cx;
//        narrow_line[narrow_line_index++][1] = Cy;
        float right_dis = distance_to_line(Ax, Ay, Bx, By, Cx, Cy);
        float right_ang = get_angle(Ax, Ay, Bx, By, Cx, Cy);
        if(left_dis > right_dis){
            flag.advance_avoid_obstacle_dir = -1;
        } else {
            flag.advance_avoid_obstacle_dir = 1;
        }
    } else {
        flag.advance_avoid_obstacle_dir = 0;
    }
    t1 = count;
    return flag.found_garage || flag.found_obstacle;
}

// Ĩ�����Ϸ���·
// ��ͼ�����Ϸ��ĵ�·Ĩ���������ص���Ϊ��ɫ��������Ĩ�������ص����
void erase_top_left_road(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
    float slope = (float)(y2 - y1) / (x2 - x1);
    float b = y1 - slope * x1;
    for(int i = y1; i < y2; i++){
        int x_end = (i - b) / slope;
        binary_image[i][x_end - 2] = 1;
        binary_image[i][x_end - 1] = 1;
        binary_image[i][x_end] = 1;
    }
}

// Ĩ�����Ϸ���·
// ��ͼ�����Ϸ��ĵ�·Ĩ���������ص���Ϊ��ɫ��������Ĩ�������ص����
void erase_top_right_road(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
    float slope = (float)(y2 - y1) / (x2 - x1);
    float b = y1 - slope * x1;
    for(int i = y1; i < y2; i++){
        int x_start = (i - b) / slope;
        binary_image[i][x_start] = 1;
        binary_image[i][x_start + 1] = 1;
        binary_image[i][x_start + 2] = 1;
    }
}

void fix_left_break(uint16_t y1, uint16_t y2){
    for(int i = y1; i < y2; i++){
        int x_end = right_distance_line[i][0] - road_distances[i];
        binary_image[60 - i][x_end - 2] = 1;
        binary_image[60 - i][x_end - 1] = 1;
        binary_image[60 - i][x_end] = 1;
    }
}

void fix_right_break(uint16_t y1, uint16_t y2){
    for(int i = y1; i < y2; i++){
        int x_start = left_distance_line[i][0] + road_distances[i];
        binary_image[60 - i][x_start] = 1;
        binary_image[60 - i][x_start + 1] = 1;
        binary_image[60 - i][x_start + 2] = 1;
    }
}

//2x2ѹ��ͼ��
void compress_image(uint8_t *Dst, const uint8_t *Src){
    uint16_t i;
    uint16_t j;
    uint16_t compress_width = MT9V03X_W / 2;
    uint16_t compress_height = MT9V03X_H / 2;
    for(i = 0; i < compress_height; i++){
        for(j = 0; j < compress_width; j++){
            Dst[i * compress_width + j] = (Src[i*2*MT9V03X_W + j*2] + Src[i*2*MT9V03X_W + j*2+1] + Src[(i*2+1)*MT9V03X_W + j*2] + Src[(i*2+1)*MT9V03X_W + j*2+1]) / 4;
        }
    }
}

void get_lost_count() {
    left_lost_count = 0;
    right_lost_count = 0;
    left_lost_dir = 0;
    right_lost_dir = 0;
    int target_distance = 5;
    int target_lost_y1;
    if(force_roundabout == 0){
        target_lost_y1 = 13;
    } else {
        target_lost_y1 = 13;
    }
    if (lost_x1 == 0 || lost_y1 == 0 || lost_x2 == 0 || lost_y2 == 0 || max_white_column.left_height < 39 || lost_y1 < target_lost_y1) {
        return;
    }
    if(lost_x2 - lost_x1 == 0){
        for(int y = lost_y1; y > lost_y2; y--){
            int left_dist = 0;
            int right_dist = 0;
            for(int x = lost_x1; x >= 0; x--){
                if(binary_image_bak[y][x] == 0){
                    left_dist++;
                    if(left_dist > road_distances[60 - y] / 2 + target_distance){
                        left_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
            for(int x = lost_x1; x < 80; x++){
                if(binary_image_bak[y][x] == 0){
                    right_dist++;
                    if(right_dist > road_distances[60 - y] / 2 + target_distance){
                        right_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
        }
    } else {
        float slope = (float)(lost_y2 - lost_y1) / (float)(lost_x2 - lost_x1);
        float b = lost_y1 - slope * lost_x1;
        for (int y = lost_y1; y > lost_y2; y--) {
            int left_dist = 0;
            int right_dist = 0;
            for (int x = (y - b) / slope; x >= 0; x--) {
                if (binary_image_bak[y][x] == 0) {
                    left_dist++;
                    if(left_dist > road_distances[60 - y] / 2 + target_distance){
                        left_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
            for (int x = (y - b) / slope; x < 80; x++) {
                if (binary_image_bak[y][x] == 0) {
                    right_dist++;
                    if(right_dist > road_distances[60 - y] / 2 + target_distance){
                        right_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
        }
    }
    for(int dir_index = left_skip_index - 10; dir_index < left_skip_index; dir_index++){
        int left_dir = left_dirs[dir_index];
        if(left_dir == 4 || left_dir == 7){
            left_lost_dir++;
        }
    }
    for(int dir_index = right_skip_index - 10; dir_index < right_skip_index; dir_index++){
        int right_dir = right_dirs[dir_index];
        if(right_dir == 5 || right_dir == 6){
            right_lost_dir++;
        }
    }
}

void get_narrow_line(){
    memset(narrow_line, 0, sizeof(narrow_line));
    narrow_line_index = 0;
    if(max_white_column_pers.left_height < 14) return;
    int max_y = max_white_column_pers.left_height - 5;
    if(max_y > 30) max_y = 30;
    for(int q = 0; q < max_y; q++){
        if(distances_pers[q] < 8){
            int _x = distance_middle_line_pers[q][0];
            int _y = distance_middle_line_pers[q][1];
            if(narrow_line_index > 0 && abs(_y - narrow_line[narrow_line_index - 1][1]) > 3) break;
            narrow_line[narrow_line_index][0] = _x;
            narrow_line[narrow_line_index][1] = _y;
            narrow_line_index++;
        }
    }
}

void get_distance_line(){
    memset(left_distance_line, 0, sizeof(left_distance_line));
    memset(right_distance_line, 0, sizeof(right_distance_line));
    memset(left_distance, 0, sizeof(left_distance));
    memset(right_distance, 0, sizeof(right_distance));
    memset(distances, 0, sizeof(distances));
    memset(distance_middle_line, 0, sizeof(distance_middle_line));
    int left_x = max_white_column.left_x;
    int right_x = max_white_column.right_x;
    distance_index = 0;
    distance_middle_line_index = 0;
    lost_x1 = 0;
    lost_x2 = 0;
    lost_y1 = 0;
    lost_y2 = 0;
    for(int y = max_white_column.start_y - 1; y >= max_white_column.end_y; y--){
        for(int x = left_x; x >= 0; x--) {
            if (binary_image_bak[y][x] == 1 || x == 1) {
                left_distance_line[distance_index][0] = x;
                left_distance_line[distance_index][1] = y;
                left_distance[distance_index][0] = left_x - x;
                left_distance[distance_index][1] = y;
                break;
            }
        }
        for(int x = right_x; x < 80; x++) {
            if (binary_image_bak[y][x] == 1 || x == 92) {
                right_distance_line[distance_index][0] = x;
                right_distance_line[distance_index][1] = y;
                right_distance[distance_index][0] = x - right_x;
                right_distance[distance_index][1] = y;
                break;
            }
        }
        distances[distance_index] = (right_distance_line[distance_index][0] - left_distance_line[distance_index][0]);
        if(distances[distance_index] < road_distances[distance_index] + 7){
            distance_middle_line[distance_middle_line_index][0] = (right_distance_line[distance_index][0] + left_distance_line[distance_index][0]) / 2;
            distance_middle_line[distance_middle_line_index][1] = y;
            distance_middle_line_index++;
            if(lost_x1 != 0 && lost_y1 != 0 && lost_x2 == 0 && lost_y2 == 0){
                lost_x2 = (right_distance_line[distance_index][0] + left_distance_line[distance_index][0]) / 2;
                lost_y2 = y;
            }
        } else {
            if(lost_x1 == 0 && lost_y1 == 0 && lost_x2 == 0 && lost_y2 == 0){
                lost_x1 = (right_distance_line[distance_index-1][0] + left_distance_line[distance_index-1][0]) / 2;
                lost_y1 = y;
            }
        }
        distance_index++;
    }
}

void get_distance_line_pers(){
    memset(left_distance_line_pers, 0, sizeof(left_distance_line_pers));
    memset(right_distance_line_pers, 0, sizeof(right_distance_line_pers));
    memset(left_distance_pers, 0, sizeof(left_distance_pers));
    memset(right_distance_pers, 0, sizeof(right_distance_pers));
    memset(distances_pers, 0, sizeof(distances_pers));
    memset(distance_middle_line_pers, 0, sizeof(distance_middle_line_pers));
    int left_x = max_white_column_pers.left_x;
    int right_x = max_white_column_pers.right_x;
    distance_index_pers = 0;
    distance_middle_line_index_pers = 0;
    for(int y = max_white_column_pers.start_y - 1; y >= max_white_column_pers.end_y; y--){
        for(int x = left_x; x >= 0; x--) {
            if (binary_pers_image[y][x] == 1 || x == 1) {
                left_distance_line_pers[distance_index_pers][0] = x;
                left_distance_line_pers[distance_index_pers][1] = y;
                left_distance_pers[distance_index_pers][0] = left_x - x;
                left_distance_pers[distance_index_pers][1] = y;
                break;
            }
        }
        for(int x = right_x; x < 47; x++) {
            if (binary_pers_image[y][x] == 1 || x == 60) {
                right_distance_line_pers[distance_index_pers][0] = x;
                right_distance_line_pers[distance_index_pers][1] = y;
                right_distance_pers[distance_index_pers][0] = x - right_x;
                right_distance_pers[distance_index_pers][1] = y;
                break;
            }
        }
        distances_pers[distance_index_pers] = (right_distance_line_pers[distance_index_pers][0] - left_distance_line_pers[distance_index_pers][0]);
        distance_middle_line_pers[distance_middle_line_index_pers][0] = (right_distance_line_pers[distance_index_pers][0] + left_distance_line_pers[distance_index_pers][0]) / 2;
        distance_middle_line_pers[distance_middle_line_index_pers][1] = y;
        distance_middle_line_index_pers++;
        distance_index_pers++;
    }
}

uint8_t get_otsu_threshold(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint8_t *image) {
#define GrayScale 256
    int Pixel_Max = 0;
    int Pixel_Min = 255;
    uint16_t width = x2 - x1;
    uint16_t height = y2 - y1;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    uint8_t threshold = 120;
    const uint8_t* data = image;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32_t gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = y1; i < y2; i += 2)
    {
        for (j = x1; j < x2; j += 2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
            if(data[i * width + j]>Pixel_Max)   Pixel_Max=data[i * width + j];
            if(data[i * width + j]<Pixel_Min)   Pixel_Min=data[i * width + j];
        }
    }

    //����ÿ������ֵ�ĵ�������ͼ���еı���

    for (i = Pixel_Min; i < Pixel_Max; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }

    //�����Ҷȼ�[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = Pixel_Min; j < Pixel_Max; j++)
    {

        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
        u0tmp += (float)j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

        w1 = 1 - w0;
        u1tmp= gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;              //����ƽ���Ҷ�
        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
//        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8_t)j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }

    return threshold;
}

// ����
void draw_rectan(){
    uint8_t i;
    switch (flag.image_preprocess_method) {
        case 0:
        case 1:
            road_color = 1;
        border_color = 0;
        for (i = 0; i < 40; i++){
            binary_image[i][0] = border_color;
            binary_image[i][1] = border_color;
            binary_image[i][80 - 1] = border_color;
            binary_image[i][80 - 2] = border_color;
        }
        for (i = 0; i < 80; i++){
            binary_image[0][i] = border_color;
            binary_image[1][i] = border_color;
        }
        break;
        case 2:
            road_color = 0;
            border_color = 1;
            for (i = 0; i < 60; i++){
                binary_image[i][0] = border_color;
                binary_image[i][80 - 1] = border_color;
            }
            for (i = 0; i < 80; i++){
                binary_image[0][i] = border_color;
            }
            break;
    }
}

void cover_car_head(){
    for(int x = 29; x < 65; x++){
        for(int y = 39; y < 60; y++){
            gray_image[y][x] = gray_image[y-1][x];
        }
    }
}

int get_border_line(int detect_count_max) {
    int left_x, left_y, right_x, right_y;  // ����ʱ����
    left_border_index = 0;  // ����ʱ��߽�����
    right_border_index = 0;  // ����ʱ�ұ߽�����
    left_skip_index = 0;  // ����ʱ��߽���������
    right_skip_index = 0;  // ����ʱ�ұ߽���������
    int left_dir;  // ����ʱ��߽緽��
    int right_dir;  // ����ʱ�ұ߽緽��
    int detect_count;  // ����ʱ������


    road_color = 0;
    border_color = 1;


    memset(left_border, 0, sizeof(left_border));
    memset(right_border, 0, sizeof(right_border));
    memset(left_dirs, 0, sizeof(left_dirs));
    memset(right_dirs, 0, sizeof(right_dirs));
    // ��ʼ�߽���ڵ�·��
    int first_x;
    int first_y = 43;
    for (first_x = (bottom_start_x + bottom_end_x) / 2; first_x > 0; first_x--) {
        if (binary_image[first_y][first_x] == road_color && binary_image[first_y][first_x - 1] != road_color) {
            left_border[left_border_index][0] = first_x;
            left_border[left_border_index][1] = first_y;
            break;
        }
    }

    for (first_x = (bottom_start_x + bottom_end_x)/2; first_x < 80; first_x++) {
        if (binary_image[first_y][first_x] == road_color && binary_image[first_y][first_x + 1] != road_color) {
            right_border[right_border_index][0] = first_x;
            right_border[right_border_index][1] = first_y;
            break;
        }
    }
    left_x = left_border[left_border_index][0];
    left_y = left_border[left_border_index][1];
    right_x = right_border[right_border_index][0];
    right_y = right_border[right_border_index][1];
    left_dir = -1;  // ������߽緽��
    right_dir = -1;  // �����ұ߽緽��
    for(detect_count = 0; detect_count < detect_count_max; detect_count++) {
        // ��߽�
        if(detect_count > 15 && sqrt((left_x - left_border[left_border_index-5][0]) * (left_x - left_border[left_border_index-5][0]) + (left_y - left_border[left_border_index-5][1]) * (left_y - left_border[left_border_index-5][1])) < 4){
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else if(left_y < 3 || (detect_count > 15 && left_x < 4)) {
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else if (left_dir != 3 && binary_image[left_y - 1][left_x + 1] == road_color &&
                   binary_image[left_y - 1][left_x] == border_color){  // ���Ϻڣ�����3�����ϰ�
            left_y = left_y - 1;
            left_x = left_x + 1;
            left_dir = 6;
        } else if (left_dir != 4 && binary_image[left_y][left_x + 1] == road_color &&
                   binary_image[left_y - 1][left_x + 1] == border_color){  // ���Һڣ�����4�����ϰ�
            left_x = left_x + 1;
            left_dir = 5;
        } else if (left_dir != 1 && binary_image[left_y - 1][left_x] == road_color &&
                   binary_image[left_y - 1][left_x - 1] == border_color){  // ���Ϻڣ�����1�����ϰ�
            left_y = left_y - 1;
            left_dir = 0;
        } else if (left_dir != 2 && binary_image[left_y - 1][left_x - 1] == road_color &&
                   binary_image[left_y][left_x - 1] == border_color){  // ���Ϻڣ�����2�����Ұ�
            left_y = left_y - 1;
            left_x = left_x - 1;
            left_dir = 7;
        } else if (left_dir !=5 && binary_image[left_y][left_x - 1] == road_color &&
                   binary_image[left_y + 1][left_x - 1] == border_color){  // ����ڣ�����5�����°�
            left_x = left_x - 1;
            left_dir = 4;
        } else if (left_dir != 6 && binary_image[left_y + 1][left_x - 1] == road_color &&
                   binary_image[left_y + 1][left_x] == border_color){  // ���ºڣ�����6�����°�
            left_y = left_y + 1;
            left_x = left_x - 1;
            left_dir = 3;
        } else if (left_dir != 7 && binary_image[left_y + 1][left_x + 1] == road_color &&
                   binary_image[left_y][left_x + 1] == border_color){  // ���ºڣ�����7�����Ұ�
            left_y = left_y + 1;
            left_x = left_x + 1;
            left_dir = 2;
        } else if (left_dir != 0 && binary_image[left_y + 1][left_x] == road_color &&
                   binary_image[left_y + 1][left_x + 1] == border_color){  // ���ºڣ�����8�����°�
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else {
            break;
        }

        left_border_index++;
        left_border[left_border_index][0] = left_x;
        left_border[left_border_index][1] = left_y;
        left_dirs[left_border_index] = left_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }

        // �ұ߽�
        if(detect_count > 15 && sqrt((right_x - right_border[right_border_index-5][0]) * (right_x - right_border[right_border_index-5][0]) + (right_y - right_border[right_border_index-5][1]) * (right_y - right_border[right_border_index-5][1])) < 4){
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else if(right_y < 3 || (detect_count > 15 && right_x > 90)) {
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else if(right_dir != 2 && binary_image[right_y - 1][right_x - 1] == road_color &&
                  binary_image[right_y - 1][right_x] == border_color) {  // ���Ϻڣ�����2�����ϰ�
            right_y = right_y - 1;
            right_x = right_x - 1;
            right_dir = 7;
        } else if(right_dir != 5 && binary_image[right_y][right_x - 1] == road_color &&
                  binary_image[right_y - 1][right_x - 1] == border_color){  // ����ڣ�����5�����ϰ�
            right_x = right_x - 1;
            right_dir = 4;
        } else if(left_dir != 1 && binary_image[right_y - 1][right_x] == road_color &&
                  binary_image[right_y - 1][right_x + 1] == border_color){  // ���Ϻڣ�����1�����ϰ�
            right_y = right_y - 1;
            right_dir = 0;
        } else if (right_x < 80 && right_dir != 3 && binary_image[right_y - 1][right_x + 1] == road_color &&
                   binary_image[right_y][right_x + 1] == border_color){  // ���Ϻڣ�����3�����Ұ�
            right_y = right_y - 1;
            right_x = right_x + 1;
            right_dir = 6;
        } else if(right_dir != 4 && binary_image[right_y][right_x + 1] == road_color &&
                  binary_image[right_y + 1][right_x + 1] == border_color){  // ���Һڣ�����4�����°�
            right_x = right_x + 1;
            right_dir = 5;
        } else if(right_dir != 6 && binary_image[right_y + 1][right_x - 1] == road_color &&
                  binary_image[right_y][right_x - 1] == border_color){  // ���ºڣ�����6�������
            right_y = right_y + 1;
            right_x = right_x - 1;
            right_dir = 3;
        } else if(right_dir != 7 && binary_image[right_y + 1][right_x + 1] == road_color &&
                  binary_image[right_y + 1][right_x] == border_color) {  // ���ºڣ�����7�����°�
            right_y = right_y + 1;
            right_x = right_x + 1;
            right_dir = 2;
        } else if(right_dir != 0 && binary_image[right_y + 1][right_x] == road_color &&
                  binary_image[right_y + 1][right_x - 1] == border_color){  // ���ºڣ�����8�����°�
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else {
            break;
        }
        right_border_index++;
        right_border[right_border_index][0] = right_x;
        right_border[right_border_index][1] = right_y;
        right_dirs[right_border_index] = right_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }
    }
    detect_count_max = detect_count;

    // ��������
    memset(middle_line, 0, sizeof(middle_line));
    middle_line_index = 0;
    for(int q = 0; q < detect_count_max; q++){
        middle_line[q][0] = (left_border[q][0] + right_border[q][0]) / 2;
        middle_line[q][1] = (left_border[q][1] + right_border[q][1]) / 2;
        middle_line_index++;
    }

    memset(middle_line_single, 0, sizeof(middle_line_single));
    middle_line_single_index = 0;
    int middle_line_current_y = -1;
    for(int q = 0; q < detect_count_max; q++){
        if(middle_line[q][1] != middle_line_current_y && middle_line[q][1] != 0){
            middle_line_single[middle_line_single_index][0] = middle_line[q][0];
            middle_line_single[middle_line_single_index][1] = middle_line[q][1];
            middle_line_single_index++;
            middle_line_current_y = middle_line[q][1];
        }
    }

    return detect_count_max;
}

int get_border_line_pers(int detect_count_max) {
    int left_x, left_y, right_x, right_y;  // ����ʱ����
    left_border_index_pers = 0;  // ����ʱ��߽�����
    right_border_index_pers = 0;  // ����ʱ�ұ߽�����
    left_skip_index_pers = 0;  // ����ʱ��߽���������
    right_skip_index_pers = 0;  // ����ʱ�ұ߽���������
    int left_dir;  // ����ʱ��߽緽��
    int right_dir;  // ����ʱ�ұ߽緽��
    int detect_count;  // ����ʱ������


    road_color = 0;
    border_color = 1;


    memset(left_border_pers, 0, sizeof(left_border_pers));
    memset(right_border_pers, 0, sizeof(right_border_pers));
    memset(left_dirs_pers, 0, sizeof(left_dirs_pers));
    memset(right_dirs_pers, 0, sizeof(right_dirs_pers));
    // ��ʼ�߽���ڵ�·��
    int first_x;
    int first_y = 43;
    for (first_x = (bottom_start_x_pers + bottom_end_x_pers) / 2; first_x > 0; first_x--) {
        if (binary_pers_image[first_y][first_x] == road_color && binary_pers_image[first_y][first_x - 1] != road_color) {
            left_border_pers[left_border_index_pers][0] = first_x;
            left_border_pers[left_border_index_pers][1] = first_y;
            break;
        }
    }

    for (first_x = (bottom_start_x_pers + bottom_end_x_pers)/2; first_x < 47; first_x++) {
        if (binary_pers_image[first_y][first_x] == road_color && binary_pers_image[first_y][first_x + 1] != road_color) {
            right_border_pers[right_border_index_pers][0] = first_x;
            right_border_pers[right_border_index_pers][1] = first_y;
            break;
        }
    }
    left_x = left_border_pers[left_border_index_pers][0];
    left_y = left_border_pers[left_border_index_pers][1];
    right_x = right_border_pers[right_border_index_pers][0];
    right_y = right_border_pers[right_border_index_pers][1];
    left_dir = -1;  // ������߽緽��
    right_dir = -1;  // �����ұ߽緽��
    for(detect_count = 0; detect_count < detect_count_max; detect_count++) {
        // ��߽�
        if(detect_count > 15 && sqrt((left_x - left_border_pers[left_border_index_pers-5][0]) * (left_x - left_border_pers[left_border_index_pers-5][0]) + (left_y - left_border_pers[left_border_index_pers-5][1]) * (left_y - left_border_pers[left_border_index_pers-5][1])) < 4){
            if(left_skip_index == 0){
                left_skip_index = left_border_index_pers;
            }
        } else if(left_y < 3 || left_x < 3 || left_x > 44) {
            if(left_skip_index == 0){
                left_skip_index = left_border_index_pers;
            }
        } else if (left_dir != 3 && binary_pers_image[left_y - 1][left_x + 1] == road_color &&
                   binary_pers_image[left_y - 1][left_x] == border_color){  // ���Ϻڣ�����3�����ϰ�
            left_y = left_y - 1;
            left_x = left_x + 1;
            left_dir = 6;
        } else if (left_dir != 4 && binary_pers_image[left_y][left_x + 1] == road_color &&
                   binary_pers_image[left_y - 1][left_x + 1] == border_color){  // ���Һڣ�����4�����ϰ�
            left_x = left_x + 1;
            left_dir = 5;
        } else if (left_dir != 1 && binary_pers_image[left_y - 1][left_x] == road_color &&
                   binary_pers_image[left_y - 1][left_x - 1] == border_color){  // ���Ϻڣ�����1�����ϰ�
            left_y = left_y - 1;
            left_dir = 0;
        } else if (left_dir != 2 && binary_pers_image[left_y - 1][left_x - 1] == road_color &&
                   binary_pers_image[left_y][left_x - 1] == border_color){  // ���Ϻڣ�����2�����Ұ�
            left_y = left_y - 1;
            left_x = left_x - 1;
            left_dir = 7;
        } else if (left_dir !=5 && binary_pers_image[left_y][left_x - 1] == road_color &&
                   binary_pers_image[left_y + 1][left_x - 1] == border_color){  // ����ڣ�����5�����°�
            left_x = left_x - 1;
            left_dir = 4;
        } else if (left_dir != 6 && binary_pers_image[left_y + 1][left_x - 1] == road_color &&
                   binary_pers_image[left_y + 1][left_x] == border_color){  // ���ºڣ�����6�����°�
            left_y = left_y + 1;
            left_x = left_x - 1;
            left_dir = 3;
        } else if (left_dir != 7 && binary_pers_image[left_y + 1][left_x + 1] == road_color &&
                   binary_pers_image[left_y][left_x + 1] == border_color){  // ���ºڣ�����7�����Ұ�
            left_y = left_y + 1;
            left_x = left_x + 1;
            left_dir = 2;
        } else if (left_dir != 0 && binary_pers_image[left_y + 1][left_x] == road_color &&
                   binary_pers_image[left_y + 1][left_x + 1] == border_color){  // ���ºڣ�����8�����°�
            if(left_skip_index == 0){
                left_skip_index = left_border_index_pers;
            }
        } else {
            break;
        }

        left_border_index_pers++;
        left_border_pers[left_border_index_pers][0] = left_x;
        left_border_pers[left_border_index_pers][1] = left_y;
        left_dirs[left_border_index_pers] = left_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }

        // �ұ߽�
        if(detect_count > 15 && sqrt((right_x - right_border_pers[right_border_index_pers-5][0]) * (right_x - right_border_pers[right_border_index_pers-5][0]) + (right_y - right_border_pers[right_border_index_pers-5][1]) * (right_y - right_border_pers[right_border_index_pers-5][1])) < 4){
            if(right_skip_index == 0){
                right_skip_index = right_border_index_pers;
            }
        } else if(right_y < 3 || right_x > 44 || right_x < 3) {
            if(right_skip_index == 0){
                right_skip_index = right_border_index_pers;
            }
        } else if(right_dir != 2 && binary_pers_image[right_y - 1][right_x - 1] == road_color &&
                  binary_pers_image[right_y - 1][right_x] == border_color) {  // ���Ϻڣ�����2�����ϰ�
            right_y = right_y - 1;
            right_x = right_x - 1;
            right_dir = 7;
        } else if(right_dir != 5 && binary_pers_image[right_y][right_x - 1] == road_color &&
                  binary_pers_image[right_y - 1][right_x - 1] == border_color){  // ����ڣ�����5�����ϰ�
            right_x = right_x - 1;
            right_dir = 4;
        } else if(left_dir != 1 && binary_pers_image[right_y - 1][right_x] == road_color &&
                  binary_pers_image[right_y - 1][right_x + 1] == border_color){  // ���Ϻڣ�����1�����ϰ�
            right_y = right_y - 1;
            right_dir = 0;
        } else if (right_x < 80 && right_dir != 3 && binary_pers_image[right_y - 1][right_x + 1] == road_color &&
                   binary_pers_image[right_y][right_x + 1] == border_color){  // ���Ϻڣ�����3�����Ұ�
            right_y = right_y - 1;
            right_x = right_x + 1;
            right_dir = 6;
        } else if(right_dir != 4 && binary_pers_image[right_y][right_x + 1] == road_color &&
                  binary_pers_image[right_y + 1][right_x + 1] == border_color){  // ���Һڣ�����4�����°�
            right_x = right_x + 1;
            right_dir = 5;
        } else if(right_dir != 6 && binary_pers_image[right_y + 1][right_x - 1] == road_color &&
                  binary_pers_image[right_y][right_x - 1] == border_color){  // ���ºڣ�����6�������
            right_y = right_y + 1;
            right_x = right_x - 1;
            right_dir = 3;
        } else if(right_dir != 7 && binary_pers_image[right_y + 1][right_x + 1] == road_color &&
                  binary_pers_image[right_y + 1][right_x] == border_color) {  // ���ºڣ�����7�����°�
            right_y = right_y + 1;
            right_x = right_x + 1;
            right_dir = 2;
        } else if(right_dir != 0 && binary_pers_image[right_y + 1][right_x] == road_color &&
                  binary_pers_image[right_y + 1][right_x - 1] == border_color){  // ���ºڣ�����8�����°�
            if(right_skip_index == 0){
                right_skip_index = right_border_index_pers;
            }
        } else {
            break;
        }
        right_border_index_pers++;
        right_border_pers[right_border_index_pers][0] = right_x;
        right_border_pers[right_border_index_pers][1] = right_y;
        right_dirs[right_border_index_pers] = right_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }
    }
    detect_count_max = detect_count;

    // ��������
    memset(middle_line_pers, 0, sizeof(middle_line_pers));
    middle_line_index_pers = 0;
    for(int q = 0; q < detect_count_max; q++){
        middle_line_pers[q][0] = (left_border_pers[q][0] + right_border_pers[q][0]) / 2;
        middle_line_pers[q][1] = (left_border_pers[q][1] + right_border_pers[q][1]) / 2;
        middle_line_index_pers++;
    }

    memset(middle_line_single_pers, 0, sizeof(middle_line_single_pers));
    middle_line_single_index_pers = 0;
    int middle_line_current_y = -1;
    for(int q = 0; q < detect_count_max; q++){
        if(middle_line_pers[q][1] != middle_line_current_y && middle_line_pers[q][1] != 0){
            middle_line_single_pers[middle_line_single_index_pers][0] = middle_line_pers[q][0];
            middle_line_single_pers[middle_line_single_index_pers][1] = middle_line_pers[q][1];
            middle_line_single_index_pers++;
            middle_line_current_y = middle_line_pers[q][1];
        }
    }

    return detect_count_max > 41 ? 41 : detect_count_max;
}

void get_max_middle_line_height(){
    max_middle_line_height = 0;
    cornering = 0;
    if(middle_line_index_pers < 1)middle_line_index_pers = 1;
    for(int q = middle_line_index_pers - 1; q > 0; q--){
        if(middle_line_pers[q][0] > 14 && middle_line_pers[q][0] < 34 &&
        DISTANCE(left_border_pers[q][0],left_border_pers[q][1],right_border_pers[q][0],right_border_pers[q][1]) < 10){
            max_middle_line_height = 60 - middle_line_pers[q][1];
            int x1 = middle_line_pers[q][0];
            int y1 = middle_line_pers[q][1];
            int x2 = middle_line_pers[middle_line_index_pers - 1][0];
            int y2 = middle_line_pers[middle_line_index_pers - 1][1];
            if(max_white_column.left_height > 35 && abs(y1 - y2) < 15 && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0) {
                if (x2 - x1 > 6) cornering = 1;
                if (x2 - x1 < -6) cornering = -1;
            } else {
                cornering = 0;
            }
            break;
        }
    }
    max_middle_line_height = gray_binary_pers_image[44][23];
}

// ����A��BC���ɵ�ֱ�ߵľ���
float distance_to_line(float Ax, float Ay, float Bx, float By, float Cx, float Cy){
    float A = Cy - By;
    float B = Bx - Cx;
    float C = Cx * By - Bx * Cy;
    if(A == 0 && B == 0){
        return 0;
    }
    return fabsf(A * Ax + B * Ay + C) / sqrtf(A * A + B * B);
}

// ����ABC����ļнǣ�����B��Ϊ�˵�
float get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy) {
    // ���Ҷ�����Ƕ�
    float AB = sqrt((Bx - Ax) * (Bx - Ax) + (By - Ay) * (By - Ay));
    float BC = sqrt((Bx - Cx) * (Bx - Cx) + (By - Cy) * (By - Cy));
    float AC = sqrt((Cx - Ax) * (Cx - Ax) + (Cy - Ay) * (Cy - Ay));
    float angle = acos((AB * AB + BC * BC - AC * AC) / (2 * AB * BC)) * 180 / 3.1415926;
    // ��BAΪ0�ȣ���ʱ��Ϊ�Ƕ��������жϽǶ��Ƿ����180�ȣ�������180����ȡ��ֵ
    if (((Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax)) < 0) {
        angle = -angle;
    }
    return angle;
}

uint8_t is_left_corner(float left_Ax, float left_Ay, float left_Bx, float left_By, float left_Cx, float left_Cy,
                     float right_Ax, float right_Ay, float right_Bx, float right_By, float right_Cx, float right_Cy){
/*C---B
 *  \ |
 *    A
 * */
    // ��һ���жϽǶ�
    float _left_angle = get_angle(left_Ax, left_Ay, left_Bx, left_By, left_Cx, left_Cy);
    float _right_angle = get_angle(right_Ax, right_Ay, right_Bx, right_By, right_Cx, right_Cy);
    if(_left_angle > -110 && _left_angle < -70 && _right_angle > -120){  // ��Ƕ���-75~-125��֮�䣬�ҽǶȴ���-120
        // �ڶ����ж�б�ʺ�B��λ��
        float slope_AC = (left_Cy - left_Ay) / (left_Cx - left_Ax);  // б�ʴ���0����ֹʮ��·�ڼ�⵽�����յ㣩
        float det = left_Ax * (left_By - left_Cy) - left_Ay * (left_Bx - left_Cx) + left_Bx * left_Cy - left_By * left_Cx;  // �ж�B����ACֱ�ߵ��·�����ֹʮ��·�ڼ�⵽�����յ㣩
        if(slope_AC > 0 && det < 0 && left_By > 10){
            // ��������ȡ��ߵ���ߵ�����ֵ������ԭͼ�����Ϸ�����3�����ص��Ƿ���ڱ߽�
            int left_highest_y = left_By;
            int left_highest_x = left_Bx;
            // ��ֹԽ��
            if(left_highest_x < 6) left_highest_x = 6;
            int left_has_border = 0;
            int i = 1;
            for(int j = -3; j <= 3; j++){
                if(binary_image[left_highest_y - i][left_highest_x + j] == border_color){
                    left_has_border++;
                }
            }
            if(left_has_border == 0){  // ����Ϸ�û�ҵ��߽�
                int diff = left_By - left_Cy;  // B���C���y����С��2����ֹ��⵽�����м��Ȧ��
                if(diff < 2){
                    flag.left_B_higher_than_C = true;
                } else {
                    flag.left_B_higher_than_C = false;
                }
                left_corner_angle = _left_angle;
                return 1;
            }
        }
    }
    return 0;
}

uint8_t is_right_corner(float right_Ax, float right_Ay, float right_Bx, float right_By, float right_Cx, float right_Cy,
                      float left_Ax, float left_Ay, float left_Bx, float left_By, float left_Cx, float left_Cy){
/*B---C
 *| /
 *A
 * */
    // ��һ���жϽǶ�
    float _right_angle = get_angle(right_Ax, right_Ay, right_Bx, right_By, right_Cx, right_Cy);
    float _left_angle = get_angle(left_Ax, left_Ay, left_Bx, left_By, left_Cx, left_Cy);
    if(_right_angle < 110 && _right_angle > 70 && _left_angle < 120){  // �ҽǶ���75~125��֮�䣬��Ƕ�С��120
        // �ڶ����ж�б�ʺ�B��λ��
        float slope_AC = (right_Cy - right_Ay) / (right_Cx - right_Ax);  // б��С��0����ֹʮ��·�ڼ�⵽�����յ㣩
        float det = right_Ax * (right_By - right_Cy) - right_Ay * (right_Bx - right_Cx) + right_Bx * right_Cy - right_By * right_Cx;  // �ж�B����ACֱ�ߵ��·�����ֹʮ��·�ڼ�⵽�����յ㣩
        if(slope_AC < 0 && det > 0 && right_By > 10){
            // ��������ȡ�ұߵ���ߵ�����ֵ������ԭͼ�����Ϸ�����3�����ص��Ƿ���ڱ߽�
            int right_highest_y = right_By;
            int right_highest_x = right_Bx;
            // ��ֹԽ��
            if(right_highest_x > 88) right_highest_x = 88;
            int right_has_border = 0;
            int i = 1;
            for(int j = -3; j <= 3; j++){
                if(binary_image[right_highest_y - i][right_highest_x + j] == border_color){
                    right_has_border++;
                }
            }
            if(right_has_border == 0){  // �ұ��Ϸ�û�ҵ��߽�
                int diff = right_By - right_Cy;  // B���C���y����С��2����ֹ��⵽�����м��Ȧ��
                if(diff < 2){
                    flag.right_B_higher_than_C = true;
                } else {
                    flag.right_B_higher_than_C = false;
                }
                right_corner_angle = _right_angle;
                return 1;
            }
        }
    }
    return 0;
}

void calculate_contrast(uint8_t *dst_image, const uint8_t *src_image, int16_t image_w, int16_t image_h) {
    for (int y = 0; y < image_h; y++) {
        for (int x = 1; x < image_w - 1; x++) {
            // ��������2������
            int pixel1 = src_image[y * image_w + (x - 1)];
            int pixel2 = src_image[y * image_w + (x + 1)];

            // ��������֮��Ĳ�����ܺ�
            int diff = pixel1 - pixel2;
            int sum = pixel1 + pixel2;

            // �����ȺͲ��Ŵ�255��
            // (sum != 0)�������0
            uint8_t contrast = (sum != 0) ? abs(diff * 255 / sum) : 0;
            dst_image[y * image_w + x] = contrast;
        }
    }
}

void calculate_contrast_x8(uint8_t *dst_image, const uint8_t *src_image, int16_t image_w, int16_t image_h) {
    for (int y = 1; y < image_h - 1; y++) {
        for (int x = 1; x < image_w - 1; x++) {
            int pixel1 = src_image[y * image_w + (x - 1)];
            int pixel2 = src_image[y * image_w + (x + 1)];
            int pixel3 = src_image[(y - 1) * image_w + x];
            int pixel4 = src_image[(y + 1) * image_w + x];
            int pixel5 = src_image[(y - 1) * image_w + (x - 1)];
            int pixel6 = src_image[(y - 1) * image_w + (x + 1)];
            int pixel7 = src_image[(y + 1) * image_w + (x - 1)];
            int pixel8 = src_image[(y + 1) * image_w + (x + 1)];


            int diff1 = pixel1 - pixel2;
            int diff2 = pixel3 - pixel4;
            int diff3 = pixel5 - pixel6;
            int diff4 = pixel7 - pixel8;
            int sum1 = pixel1 + pixel2;
            int sum2 = pixel3 + pixel4;
            int sum3 = pixel5 + pixel6;
            int sum4 = pixel7 + pixel8;

            int contrast1 = (sum1 != 0) ? abs(diff1 * 80 / sum1) : 0;
            int contrast2 = (sum2 != 0) ? abs(diff2 * 80 / sum2) : 0;
            int contrast3 = (sum3 != 0) ? abs(diff3 * 32 / sum3) : 0;
            int contrast4 = (sum4 != 0) ? abs(diff4 * 32 / sum4) : 0;
            dst_image[y * image_w + x] = contrast1 + contrast2 + contrast3 + contrast4;
        }
    }
}

void tft180_draw_border_line(cv::Mat& dst_image, uint16_t x, uint16_t y, const uint8_t line[][2], uint16_t color){
    uint16_t i = 0;
    while(line[i][0] || line[i][1]) {
        dst_image.at<cv::Vec3b>(y + line[i][1], x + line[i][0]) = cv::Vec3b(0, 255, 0);
        i++;
    }
}
