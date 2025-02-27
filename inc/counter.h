//
// Created by ashkore on 2023/11/2.
//

#ifndef SMART_CAR_CAMERA_COUNTER_H
#define SMART_CAR_CAMERA_COUNTER_H

#include "cstdint"

typedef struct{
    int erase;  // �����İ�ɫ���ص�����
    int16_t found_left_roundabout;  // �����󻷵�����
    int16_t found_right_roundabout;  // �����һ�������
    int16_t found_crossroad;  // ����ʮ��·�ڼ���
    int16_t found_ramp;  // �����µ�����
    int16_t found_garage;  // ���ֳ������
    int16_t found_obstacle;  // �����ϰ������
    int16_t drive_in_left_roundabout;  // ����ʻ���󻷵�����ʱ(ms)
    int16_t drive_in_right_roundabout;  // ����ʻ���һ�������ʱ(ms)
    int16_t drive_in_crossroad;  // ����ʻ��ʮ��·�ڵ���ʱ(ms)
    int16_t drive_in_ramp;  // ����ʻ���µ�����ʱ(ms)
    int16_t drive_in_obstacle;  // ����ʻ���ϰ��ﵹ��ʱ(ms)
    int16_t avoid_roundabout;  // �����л�����⵹��ʱ(ms)
    int16_t avoid_obstacle;  // �������ϰ����⵹��ʱ(ms)
    int16_t serial_send;  // ���ڷ��ͼ�������
    int16_t stop_motor;  // ֹͣ�������
    int16_t out_of_bound;  // �������
    uint16_t beep_ms;  // ����������
    int16_t boost;  // ���ټ���
    int32_t stop_delay;  // ͣ������
    int16_t start_motor_delay;  // �����������
    int32_t save_flash_led;  // ��������LED����
    int32_t read_flash_led;  // ��ȡ����LED����
} Counter;

extern Counter counter;

#endif //SMART_CAR_CAMERA_COUNTER_H
