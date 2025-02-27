//
// Created by ashkore on 2023/12/2.
//

#ifndef SMART_CAR_CAMERA_FLAG_H
#define SMART_CAR_CAMERA_FLAG_H


#define LOCK 1
#define UNLOCK 0
#include <cstdint>

typedef struct{
    int8_t drive_in_garage;  // ʻ�복���־
    int8_t drive_out_garage;  // ʻ�������־
    int8_t change_speed_setpoint;  // ����ı��ٶ��趨���־
    int8_t already_ramp;  // �Ѿ����±�־
    int8_t ramp_up;  // ���±�־
    int8_t ramp_down;  // ���±�־
    int8_t found_crossroad;  // ����ʮ��·�ڱ�־
    int8_t found_left_roundabout;  // �����󻷵���־
    int8_t found_right_roundabout;  // �����һ�����־
    int8_t found_left_corner;  // ������սǱ�־
    int8_t found_right_corner;  // �����ҹսǱ�־
    int8_t found_ramp;  // �����µ���־
    int8_t found_garage;  // ���ֳ����־
    int8_t found_obstacle;  // �����ϰ����־
    int8_t left_roundabout_type;  // �󻷵�����
    int8_t right_roundabout_type;  // �һ�������
    int8_t left_B_higher_than_C;  // ��ս�B�����C���־��������Ҫʹ��������ϸ������
    int8_t right_B_higher_than_C;  // �ҹս�B�����C���־��������Ҫʹ��������ϸ������
    int8_t start;
    int8_t stop;  // ǿ��ͣ����־
    int8_t icm20602_error;  // �����Ǵ����־
    int8_t lost_control;  // ʧ�ر�־
    int8_t image_preprocess_method;  // ͼ��Ԥ������
    int8_t crossroad_By_diff;  // ʮ��·��B���ֵ
    int8_t boost;  // ���ٱ�־
    int8_t boost_drive;  // ����������־
    int8_t break_drive;  // ɲ��������־
    int8_t image_var_lock;  // ͼ�������
    int8_t force_angle;  // ǿ�ƽǶ�
    int8_t advance_avoid_obstacle;  // ��ǰ���ϱ�־
    int8_t advance_avoid_obstacle_dir;  // ��ǰ���Ϸ���
} Flag;

extern Flag flag;

#endif //SMART_CAR_CAMERA_FLAG_H
