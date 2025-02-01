//
// Created by ashkore on 25-2-1.
//

#ifndef CV_MAIN_H
#define CV_MAIN_H

#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <iostream>
#include <cstring>
#include <sys/timerfd.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <pthread.h>
#include "opencv2/opencv.hpp"
// 图像和传感器数据结构
struct FrameData {
    cv::Mat frame;
    int sensor_value{}; // 模拟传感器数据
};

// 无锁环形缓冲区
class RingBuffer {
public:
    explicit RingBuffer(int size) : buffer(size), head(0), tail(0) {}

    bool push(const FrameData& data) {
        int next_head = (head + 1) % buffer.size();
        if (next_head == tail) return false; // 缓冲区满
        buffer[head] = data;
        head = next_head;
        return true;
    }

    bool pop(FrameData& data) {
        if (tail == head) return false; // 缓冲区空
        data = buffer[tail];
        tail = (tail + 1) % buffer.size();
        return true;
    }

private:
    std::vector<FrameData> buffer;
    std::atomic<int> head;
    std::atomic<int> tail;
};

#endif //CV_MAIN_H
