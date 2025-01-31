#include "opencv2/opencv.hpp"
#include "mytag.h"
#include <csignal>
#include <ctime>
#include "pthread.h"
#include <chrono>
#define MEASURE_TIME(TAG, code_block) \
    do { \
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); \
    code_block \
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); \
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start); \
    fprintf(stdout, "%s %.2lfus\n", TAG, time_used.count() * 1000000); \
    } while (0)

#define MEASURE_TIME_TO_VAR(duration_var, code_block) \
    do { \
    double start = cv::getTickCount(); \
    code_block \
    double end = cv::getTickCount(); \
    duration_var = (end - start) / cv::getTickFrequency(); \
    } while (0)


int main()
{
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.open(0);
    cv::VideoWriter http;
    http.open("httpjpg", 7766);

    cv::Mat frame;
    cv::Mat gray;
    auto atag = mytag("tag36h11", 1.5, 0, 1, false, false);
    int id;
    int i = 0;
    double distance;
    while (true) {
        MEASURE_TIME("read frame:", {
            cap >> frame;
        });
        MEASURE_TIME("convert gray:", {
            cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        });
        MEASURE_TIME("detect_time:", {
            atag.detect(gray);
        });
        MEASURE_TIME("getclosettagindex:", {
            atag.getClosetTagIndex();
        });
        MEASURE_TIME("draw:", {
            atag.draw(frame);
        });
        MEASURE_TIME("getid:", {
            id = atag.getClosetTagID();
        });
        MEASURE_TIME("getdistance:", {
            distance = atag.getClosetTagDistance(1500);
            cv::putText(frame, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
        });
        MEASURE_TIME("http write:", {
            http << frame;
        });
        fprintf(stdout, "frame:%d\n", i++);
    }
    atag.clean();
    return 0;
}