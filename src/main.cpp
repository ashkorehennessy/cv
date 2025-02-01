#include "main.h"
#include "mytag.h"
#include "log.h"
const int timer_period = 35;  // 定时器周期(ms)
const int BUFFER_SIZE = 5;
RingBuffer frame_buffer(BUFFER_SIZE); // 环形缓冲区
std::atomic<bool> running{true};      // 控制线程运行标志
void* realtime_task(void* arg) {
    // 设置实时线程优先级
    struct sched_param param = {.sched_priority = 99};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    // 创建高精度定时器
    int timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);
    struct itimerspec timer_spec = {
        .it_interval = {0, timer_period * 1000 * 1000}, // 10ms周期
        .it_value = {0, timer_period * 1000 * 1000}
    };
    timerfd_settime(timer_fd, 0, &timer_spec, nullptr);

    // 创建epoll实例
    int epoll_fd = epoll_create1(0);
    struct epoll_event event = {.events = EPOLLIN, .data = {.fd = timer_fd}};
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &event);

    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;

    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.open(0);

    while (running) {
        // 等待定时器事件
        struct epoll_event events[1];
        int num_events = epoll_wait(epoll_fd, events, 1, -1);
        if (num_events > 0 && events[0].data.fd == timer_fd) {
            uint64_t expirations;
            read(timer_fd, &expirations, sizeof(expirations));

            // 模拟任务
            FrameData new_frame;
            cap >> new_frame.frame;                                       // 读取图像
            // 其他操作

            // 将数据推入缓冲区
            if (!frame_buffer.push(new_frame)) {
                LOGW("timer_event", "Buffer full, dropping frame!");
            }

            end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            start = std::chrono::steady_clock::now();
            fprintf(stdout,"timer_event 距离上次事件%.2lfms\n", time_used.count() * 1000);
            LOGI("timer_event", "距离上次事件%.2lfms", time_used.count() * 1000);
            if(std::abs(time_used.count() * 1000 - timer_period) > 1) {
                fprintf(stdout,"timer_event 定时器周期不准确，误差: %.2lfms\n", time_used.count() * 1000 - timer_period);
                LOGW("timer_event", "定时器周期不准确，误差: %.2lfms", time_used.count() * 1000 - timer_period);
            }
        }
    }

    close(timer_fd);
    close(epoll_fd);
    return nullptr;
}

// 非实时任务线程函数
void *non_realtime_task(void *arg) {
    cv::VideoWriter http;
    http.open("httpjpg", 7766);
    auto atag = mytag("tag36h11", 1.5, 0, 1, false, false);
    int id;
    cv::Mat gray;
    double distance;
    while (running) {
        FrameData frame_data;
        if (frame_buffer.pop(frame_data)) {
            MEASURE_TIME("convert gray", {
                cvtColor(frame_data.frame, gray, cv::COLOR_BGR2GRAY);
            });
            MEASURE_TIME("detect_time", {
                atag.detect(gray);
            });
            MEASURE_TIME("getclosettagindex", {
                atag.getClosetTagIndex();
            });
            MEASURE_TIME("draw", {
                atag.draw(frame_data.frame);
            });
            MEASURE_TIME("getid", {
                id = atag.getClosetTagID();
            });
            MEASURE_TIME("getdistance", {
                distance = atag.getClosetTagDistance(1500);
                cv::putText(frame_data.frame, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
            });
            MEASURE_TIME("http write", {
                http << frame_data.frame;
            });
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 缓冲区空，短暂等待
        }
    }
    return nullptr;
}


int main()
{
    if(!log_init("app.logcat")) {
        return 1;
    }
    LOGW("MAIN", "Application starting...");

    // 创建线程
    pthread_t rt_thread;
    pthread_t nrt_thread;
    pthread_create(&rt_thread, nullptr, realtime_task, nullptr);
    pthread_create(&nrt_thread, nullptr, non_realtime_task, nullptr);

    std::this_thread::sleep_for(std::chrono::seconds(10)); // 运行10秒

    running = false; // 停止线程

    // 等待线程结束
    log_shutdown();
    pthread_join(rt_thread, nullptr);
    pthread_join(nrt_thread, nullptr);
}