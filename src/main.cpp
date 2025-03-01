#include "main.h"
#include "mytag.h"
#include "log.h"
#include "vofa.h"
#include <csignal>
#include <my_cv2.h>

#include "image_process.h"
int cornering;
int image_diff;
int force_roundabout;
volatile sig_atomic_t g_signal_received = 0;
const int timer_period = 10;  // 定时器周期(ms)
std::atomic<bool> running{true};      // 控制线程运行标志
cv::Mat frame;
cv::Mat result_image;
uint8_t    gray1ch_image[60][80];
auto tcp_transport = std::make_unique<TCPTransport>("0.0.0.0", 1347);
auto vofa = VOFA(std::move(tcp_transport));
auto udp_transport = std::make_unique<UDPTransport>("192.168.5.194", 1349);
auto vofa_udp = VOFA(std::move(udp_transport));
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

    int bottom_threshold = 100;
    int contrast_threshold = 20;
    int canny_lowThreshold = 16;
    int canny_highThreshold = 40;
    int incision = 6;
    int incision_max = 6;
    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 120);
    cap.open(0);
    cv::Mat myframe;
    result_image = cv::Mat(60, 80, CV_8UC1);
    while (running) {
        // 等待定时器事件
        struct epoll_event events[1];
        int num_events = epoll_wait(epoll_fd, events, 1, -1);
        if (num_events > 0 && events[0].data.fd == timer_fd) {
            uint64_t expirations;
            read(timer_fd, &expirations, sizeof(expirations));
            // end = std::chrono::steady_clock::now();
            // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            // start = std::chrono::steady_clock::now();
            // LOGI("timer_event", "距离上次事件%.2lfms", time_used.count() * 1000);
            // if(std::abs(time_used.count() * 1000 - timer_period) > 1) {
            //     LOGW("timer_event", "定时器周期不准确，误差: %.2lfms", time_used.count() * 1000 - timer_period);
            // }
            // MEASURE_TIME("rt task", {
            cap >> frame;
            // vofa.imwrite(frame);
            frame.copyTo(myframe);
            cv::resize(myframe, myframe, cv::Size(80,60));
            memcpy(gray_image, myframe.data, 80 * 60);
            calculate_contrast_x8((uint8_t *)contrast_image, (const uint8_t *)gray_image, 80, 60);
            memcpy((uint8_t *) binary_image, (const uint8_t *) contrast_image, 80 * 60);
            my_cv2_doubleThreshold((uint8_t *) binary_image, 80, 0, 0, 80, 60, canny_lowThreshold, canny_highThreshold);
            my_cv2_checkConnectivity((uint8_t *) binary_image, 80, 0, 0, 80, 60);
            my_cv2_threshold((uint8_t *) binary_image, 80, 0, 0, 80, 60, 127, 255);
            memcpy((uint8_t *) gray_binary_image, (const uint8_t *) gray_image, 80 * 60);
            my_cv2_threshold((uint8_t *) gray_binary_image, 80, 0, 0, 80, 60, 128, 255);
            // vofa.imwrite((uint8_t *)gray_binary_image, 80, 60);
            bottom_start_end_x_get();

            get_max_middle_line_height();

            incision = incision_max;
            max_white_column_get(bottom_start_x > 15 ? bottom_start_x : 15, 1, bottom_end_x < 64 ? bottom_end_x : 64 , 59);

            get_distance_line();
            get_lost_count();
            check_garage_and_obstacle();

            check_ramp();
            check_crossroad();
            check_roundabout();
            get_narrow_line();
            check_garage_and_obstacle();
            draw_rectan();
            int detect_count_max = get_border_line(80);
            // });
            // vofa.imwrite((uint8_t *)contrast_image, 80, 60);
        }
    }

    close(timer_fd);
    close(epoll_fd);
    return nullptr;
}

// 非实时任务线程函数
void *non_realtime_task(void *arg) {
    auto atag = mytag("tag36h11", 1.5, 0, 1, false, false);
    int id;
    cv::Mat gray;
    cv::Mat gray1ch(60, 80, CV_8UC1, (void*)gray1ch_image);
    cv::Mat gray3ch;
    double distance;
    std::vector<uchar> jpg;
    while (running) {
            frame.copyTo(gray);
            // MEASURE_TIME("non rt task", {
                memcpy(gray1ch_image, gray_image, 80 * 60);
                cv::cvtColor(gray1ch, gray3ch, cv::COLOR_GRAY2BGR);
            // });
            MEASURE_TIME("detect_time", {
                atag.detect(gray);
            });
            // MEASURE_TIME("getclosettagindex", {
                atag.getClosetTagIndex();
//            });
//            MEASURE_TIME("draw", {
                atag.draw(gray3ch, 0.25);
//            });detect_count_max:
//            MEASURE_TIME("getid", {
                id = atag.getClosetTagID();
//            });
//            MEASURE_TIME("getdistance", {
                distance = atag.getClosetTagDistance(1500);
                // cv::putText(gray3ch, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
//            });
                tft180_draw_border_line(gray3ch, 0,0,left_border, cv::Scalar(0, 0xff, 0));
                tft180_draw_border_line(gray3ch, 0, 0, right_border, cv::Scalar(0, 0xff, 0));
                tft180_draw_border_line(gray3ch, 0, 0, middle_line, cv::Scalar(0, 0, 0xff));
            // MEASURE_TIME("http write", {
                vofa.imwrite(gray3ch);
                // http << gray3ch;
            // });
    }
    return nullptr;
}

void signal_handler(int sig) {
    running = false;
    g_signal_received = sig;
    log_shutdown();
}

int main()
{
    // 注册信号处理
    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    if(!log_init("app.logcat")) {
        return 1;
    }
    LOGW("MAIN", "Application starting...");
    system("v4l2-ctl -d /dev/video0 -c contrast=100 -c gamma=500 -c auto_exposure=1 -c exposure_time_absolute=100");
    // 创建posix线程
    pthread_t rt_thread;
    pthread_t nrt_thread;
    pthread_create(&rt_thread, nullptr, realtime_task, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 运行1秒
    pthread_create(&nrt_thread, nullptr, non_realtime_task, nullptr);

    std::this_thread::sleep_for(std::chrono::seconds(1000)); // 运行100秒

    running = false; // 停止线程

    log_shutdown();
}