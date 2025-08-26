#include <stdio.h>
#include <cmath>
#include<Eigen/Dense>

// #include "timer.h"
// #pragma warning(disable:4996)
#include "net/robocup_ssl_client.h"
// #include "timer.h"
// #pragma warning(disable:4996)
#include "visionProto/messages_robocup_ssl_detection.pb.h"
#include "visionProto/messages_robocup_ssl_geometry.pb.h"
#include "visionProto/messages_robocup_ssl_wrapper.pb.h"

#include "gcProto/ssl_gc_referee_message.pb.h"

#include <iostream>
#include <iomanip>
using namespace std;
using namespace Eigen;



#include <windows.h>
#include <time.h>
#pragma comment(lib, "winmm.lib")

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#define pi222 3.1415926
#define M_PI pi222
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)
#include <iostream>
#include <cmath>
#include <vector>

#include <iostream>
#include <cmath>
#include <vector>

#include <fstream>
#include <iostream>

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <complex>

struct Point
{
    double x;
    double y;
    double t; // time
};
Point ballPos;
std::mutex mtx;
std::ofstream outfile;


void getVision();


struct Config
{
    int ball_number = 4; // 定位球的数量
    double camar_height = 200;//相机高度
    double ball_radius = 20; // 球的半径
    int group_number = 4; // 分组数量
    double compare = 100000;//合理误差值
} config;


static double average_a_x;//ax+b
static double average_b_x;
static double average_c_x;
static double average_a_y;//ay+b
static double average_b_y;
static double average_c_y;

static double* a_x = new double[config.group_number];
static double* a_y = new double[config.group_number];
static double* b_x = new double[config.group_number];
static double* b_y = new double[config.group_number];
static double* c_x = new double[config.group_number];
static double* c_y = new double[config.group_number];


std::map<int, std::pair<double, double>> real_position = {
    {1, {200,   582.5}},
    {2, {200,  -582.5}},
    {3, {1345, -582.5}},
    {4, {1345,  582.5}} }; // 定位球的编号和真实位置
//2540, 1345, 1395

std::map<int, std::pair<double, double>> vision_position;



// 函数：持续接收视觉数据
void getVision()
{
    RoboCupSSLClient client;
    client.open(true); // 打开客户端连接
    SSL_WrapperPacket packet;
    std::vector<Point> ballP; // 存储球的位置历史记录

    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_detection())
            { // 检查是否有检测到的数据
                SSL_DetectionFrame detection = packet.detection();
                int balls_n = detection.balls_size(); // 获取检测到的球的数量

                // 更新球的位置
                for (int i = 0; i < balls_n; i++)
                {
                    SSL_DetectionBall ball = detection.balls(i);
                    Point pp;
                    pp.x = ball.x(); // 调整 x 位置，减去 20 个单位
                    pp.y = ball.y();
                    pp.t = (detection.t_capture() / 10 - (int)(detection.t_capture() / 10)); // 计算时间信息
                    mtx.lock();                                                              // 加锁以确保线程安全
                    ballPos = pp;                                                            // 更新全局的球位置
                    mtx.unlock();                                                            // 解锁

                    ballP.push_back(pp); // 将球的位置添加到历史记录中
                    if (pp.x < 300)      // 如果球的位置接近（x 坐标小于 300），记录数据
                    {
                        for (auto ball2 : ballP)
                        {
                            // outfile << ball2.x << "," << ball2.y << "," << ball2.t << '\n'; // 将球的位置和时间写入文件
                        }
                        // outfile.close(); // 关闭文件
                    }
                }
            }
        }
    }
    return;
}


int main()
{
    std::thread t1(getVision);
    t1.detach(); // 分离线程，独立运行

    //怎么把视觉的位置传到索引里
    for (int n = 0; n < config.ball_number; ++n) {
        char ch;
        cout << "输入空格，记录第" << n + 1 << "个数据\n";
        cin >> ch;
        mtx.lock();
        vision_position[n + 1] = { ballPos.x, ballPos.y };
        cout << "(" << vision_position[n + 1].first << ", " << vision_position[n + 1].second << ")\n";
        mtx.unlock();
    }

    MatrixXd A(6, 6);
    VectorXd b(6);

    int n = 1;
    for (int i = 1; i <= config.ball_number; i++) {
        for (int m = i + 1; m <= config.ball_number; m++) {
            for (int j = m + 1; j <= config.ball_number; j++) {
                b << real_position[i].first,
                    real_position[i].second,
                    real_position[m].first,
                    real_position[m].second,
                    real_position[j].first,
                    real_position[j].second;
                A << vision_position[i].first, vision_position[i].second, 1, 0, 0, 0,
                    0, 0, 0, vision_position[i].first, vision_position[i].second, 1,
                    vision_position[m].first, vision_position[m].second, 1, 0, 0, 0,
                    0, 0, 0, vision_position[m].first, vision_position[m].second, 1,
                    vision_position[j].first, vision_position[j].second, 1, 0, 0, 0,
                    0, 0, 0, vision_position[j].first, vision_position[j].second, 1;
                VectorXd x = A.colPivHouseholderQr().solve(b);
                a_x[n] = x(0);
                a_y[n] = x(3);
                b_x[n] = x(1);
                b_y[n] = x(4);
                c_x[n] = x(2);
                c_y[n] = x(5);
                n++;
            }
        }
    }
    // 计算平均值
    double sum_a_x = 0, sum_b_x = 0, sum_c_x = 0;
    double sum_a_y = 0, sum_b_y = 0, sum_c_y = 0;
    for (int i = 1; i <= config.group_number; i++) {
        sum_a_x += a_x[i];
        sum_b_x += b_x[i];
        sum_c_x += c_x[i];
        sum_a_y += a_y[i];
        sum_b_y += b_y[i];
        sum_c_y += c_y[i];
    }
    int valid_count = 0;
    for (int i = 1; i <= config.group_number; i++) {
        if (abs(a_x[i] - sum_a_x / config.group_number - 1) < config.compare &&
            abs(b_x[i] - sum_b_x / config.group_number - 1) < config.compare &&
            abs(c_x[i] - sum_c_x / config.group_number - 1) < config.compare &&
            abs(a_y[i] - sum_a_y / config.group_number - 1) < config.compare &&
            abs(b_y[i] - sum_b_y / config.group_number - 1) < config.compare &&
            abs(c_y[i] - sum_c_y / config.group_number - 1) < config.compare) {
            average_a_x += a_x[i];
            average_b_x += b_x[i];
            average_c_x += c_x[i];
            average_a_y += a_y[i];
            average_b_y += b_y[i];
            average_c_y += c_y[i];
            valid_count++;
        }
    }
    average_a_x /= valid_count;
    average_b_x /= valid_count;
    average_c_x /= valid_count;
    average_a_y /= valid_count;
    average_b_y /= valid_count;
    average_c_y /= valid_count;






    //怎么输出输入到文件里
    outfile.open("C:\\Robot\\2025PingPongCampus\\codes\\special\\visual_correction.csv", ios::trunc);
    if (outfile.is_open()) {
        // 表头
        outfile << "a_x, b_x, c_x, a_y, b_y, c_y\n";
        // 写入平均值
        outfile << average_a_x << ", " << average_b_x << ", " << average_c_x << ", "
            << average_a_y << ", " << average_b_y << ", " << average_c_y << "\n";
        cout << average_a_x << ", " << average_b_x << ", " << average_c_x << ", "
            << average_a_y << ", " << average_b_y << ", " << average_c_y << "\n";
        outfile.close();
    }
    else {
        cerr << "Unable to open file";
    }
    return 0;

}
