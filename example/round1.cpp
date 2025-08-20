#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "visionProto/messages_robocup_ssl_detection.pb.h"
#include "visionProto/messages_robocup_ssl_geometry.pb.h"
#include "visionProto/messages_robocup_ssl_wrapper.pb.h"

#include "gcProto/ssl_gc_referee_message.pb.h"

#include <iostream>
#include "endMotor/MotorIK.h"
#include <windows.h>
#include <time.h>
#pragma comment(lib, "winmm.lib")

#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jkerr.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jktypes.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/JAKAZuRobot.h"
#include <chrono>
#include <thread>
#include <mutex>
#define pi222 3.1415926
#define M_PI pi222
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)
#include <cmath>
#include <vector>
#include <fstream>
#include <complex>
#include <conio.h> // Windows specific

#include <mmsystem.h>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

#define HEIGHT_LEFT  245
#define HEIGHT_RIGHT 238
#define BALL_DELTA_X 0
#define BALL_DELTA_Y 0
#define TABLE_LENGTH 2740
#define TABLE_WIDTH 1525
#define GATE_WIDTH 80
#define BALL_WIDTH 0 // FIX

struct Point
{
    double x;
    double y;
    double t;  // time
    double vx; // x方向速度
    double vy; // y方向速度
};

// 全局变量，用于存储球的位置和管理线程
Point ballPos;         // 球的位置
std::mutex mtx;        // 互斥锁，用于保护共享资源的访问

std::map<int, std::tuple<double, double, int>> gates = {
    {1, {TABLE_LENGTH,  710, 6}},
    {2, {TABLE_LENGTH,  540, 5}},
    {3, {TABLE_LENGTH,  360, 5}},
    {4, {TABLE_LENGTH,  180, 4}},
    {5, {TABLE_LENGTH,    0, 4}},
    {6, {TABLE_LENGTH, -180, 4}},
    {7, {TABLE_LENGTH, -360, 5}},
    {8, {TABLE_LENGTH, -540, 5}},
    {9, {TABLE_LENGTH, -710, 6}},
};  // 目标门的序号及对应的坐标和分数

void getVision();

// 计算击球角度
double goAvalue(double ball_x, double ball_y, double posi_x, double posi_y)
{
    std::cout << std::endl << "==== 角度计算调试信息 ====" << std::endl;
    std::cout << "球的位置: (" << ball_x << ", " << ball_y << ")" << std::endl;
    std::cout << "目标位置: (" << posi_x << ", " << posi_y << ")" << std::endl;

    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;
    std::cout << "X方向距离: " << run_x << " mm" << std::endl;
    std::cout << "Y方向距离: " << run_y << " mm" << std::endl;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;
    std::cout << "计算得到角度: " << angle_deg << " 度" << std::endl;
    std::cout << "========================" << std::endl << std::endl;

    return angle_deg;
}

// 判断球是否能穿过一个门
bool can_pass_gate(double ball_x, double ball_y, double gate_x, double gate_y) {
    double dx = gate_x - ball_x;
    if (fabs(dx) < 1e-5) return false;  // 避免除以0
    double dy = gate_y - ball_y;
    double slope = dy / dx;
    double y_at_gate = ball_y + slope * dx;
    return fabs(y_at_gate - gate_y) <= GATE_WIDTH / 2;
}

void add_delta(int id, double ball_y, Point& target) {
    switch (id) {
    case 1:
        target.y += fabs(ball_y) / TABLE_WIDTH * 2.0 * 10;
        break;
    case 9:
        target.y -= fabs(ball_y) / TABLE_WIDTH * 2.0 * 10;
        break;
    }
}

// 基于目标门的位置和得分，自动选择合适的目标门
Point choice(double ball_x, double ball_y) {
    Point ball{ ball_x, ball_y, 0, 0, 0 };
    int best_gate = -1;
    int best_score = -1;
    double min_center_dist = 1e9;
    Point target;

    for (std::map<int, std::tuple<double, double, int>>::const_iterator it = gates.begin(); it != gates.end(); ++it) {
        int id = it->first;
        double gx = std::get<0>(it->second);
        double gy = std::get<1>(it->second);
        int score = std::get<2>(it->second);

        if (!can_pass_gate(ball_x, ball_y, gx, gy)) continue;

        bool prefer_direction = (ball_y > 0 && gy < 0) || (ball_y < 0 && gy > 0);

        if (prefer_direction) {
            if (score > best_score || (score == best_score && fabs(gy) < min_center_dist)) {
                best_score = score;
                min_center_dist = fabs(gy);
                best_gate = id;
                target.x = gx;
                target.y = gy;
            }
        }
    }

    if (best_gate == -1) {
        for (std::map<int, std::tuple<double, double, int>>::const_iterator it = gates.begin(); it != gates.end(); ++it) {
            int id = it->first;
            double gx = std::get<0>(it->second);
            double gy = std::get<1>(it->second);
            int score = std::get<2>(it->second);

            if (!can_pass_gate(ball_x, ball_y, gx, gy)) continue;

            if (score > best_score || (score == best_score && fabs(gy) < min_center_dist)) {
                best_score = score;
                min_center_dist = fabs(gy);
                best_gate = id;
                target.x = gx;
                target.y = gy;
            }
        }
    }

    if (best_gate == -1) {  // 找不到门，默认打5
        std::cout << "未找到可击打目标门，使用默认门 5" << std::endl;
        best_gate = 5;
        target.x = std::get<0>(gates[5]);
        target.y = std::get<1>(gates[5]);
    }
    else {
        std::cout << "自动选择目标门：" << best_gate << "（得分：" << best_score << "）" << std::endl;
    }

    add_delta(best_gate, ball_y, target);

    //target.x = std::get<0>(gates[1]);
    //target.y = std::get<1>(gates[1]);
    return target;
}

void record_trajectory() {
    // 打开CSV文件（每次打开时清空内容，如果文件不存在则会创建）
    std::ofstream outfile("pingpong_trajectory.csv", std::ios::trunc); // trunc模式会覆盖文件内容
    if (!outfile.is_open()) {
        std::cerr << "无法打开文件！" << std::endl;
        return;
    }

    // 写入表头
    outfile << "time,x,y" << std::endl;

    // 初始化上一帧的位置
    Point last_ball_pos = { 0, 0, 0, 0, 0 };  // 假设 (0, 0) 位置为初始值，或上次记录的球的位置

    while (fabs(ballPos.x) < 0.1 && fabs(ballPos.y) < 0.1);

    while (true) {
        // 读取球的位置时加锁，避免数据竞争
        double ball_x, ball_y, time_stamp;
        {
            std::lock_guard<std::mutex> lock(mtx);  // 使用 std::lock_guard 自动管理锁
            ball_x = ballPos.x;
            ball_y = ballPos.y;
            time_stamp = ballPos.t;
        }

        // 检测球坐标是否有效
        if (fabs(ball_x) < 0.1 || fabs(ball_y) < 0.1) {
            std::cout << "球检测不到，停止记录。" << std::endl;
            break;  // 如果检测不到球，停止记录
        }

        // 检查球是否超出球桌范围
        if (fabs(ball_x) > TABLE_LENGTH - 300 || fabs(ball_y) > TABLE_WIDTH) {
            std::cout << "球超出球桌范围，停止记录。" << std::endl;
            break;  // 如果球超出球桌范围，停止记录
        }

        // 判断当前坐标和上一次坐标的变化，如果变化较小则不记录
        if (fabs(ball_x - last_ball_pos.x) < 0.5 && fabs(ball_y - last_ball_pos.y) < 0.5) {
            continue;  // 跳过本次记录
        }

        // 将时间戳和坐标写入文件
        outfile << time_stamp << "," << ball_x << "," << ball_y << std::endl;

        // 更新上一次坐标
        last_ball_pos.x = ball_x;
        last_ball_pos.y = ball_y;
        last_ball_pos.t = time_stamp;
    }

    outfile.close();  // 关闭文件
}


JAKAZuRobot demo;
MotorIK mkmotor;

int main()
{
    std::cout << "start main" << endl;

    // 启动一个线程运行 getVision()，用于持续接收球的位置数据
    std::cout << "创建视觉线程" << std::endl;
    std::thread t1(getVision);
    t1.detach(); // 分离线程，独立运行

    std::cout << "连接机器人中..." << std::endl;
    demo.login_in("192.168.1.10"); // 使用机器人LAN2口，其IP地址需要在图形化开发软件中进行设置。伺服控制需要用千兆网线连接。

    std::cout << "机器人连接完成，准备上电" << std::endl;
    demo.power_on();                // 本体上电
    demo.enable_robot();            // 电机使能

    std::cout << "初始化电机中..." << std::endl;
    
    mkmotor.ID = 1;               // 电机ID 为1
    mkmotor.reduceRatio = 10;     // 设置减速比
    mkmotor.setSerial(4, 115200); // 设置串口COM号，及波特率
    mkmotor.start();

    std::cout << "机械臂位置初始化..." << std::endl;
    JointValue jval, jvalH;
    jval.jVal[0] = 0 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 90 * Deg2Rad;
    jval.jVal[3] = 0 * Deg2Rad;
    jval.jVal[4] = 0 * Deg2Rad;
    jval.jVal[5] = 0 * Deg2Rad;

    errno_t ret = ERR_SUCC;
    ret = demo.joint_move(&jval, MoveMode::ABS, true, 5); // 请确认空间不会发生碰撞

    std::cout << "电机位置初始化..." << std::endl;
    double ini_dianji_degree = 180;
    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);

    int hit_round = 1;

    while (1)
    {
        std::cout << std::endl << "======== 第 " << hit_round << " 次击球 ========" << std::endl;
        std::cout << "请准备好球，按下回车键开始击球..." << std::endl;
        //if (hit_round == 1) std::cin.ignore();
        std::string dummy;
        //std::getline(std::cin, dummy);
        std::cin >> dummy;
        std::cout << "收到信号，准备击球..." << std::endl;

        CartesianPose pos;
        double goA = 0;

        double ball_x, ball_y;
        mtx.lock();
        ball_x = ballPos.x;
        ball_y = ballPos.y;
        mtx.unlock();

        std::printf("-Ball POS=<%9.2f,%9.2f> ", ball_x, ball_y);
        if (fabs(ball_x) < 0.01 && fabs(ball_y) < 0.01)
        {
            std::cout << "未检测到球，跳过本次击球" << std::endl;
            continue;
        }

        Point target = choice(ball_x, ball_y);
        std::printf("-Target POS=<%9.2f,%9.2f>\n", target.x, target.y);

        goA = goAvalue(ballPos.x, ballPos.y, target.x, target.y);

        pos.tran.x = ball_x + BALL_DELTA_X;
        pos.tran.y = ball_y + BALL_DELTA_Y;
        pos.tran.z = ball_y > 0 ? HEIGHT_LEFT : HEIGHT_RIGHT;

        if (ball_y > 0) {
            pos.rpy.rx = (-90 + goA) * Deg2Rad;
            pos.rpy.ry = -90 * Deg2Rad;
            pos.rpy.rz = 0;
        }
        else {
            pos.rpy.rx = (90 - goA) * Deg2Rad;
            pos.rpy.ry = 90 * Deg2Rad;
            pos.rpy.rz = 0;
        }

        double ini_angle = 90;

        try {
            demo.kine_inverse(&jval, &pos, &jvalH);
        }
        catch (std::runtime_error) {
            std::cout << "inverse error，跳过本次击球" << std::endl;
            continue;
        }

        mkmotor.goToPos2(ini_angle, 1, 200, false);
        demo.joint_move(&jvalH, MoveMode::ABS, true, 1);

        Sleep(1000);
        double speed = 900;
        mkmotor.goToPos2((ball_y > 0) ? (ini_angle - 180) : (ini_angle + 180), 1, speed, false);

        //record_trajectory();
        //std::cout << "轨迹记录完成" << std::endl;

        Sleep(1000);

        demo.joint_move(&jval, MoveMode::ABS, true, 7);
        mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);

        std::cout << "本轮击球完成" << std::endl << std::endl;
        hit_round++;
    }
    return 0;
}


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
                //std::cout << ball.x();
                // 更新球的位置
                for (int i = 0; i < balls_n; i++)
                {
                    SSL_DetectionBall ball = detection.balls(i);
                    Point pp;
                    pp.x = ball.x(); // 调整 x 位置，减去 20 个单位
                    pp.y = ball.y();
                    // pp.t = (detection.t_capture() / 10 - (int)(detection.t_capture() / 10)); // 计算时间信息
                    pp.t = detection.t_capture(); // 使用完整的时间戳
                    mtx.lock();                   // 加锁以确保线程安全
                    ballPos = pp;                 // 更新全局的球位置
                    mtx.unlock();                 // 解锁

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


