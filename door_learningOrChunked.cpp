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
#define ADDANGLE 120
#include <cmath>
#include <vector>
#include <fstream>
#include <complex>
#include <conio.h> // Windows specific

#include <mmsystem.h>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
JAKAZuRobot demo;
MotorIK mkmotor;

// 日志打印函数
enum log_class { Info, Error, Hint, Move };
static void logger(const log_class l)
{
    switch (l)
    {
    case Info:
        std::cout << "\t[info]  "; break;
    case Error:
        std::cout << "\t[error] "; break;
    case Hint:
        std::cout << "\t[hint]\n"; break;
    case Move:
        std::cout << "\t[move]  "; break;
    }
}

// 带日志与提示的输入
template <typename T>
void std_in(T& variable, const char* hint = nullptr)
{
    if (hint != nullptr)
        std::cout << "\t[hint]\n" << hint << std::endl;

    std::cin >> variable;
    std::cout << "\t[input] " << variable << std::endl;
}

struct Point
{
    double x;
    double y;
    double t;  // time
    double vx; // x方向速度
    double vy; // y方向速度
};
struct AffineTransform
{
    double scale_x = 1.0;  // x方向缩放
    double scale_y = 1.0;  // y方向缩放
    double offset_x = 0.0; // x方向偏移
    double offset_y = 0.0; // y方向偏移
    bool valid = false;    // 是否已初始化
} transform_correction;

// 添加目标点变换函数
Point transformTarget(const double target_x, const double target_y, const AffineTransform& transform)
{
    Point transformed;
    // 应用仿射变换
    transformed.x = target_x * transform.scale_x + transform.offset_x;
    transformed.y = target_y * transform.scale_y + transform.offset_y;
    return transformed;
}

// 修改的返回类型和名称，包括球的离桌速度、角度、实际落点，以及数据是否有效
struct BallTrajectoryInfo
{
    double exit_speed;    // 离桌速度
    double exit_angle;    // 离桌角度
    Point exit_point; // 实际落点
    bool valid;           // 数据是否有效
};

// 全局变量，用于存储球的位置和管理线程
Point ball_pos;         // 球的位置
std::mutex mtx;        // 互斥锁，用于保护共享资源的访问
std::ofstream outfile; // 输出文件流，用于保存球的数据

map<int, std::pair<double, double>> door_position = {
    {1, {2740, -720}},
    {2, {2740, 720}}
};
int door_index;
int chunk_index = -1;

std::map<int, int> hit_score; // 第几轮击球和每轮的得分

// 调试设置
struct Config {
    bool if_theoretic_test = false;      // 是否进行理论测试
    const char* ip = "192.168.1.10";
    bool if_prepare = false;             // 是否有准备动作
    bool if_fake_ball_position = false; // 是否使用假球位置
    int com = 9;
	int speed = 900; // 速度
    bool if_self_correction = false;
	bool if_chunked = true; // 是否分块处理
	// 0: 0-170, 1:170-340, 2: 340-500, 3: 0--170, 4: 170--340, 5: 340--500
	double y_offset[6] = { 0, -20, 0, 40, 70, 90 }; // y方向偏移
	double x_offset[6] = { 0, 0, 0, 0, 0, 0 }; // x方向偏移
} config;

void getVision();
void getVision_fake();
double getdistance(double ball_x, double ball_y, double posi_x, double posi_y);


// 分析本次击球信息：球拍速度、出球速度、落桌速度、桌面滑动距离
BallTrajectoryInfo get2Speed(double ball_x)
{
    logger(Info);
    cout << "Trajectory analysis begins. x = " << ball_x << endl;
    vector<Point> outball, fallball;
    BallTrajectoryInfo result = { 0, 0, {0, 0, 0, 0, 0}, false };
    Point last_saved_point = { 0, 0, 0, 0, 0 }; // 用于记录上一次保存的点

    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();
    const double MAX_COLLECTION_TIME = 2.0; // 最大收集时间（秒）
    const double MIN_CHANGE = 1.5;          // 最小位置变化阈值
    const double MIN_POINTS = 3;            // 最小有效点数
    const double MAX_SPEED = 10000.0;       // 最大合理速度 (mm/s)
    const int MASK = 2300;

    // 数据收集阶段
    while (ball_pos.x < MASK)
    {
        Point current_pos = ball_pos; // 创建当前位置的副本

        double position_change = sqrt(
            pow(current_pos.x - last_saved_point.x, 2) +
            pow(current_pos.y - last_saved_point.y, 2));

        // 收集数据的条件判断
        // 刚出球
        bool should_save_outball = (current_pos.x > ball_x + 10 &&
            current_pos.x <= ball_x + 150);
        // 准备落桌
        bool should_save_fallball = (current_pos.x > MASK - 300 && // 调整范围
            current_pos.x < MASK - 10);

        if (position_change > MIN_CHANGE)
        {
            if (should_save_outball)
            {
                logger(Info);
                cout << "Ball position: (" << current_pos.x << ", " << current_pos.y
                    << ") when " << current_pos.t << endl;
                outball.push_back(current_pos);
                last_saved_point = current_pos;
            }
            else if (should_save_fallball)
            {
                logger(Info);
                cout << "Ball position: (" << current_pos.x << ", " << current_pos.y
                    << ") when " << current_pos.t << endl;
                fallball.push_back(current_pos);
                last_saved_point = current_pos;
            }
        }

        // 检查时间限制
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed > MAX_COLLECTION_TIME)
        {
            logger(Info);
            cout << "Reached time limit " << MAX_COLLECTION_TIME << " s. Collection ends.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 数据处理阶段
    if (fallball.size() >= MIN_POINTS)
    {
        // 只使用最后几个点计算速度
        const int LAST_POINTS_COUNT = 3; // 使用最后3个点
        vector<double> finalSpeeds;      // 存储最后几段的速度

        // 计算最后几个点的速度
        int startIdx = max(0, (int)fallball.size() - LAST_POINTS_COUNT - 1);
        double totalVx = 0, totalVy = 0;
        int validCount = 0;

        for (size_t i = startIdx + 1; i < fallball.size(); ++i)
        {
            double deltaX = fallball[i].x - fallball[i - 1].x;
            double deltaY = fallball[i].y - fallball[i - 1].y;
            double deltaT = fallball[i].t - fallball[i - 1].t;

            // 验证速度的合理性
            if (deltaT > 0.001) // 避免除以零
            {
                double vx = deltaX / deltaT;
                double vy = deltaY / deltaT;

                // 速度合理性检查
                if (abs(vx) < MAX_SPEED && abs(vy) < MAX_SPEED)
                {
                    // 使用线性加权，越靠近落桌点的速度权重越大
                    double weight = (i - startIdx) / (double)(fallball.size() - startIdx - 1);
                    totalVx += vx * weight;
                    totalVy += vy * weight;
                    validCount++;
                }
            }
        }

        if (validCount > 0)
        {
            // 计算加权平均速度
            double avgSpeedX = totalVx / validCount;
            double avgSpeedY = totalVy / validCount;

            // 考虑摩擦力导致的速度衰减
            const double FRICTION_FACTOR = 1.0; // 根据实际情况调整
            result.exit_speed = sqrt(avgSpeedX * avgSpeedX + avgSpeedY * avgSpeedY) * FRICTION_FACTOR;
            result.exit_angle = atan2(avgSpeedY, avgSpeedX) * Rad2Deg;

            // 计算离桌点
            Point last_point = fallball.back();
            double exit_y = last_point.y + (door_position[door_index].second - last_point.y) *
                (2740 - last_point.x) / ((door_position[door_index].first + 2740) - last_point.x);

            // 输出详细的速度分析
            logger(Info);
            cout << "Calculation using last " << validCount << " points: " << "Original speed is (" << avgSpeedX << ", "
                << avgSpeedY << ") mm/s, speed considering fiction is " << result.exit_speed << " mm/s with direction at "
                << result.exit_angle << " degrees." << std::endl;

			result.exit_point.x = 2740; // 固定出口点x坐标
			result.exit_point.y = exit_y; // 计算出口点y坐标
            result.valid = true;

            //轨迹分析结果
            logger(Info);
            std::cout << "Calculation: Last observation point: (" << last_point.x << ", " << last_point.y << "), "
                << "fitted exit point: (2740, " << exit_y << "), "
                << "exit speed: " << result.exit_speed << " mm/s, "
                << "exit angle: " << result.exit_angle << " degrees."
                << "\n";
        }
    }
    else
    {
        logger(Error);
        cout << "No sufficient data.\n";
    }

    return result;
}

// 添加全局计数器来跟踪击球次数
struct CalibrationState
{
    int hit_count = 0;
    bool in_game = false; // 标记是否进入比赛阶段
} calib_state;

// 获取动态学习率
double getDynamicAlpha()
{
    switch (calib_state.hit_count)
    {
    case 0: // 第一次击球
        return 0.7;
    case 1: // 第二次击球
        return 0.5;
    default:
        // if (calib_state.hit_count >= 8) {  // 第8次及以后
        //     return 0.1;
        // } else if (calib_state.hit_count >= 3) {  // 第3-7次
        //     return 0.3;
        // }
        return 0.3; // 默认值
    }
}//击球策略，次数不同，瞄准的目标不同


// 修改校准函数，添加x方向的谨慎修正
void analyzeAndUpdateCalibration(const BallTrajectoryInfo& trajectory, bool is_left_side)//校准函数
{
    // 限制调整量
    const double MAX_ADJUSTMENT_Y = 80.0; // y方向最大调整
    const double MAX_ADJUSTMENT_X = 30.0; // x方向最大调整
    if (trajectory.valid)
    {
        // 计算与目标点的实际误差
        double actual_error_x = trajectory.exit_point.x - door_position[door_index].first;
        double actual_error_y = trajectory.exit_point.y - door_position[door_index].second;

        // 打印调试信息
        logger(Info);
        printf("Calibration information updated. Target position: (%.2f, %.2f); Error value: (%.2f, %.2f).\n",
            door_position[door_index].first, door_position[door_index].second, actual_error_x, actual_error_y);

        // 如果是比赛阶段且得分，跳过校准
        if (calib_state.hit_count >= 2 && hit_score[calib_state.hit_count + 1] > 0)
        {
            logger(Info);
            std::cout << "Scored.\n";
            calib_state.hit_count++;
            return;
        }

        const double MAX_REASONABLE_ERROR = 300.0; // 最大合理误差
        if (abs(actual_error_y) >= MAX_REASONABLE_ERROR || abs(actual_error_x) >= MAX_REASONABLE_ERROR)
        {
            logger(Error);
            std::cout << "Too large error (" << actual_error_x << "mm, " << actual_error_y << "mm).\n";
            return;
        }

        double alpha = getDynamicAlpha(); // 获取动态学习率
        double alpha_x = alpha;           // x方向使用更小的学习率

        // 更新仿射变换参数
        if (!transform_correction.valid)
        {
            transform_correction.scale_x = 1.0;
            transform_correction.scale_y = 1.0;
            transform_correction.offset_y = -actual_error_y * alpha;
            transform_correction.offset_x = -actual_error_x * alpha_x;
            if (abs(transform_correction.offset_x) > MAX_ADJUSTMENT_X)
            {
                transform_correction.offset_x = (transform_correction.offset_x > 0 ? MAX_ADJUSTMENT_X : -MAX_ADJUSTMENT_X);
            }
            if (abs(transform_correction.offset_y) > MAX_ADJUSTMENT_Y)
            {
                transform_correction.offset_y = (transform_correction.offset_y > 0 ? MAX_ADJUSTMENT_Y : -MAX_ADJUSTMENT_Y);
            }
            transform_correction.valid = true;

            logger(Info);
            std::cout << "Initialization: offset_y = " << transform_correction.offset_y
                << ", offset_x = " << transform_correction.offset_x << ".\n";
        }
        else
        {

            // 记录旧的偏移量
            double old_offset_y = transform_correction.offset_y;
            double old_offset_x = transform_correction.offset_x;

            // 计算y方向调整量
            double adjustment_y = -actual_error_y * alpha;

            // 计算x方向调整量（只在误差较大且连续时才调整）
            double adjustment_x = 0;
            static int consistent_x_error_count = 0;
            static double last_error_x = 0;

            // 检查x方向误差的一致性
            if (abs(actual_error_x) > 50) // 误差大于限额
            {
                adjustment_x = -actual_error_x * alpha_x;
            }
            else if (abs(actual_error_x) > 10 &&                  // 误差大于限额
                ((actual_error_x > 0 && last_error_x > 0) || // 连续正误差
                    (actual_error_x < 0 && last_error_x < 0)))  // 连续负误差
            {
                consistent_x_error_count++;
                if (consistent_x_error_count >= 2) // 连续多次同向误差
                {
                    adjustment_x = -actual_error_x * alpha_x;
                }
            }
            else
            {
                consistent_x_error_count = 0;
            }
            last_error_x = actual_error_x;

            if (abs(adjustment_y) > MAX_ADJUSTMENT_Y)
            {
                adjustment_y = (adjustment_y > 0 ? MAX_ADJUSTMENT_Y : -MAX_ADJUSTMENT_Y);
            }
            if (abs(adjustment_x) > MAX_ADJUSTMENT_X)
            {
                adjustment_x = (adjustment_x > 0 ? MAX_ADJUSTMENT_X : -MAX_ADJUSTMENT_X);
            }

            // 应用调整
            transform_correction.offset_y = transform_correction.offset_y + adjustment_y;
            transform_correction.offset_x = transform_correction.offset_x + adjustment_x;

            // 限制总偏移量
            const double MAX_TOTAL_OFFSET_Y = 100.0; // y方向最大总偏移100mm
            const double MAX_TOTAL_OFFSET_X = 100.0; // x方向最大总偏移100mm

            transform_correction.offset_y = max(-MAX_TOTAL_OFFSET_Y,
                min(MAX_TOTAL_OFFSET_Y, transform_correction.offset_y));
            transform_correction.offset_x = max(-MAX_TOTAL_OFFSET_X,
                min(MAX_TOTAL_OFFSET_X, transform_correction.offset_x));

            cout << "Parameters updated: "
                << "Learning rate: Y=" << alpha << ", X=" << alpha_x << "; "
                << "Continuing X error: " << consistent_x_error_count << " times; "
                << "Last correction: Y=" << old_offset_y << ", X=" << old_offset_x << "; "
                << "New correction: Y=" << transform_correction.offset_y
                << ", X=" << transform_correction.offset_x << ".\n";
        }

        // 增加击球计数
        calib_state.hit_count++;
    }
    else
    {
        logger(Error);
        std::cout << "No valid data.\n";
    }
}


void prepare() {

    JointValue jval, jvalH;
    errno_t ret = ERR_SUCC;
    //ret = demo.joint_move(&jval, MoveMode::ABS, true, 5, 20, 1, NULL); // 请确认空间不会发生碰撞


    jval.jVal[0] = -30 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 0 * Deg2Rad;
    jval.jVal[3] = 90 * Deg2Rad;
    jval.jVal[4] = 45 * Deg2Rad;

    jval.jVal[5] = 0 * Deg2Rad;

	logger(Move);
    std::cout << "Jaka mini preparing. (No.1_1)" << std::endl;
    ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞

    Sleep(1);

    jval.jVal[0] = 70 * Deg2Rad;
    jval.jVal[1] = 15 * Deg2Rad;
    jval.jVal[2] = 30 * Deg2Rad;
    jval.jVal[3] = 30 * Deg2Rad;
    jval.jVal[4] = 40 * Deg2Rad;

    jval.jVal[5] = -20 * Deg2Rad;

    logger(Move);
    std::cout << "Jaka mini preparing. (No.1_2)" << std::endl;
    ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞

    Sleep(1);


}


// 计算球的运行轨迹的角度
double goAvalue(const double ball_x, const double ball_y, const double posi_x, const double posi_y)
{
    const double run_x = posi_x - ball_x;
    const double run_y = posi_y - ball_y;

    const double angle_rad = atan2(run_y, run_x);
    const double angle_deg = angle_rad * Rad2Deg;
    logger(Info);
    std::cout << "Hitting angle: " << angle_deg << "\n";

    return angle_deg;
}

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
                    pp.x = ball.x() - 20; // 调整 x 位置，减去 20 个单位
                    pp.y = ball.y();
                    // pp.t = (detection.t_capture() / 10 - (int)(detection.t_capture() / 10)); // 计算时间信息
                    pp.t = detection.t_capture(); // 使用完整的时间戳
                    mtx.lock();                   // 加锁以确保线程安全
                    ball_pos = pp;                 // 更新全局的球位置
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

// 假的视觉数据：一个固定的位置
void getVision_fake()
{
    RoboCupSSLClient client;
    client.open(true); // 打开客户端连接
    SSL_WrapperPacket packet;
    std::vector<Point> ballP; // 存储球的位置历史记录

    while (true)
    {
        if (true)
        {
            if (true)
            { // 检查是否有检测到的数据
                SSL_DetectionFrame detection = packet.detection();
                int balls_n = 1; // 获取检测到的球的数量
                //std::cout << ball.x();
                // 更新球的位置
                for (int i = 0; i < balls_n; i++)
                {

                    Point pp;
                    pp.x = 100; // 调整 x 位置，减去 20 个单位
                    pp.y = -600;
                    // pp.t = (detection.t_capture() / 10 - (int)(detection.t_capture() / 10)); // 计算时间信息
                    pp.t = detection.t_capture(); // 使用完整的时间戳
                    mtx.lock();                   // 加锁以确保线程安全
                    ball_pos = pp;                 // 更新全局的球位置
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

int main()
{
    logger(Info);
    std::cout << "main starts." << endl;

    // 独立线程以持续获取球的位置信息
    if (config.if_fake_ball_position) {
        std::thread t1(getVision_fake);
        t1.detach(); // 分离线程，独立运行
        logger(Info);
        std::cout << "Using fake ball position (" << ball_pos.x << ", " << ball_pos.y << ")." << std::endl;
    }
    else {
        std::thread t1(getVision);
        t1.detach(); // 分离线程，独立运行
        logger(Info);
        std::cout << "Vision system connected." << std::endl;
    }

    demo.login_in(config.ip);
    logger(Info);
    std::cout << "Jaka mini connected." << std::endl;
    demo.power_on();                // 本体上电
    demo.enable_robot();            // 电机使能
    logger(Info);
    std::cout << "Jaka mini powered." << std::endl;

    //末端电机初始化
    logger(Info);
    std::cout << "Motor set." << std::endl;

    mkmotor.ID = 1;               // 电机ID 为1
    mkmotor.reduceRatio = 10;     // 设置减速比
    mkmotor.setSerial(config.com, 115200); // 设置串口COM号，及波特率
    mkmotor.start();

    // 确定机械臂的初始位置
    JointValue jval, jvalH;
    logger(Move);
    std::cout << "Jaka mini initializing." << std::endl;
    // 确定位置
    jval.jVal[0] = 0 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 90 * Deg2Rad;
    jval.jVal[3] = 0 * Deg2Rad;
    jval.jVal[4] = 0 * Deg2Rad;
    jval.jVal[5] = 0 * Deg2Rad;
    errno_t ret = ERR_SUCC;
    ret = demo.joint_move(&jval, MoveMode::ABS, true, 5); // 一般都是ABS（绝对运动模式）

    // 电机初始化
    logger(Move);
    std::cout << "Motor initializing." << std::endl;
    double ini_dianji_degree = jval.jVal[5] * Rad2Deg + 180;//180
    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false); // angle, direction, speed, check

    double goA = -10;




    int count = 0;
    while (true)
    {
        count++;

        logger(Hint);
        std::cout << "Pre ENTER to begin..." << std::endl;
        char ch = std::cin.get();

        if (ch == '\n') {
            // 用户按下了 Enter 键
            std::cout << "[input] ENTER" << std::endl;


            

            double record_x;
            

            if (ball_pos.y > 0) // 位置在左侧的球
            {
                if (ball_pos.y < 170) chunk_index = 0; // 0-170
                else if (ball_pos.y < 340) chunk_index = 1; // 170-340
				else chunk_index = 2; // 340-500 
				door_index = 1; // 左侧门
                logger(Info);
                std::cout << "Target No." << 9 << std::endl;
                CartesianPose pos;
                mtx.lock();
                double initial_ball_x = ball_pos.x; // 在这里声明和初始化
                double initial_ball_y = ball_pos.y;
                logger(Info);
                std::printf("Ball position: (%9.2f,%9.2f) \n", ball_pos.x, ball_pos.y);
                pos.tran.x = initial_ball_x - 7;
                pos.tran.y = initial_ball_y + 7;
                mtx.unlock();
                pos.tran.z = 240;
                Point target;

                if (transform_correction.valid) ///// 什么时候 valid
                {
                    target = transformTarget(door_position[1].first, door_position[1].second, transform_correction);
                    ///// 为什么要 transformTarget
                    logger(Info);
                    std::cout << "Target position transformed from (" << door_position[1].first << ", " << door_position[1].second << ") ";
                    std::cout << "to (" << target.x << ", " << target.y << ").\n";
                }
                else
                {
                    target.x = door_position[1].first + config.x_offset[chunk_index];
                    target.y = door_position[1].second + config.y_offset[chunk_index];
                    logger(Info);
                    std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
                }

                goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
				pos.tran.x -= 20 * cos(goA * Deg2Rad);
				pos.tran.y -= 20 * sin(goA * Deg2Rad);

                // std::cout << goA << std::endl;
                pos.rpy.rx = (-90 + goA) * Deg2Rad; // 改为加号
                pos.rpy.ry = -90 * Deg2Rad;
                pos.rpy.rz = 0;
                double ini_angle = 360;
                // 控制机械臂移动到球上方
                try {
                    ret = demo.kine_inverse(&jval, &pos, &jvalH);
                }
                catch (std::runtime_error) {
					logger(Error);
                    std::cout << "Inverse error!" << std::endl;
                    logger(Info);
                    std::cout << "Jaka mini and motor returning to initial position." << std::endl;
                    ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
                    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
                    continue;
                }
                if (ret == 0)
                {
					logger(Move);
					std::cout << "Motor preparing." << std::endl;
                    ini_angle = 90;
                    mkmotor.goToPos2(ini_angle, 1, 200, false);
					logger(Move);
                    std::cout << "Jaka mini preparing." << std::endl;
                	demo.joint_move(&jvalH, MoveMode::ABS, true, 1);
                }
                else
                {
                    logger(Error);
                    std::cout << "Inverse error!" << std::endl;
                    logger(Info);
                    std::cout << "Jaka mini and motor returning to initial position." << std::endl;
                    ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
                    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
                    continue;
                }
                JointValue jvalT;
                demo.get_joint_position(&jvalT);
            	Sleep(1000);

                double speed = config.speed;

                logger(Move);
                std::cout << "Motor hitting." << std::endl;
                mkmotor.goToPos2((ini_angle - 180 - ADDANGLE), 1, speed, false);

                BallTrajectoryInfo trajectory;
                if (config.if_self_correction)
                {
                    logger(Info);
                    std::cout << "Recording trajectory.\n";
                    trajectory = get2Speed(initial_ball_x);
                }

                Sleep(1000);

                demo.get_joint_position(&jvalT);
                logger(Move);
                std::cout << "Jaka mini lifting." << std::endl;
            	jvalT.jVal[1] -= 50 * Deg2Rad;
                demo.joint_move(&jvalT, MoveMode::ABS, true, 1);

                if (config.if_self_correction)
                {
                    logger(Info);
                    std::cout << "Analyzing and updating calibration.\n";
                    analyzeAndUpdateCalibration(trajectory, true);
                }

                // 然后再返回等待位置
                Sleep(1000);
                logger(Move);
                std::cout << "Jaka mini and motor returning to initial position." << std::endl;
                ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
                mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
            }
            //如果球在右侧
            else
            {
                if (ball_pos.y > -170) chunk_index = 3; // 0-170
                else if (ball_pos.y > -340) chunk_index = 4; // 170-340
                else chunk_index = 5; // 340-500 
                door_index = 2;
                logger(Info);
                std::cout << "Target No." << 1 << std::endl;
                CartesianPose pos;
                mtx.lock();
                double initial_ball_x = ball_pos.x; // 在这里声明和初始化
                record_x = initial_ball_x;
                double initial_ball_y = ball_pos.y;
                logger(Info);
                std::printf("Ball position: (%.2f, %.2f) \n", initial_ball_x, initial_ball_y);
                pos.tran.x = initial_ball_x;
                pos.tran.y = initial_ball_y;
                mtx.unlock();
                pos.tran.z = 240;

                Point target;

                if (transform_correction.valid) ///// 什么时候 valid
                {
                    target = transformTarget(door_position[door_index].first, door_position[door_index].second, transform_correction);
                    ///// 为什么要 transformTarget
                    logger(Info);
                    std::cout << "Target position transformed from (" << door_position[door_index].first << ", " << door_position[door_index].second << ") ";
                    std::cout << "to (" << target.x << ", " << target.y << ").\n";
                }
                else
                {
                    target.x = door_position[door_index].first + config.x_offset[chunk_index];
                    target.y = door_position[door_index].second + config.y_offset[chunk_index];
                    logger(Info);
                    std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
                }

                goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
                pos.tran.x -= 20 * cos(goA * Deg2Rad);
                pos.tran.y -= 20 * sin(goA * Deg2Rad);
                
                pos.rpy.rx = (90 - goA) * Deg2Rad; // 改为减号
                pos.rpy.ry = 90 * Deg2Rad;
                pos.rpy.rz = 0;
                double ini_angle = 0;

                // 控制机械臂移动到球上方
                try {
                    ret = demo.kine_inverse(&jval, &pos, &jvalH);
                }
                catch (std::runtime_error) {
                    logger(Error);
                    std::cout << "Inverse error!" << std::endl;
                    logger(Info);
                    std::cout << "Jaka mini and motor returning to initial position." << std::endl;
                    ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
                    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
                    continue;
                }
                if (ret == 0)
                {
                    ini_angle = 90;
                    logger(Move);
                    std::cout << "Motor preparing." << std::endl;
                    mkmotor.goToPos2(ini_angle, 1, 200, false);
                    logger(Move);
                    std::cout << "Jaka mini preparing." << std::endl;
                    demo.joint_move(&jvalH, MoveMode::ABS, true, 1);
                    // Sleep(30000);
                }
                else
                {
                    logger(Error);
                    std::cout << "Inverse error!" << std::endl;
                    logger(Info);
                    std::cout << "Jaka mini and motor returning to initial position." << std::endl;
                    ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
                    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
                    continue;
                }
                JointValue jvalT;
                demo.get_joint_position(&jvalT);
                Sleep(1000);

                double speed = config.speed;

                logger(Move);
                std::cout << "Motor hitting." << std::endl;
            	mkmotor.goToPos2((ini_angle + 180 + ADDANGLE), 1, speed, false);

                BallTrajectoryInfo trajectory;
                if (config.if_self_correction)
                {
                    logger(Info);
                    std::cout << "Recording trajectory.\n";
                    trajectory = get2Speed(initial_ball_x);
                }

                Sleep(1000);

                demo.get_joint_position(&jvalT);
                jvalT.jVal[1] -= 50 * Deg2Rad;

            	logger(Move);
                std::cout << "Jaka mini lifting." << std::endl;
                demo.joint_move(&jvalT, MoveMode::ABS, true, 1);

                if (config.if_self_correction)
                {
                    logger(Info);
                    std::cout << "Analyzing and updating calibration.\n";
                    analyzeAndUpdateCalibration(trajectory, true);
                }

                // 然后再返回等待位置
                Sleep(1000);

                logger(Move);
                std::cout << "Jaka mini and motor returning to initial position." << std::endl;
                ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
                mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
            }
        }
        else {
            // 其他按键，退出循环
			std::cout << "[input] " << ch << std::endl;
			logger(Error);
			std::cout << "Invalid input. Please press ENTER to continue." << std::endl;

            count--;
        	continue;
        }
    }
    logger(Info);
        std::cout << "Hitting end." << std::endl;

    return 0;
}