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

constexpr auto pi222 = 3.1415926;  // pi 的值;
#define Deg2Rad (pi222 / 180)//用来将角度值转化为弧度制
#define Rad2Deg (180 / pi222)//用来将弧度制转化为角度值

#include <cmath>
#include <vector>
#include <fstream>
#include <complex>
#include <conio.h> // Windows specific

#include <mmsystem.h>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

constexpr auto ADDANGLE = 120;
//站在击球点看目标是x，击球点右侧是y轴负半轴，左半轴是y轴正半轴
// 存储函数：输入为第几轮击球和总得分，要那一轮的得分，存到一个字典里。
// 策略函数：输入的是存储函数的结果，以及静态击球的球坐标，输出这一轮的目标桶序号。
// 计算函数：输入的是目标桶序号，球的坐标，输出角度和速度，以及位置
// 击球函数：传入上面三个参数

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

// 点的定义：x, y, t, vx, vy
struct Point
{
    double x;
    double y;
    double t;  // time
    double vx; // x方向速度
    double vy; // y方向速度
};

// 调试设置
struct Config {
    bool if_theoretic_test = false;      // 是否进行理论测试
    const char* ip = "192.168.1.112";
    bool if_prepare = false;             // 是否有准备动作
    bool if_fake_ball_position = false; // 是否使用假球位置
    bool if_self_correction = false;     // 是否使用自我校正
    int com = 3;
    double v = 0.157;
    int alpha_group = 3;
    bool if_chunked = true;
    double x_offset[9] = { -50, 0, 0,-50, -60, 0, 0, 30, 0 };
    double y_offset[9] = { -150, -200, -140, 130, -595, -150, -140, -140, -125 };
} config;


double alpha1, alpha2, alpha3;

void alpha_group(int alpha_group)
{
    switch (alpha_group) {
    case 0:
        alpha1 = 0.8;
        alpha2 = 0.7;
        alpha3 = 0.3;
    case 1:
        alpha1 = 0.6;
        alpha2 = 0.5;
        alpha3 = 0.2;
    case 2:
        alpha1 = 0.5;
        alpha2 = 0.4;
        alpha3 = 0.1;
    case 3:
        alpha1 = alpha2 = alpha3 = 0.5;
    }
};

// 加速系数计算函数
double calculateAccFactor(double ball_y)
{
    if (ball_y > 0)
    {
        if (ball_y < 200)
            return 1.054;
        if (ball_y <= 400)
            return 1.057;
        return 1.054;
    }
    else
    {
        if (ball_y > -200)
            return 1.058;
        if (ball_y >= -400)
            return 1.061;
        return 1.058;
    }
}

Point ball_pos;         // 球的位置
std::mutex mtx;        // 互斥锁，用于保护共享资源的访问
std::ofstream outfile; // 输出文件流，用于保存球的数据

// 函数声明
char cmd = 'H';
void getVision();  // 得到球的坐标
void getVision_fake();  // 特定击球点，如果视觉系统未连接则可以使用这个
void special();
double goAvalue(double ball_x, double ball_y, double posi_x, double posi_y = 0);
double getdistance(double ball_x, double ball_y, double posi_x, double posi_y);
double speedX(double ball_x, double ball_y, double posi_x, double posi_y);
double calculateAccFactor(double ball_y);
double calculateRequiredExitSpeed(double start_x, double start_y, double exit_x, double exit_y, double target_x, double target_y);
Point predictLandingPoint(double ball_x, double ball_y, double exit_speed, double exit_angle, double exit_height = 660);
int findNearestBucket(double ball_y);  // 找到最近的桶

std::ofstream debug_log("debug_log.csv", std::ios::app);  // debug 日志

// 添加全局变量来存储上一次的实际数据
struct HitResult
{
    double exit_speed;
    double exit_angle;
    double landing_x;
    double landing_y;
    bool valid;
} last_hit_result = { 0, 0, 0, 0, false };

// 修改物理参数常量，统一使用毫米(mm)、秒(s)、千克(kg)作为基本单位
constexpr double GRAVITY = 9796.7; // mm/s^2   重力

// 添加仿射变换结构体
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
    Point actual_landing; // 实际落点
    bool valid;           // 数据是否有效
};

int bucket_index;
double bucket_position1_fixed, bucket_position2_fixed;

std::map<int, int> hit_score; // 第几轮击球和每轮的得分

// 桶的位置
std::map<int, std::pair<double, double>> bucket_position = {
    {1, {3020, 560}},
    {2, {3020, 0}},
    {3, {3020, -560}},
    {4, {3380, 250}},
    {5, {3380, -250}},
    {6, {3620, 0}}
}; // 目标桶的序号和他的坐标    

BallTrajectoryInfo get2Speed(double ball_x);
// 添加全局计数器来跟踪击球次数
struct CalibrationState
{
    int hit_count = 0;
    bool in_game = false; // 标记是否进入比赛阶段
} calib_state;

// 获取动态学习率
double getDynamicAlpha()
{
    alpha_group(config.alpha_group);
    switch (calib_state.hit_count)
    {
    case 0: // 第一次击球
        return alpha1;
    case 1: // 第二次击球
        return alpha2;
    default:
        return alpha3; // 默认值
    }
}//击球策略，次数不同，瞄准的目标不同

// 修改校准函数，添加x方向的谨慎修正
void analyzeAndUpdateCalibration(const BallTrajectoryInfo& trajectory, bool is_left_side)//校准函数
{
    // 限制调整量
    const double MAX_ADJUSTMENT_Y = 50.0; // y方向最大调整
    const double MAX_ADJUSTMENT_X = 30.0; // x方向最大调整
    if (trajectory.valid)
    {
        // 计算与目标点的实际误差
        double actual_error_x = trajectory.actual_landing.x - bucket_position[bucket_index].first;
        double actual_error_y = trajectory.actual_landing.y - bucket_position[bucket_index].second;

        // 打印调试信息
        logger(Info);
        printf("calibration information updated. Target position: (%.2f, %.2f); Error value: (%.2f, %.2f).\n",
            bucket_position[bucket_index].first, bucket_position[bucket_index].second, actual_error_x, actual_error_y);

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
            transform_correction.offset_x = 0;
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
            transform_correction.offset_x = 0;

            // 限制总偏移量
            const double MAX_TOTAL_OFFSET_Y = 100.0; // y方向最大总偏移100mm
            const double MAX_TOTAL_OFFSET_X = 100.0; // x方向最大总偏移100mm

            transform_correction.offset_y = max(-MAX_TOTAL_OFFSET_Y,
                min(MAX_TOTAL_OFFSET_Y, transform_correction.offset_y));
            transform_correction.offset_x = 0;

            cout << "Parameters updated: "
                << "Learning rate: Y=" << alpha << ", X=" << alpha_x << "; "
                << "Continuing X error: " << consistent_x_error_count << " times; "
                << "Last correction: Y=" << old_offset_y << ", X=" << old_offset_x << "; "
                << "New correction: Y=" << transform_correction.offset_y
                << ", X=" << transform_correction.offset_x << ".\n";
        }

        // 增加击球计数
        calib_state.hit_count++;

        // 记录到日志文件
        debug_log << "校准更新,"
            << calib_state.hit_count << ","
            << actual_error_x << ","
            << actual_error_y << ","
            << transform_correction.offset_x << ","
            << transform_correction.offset_y << ","
            << getDynamicAlpha() << "\n";
    }
    else
    {
        logger(Error);
        std::cout << "No valid data.\n";
    }
}

// Jaka Mini 机械臂控制类 和 电机控制类
JAKAZuRobot demo;
MotorIK mkmotor;



// 开始进洞的准备
void prepare() {
    JointValue jval, jvalH;
    errno_t ret = ERR_SUCC;
    //ret = demo.joint_move(&jval, MoveMode::ABS, true, 5, 20, 1, NULL); // 请确认空间不会发生碰撞


    if (bucket_index == 1 || bucket_index == 7 || bucket_index == 10) {
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

        ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞
        logger(Move);
        std::cout << "Jaka mini preparing. (No.1_2)" << std::endl;

        Sleep(1);
    }
    if (bucket_index == 2 || bucket_index == 5 || bucket_index == 9 || bucket_index == 4) {
        jval.jVal[0] = 0 * Deg2Rad;
        jval.jVal[1] = -50 * Deg2Rad;
        jval.jVal[2] = 40 * Deg2Rad;
        jval.jVal[3] = 0 * Deg2Rad;
        jval.jVal[4] = 0 * Deg2Rad;

        jval.jVal[5] = 0 * Deg2Rad;

        ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞
        logger(Move);
        std::cout << "Jaka mini preparing. (No.2_1)" << std::endl;

        Sleep(1);

        jval.jVal[0] = 11 * Deg2Rad;
        jval.jVal[1] = -45 * Deg2Rad;
        jval.jVal[2] = 80 * Deg2Rad;
        jval.jVal[3] = 0 * Deg2Rad;
        jval.jVal[4] = 0 * Deg2Rad;

        jval.jVal[5] = 0 * Deg2Rad;

        ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞
        logger(Move);
        std::cout << "Jaka mini preparing. (No.2_2)" << std::endl;

        Sleep(1);
    }
    if (bucket_index == 3 || bucket_index == 6 || bucket_index == 8) {
        jval.jVal[0] = 10 * Deg2Rad;
        jval.jVal[1] = -70 * Deg2Rad;
        jval.jVal[2] = 0 * Deg2Rad;
        jval.jVal[3] = 40 * Deg2Rad;
        jval.jVal[4] = 0 * Deg2Rad;

        jval.jVal[5] = 0 * Deg2Rad;

        ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞
        logger(Move);
        std::cout << "Jaka mini preparing. (No.3_1)" << std::endl;

        Sleep(1);

        jval.jVal[0] = 11 * Deg2Rad;
        jval.jVal[1] = -45 * Deg2Rad;
        jval.jVal[2] = 80 * Deg2Rad;
        jval.jVal[3] = 0 * Deg2Rad;
        jval.jVal[4] = 0 * Deg2Rad;

        jval.jVal[5] = 0 * Deg2Rad;

        ret = demo.joint_move(&jval, MoveMode::ABS, true, 2, 5, 1, NULL); // 请确认空间不会发生碰撞
        logger(Move);
        std::cout << "Jaka mini preparing. (No.3_2)" << std::endl;

        Sleep(1);
    }
}



// 主函数
int main()
{
    if (config.if_theoretic_test)
    {
        logger(Info);
        std::cout << "Theoretic test mode enabled." << std::endl;

        while (true)
        {
            std_in<double>(ball_pos.x, "Please enter the ball's x position (mm).");
            std_in<double>(ball_pos.y, "Please enter the ball's y position (mm).");

            // 输入击打目标
            while (true) {
                std_in<int>(bucket_index, "Please enter your target (1-6).");

                if (bucket_index >= 1 && bucket_index <= 10)
                    break;
                else {
                    logger(Error);
                    std::cout << "Invalid Input." << std::endl;
                }
            }

            logger(Info);
            std::cout << "Target No." << bucket_index << std::endl;
            logger(Info);
            std::printf("Ball position: (%.2f, %.2f) \n", ball_pos.x, ball_pos.y);
            logger(Info);
            std::printf("Target position: (%.2f, %.2f) \n", bucket_position[bucket_index].first, bucket_position[bucket_index].second);

            Point target;
            target.x = bucket_position[bucket_index].first;
            target.y = bucket_position[bucket_index].second;

            // 决定速度
            double speed = speedX(ball_pos.x, ball_pos.y, target.x, target.y);
            logger(Info);
            std::printf("Calculated speed: %.2f mm/s\n", speed);
            // 决定角度
            double goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
            logger(Info);
            std::printf("Calculated angle: %.2f degrees\n", goA);

            double pat_x_correction = 20 * cos(goA * Deg2Rad); // 计算x方向的修正
            double pat_y_correction = 20 * sin(goA * Deg2Rad); // 计算y方向的修正
            double pat_correction = sqrt(pat_x_correction * pat_x_correction + pat_y_correction * pat_y_correction);

            logger(Info);
            if (ball_pos.y > 0)
                std::cout << "Pat position (relative to ball): ("
                << -pat_x_correction << ", "
                << -pat_y_correction << ") mm, "
                << pat_correction << " mm.\n";
            else
                std::cout << "Pat position (relative to ball): ("
                << -pat_x_correction << ", "
                << -pat_y_correction << ") mm, "
                << pat_correction << " mm.\n";
        }

        return 0;
    }

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




    // 开始比赛进程
    while (true)
    {
        //输入击球信号
        std::cout << "输入“enter”开始击球" << std::endl;
        char ch = std::cin.get();
        std::cout << "开始击球！" << std::endl;
        // 准备动作
        if (config.if_prepare) prepare();

        logger(Info);


        if (ball_pos.y >= 100) // 位置在左侧的球，向4号桶击球
        {

            CartesianPose pos;
            mtx.lock();
            double initial_ball_x = ball_pos.x;
            double initial_ball_y = ball_pos.y;
            logger(Info);
            std::printf("Ball position: (%.2f,%.2f) \n", initial_ball_x, initial_ball_y);
            pos.tran.x = initial_ball_x;
            pos.tran.y = initial_ball_y;
            mtx.unlock();
            pos.tran.z = 240;
            Point target;
            if (transform_correction.valid) ///// 什么时候 valid
            {
                target = transformTarget(bucket_position[4].first, bucket_position[4].second, transform_correction);
                ///// 为什么要 transformTarget
                logger(Info);
                std::cout << "Target position transformed from (" << bucket_position[4].first << ", " << bucket_position[4].second << ") ";
                std::cout << "to (" << target.x << ", " << target.y << ").\n";
            }
            else
            {
                target.x = bucket_position[4].first;
                target.y = bucket_position[4].second;
                logger(Info);
                std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
            }

            int index;

            if (config.if_chunked)
            {
                if (ball_pos.x >= 100)
                    if (ball_pos.y >= 350)
                    {
                        index = 0;
                    }
                    else if (ball_pos.y >= 200) index = 1;
                    else index = 2;
                else
                    if (ball_pos.y >= 350)
                    {
                        index = 0;
                    }
                    else index = 2;
                target.x += config.x_offset[index];
                target.y += config.y_offset[index];
            }


            goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
            pos.tran.x -= 20 * cos(goA * Deg2Rad);
            pos.tran.y -= 20 * sin(goA * Deg2Rad);

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
                continue; // 跳出循环，不打球
            }
            if (ret == 0)
            {
                logger(Move);
                std::cout << "Motor preparing." << std::endl;
                ini_angle = 90;
                mkmotor.goToPos2(ini_angle, 1, 200, false);
                logger(Move);
                std::cout << "Jaka mini preparing." << std::endl;
                demo.joint_move(&jvalH, MoveMode::ABS, true, 7);
            }
            else
            {
                logger(Error);
                std::cout << "Inverse error!" << std::endl;
                continue;
            }
            JointValue jvalT;
            demo.get_joint_position(&jvalT); ///// 意义在哪
            Sleep(200);

            // 决定速度
            double speed = speedX(initial_ball_x, initial_ball_y, target.x, target.y);

            // 控制电机击球
            logger(Move);
            std::cout << "Motor hitting." << std::endl;
            mkmotor.goToPos2((ini_angle - 180 - ADDANGLE), 1, speed, false);
            Sleep(1500);

            BallTrajectoryInfo trajectory;
            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Recording trajectory.\n";
                trajectory = get2Speed(initial_ball_x);
            }

            demo.get_joint_position(&jvalT);
            jvalT.jVal[1] -= 50 * Deg2Rad;  // 在击球后抬起机械臂


            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Analyzing and updating calibration.\n";
                analyzeAndUpdateCalibration(trajectory, true);
            }

            // 然后再返回等待位置

            logger(Move);
            std::cout << "Jaka mini and motor returning to initial position." << std::endl;
            mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
            ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);

            if (config.if_chunked)
            {

                target.x -= config.x_offset[index];
                target.y -= config.y_offset[index];
            }
        }
        else if (ball_pos.y < -250)// 位置在右边的球，向5号桶击打
        {
            CartesianPose pos;
            mtx.lock();
            double initial_ball_x = ball_pos.x;
            double initial_ball_y = ball_pos.y;
            logger(Info);
            std::printf("Ball position: (%.2f, %.2f) \n", initial_ball_x, initial_ball_y);
            pos.tran.x = initial_ball_x;
            pos.tran.y = initial_ball_y;
            mtx.unlock();
            pos.tran.z = 240;
            Point target;
            if (transform_correction.valid)
            {
                target = transformTarget(bucket_position[5].first, bucket_position[5].second, transform_correction);
                logger(Info);
                std::cout << "Target position transformed from (" << bucket_position[5].first << ", " << bucket_position[5].second << ") ";
                std::cout << "to (" << target.x << ", " << target.y << ").\n";
            }
            else
            {
                target.x = bucket_position[5].first;
                target.y = bucket_position[5].second;
                logger(Info);
                std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
            }

            int index;

            if (config.if_chunked)
            {
                if (ball_pos.x >= 100)
                    if (ball_pos.y <= -400)
                    {
                        index = 6;
                    }
                    else index = 5;
                else
                    if (ball_pos.y <= -400)
                    {
                        index = 8;
                    }
                    else index = 7;
                target.x += config.x_offset[index];
                target.y += config.y_offset[index];
            }

            goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
            pos.tran.x -= 20 * cos(goA * Deg2Rad);
            pos.tran.y -= 20 * sin(goA * Deg2Rad);

            pos.rpy.rx = (90 - goA) * Deg2Rad;
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
            Sleep(200);

            // 决定速度
            double speed = speedX(initial_ball_x, initial_ball_y, target.x, target.y);

            // 控制电机击球
            logger(Move);
            std::cout << "Motor hitting." << std::endl;
            mkmotor.goToPos2((ini_angle + 180 + ADDANGLE), 1, speed, false);
            Sleep(1500);

            BallTrajectoryInfo trajectory;
            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Recording trajectory.\n";
                trajectory = get2Speed(initial_ball_x);
            }


            demo.get_joint_position(&jvalT);
            jvalT.jVal[1] -= 50 * Deg2Rad;


            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Analyzing and updating calibration.\n";
                analyzeAndUpdateCalibration(trajectory, true);
            }

            // 然后再返回等待位置

            logger(Move);
            std::cout << "Jaka mini and motor returning to initial position." << std::endl;
            mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
            ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);

            if (config.if_chunked)
            {

                target.x -= config.x_offset[index];
                target.y -= config.y_offset[index];
            }
        }
        //球在左中侧，向对侧5号桶击球
        else if (ball_pos.y > 0 && ball_pos.y < 150)
        {
            CartesianPose pos;
            mtx.lock();
            double initial_ball_x = ball_pos.x;
            double initial_ball_y = ball_pos.y;
            logger(Info);
            std::printf("Ball position: (%.2f,%.2f) \n", initial_ball_x, initial_ball_y);
            pos.tran.x = initial_ball_x;
            pos.tran.y = initial_ball_y;
            mtx.unlock();
            pos.tran.z = 240;
            Point target;
            if (transform_correction.valid) ///// 什么时候 valid
            {
                target = transformTarget(bucket_position[5].first, bucket_position[5].second, transform_correction);
                ///// 为什么要 transformTarget
                logger(Info);
                std::cout << "Target position transformed from (" << bucket_position[5].first << ", " << bucket_position[5].second << ") ";
                std::cout << "to (" << target.x << ", " << target.y << ").\n";
            }
            else
            {
                target.x = bucket_position[5].first;
                target.y = bucket_position[5].second;
                logger(Info);
                std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
            }
            goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
            pos.tran.x -= 20 * cos(goA * Deg2Rad);
            pos.tran.y -= 20 * sin(goA * Deg2Rad);

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
                continue; // 跳出循环，不打球
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
                continue;
            }
            JointValue jvalT;
            demo.get_joint_position(&jvalT); ///// 意义在哪
            Sleep(200);

            // 决定速度
            double speed = speedX(initial_ball_x, initial_ball_y, target.x, target.y);

            // 控制电机击球
            logger(Move);
            std::cout << "Motor hitting." << std::endl;
            mkmotor.goToPos2((ini_angle - 180 - ADDANGLE), 1, speed, false);
            Sleep(1500);

            BallTrajectoryInfo trajectory;
            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Recording trajectory.\n";
                trajectory = get2Speed(initial_ball_x);
            }


            demo.get_joint_position(&jvalT);
            jvalT.jVal[1] -= 50 * Deg2Rad;  // 在击球后抬起机械臂


            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Analyzing and updating calibration.\n";
                analyzeAndUpdateCalibration(trajectory, true);
            }

            // 然后再返回等待位置

            logger(Move);
            std::cout << "Jaka mini and motor returning to initial position." << std::endl;
            mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
            ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
        }
        //球在右中侧，向4号桶击球
        else {
            CartesianPose pos;
            mtx.lock();
            double initial_ball_x = ball_pos.x;
            double initial_ball_y = ball_pos.y;
            logger(Info);
            std::printf("Ball position: (%.2f,%.2f) \n", initial_ball_x, initial_ball_y);
            pos.tran.x = initial_ball_x;
            pos.tran.y = initial_ball_y;
            mtx.unlock();
            pos.tran.z = 240;
            Point target;
            if (transform_correction.valid) ///// 什么时候 valid
            {
                target = transformTarget(bucket_position[4].first, bucket_position[4].second, transform_correction);
                ///// 为什么要 transformTarget
                logger(Info);
                std::cout << "Target position transformed from (" << bucket_position[4].first << ", " << bucket_position[4].second << ") ";
                std::cout << "to (" << target.x << ", " << target.y << ").\n";
            }
            else
            {
                target.x = bucket_position[4].first;
                target.y = bucket_position[4].second;
                logger(Info);
                std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
            }
            goA = goAvalue(ball_pos.x, ball_pos.y, target.x, target.y);
            pos.tran.x -= 20 * cos(goA * Deg2Rad);
            pos.tran.y -= 20 * sin(goA * Deg2Rad);

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
                continue; // 跳出循环，不打球
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
                continue;
            }
            JointValue jvalT;
            demo.get_joint_position(&jvalT); ///// 意义在哪
            Sleep(200);

            // 决定速度
            double speed = speedX(initial_ball_x, initial_ball_y, target.x, target.y);

            // 控制电机击球
            logger(Move);
            std::cout << "Motor hitting." << std::endl;
            mkmotor.goToPos2((ini_angle - 180 - ADDANGLE), 1, speed, false);
            Sleep(1500);

            BallTrajectoryInfo trajectory;
            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Recording trajectory.\n";
                trajectory = get2Speed(initial_ball_x);
            }


            demo.get_joint_position(&jvalT);
            jvalT.jVal[1] -= 50 * Deg2Rad;  // 在击球后抬起机械臂


            if (config.if_self_correction)
            {
                logger(Info);
                std::cout << "Analyzing and updating calibration.\n";
                analyzeAndUpdateCalibration(trajectory, true);
            }

            // 然后再返回等待位置

            logger(Move);
            std::cout << "Jaka mini and motor returning to initial position." << std::endl;
            mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
            ret = demo.joint_move(&jval, MoveMode::ABS, true, 7);
        }

        logger(Info);
        std::cout << "Hitting end." << std::endl;
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
                    pp.x = ball.x() - 20; // 调整 x 位置，减去 20 个单位
                    pp.y = ball.y();
                    // pp.t = (detection.t_capture() / 10 - (int)(detection.t_capture() / 10)); // 计算时间信息
                    pp.t = detection.t_capture(); // 使用完整的时间戳
                    mtx.lock();                   // 加锁以确保线程安全
                    ball_pos = pp;                // 更新全局的球位置
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
    cout << "fake";

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

// 计算离桌点和电机速度
double speedX(const double ball_x, const double ball_y, const double posi_x, const double posi_y)
{
    // 如果已经有校准数据，使用变换后的目标点
    Point target;
    //target.x = posi_x;
    //target.y = posi_y;
    if (transform_correction.valid)
    {
        target = transformTarget(posi_x, posi_y, transform_correction);
    }
    else
    {
        target.x = posi_x;
        target.y = posi_y;
    }

    // 计算离桌点y坐标（球初始位置和目标点连线在x=2740处的y值）
    double exit_y = ball_y + (target.y - ball_y) *
        (2740 - ball_x) / ((target.x) - ball_x);

    logger(Info);
    std::cout << "Calculation: The ball will leave table at (2740, " << exit_y << ").\n";

    // 使用调整后的目标点计算速度
    double required_velocity = calculateRequiredExitSpeed(
        ball_x, ball_y,
        2740, exit_y,
        target.x, target.y);

    // 转换为电机速度
    double motor_speed = (required_velocity / 1000.0 / config.v) * Rad2Deg;

    // 限制最大速度
    double final_speed = min(motor_speed, 1200.0);
    logger(Info);
    std::cout << "Calculation: Motor speed is " << final_speed << " deg/s\n";

    return final_speed;
}


// 计算与落点的距离
double getdistance(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double sqr = (ball_x - posi_x) * (ball_x - posi_x) + (ball_y - posi_y) * (ball_y - posi_y);
    return sqrt(sqr);
}



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
    const double MAX_COLLECTION_TIME = 3.0; // 最大收集时间（秒）
    const double MIN_CHANGE = 0.5;          // 最小位置变化阈值
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
            double exit_y = last_point.y + (bucket_position[bucket_index].second - last_point.y) *
                (2740 - last_point.x) / ((bucket_position[bucket_index].first + 2740) - last_point.x);

            // 输出详细的速度分析
            logger(Info);
            cout << "Calculation using last " << validCount << " points: " << "Original speed is (" << avgSpeedX << ", "
                << avgSpeedY << ") mm/s, speed considering fiction is " << result.exit_speed << " mm/s with direction at "
                << result.exit_angle << " degrees." << std::endl;

            // 使用修正后的速度预测落点
            Point predicted = predictLandingPoint(2740, exit_y,
                result.exit_speed,
                result.exit_angle, 660);
            result.actual_landing = predicted;
            result.valid = true;

            //轨迹分析结果
            logger(Info);
            std::cout << "Calculation: Last observation point: (" << last_point.x << ", " << last_point.y << "), "
                << "fitted exit point: (2740, " << exit_y << "), "
                << "exit speed: " << result.exit_speed << " mm/s, "
                << "exit angle: " << result.exit_angle << " degrees, "
                << "predicted landing point: (" << result.actual_landing.x << ", "
                << result.actual_landing.y << ").\n";

            // 记录到日志
            debug_log << "轨迹数据,"
                << result.exit_speed << ","
                << result.exit_angle << ","
                << exit_y << ","
                << result.actual_landing.x << ","
                << result.actual_landing.y << "\n";
        }
    }
    else
    {
        logger(Error);
        cout << "No sufficient data.\n";
    }

    return result;
}

// 落点预测
Point predictLandingPoint(double exit_x, double exit_y, double exit_speed, double exit_angle, double exit_height)
{
    const double dt = 0.001; // 时间步长(s)
    double t = 0;
    double x = exit_x;
    double y = exit_y;
    double h = exit_height;

    // 计算总时间，假设自由落体运动
    double total_time = sqrt(2.0 * exit_height / GRAVITY);

    // 初始速度分量
    double vx = exit_speed * cos(exit_angle * Deg2Rad);
    double vy = exit_speed * sin(exit_angle * Deg2Rad);
    double vz = 0; // 初始垂直速度

    // 模拟运动直到球落地或达到总时间
    while (h > 0 && t < total_time)
    {
        // 更新垂直速度
        vz -= GRAVITY * dt;

        // 更新位置
        x += vx * dt;
        y += vy * dt;
        h += vz * dt;

        // 更新时间
        t += dt;

        // 检查数值是否合理
        if (std::isnan(x) || std::isnan(y) || std::isnan(h) ||
            std::abs(x) > 1e6 || std::abs(y) > 1e6 || std::abs(h) > 1e6)
        {
            logger(Error);
            std::cout << "Calculation error!\n";
            break;
        }
    }

    // 返回预测的落点信息
    Point landing;
    landing.x = x;
    landing.y = y;
    landing.t = t;
    landing.vx = vx;
    landing.vy = vy;

    return landing;
}

// 计算所需出球速度
double calculateRequiredExitSpeed(double start_x, double start_y, double exit_x, double exit_y, double target_x, double target_y)
{
    double acc = calculateAccFactor(start_y);

    // 计算飞行时间
    double flight_time = sqrt(2.0 * 660.0 / GRAVITY);

    // 计算水平飞行距离 (mm)
    double flight_distance = sqrt(
        pow(target_x - exit_x, 2) +
        pow(target_y - exit_y, 2));

    double s = getdistance(start_x, start_y, target_x, target_y);
    double s1 = ((2740 - start_x) / cos(goAvalue(start_x, start_y, target_x, target_y) * Deg2Rad)) / acc;
    double dis = s - s1;



    // 计算基础速度 (mm/s)
    // double base_speed = flight_distance / flight_time;
    double base_speed = dis / flight_time;

    // 防止 base_speed 过小
    if (base_speed < 1e-6)
    {
        logger(Error);
        std::cout << "Calculated speed is too small. Adjusted to 1 mm/s.\n";
        base_speed = 1.0; // 设置一个最小值
    }

    // 5. 计算桌面运动部分
    double table_distance = sqrt(pow(exit_x - start_x, 2) - pow(exit_y - start_y, 2)); // mm

    // 打印详细计算过程
    logger(Info);
    std::cout << "Adapt: "
        << "Flight time: " << flight_time << " s, "
        << "Flight distance: " << flight_distance << " mm, "
        << "Adjusted distance: " << dis << " mm with acceleration factor " << acc << ", "
        << "Base speed: " << base_speed << " mm/s, "
        << "Table distance: " << table_distance << " mm.\n";

    return base_speed;
}

