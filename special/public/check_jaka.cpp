#pragma comment(lib, "winmm.lib")
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
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jkerr.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jktypes.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/JAKAZuRobot.h"
#include <chrono>

#include <thread>
#include <mutex>
#include <mmsystem.h>
#include <cmath>
#include <complex>
#include <conio.h> // Windows specific
#include <list>
#include <queue>
#include <vector>
#include <fstream>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;


#define pi222 3.1415926
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)

#define TABLE_LENGTH 2740
#define TABLE_WIDTH 1525
#define GOAL_WIDTH 80
#define GOAL_THICKNESS 20


enum Logger { Info, Error, Move, Hint, Input };

namespace detail {
    template <typename... Args>
    inline void print_all(std::ostream& os, const Args&... args) {
        int _[] = { 0, ((void)(os << args), 0)... };
        (void)_;
    }
}

template <typename... Args>
void logger(Logger l, const Args&... args) {
    switch (l) {
    case Info:  std::cout << "\t[Info]  "; break;
    case Error: std::cout << "\t\033[0m\033[1;31m[Error] "; break;
    case Move:  std::cout << "\t\033[0m\033[1;32m[Move]  "; break;
    case Input: std::cout << "\t[Input] "; break;
    case Hint:  std::cout << "\t\033[0m\033[1;36m[Hint]  "; break;
    }

    detail::print_all(std::cout, args...);

    switch (l) {
    case Error: case Move: case Hint:
        std::cout << "\033[0m";
        break;
    default:
        break;
    }
    std::cout << '\n';
}


struct Config {
    // config.csv
    char* jaka_ip; // jaka mini ip
    int motor_com; // COM port for the motor
    char* repo_path;
    bool if_referee;

    // motor speed
    const double speed = 740; // Motor hitting speed

    // paddle position
    const double paddle_z_left = 240, paddle_z_right = 240; // Paddle heights for left and right sides     177
    const double paddle_ball_center_distance = 60; // Distance from paddle center to ball center

    // jaka mini position
    const bool if_jaka_offset = false;
    const double jaka_offset_x = 0;
    const double jaka_offset_y = 0;

    // IFs
    const bool if_add_delta = false; // If add offset for side goals
    const bool if_record_trajectory = true; // If record trajectory
    const bool if_vision_correction = false;

    // Adjacent learning
    const bool if_adjacent_learning = true; // If use adjacent learning
    const int offset_class = 1; // 1: , 2: Exponential
    const double correction_coefficient = 0.8;
    const int deviser_x = 50, deviser_y = 100; // Deviser for adjacent learning, used to calculate the distance

} config;


struct Point
{
    double x;
    double y;
    double t;  // time
    double vx;
    double vy;
};

Point ball_position;         // location of the ball
int balls_n;            // The amount of balls that are detected
std::mutex mtx;        // mutex for protecting shared resources

JAKAZuRobot demo;
MotorIK mkmotor;

void get_vision();      // Function to get vision data


void read_config()
{
    ifstream config_file("config.csv");
    string line;

    getline(config_file, line); // Skip the header line
    getline(config_file, line); // Skip the header line

    stringstream ss(line);
    config.motor_com = 0;
    config.jaka_ip = new char[16];
    config.repo_path = new char[256];
    config.if_referee = false;
    ss >> config.motor_com;
    ss.ignore(1, ','); // Ignore the comma
    ss.getline(config.jaka_ip, 16, ',');
    ss.getline(config.repo_path, 256, ',');
    ss >> config.if_referee;
    // Log the configuration

    logger(Info, "Configuration loaded:",
        "Motor COM:", config.motor_com,
        "; Jaka IP:", config.jaka_ip,
        "; Repo Path:", config.repo_path,
        "; If Referee:", config.if_referee);
}

int main()
{
    read_config(); // Read configuration from config.csv

    // Another thread to get vision data
    std::thread t1(get_vision);
    t1.detach();
    logger(Info, "Vision system connected.");

    demo.login_in(config.jaka_ip);
    logger(Info, "Jaka mini connected.");

    demo.power_on();
    demo.enable_robot();
    logger(Info, "Jaka mini empowered.");


    mkmotor.ID = 1;
    mkmotor.reduceRatio = 10;
    mkmotor.setSerial(config.motor_com, 115200);
    mkmotor.start();
    logger(Info, "Motor connected.");

    JointValue jval;
    jval.jVal[0] = 0 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 90 * Deg2Rad;
    jval.jVal[3] = 0 * Deg2Rad;
    jval.jVal[4] = 0 * Deg2Rad;
    jval.jVal[5] = 0 * Deg2Rad;

    logger(Move, "Jaka mini initializing.");
    demo.joint_move(&jval, MoveMode::ABS, true, 5);

    logger(Move, "Motor initializing.");
    double ini_dianji_degree = 0;
    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);

    Point recorded_ball_pos;

    char ch = '\0';
    logger(Hint, "Pre ENTER to check balls_n...");
    while (ch != '\n')
    {
        ch = std::cin.get();
    }

    logger(Info, "Checking the amount of balls detected.");
    int threshold = 300; // Threshold time for ball detection
    for (int i = 0; i < threshold; i++)
    {
        if (balls_n == 1) i++;
        else if (balls_n > 1)
        {
            logger(Error, "Multiple balls detected! Please ensure only one ball is present.");
            break;
        }
        else if (balls_n == 0)
        {
            logger(Error, "No ball detected! Please ensure a ball is present.");
            break;
        }
    }

    while (true)
    {
        mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
        jval.jVal[0] = 0 * Deg2Rad;
        jval.jVal[1] = 0 * Deg2Rad;
        jval.jVal[2] = 90 * Deg2Rad;
        jval.jVal[3] = 0 * Deg2Rad;
        jval.jVal[4] = 0 * Deg2Rad;
        jval.jVal[5] = 0 * Deg2Rad;

        logger(Move, "Jaka mini initializing.");
        demo.joint_move(&jval, MoveMode::ABS, true, 5);

        logger(Move, "Motor initializing.");

        logger(Hint, "Pre ENTER to hit...");
        char ch = std::cin.get();
        if (ch != '\n') continue;

        mtx.lock();
        recorded_ball_pos = ball_position;
        mtx.unlock();

        if (fabs(recorded_ball_pos.x) < 0.01 && fabs(recorded_ball_pos.y) < 0.01) {
            logger(Error, "No ball detected!");
            continue;
        }
        logger(Info, "Ball position: (", recorded_ball_pos.x, ", ", recorded_ball_pos.y, ")");

        // 5个固定角度测试
        const std::vector<double> test_angles = { -50, -30, -10, 0, 10, 30, 50 };
        for (double goA : test_angles) {
            logger(Info, "Testing angle: ", goA, " degrees.");

            CartesianPose pos;

            // 基于锁定位置计算（不再需要target）
            pos.tran.x = recorded_ball_pos.x - config.paddle_ball_center_distance * cos(goA * Deg2Rad);
            pos.tran.y = recorded_ball_pos.y - config.paddle_ball_center_distance * sin(goA * Deg2Rad) + config.jaka_offset_y;
            pos.tran.z = recorded_ball_pos.y > 0 ? config.paddle_z_left : config.paddle_z_right;

            // 固定角度击打逻辑
            if (recorded_ball_pos.y > 0) {
                pos.rpy.rx = (-90 + goA) * Deg2Rad;  // 右侧击球角度
                pos.rpy.ry = -90 * Deg2Rad;
            }
            else {
                pos.rpy.rx = (90 - goA) * Deg2Rad;   // 左侧击球角度 
                pos.rpy.ry = 90 * Deg2Rad;
            }
            pos.rpy.rz = 0;

            // 运动执行（保持原有电机控制）
            JointValue jvalH;
            try { demo.kine_inverse(&jval, &pos, &jvalH); }
            catch (std::runtime_error) {
                logger(Error, "Inverse error!");
                continue;
            }

            logger(Move, "Preparing...");
            mkmotor.goToPos2(-90, 1, 200, false);  // 固定初始角度
            demo.joint_move(&jvalH, MoveMode::ABS, true, 1);

            Sleep(200);
            logger(Move, "Hitting...");
            mkmotor.goToPos2((recorded_ball_pos.y > 0) ? -180 : 0, 0, 500, false);
        }
    }
    return 0;
}


// Constantly get vision data in a separate thread
void get_vision()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    std::vector<Point> ballP;

    double a_x, b_x, c_x, a_y, b_y, c_y;

    if (config.if_vision_correction)
    {
        ifstream correction_file(string(config.repo_path) + "\\special\\public\\visual_correction.csv");
        string line;
        getline(correction_file, line); // Skip the header line
        getline(correction_file, line); // Read the first line

        stringstream ss(line);

        ss >> a_x;
        ss.ignore(1, ','); // Ignore the comma
        ss >> b_x;
        ss.ignore(1, ','); // Ignore the comma
        ss >> c_x;
        ss.ignore(1, ','); // Ignore the comma
        ss >> a_y;
        ss.ignore(1, ','); // Ignore the comma
        ss >> b_y;
        ss.ignore(1, ','); // Ignore the comma
        ss >> c_y;
    }
    else
    {
        a_x = 1.0; b_x = 0.0; c_x = 0.0;
        a_y = 0.0; b_y = 1.0; c_y = 0.0;
    }

    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_detection())
            {
                // Check if the packet contains detection data
                SSL_DetectionFrame detection = packet.detection();
                balls_n = detection.balls_size(); // Get the number of detected balls
                //std::cout << ball.x();
                // Update the ball position
                for (int i = 0; i < balls_n; i++)
                {
                    SSL_DetectionBall ball = detection.balls(i);
                    Point pp;
                    pp.x = a_x * ball.x() + b_x * ball.y() + c_x; // Apply correction
                    pp.y = a_y * ball.x() + b_y * ball.y() + c_y; // Apply correction
                    pp.t = detection.t_capture();
                    mtx.lock();
                    ball_position = pp;
                    mtx.unlock();

                    ballP.push_back(pp);
                }
            }
        }
    }
}